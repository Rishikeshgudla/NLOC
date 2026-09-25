"""
N-LOC golden model (M0) - integer / fixed-point reference for the VHDL design.
Fixed point: Q2.6 style, 1.0 == 64. Pixels are 8-bit gray >> 2 -> 0..63.
Every operation here is one the FPGA can do (add, abs, compare, shift, LUT).
"""
import numpy as np

# ---- Parameters (scaled down to fit Basys-3 BRAM; change here only) ----
ONE   = 64          # 1.0 in fixed point
NL    = 16          # landmarks per image
PATCH = 144         # 12 x 12 pixels per landmark
NS    = 32          # signature-layer neurons (max distinct landmark types)
NA    = 3           # azimuth groups in the SWM
NAZ   = 180         # azimuth-layer neurons (2 deg each)
NP    = 30          # place cells (max learnable places)
NSWM  = NS * NA
VIG   = 56          # vigilance: below this a new signature neuron is recruited
SIGMA = 3           # azimuth bubble width, in neurons

# Azimuth bubble LUT: activity vs circular distance (in neurons) from the centre
_d  = np.arange(NAZ // 2 + 1)
BUB = np.round(ONE * np.exp(-_d**2 / (2.0 * SIGMA**2))).astype(np.int32)


def azimuth(theta_deg):
    """Azimuth layer: activity bubble around theta (Eq. 2 result), 180 values.
    Center uses round-half-up ((theta+1)//2), matching the VHDL shifter
    (theta+1)>>1 exactly -- NOT Python's banker's-rounding round(), which is
    harder to reproduce in hardware and gives no accuracy benefit here."""
    c = (((int(round(theta_deg)) % 360) + 1) // 2) % NAZ
    d = np.abs(np.arange(NAZ) - c)
    return BUB[np.minimum(d, NAZ - d)]


class NLOC:
    def __init__(self):
        self.sl_w = np.zeros((NS, PATCH), np.int32); self.sl_n = 0
        self.pc_w = np.zeros((NP, NSWM), np.int32);  self.pc_n = 0

    # --- Signature layer + WTA (Eq. 1). Winner = min SAD (== max activity) ---
    def signature(self, patch, learn):
        p = np.asarray(patch, np.int32) >> 2
        if self.sl_n:
            sad = np.abs(self.sl_w[:self.sl_n] - p).sum(1)
            i = int(np.argmin(sad))
            s = ONE - ((int(sad[i]) * 455) >> 16)      # 455/65536 ~ 1/144
        else:
            i, s = -1, -1
        if learn and s < VIG and self.sl_n < NS:       # recruit a new neuron
            i = self.sl_n; self.sl_w[i] = p; self.sl_n += 1; s = ONE
        return (i, s) if i >= 0 else None

    # --- Azimuth (Eq. 2) + Spatial Working Memory (Eq. 3-4, simplified) ---
    def swm(self, landmarks, yaw, learn):
        x = np.zeros((NS, NA), np.int32)
        for patch, ego in landmarks:
            r = self.signature(patch, learn)
            if r is None:
                continue
            i, s = r
            a = azimuth((ego + yaw) % 360).reshape(NA, NAZ // NA).max(1)
            x[i] = np.minimum(ONE, x[i] + ((s * a) >> 6))  # saturating, no sigmoid
        return x.ravel()

    # --- Place cells + WTA (Eq. 5). Winner = min SAD (== max activity) ---
    def learn(self, landmarks, yaw):
        self.pc_w[self.pc_n] = self.swm(landmarks, yaw, True); self.pc_n += 1

    def query(self, landmarks, yaw):
        x = self.swm(landmarks, yaw, False)
        sad = np.abs(self.pc_w[:self.pc_n] - x).sum(1)
        return int(np.argmin(sad)), sad


# ---------------- Synthetic world for testing ----------------
def make_world(rng, n_proto=24):
    protos = rng.integers(0, 256, (n_proto, PATCH))
    places = [(rng.choice(n_proto, NL, replace=False),   # which landmarks
               rng.uniform(0, 360, NL))                  # absolute bearing of each
              for _ in range(NP)]
    return protos, places


def view(protos, place, yaw, rng, pix_noise=0, ang_noise=0, drop=0):
    ids, bearing = place
    keep = rng.permutation(NL)[: NL - drop]
    lm = []
    for k in keep:
        patch = np.clip(protos[ids[k]] + rng.normal(0, pix_noise, PATCH), 0, 255)
        ego = (bearing[k] - yaw + rng.normal(0, ang_noise)) % 360
        lm.append((patch.astype(np.int32), ego))
    return lm


if __name__ == "__main__":
    rng = np.random.default_rng(1)
    protos, places = make_world(rng)
    net = NLOC()
    for pl in places:                                    # learning mode
        y = rng.uniform(0, 360)
        net.learn(view(protos, pl, y, rng), y)
    print(f"learned {net.pc_n} places, {net.sl_n} signature neurons")

    def run(label, trials=20, **kw):
        ok = tot = 0
        for pid, pl in enumerate(places):
            for _ in range(trials):
                y = rng.uniform(0, 360)
                yaw_meas = y + rng.normal(0, kw.get("yaw_err", 0))
                lm = view(protos, pl, y, rng, kw.get("pix", 0),
                          kw.get("ang", 0), kw.get("drop", 0))
                ok += net.query(lm, yaw_meas % 360)[0] == pid; tot += 1
        print(f"{label:<44s}{100.0 * ok / tot:6.1f} %")

    run("clean views, random yaw")
    run("pixel noise sigma=10")
    run("+ angle noise 2deg, yaw error 2deg", pix=10, ang=2, yaw_err=2)
    run("+ 2 landmarks missing", pix=10, ang=2, yaw_err=2, drop=2)


# ---------------- M2 test-vector generator (Signature Layer only) ----------------
def dump_signature_vectors(path, n_calls=60, seed=2):
    """
    Drives NLOC.signature() directly (bypasses azimuth/SWM/place) and writes a
    plain-text vector file the VHDL testbench reads with textio:
        <line 1>            : n_calls
        <one line per call> : learn(0/1) idx(-1 if none) score(-1 if none) <144 pixel values 0..255>
    idx = -1 / score = -1 means "no stored signature yet" (query before any learn).
    """
    rng = np.random.default_rng(seed)
    net = NLOC()
    lines = []
    for k in range(n_calls):
        # first 8 calls: forced learn, so there is always something to query later
        learn = 1 if (k < 8 or rng.random() < 0.35) else 0
        if rng.random() < 0.5 and k > 5:
            # reuse a previously learned patch (± pixel noise) so matches happen
            src = rng.integers(0, net.sl_n) if net.sl_n else 0
            patch = np.clip(net.sl_w[src].astype(np.int32) * 4
                             + rng.normal(0, 6, PATCH), 0, 255).astype(np.int32)
        else:
            patch = rng.integers(0, 256, PATCH)
        r = net.signature(patch, bool(learn))
        idx, score = r if r is not None else (-1, -1)
        lines.append(f"{learn} {idx} {score} " + " ".join(map(str, patch.tolist())))
    with open(path, "w") as f:
        f.write(f"{n_calls}\n")
        f.write("\n".join(lines) + "\n")
    print(f"wrote {n_calls} signature test vectors to {path} "
          f"({net.sl_n} signatures learned)")


if __name__ == "__main__" and False:
    pass  # placeholder, real __main__ block is above; kept for import safety


# ---------------- M3 test-vector generator (Azimuth Layer) ----------------
def dump_azimuth_vectors(path, thetas=None, seed=3):
    """
    Each line: theta(0..359) then NAZ=180 expected bubble values.
    Covers every theta 0..359 by default (exhaustive -- there are only 360
    possible inputs and the module is pure combinational-ish, so this is cheap).
    """
    if thetas is None:
        thetas = list(range(360))          # exhaustive sweep
    with open(path, "w") as f:
        f.write(f"{len(thetas)}\n")
        for th in thetas:
            row = azimuth(th)
            f.write(f"{th} " + " ".join(map(str, row.tolist())) + "\n")
    print(f"wrote {len(thetas)} azimuth test vectors to {path}")


# ---------------- M4 test-vector generator (Spatial Working Memory) ----------------
def _swm_direct(landmarks):
    """landmarks: list of (idx, score, az_arr[NAZ]). Same math as NLOC.swm(),
    but takes idx/score/azimuth directly instead of calling signature()/azimuth()
    -- M2 and M3 are already verified on their own, so M4's test isolates just
    the group-max reduction + saturating accumulate that SWM itself is responsible for."""
    x = np.zeros((NS, NA), np.int32)
    for idx, score, az in landmarks:
        a = np.asarray(az).reshape(NA, NAZ // NA).max(1)
        x[idx] = np.minimum(ONE, x[idx] + ((score * a) >> 6))
    return x.ravel()


def dump_swm_vectors(path, n_images=25, seed=4):
    rng = np.random.default_rng(seed)
    images = []

    for _ in range(n_images):                     # random images
        nlm = rng.integers(1, 7)
        lms = []
        for _ in range(nlm):
            idx = int(rng.integers(0, NS))
            score = int(rng.integers(0, ONE + 1))
            az = rng.integers(0, ONE + 1, NAZ)
            lms.append((idx, score, az))
        images.append(lms)

    # crafted edge cases, appended after the random ones
    hi = np.full(NAZ, ONE, dtype=np.int32)          # max-everywhere bubble
    lo = np.zeros(NAZ, dtype=np.int32)
    images.append([(5, ONE, hi)])                                    # exact single-shot saturation (64*64>>6=64)
    images.append([(5, ONE, hi), (5, ONE, hi), (5, ONE, hi)])          # repeated hits on same idx -> must clamp at 64
    images.append([(0, 0, hi), (1, ONE, lo)])                          # zero score / zero azimuth -> no contribution
    images.append([(7, 40, hi), (7, 30, hi)])                          # partial then partial -> sums but may or may not clamp
    images.append([(3, ONE, hi), (9, ONE, hi), (3, 10, lo)])           # two different idx + a no-op third hit

    with open(path, "w") as f:
        f.write(f"{len(images)}\n")
        for lms in images:
            f.write(f"{len(lms)}\n")
            for idx, score, az in lms:
                f.write(f"{idx} {score} " + " ".join(map(str, np.asarray(az).tolist())) + "\n")
            f.write(" ".join(map(str, _swm_direct(lms).tolist())) + "\n")
    print(f"wrote {len(images)} SWM test images to {path}")


# ---------------- M5 test-vector generator (Place Cell Layer + WTA) ----------------
def dump_place_vectors(path, n_calls=40, seed=5, np_cap=NP):
    """
    Drives a standalone Place Cell model (same rule as NLOC.learn()/query(), but
    fed raw NSWM-length vectors directly -- M4's SWM math is already verified on
    its own, so this isolates just the match-or-append + argmin logic).
      LEARN : unconditionally appends the vector as a new place (until capacity).
      QUERY : finds the stored place with smallest SAD (ties -> first/lowest index).
    Each line: learn(0/1) idx(-1=none) sad(-1=none) full(0/1) <NSWM values 0..64>
      idx/sad=-1        -> nothing stored yet (query with pc_n==0)
      full=1            -> learn attempted with no capacity left (ignored)
    """
    rng = np.random.default_rng(seed)
    pc_w = np.zeros((np_cap, NSWM), np.int32)
    pc_n = 0
    lines = []

    def call(vec, learn):
        nonlocal pc_n
        vec = np.asarray(vec, np.int32)
        idx, sad, full = -1, -1, 0
        if pc_n > 0:
            sads = np.abs(pc_w[:pc_n] - vec).sum(1)
            idx = int(np.argmin(sads)); sad = int(sads[idx])
        if learn:
            if pc_n < np_cap:
                pc_w[pc_n] = vec; idx = pc_n; sad = 0; pc_n += 1
            else:
                full = 1
        lines.append(f"{int(learn)} {idx} {sad} {full} " + " ".join(map(str, vec.tolist())))

    # 1: query before anything is learned -> none
    call(rng.integers(0, ONE + 1, NSWM), False)

    # 2: a run of ordinary learns and queries
    for k in range(n_calls):
        learn = 1 if (k < 5 or rng.random() < 0.4) else 0
        if rng.random() < 0.5 and pc_n > 0:
            src = rng.integers(0, pc_n)
            vec = np.clip(pc_w[src] + rng.integers(-8, 9, NSWM), 0, ONE)
        else:
            vec = rng.integers(0, ONE + 1, NSWM)
        call(vec, bool(learn))

    # 3: deliberate tie -- two identical places, query exactly between them -> picks the first
    twin = rng.integers(0, ONE + 1, NSWM)
    call(twin, True)          # place A
    call(twin, True)          # place B, identical to A
    call(twin, False)         # query -> must report idx = A (lower index), sad = 0

    # 4: fill remaining capacity, then two more learns that must be rejected (full)
    while pc_n < np_cap:
        call(rng.integers(0, ONE + 1, NSWM), True)
    call(rng.integers(0, ONE + 1, NSWM), True)     # full=1 expected
    call(rng.integers(0, ONE + 1, NSWM), True)     # full=1 expected
    call(pc_w[0], False)                            # query still works after being full

    with open(path, "w") as f:
        f.write(f"{len(lines)}\n" + "\n".join(lines) + "\n")
    print(f"wrote {len(lines)} place-layer test calls to {path} ({pc_n}/{np_cap} places used)")
