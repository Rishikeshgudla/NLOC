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
    """Azimuth layer: activity bubble around theta (Eq. 2 result), 180 values."""
    c = int(round(theta_deg / 2.0)) % NAZ
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
