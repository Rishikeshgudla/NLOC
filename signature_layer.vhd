-- signature_layer.vhd : M2 - Signature Layer + Winner-Takes-All (recruit-or-match)
--
-- Streams in one 144-pixel landmark patch (top 6 bits of each 8-bit pixel are
-- used, matching golden model's ">> 2"), then either:
--   QUERY (learn='0') : finds the stored signature with the smallest SAD
--                       (= highest activity) and reports its index + score.
--   LEARN (learn='1') : does the same match, and if the best score is below
--                       VIGILANCE (or nothing is stored yet), recruits a new
--                       signature neuron instead of matching an old one.
-- sig_none = '1' means "no stored signature and not learning" (nothing to report).
-- Mirrors nloc_golden.py's NLOC.signature() exactly (see sim/tb_signature_layer.vhd).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity signature_layer is
  generic (
    NS       : positive := 32;     -- max signature neurons
    PATCH    : positive := 144;    -- pixels per landmark
    VIG      : natural  := 56;     -- vigilance threshold (0..64 scale)
    ONE      : natural  := 64;     -- fixed-point 1.0
    SAD_MULT : natural  := 455     -- score = ONE - (sad*SAD_MULT) >> 16
  );
  port (
    clk       : in  std_logic;
    rst       : in  std_logic;

    learn     : in  std_logic;                       -- mode for THIS landmark
    pix_data  : in  std_logic_vector(7 downto 0);
    pix_idx   : in  std_logic_vector(7 downto 0);     -- 0..PATCH-1
    pix_valid : in  std_logic;
    pix_last  : in  std_logic;                        -- with pix_valid on last pixel

    sig_valid : out std_logic;                        -- 1-clock pulse, result ready
    sig_idx   : out std_logic_vector(4 downto 0);      -- winner neuron 0..NS-1
    sig_score : out std_logic_vector(6 downto 0);      -- 0..64
    sig_new   : out std_logic;                         -- '1' if this call recruited
    sig_none  : out std_logic                          -- '1' if nothing to report
  );
end entity;

architecture rtl of signature_layer is
  constant AW : positive := 6;   -- ceil(log2(PATCH)) = 8 needed, use integer counters instead

  type buf_t is array (0 to PATCH - 1) of unsigned(5 downto 0);
  signal buf : buf_t;

  -- flat weight memory: neuron n, pixel a -> address n*PATCH + a
  type wmem_t is array (0 to NS * PATCH - 1) of unsigned(5 downto 0);
  signal wmem : wmem_t := (others => (others => '0'));

  type state_t is (IDLE, COMPARE, DECIDE, RECRUIT, DONE);
  signal state : state_t := IDLE;

  signal sl_n      : integer range 0 to NS := 0;       -- neurons learned so far
  signal n_cnt      : integer range 0 to NS := 0;       -- neuron being compared
  signal a_cnt      : integer range 0 to PATCH := 0;    -- pixel within neuron / recruit
  signal sad_acc    : unsigned(15 downto 0) := (others => '0');
  signal best_sad   : unsigned(15 downto 0) := (others => '1');
  signal best_idx   : integer range 0 to NS - 1 := 0;
  signal have_best  : std_logic := '0';
  signal learn_r    : std_logic := '0';
begin
  process (clk)
    variable diff    : unsigned(6 downto 0);
    variable prod     : unsigned(23 downto 0);
    variable score_v  : integer;
  begin
    if rising_edge(clk) then
      sig_valid <= '0'; sig_new <= '0'; sig_none <= '0';

      if rst = '1' then
        state <= IDLE; sl_n <= 0;
      else
        case state is

          when IDLE =>
            if pix_valid = '1' then
              buf(to_integer(unsigned(pix_idx))) <= unsigned(pix_data(7 downto 2));
              learn_r <= learn;
              if pix_last = '1' then
                n_cnt <= 0; a_cnt <= 0; sad_acc <= (others => '0');
                best_sad <= (others => '1'); have_best <= '0';
                if sl_n = 0 then
                  state <= DECIDE;             -- nothing stored -> skip compare
                else
                  state <= COMPARE;
                end if;
              end if;
            end if;

          when COMPARE =>                       -- 1 pixel-compare per clock
            if buf(a_cnt) >= wmem(n_cnt * PATCH + a_cnt) then
              diff := ("0" & buf(a_cnt)) - ("0" & wmem(n_cnt * PATCH + a_cnt));
            else
              diff := ("0" & wmem(n_cnt * PATCH + a_cnt)) - ("0" & buf(a_cnt));
            end if;
            sad_acc <= sad_acc + resize(diff, 16);
            if a_cnt = PATCH - 1 then
              if have_best = '0' or (sad_acc + resize(diff, 16)) < best_sad then
                best_sad <= sad_acc + resize(diff, 16); best_idx <= n_cnt;
              end if;
              have_best <= '1';
              a_cnt <= 0; sad_acc <= (others => '0');
              if n_cnt = sl_n - 1 then
                state <= DECIDE;
              else
                n_cnt <= n_cnt + 1;
              end if;
            else
              a_cnt <= a_cnt + 1;
            end if;

          when DECIDE =>
            if sl_n = 0 then
              if learn_r = '1' then
                state <= RECRUIT; a_cnt <= 0;
              else
                sig_none <= '1'; sig_valid <= '1'; state <= IDLE;
              end if;
            else
              prod := resize(best_sad * to_unsigned(SAD_MULT, 9), 24);
              score_v := ONE - to_integer(prod(23 downto 16));
              if learn_r = '1' and sl_n < NS and score_v < VIG then
                state <= RECRUIT; a_cnt <= 0;
              else
                sig_idx <= std_logic_vector(to_unsigned(best_idx, 5));
                sig_score <= std_logic_vector(to_unsigned(score_v, 7));
                sig_valid <= '1'; state <= IDLE;
              end if;
            end if;

          when RECRUIT =>                        -- write buf into wmem(sl_n)
            wmem(sl_n * PATCH + a_cnt) <= buf(a_cnt);
            if a_cnt = PATCH - 1 then
              sig_idx <= std_logic_vector(to_unsigned(sl_n, 5));
              sig_score <= std_logic_vector(to_unsigned(ONE, 7));
              sig_new <= '1'; sig_valid <= '1';
              sl_n <= sl_n + 1;
              state <= IDLE;
            else
              a_cnt <= a_cnt + 1;
            end if;

          when DONE => state <= IDLE;
        end case;
      end if;
    end if;
  end process;
end architecture;