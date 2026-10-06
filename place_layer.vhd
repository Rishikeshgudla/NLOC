-- place_layer.vhd : M5 - Place Cell Layer + Winner-Takes-All
--
-- Streams in one 96-value Spatial Working Memory vector (from swm_layer's
-- dump_* stream), then:
--   LEARN, capacity available : unconditionally appends it as a NEW place
--                                (fast path -- no comparison needed).
--   LEARN, capacity full      : finds the nearest existing place instead
--                                (reported with place_full = '1', nothing stored).
--   QUERY                     : finds the stored place with the smallest SAD
--                                (= best match); ties go to the lowest index.
-- place_none = '1' : query with nothing stored yet (nothing to report).
-- Mirrors nloc_golden.py's dump_place_vectors() reference model exactly.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity place_layer is
  generic (
    NP   : positive := 30;      -- max place cells
    NSWM : positive := 96       -- values per SWM vector (NS*NA)
  );
  port (
    clk       : in  std_logic;
    rst       : in  std_logic;

    learn     : in  std_logic;                        -- mode for THIS call
    in_data   : in  std_logic_vector(6 downto 0);       -- SWM value, 0..64
    in_idx    : in  std_logic_vector(6 downto 0);       -- 0..NSWM-1
    in_valid  : in  std_logic;
    in_last   : in  std_logic;                          -- with in_valid on last value

    place_valid : out std_logic;                        -- 1-clock pulse, result ready
    place_idx   : out std_logic_vector(4 downto 0);      -- winner / new place, 0..NP-1
    place_sad   : out std_logic_vector(12 downto 0);     -- 0..NSWM*64
    place_new   : out std_logic;                          -- '1' if this call appended a place
    place_full  : out std_logic;                          -- '1' if learn was rejected (no capacity)
    place_none  : out std_logic                           -- '1' if query found nothing stored
  );
end entity;

architecture rtl of place_layer is
  type buf_t is array (0 to NSWM - 1) of unsigned(6 downto 0);
  signal buf : buf_t;

  type wmem_t is array (0 to NP * NSWM - 1) of unsigned(6 downto 0);
  signal wmem : wmem_t := (others => (others => '0'));

  type state_t is (IDLE, COMPARE, DECIDE, RECRUIT);
  signal state : state_t := IDLE;

  signal pc_n     : integer range 0 to NP := 0;
  signal n_cnt     : integer range 0 to NP := 0;
  signal a_cnt     : integer range 0 to NSWM := 0;
  signal sad_acc   : unsigned(15 downto 0) := (others => '0');
  signal best_sad  : unsigned(15 downto 0) := (others => '1');
  signal best_idx  : integer range 0 to NP - 1 := 0;
  signal learn_r   : std_logic := '0';
  signal want_cmp  : std_logic := '0';     -- '1' -> go COMPARE, '0' -> RECRUIT/NONE directly
begin
  process (clk)
    variable diff : unsigned(6 downto 0);
  begin
    if rising_edge(clk) then
      place_valid <= '0'; place_new <= '0'; place_full <= '0'; place_none <= '0';

      if rst = '1' then
        state <= IDLE; pc_n <= 0;
      else
        case state is

          when IDLE =>
            if in_valid = '1' then
              buf(to_integer(unsigned(in_idx))) <= unsigned(in_data);
              learn_r <= learn;
              if in_last = '1' then
                n_cnt <= 0; a_cnt <= 0; sad_acc <= (others => '0');
                best_sad <= (others => '1');
                if learn = '1' and pc_n < NP then
                  state <= RECRUIT; a_cnt <= 0;             -- fast path, no compare
                elsif pc_n = 0 then
                  place_none <= '1'; place_valid <= '1';     -- nothing stored, nothing to do
                else
                  state <= COMPARE;                           -- query, or learn-when-full
                end if;
              end if;
            end if;

          when COMPARE =>
            if buf(a_cnt) >= wmem(n_cnt * NSWM + a_cnt) then
              diff := buf(a_cnt) - wmem(n_cnt * NSWM + a_cnt);
            else
              diff := wmem(n_cnt * NSWM + a_cnt) - buf(a_cnt);
            end if;
            sad_acc <= sad_acc + resize(diff, 16);
            if a_cnt = NSWM - 1 then
              if (sad_acc + resize(diff, 16)) < best_sad then
                best_sad <= sad_acc + resize(diff, 16); best_idx <= n_cnt;
              end if;
              a_cnt <= 0; sad_acc <= (others => '0');
              if n_cnt = pc_n - 1 then
                state <= DECIDE;
              else
                n_cnt <= n_cnt + 1;
              end if;
            else
              a_cnt <= a_cnt + 1;
            end if;

          when DECIDE =>
            place_idx <= std_logic_vector(to_unsigned(best_idx, 5));
            place_sad <= std_logic_vector(resize(best_sad, 13));
            if learn_r = '1' then
              place_full <= '1';                 -- got here only when pc_n = NP
            end if;
            place_valid <= '1';
            state <= IDLE;

          when RECRUIT =>                          -- write buf into wmem(pc_n)
            wmem(pc_n * NSWM + a_cnt) <= buf(a_cnt);
            if a_cnt = NSWM - 1 then
              place_idx <= std_logic_vector(to_unsigned(pc_n, 5));
              place_sad <= (others => '0');
              place_new <= '1'; place_valid <= '1';
              pc_n <= pc_n + 1;
              state <= IDLE;
            else
              a_cnt <= a_cnt + 1;
            end if;
        end case;
      end if;
    end if;
  end process;
end architecture;
