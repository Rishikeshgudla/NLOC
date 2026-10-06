-- swm_layer.vhd : M4 - Spatial Working Memory
--
-- Combines each landmark's Signature Layer result (idx, score) with its
-- Azimuth Layer bubble (180 streamed values) into a 32x3 accumulator grid:
--   for each landmark:  a[g] = max over each 60-neuron third of the bubble
--                        grid[idx][g] = min(64, grid[idx][g] + ((score*a[g])>>6))
-- The grid is cleared once per image (pulse swm_clear), then one landmark's
-- worth of (idx, score, az-stream) is fed in per call, ending with az_last.
-- After the image's last landmark, pulse dump_start to stream the 96 grid
-- values out (row-major: idx*NA + group). Mirrors nloc_golden.py's swm().
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity swm_layer is
  generic (
    NS  : positive := 32;
    NA  : positive := 3;
    NAZ : positive := 180;
    ONE : natural   := 64
  );
  port (
    clk       : in  std_logic;
    rst       : in  std_logic;

    swm_clear : in  std_logic;                       -- pulse: zero the grid (new image)

    lm_start  : in  std_logic;                        -- pulse: latch idx/score, reset group-max
    lm_idx    : in  std_logic_vector(4 downto 0);
    lm_score  : in  std_logic_vector(6 downto 0);

    az_idx    : in  std_logic_vector(7 downto 0);      -- 0..NAZ-1, from azimuth_layer
    az_data   : in  std_logic_vector(6 downto 0);
    az_valid  : in  std_logic;
    az_last   : in  std_logic;                          -- with az_valid on final sample

    swm_valid : out std_logic;                          -- pulse: landmark committed to grid

    dump_start : in  std_logic;                          -- pulse: begin streaming the grid
    dump_idx   : out std_logic_vector(6 downto 0);        -- 0..NS*NA-1
    dump_data  : out std_logic_vector(6 downto 0);
    dump_valid : out std_logic;
    dump_last  : out std_logic
  );
end entity;

architecture rtl of swm_layer is
  constant GROUP_W : positive := NAZ / NA;             -- neurons per group

  type grid_t is array (0 to NS * NA - 1) of unsigned(6 downto 0);
  type gmax_t is array (0 to NA - 1) of unsigned(6 downto 0);

  type state_t is (IDLE, DUMP);
  signal state : state_t := IDLE;
begin
  process (clk)
    variable grid     : grid_t := (others => (others => '0'));
    variable grp_max  : gmax_t := (others => (others => '0'));
    variable cur_idx   : integer range 0 to NS - 1 := 0;
    variable cur_score : unsigned(6 downto 0) := (others => '0');
    variable g         : integer range 0 to NA - 1;
    variable az_v      : unsigned(6 downto 0);
    variable contrib    : unsigned(13 downto 0);
    variable newval     : unsigned(7 downto 0);
    variable dump_i     : integer range 0 to NS * NA - 1 := 0;
  begin
    if rising_edge(clk) then
      swm_valid <= '0'; dump_valid <= '0'; dump_last <= '0';

      if rst = '1' then
        state <= IDLE; grid := (others => (others => '0'));
        grp_max := (others => (others => '0')); cur_idx := 0; dump_i := 0;
      else
        if swm_clear = '1' then
          grid := (others => (others => '0'));
        end if;

        if lm_start = '1' then
          cur_idx := to_integer(unsigned(lm_idx));
          cur_score := unsigned(lm_score);
          grp_max := (others => (others => '0'));
        end if;

        if az_valid = '1' then
          g := to_integer(unsigned(az_idx)) / GROUP_W;
          az_v := unsigned(az_data);
          if az_v > grp_max(g) then
            grp_max(g) := az_v;                       -- immediate (variable), no 1-cycle lag
          end if;

          if az_last = '1' then                        -- commit this landmark into the grid
            for k in 0 to NA - 1 loop
              contrib := cur_score * grp_max(k);        -- 7x7 -> 14 bits
              newval  := resize(grid(cur_idx * NA + k), 8)
                         + resize(contrib(13 downto 6), 8);   -- (score*a) >> 6
              if newval > to_unsigned(ONE, 8) then
                grid(cur_idx * NA + k) := to_unsigned(ONE, 7);
              else
                grid(cur_idx * NA + k) := newval(6 downto 0);
              end if;
            end loop;
            swm_valid <= '1';
          end if;
        end if;

        case state is
          when IDLE =>
            if dump_start = '1' then
              dump_i := 0; state <= DUMP;
            end if;

          when DUMP =>
            dump_idx  <= std_logic_vector(to_unsigned(dump_i, 7));
            dump_data <= std_logic_vector(grid(dump_i));
            dump_valid <= '1';
            if dump_i = NS * NA - 1 then
              dump_last <= '1'; state <= IDLE;
            else
              dump_i := dump_i + 1;
            end if;
        end case;
      end if;
    end if;
  end process;
end architecture;
