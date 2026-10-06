-- azimuth_layer.vhd : M3 - Azimuth Layer
--
-- Combines a landmark's angle in the image with the vehicle yaw (mod 360),
-- then streams out an "activity bubble": 180 values (one per 2-degree bin),
-- highest at the combined angle and falling off with a Gaussian shape,
-- wrapping around the compass circle. Mirrors nloc_golden.py's azimuth().
--
-- Center angle uses round-half-up: c = ((theta_deg + 1) >> 1) mod NAZ.
-- (Python's banker's-rounding round() is not used -- see nloc_golden.py.)
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity azimuth_layer is
  generic (
    NAZ : positive := 180        -- azimuth neurons, 2 degrees apart
  );
  port (
    clk      : in  std_logic;
    rst      : in  std_logic;

    az_theta : in  std_logic_vector(8 downto 0);  -- combined angle, 0..359
    az_start : in  std_logic;                      -- 1-clock pulse

    az_idx   : out std_logic_vector(7 downto 0);   -- 0..NAZ-1
    az_data  : out std_logic_vector(6 downto 0);   -- bubble value, 0..64
    az_valid : out std_logic;                       -- 1-clock pulse per idx
    az_last  : out std_logic;                       -- with az_valid on idx=NAZ-1
    az_busy  : out std_logic
  );
end entity;

architecture rtl of azimuth_layer is
  -- Gaussian bubble LUT, distance 0..90 (NAZ/2), sigma=3, peak=64.
  -- Generated from nloc_golden.py's BUB table -- keep the two in sync.
  type rom_t is array (0 to NAZ / 2) of unsigned(6 downto 0);
  constant BUB : rom_t := (
    to_unsigned(64, 7), to_unsigned(61, 7), to_unsigned(51, 7), to_unsigned(39, 7), to_unsigned(26, 7), to_unsigned(16, 7), to_unsigned(9, 7), to_unsigned(4, 7), to_unsigned(2, 7), to_unsigned(1, 7),
    to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7),
    to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7),
    to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7),
    to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7),
    to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7),
    to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7),
    to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7),
    to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7), to_unsigned(0, 7),
    to_unsigned(0, 7)
  );

  type state_t is (IDLE, STREAM);
  signal state : state_t := IDLE;
  signal c     : integer range 0 to NAZ - 1 := 0;
  signal idx   : integer range 0 to NAZ - 1 := 0;
begin
  az_busy <= '1' when state = STREAM else '0';

  process (clk)
    variable diff : integer range 0 to NAZ - 1;
    variable dist : integer range 0 to NAZ / 2;
  begin
    if rising_edge(clk) then
      az_valid <= '0'; az_last <= '0';

      if rst = '1' then
        state <= IDLE;
      else
        case state is
          when IDLE =>
            if az_start = '1' then
              -- c = ((theta + 1) >> 1) mod NAZ ; theta is 0..359, NAZ=180
              c <= ((to_integer(unsigned(az_theta)) + 1) / 2) mod NAZ;
              idx <= 0; state <= STREAM;
            end if;

          when STREAM =>
            if idx >= c then diff := idx - c; else diff := c - idx; end if;
            if diff > NAZ - diff then dist := NAZ - diff; else dist := diff; end if;
            az_idx  <= std_logic_vector(to_unsigned(idx, 8));
            az_data <= std_logic_vector(BUB(dist));
            az_valid <= '1';
            if idx = NAZ - 1 then
              az_last <= '1'; state <= IDLE;
            else
              idx <= idx + 1;
            end if;
        end case;
      end if;
    end if;
  end process;
end architecture;
