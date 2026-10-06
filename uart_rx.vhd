-- uart_rx.vhd : UART receiver, 8N1
-- rx_valid pulses for 1 clock when a byte is ready on rx_data.
-- rx_err pulses for 1 clock instead if the stop bit was not '1' (framing error).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity uart_rx is
  generic (
    CLK_HZ : positive := 100_000_000;
    BAUD   : positive := 115_200
  );
  port (
    clk      : in  std_logic;
    rst      : in  std_logic;
    rx       : in  std_logic;                      -- serial line in (asynchronous)
    rx_data  : out std_logic_vector(7 downto 0);
    rx_valid : out std_logic;
    rx_err   : out std_logic
  );
end entity;

architecture rtl of uart_rx is
  constant CLKS_PER_BIT : positive := CLK_HZ / BAUD;
  type state_t is (IDLE, START_BIT, DATA_BITS, STOP_BIT);
  signal state   : state_t := IDLE;
  signal clk_cnt : integer range 0 to CLKS_PER_BIT - 1 := 0;
  signal bit_idx : integer range 0 to 7 := 0;
  signal shreg   : std_logic_vector(7 downto 0) := (others => '0');
  signal rx_s1, rx_s2 : std_logic := '1';          -- 2-FF synchronizer
begin
  process (clk)
  begin
    if rising_edge(clk) then
      rx_s1 <= rx;  rx_s2 <= rx_s1;
      rx_valid <= '0'; rx_err <= '0';

      if rst = '1' then
        state <= IDLE; clk_cnt <= 0; bit_idx <= 0;
      else
        case state is
          when IDLE =>
            clk_cnt <= 0; bit_idx <= 0;
            if rx_s2 = '0' then state <= START_BIT; end if;

          when START_BIT =>                         -- check middle of start bit
            if clk_cnt = (CLKS_PER_BIT - 1) / 2 then
              if rx_s2 = '0' then
                clk_cnt <= 0; state <= DATA_BITS;
              else
                state <= IDLE;                      -- glitch, not a real start
              end if;
            else
              clk_cnt <= clk_cnt + 1;
            end if;

          when DATA_BITS =>                         -- sample each bit at its middle
            if clk_cnt = CLKS_PER_BIT - 1 then
              clk_cnt <= 0;
              shreg(bit_idx) <= rx_s2;              -- LSB first
              if bit_idx = 7 then
                bit_idx <= 0; state <= STOP_BIT;
              else
                bit_idx <= bit_idx + 1;
              end if;
            else
              clk_cnt <= clk_cnt + 1;
            end if;

          when STOP_BIT =>
            if clk_cnt = CLKS_PER_BIT - 1 then
              clk_cnt <= 0; state <= IDLE;
              if rx_s2 = '1' then
                rx_data <= shreg; rx_valid <= '1';
              else
                rx_err <= '1';
              end if;
            else
              clk_cnt <= clk_cnt + 1;
            end if;
        end case;
      end if;
    end if;
  end process;
end architecture;