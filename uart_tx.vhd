-- uart_tx.vhd : UART transmitter, 8N1 (8 data bits, no parity, 1 stop bit)
-- Usage: put a byte on tx_data, pulse tx_start for 1 clock while tx_busy = '0'.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity uart_tx is
  generic (
    CLK_HZ : positive := 100_000_000;   -- Basys-3 clock
    BAUD   : positive := 115_200
  );
  port (
    clk      : in  std_logic;
    rst      : in  std_logic;                      -- synchronous, active high
    tx_start : in  std_logic;                      -- 1-clock pulse
    tx_data  : in  std_logic_vector(7 downto 0);
    tx       : out std_logic;                      -- serial line (idle = '1')
    tx_busy  : out std_logic
  );
end entity;

architecture rtl of uart_tx is
  constant CLKS_PER_BIT : positive := CLK_HZ / BAUD;
  type state_t is (IDLE, START_BIT, DATA_BITS, STOP_BIT);
  signal state   : state_t := IDLE;
  signal clk_cnt : integer range 0 to CLKS_PER_BIT - 1 := 0;
  signal bit_idx : integer range 0 to 7 := 0;
  signal shreg   : std_logic_vector(7 downto 0) := (others => '0');
begin
  process (clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        state <= IDLE; tx <= '1'; tx_busy <= '0';
        clk_cnt <= 0; bit_idx <= 0;
      else
        case state is
          when IDLE =>
            tx <= '1'; tx_busy <= '0'; clk_cnt <= 0; bit_idx <= 0;
            if tx_start = '1' then
              shreg <= tx_data; tx_busy <= '1'; state <= START_BIT;
            end if;

          when START_BIT =>
            tx <= '0';
            if clk_cnt = CLKS_PER_BIT - 1 then
              clk_cnt <= 0; state <= DATA_BITS;
            else
              clk_cnt <= clk_cnt + 1;
            end if;

          when DATA_BITS =>                         -- LSB first
            tx <= shreg(bit_idx);
            if clk_cnt = CLKS_PER_BIT - 1 then
              clk_cnt <= 0;
              if bit_idx = 7 then
                bit_idx <= 0; state <= STOP_BIT;
              else
                bit_idx <= bit_idx + 1;
              end if;
            else
              clk_cnt <= clk_cnt + 1;
            end if;

          when STOP_BIT =>
            tx <= '1';
            if clk_cnt = CLKS_PER_BIT - 1 then
              clk_cnt <= 0; state <= IDLE; tx_busy <= '0';
            else
              clk_cnt <= clk_cnt + 1;
            end if;
        end case;
      end if;
    end if;
  end process;
end architecture;