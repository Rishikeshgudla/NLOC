-- tb_uart_loopback.vhd : uart_tx -> uart_rx, checks every byte comes back unchanged
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb_uart_loopback is
end entity;

architecture sim of tb_uart_loopback is
  constant CLK_HZ : positive := 100_000_000;
  constant BAUD   : positive := 115_200;
  constant CLK_PERIOD : time := 10 ns;

  signal clk      : std_logic := '0';
  signal rst      : std_logic := '1';
  signal tx_start : std_logic := '0';
  signal tx_data  : std_logic_vector(7 downto 0) := (others => '0');
  signal line_s   : std_logic;
  signal tx_busy  : std_logic;
  signal rx_data  : std_logic_vector(7 downto 0);
  signal rx_valid : std_logic;
  signal rx_err   : std_logic;

  type byte_array is array (natural range <>) of std_logic_vector(7 downto 0);
  constant TEST_BYTES : byte_array := (x"55", x"A3", x"00", x"FF", x"3C", x"81", x"7E");

  signal done : boolean := false;
begin
  clk <= not clk after CLK_PERIOD / 2 when not done;   -- stops when test ends

  dut_tx : entity work.uart_tx
    generic map (CLK_HZ => CLK_HZ, BAUD => BAUD)
    port map (clk => clk, rst => rst, tx_start => tx_start,
              tx_data => tx_data, tx => line_s, tx_busy => tx_busy);

  dut_rx : entity work.uart_rx
    generic map (CLK_HZ => CLK_HZ, BAUD => BAUD)
    port map (clk => clk, rst => rst, rx => line_s,
              rx_data => rx_data, rx_valid => rx_valid, rx_err => rx_err);

  -- Sender: back-to-back bytes, waits for tx_busy to drop between bytes
  sender : process
  begin
    wait for 100 ns;
    rst <= '0';
    wait until rising_edge(clk);
    for i in TEST_BYTES'range loop
      tx_data  <= TEST_BYTES(i);
      tx_start <= '1';
      wait until rising_edge(clk);
      tx_start <= '0';
      wait until tx_busy = '0' and rising_edge(clk);
    end loop;
    wait;
  end process;

  -- Checker: every received byte must match the expected one, in order
  checker : process
  begin
    for i in TEST_BYTES'range loop
      wait until rising_edge(clk) and (rx_valid = '1' or rx_err = '1');
      assert rx_err = '0'
        report "FAIL: framing error on byte " & integer'image(i) severity failure;
      assert rx_data = TEST_BYTES(i)
        report "FAIL: byte " & integer'image(i) & " mismatch" severity failure;
      report "byte " & integer'image(i) & " OK";
    end loop;
    report "PASS: all " & integer'image(TEST_BYTES'length) & " bytes received correctly";
    done <= true;
    wait;
  end process;

  -- Watchdog: fail if it takes too long
  watchdog : process
  begin
    wait until done for 10 ms;
    assert done report "FAIL: timeout" severity failure;
    wait;
  end process;
end architecture;