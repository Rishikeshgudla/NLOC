-- tb_azimuth_layer.vhd : reads sim/az_vectors.txt (all 360 possible angles,
-- written by nloc_golden.py's dump_azimuth_vectors) and checks every one of
-- the 180 streamed-out bubble values against the golden model, for each angle.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;

entity tb_azimuth_layer is
end entity;

architecture sim of tb_azimuth_layer is
  constant NAZ : positive := 180;
  signal clk : std_logic := '0';
  signal rst : std_logic := '1';
  signal finished : boolean := false;

  signal az_theta : std_logic_vector(8 downto 0) := (others => '0');
  signal az_start : std_logic := '0';
  signal az_idx   : std_logic_vector(7 downto 0);
  signal az_data  : std_logic_vector(6 downto 0);
  signal az_valid, az_last, az_busy : std_logic;

  signal n_pass, n_fail, n_angles : integer := 0;
begin
  clk <= not clk after 5 ns when not finished;

  dut : entity work.azimuth_layer
    generic map (NAZ => NAZ)
    port map (clk => clk, rst => rst, az_theta => az_theta, az_start => az_start,
              az_idx => az_idx, az_data => az_data, az_valid => az_valid,
              az_last => az_last, az_busy => az_busy);

  stim : process
    file     f     : text open read_mode is "az_vectors.txt";
    variable ln     : line;
    variable n_th   : integer;
    variable theta  : integer;
    variable exp    : integer;
    variable got_idx, got_data : integer;
    variable seen   : integer;
  begin
    wait for 20 ns; rst <= '0'; wait for 20 ns;

    readline(f, ln); read(ln, n_th);
    report "reading " & integer'image(n_th) & " azimuth test vectors (exhaustive 0..359)";

    for t in 0 to n_th - 1 loop
      readline(f, ln);
      read(ln, theta);
      az_theta <= std_logic_vector(to_unsigned(theta, 9));
      wait until rising_edge(clk);
      az_start <= '1';
      wait until rising_edge(clk);
      az_start <= '0';

      seen := 0;
      for i in 0 to NAZ - 1 loop
        wait until rising_edge(clk) and az_valid = '1';
        read(ln, exp);
        got_idx  := to_integer(unsigned(az_idx));
        got_data := to_integer(unsigned(az_data));
        if got_idx = i and got_data = exp then
          n_pass <= n_pass + 1;
        else
          n_fail <= n_fail + 1;
          report "FAIL theta=" & integer'image(theta) & " neuron " & integer'image(i) &
                 ": expected idx=" & integer'image(i) & " data=" & integer'image(exp) &
                 " got idx=" & integer'image(got_idx) & " data=" & integer'image(got_data)
                 severity error;
        end if;
        if i = NAZ - 1 then
          assert az_last = '1' report "FAIL theta=" & integer'image(theta) &
                 ": az_last missing on final neuron" severity error;
        end if;
        seen := seen + 1;
      end loop;
      n_angles <= n_angles + 1;
      wait until rising_edge(clk);
    end loop;

    if n_fail = 0 then
      report "PASS: all " & integer'image(n_th) & " angles x " & integer'image(NAZ) &
             " neurons matched the golden model" severity note;
    else
      report "FAIL: " & integer'image(n_fail) & " neuron values did not match, out of "
             & integer'image(n_th * NAZ) severity failure;
    end if;
    finished <= true;
    wait;
  end process;

  watchdog : process
  begin
    wait until finished for 100 ms;
    assert finished report "FAIL: testbench timeout" severity failure;
    wait;
  end process;
end architecture;
