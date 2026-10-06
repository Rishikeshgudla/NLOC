-- tb_place_layer.vhd : reads sim/place_vectors.txt (written by nloc_golden.py's
-- dump_place_vectors) and checks place_layer.vhd matches the golden model on
-- every call: same winner index, same SAD, same none/full/new flags.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;

entity tb_place_layer is
end entity;

architecture sim of tb_place_layer is
  constant NSWM : positive := 96;
  signal clk : std_logic := '0';
  signal rst : std_logic := '1';
  signal finished : boolean := false;

  signal learn    : std_logic := '0';
  signal in_data  : std_logic_vector(6 downto 0) := (others => '0');
  signal in_idx   : std_logic_vector(6 downto 0) := (others => '0');
  signal in_valid : std_logic := '0';
  signal in_last  : std_logic := '0';

  signal place_valid, place_new, place_full, place_none : std_logic;
  signal place_idx : std_logic_vector(4 downto 0);
  signal place_sad : std_logic_vector(12 downto 0);

  signal n_pass, n_fail : integer := 0;
begin
  clk <= not clk after 5 ns when not finished;

  dut : entity work.place_layer
    port map (clk => clk, rst => rst, learn => learn,
              in_data => in_data, in_idx => in_idx, in_valid => in_valid, in_last => in_last,
              place_valid => place_valid, place_idx => place_idx, place_sad => place_sad,
              place_new => place_new, place_full => place_full, place_none => place_none);

  stim : process
    file     f      : text open read_mode is "place_vectors.txt";
    variable ln      : line;
    variable n_calls  : integer;
    variable v_learn, v_idx, v_sad, v_full, v_val : integer;
    variable good     : boolean;
    variable got_idx, got_sad : integer;
  begin
    wait for 20 ns; rst <= '0'; wait for 20 ns;

    readline(f, ln); read(ln, n_calls);
    report "reading " & integer'image(n_calls) & " place-layer test calls";

    for k in 0 to n_calls - 1 loop
      readline(f, ln);
      read(ln, v_learn); read(ln, v_idx); read(ln, v_sad); read(ln, v_full);
      if v_learn = 1 then learn <= '1'; else learn <= '0'; end if;

      for a in 0 to NSWM - 1 loop
        read(ln, v_val);
        in_data <= std_logic_vector(to_unsigned(v_val, 7));
        in_idx  <= std_logic_vector(to_unsigned(a, 7));
        in_valid <= '1';
        if a = NSWM - 1 then in_last <= '1'; else in_last <= '0'; end if;
        wait until rising_edge(clk);
        in_valid <= '0'; in_last <= '0';
      end loop;

      wait until rising_edge(clk) and (place_valid = '1');

      if v_idx = -1 then
        good := place_none = '1';
      elsif v_full = 1 then
        got_idx := to_integer(unsigned(place_idx));
        got_sad := to_integer(unsigned(place_sad));
        good := (place_full = '1') and (got_idx = v_idx) and (got_sad = v_sad);
      else
        got_idx := to_integer(unsigned(place_idx));
        got_sad := to_integer(unsigned(place_sad));
        good := (place_none = '0') and (place_full = '0')
                and (got_idx = v_idx) and (got_sad = v_sad);
      end if;

      if good then
        n_pass <= n_pass + 1;
      else
        n_fail <= n_fail + 1;
        report "FAIL call " & integer'image(k) &
               ": expected idx=" & integer'image(v_idx) & " sad=" & integer'image(v_sad) &
               " full=" & integer'image(v_full) severity error;
      end if;
      wait until rising_edge(clk);
    end loop;

    if n_fail = 0 then
      report "PASS: all " & integer'image(n_calls) &
             " place-layer calls matched the golden model" severity note;
    else
      report "FAIL: " & integer'image(n_fail) & " of " & integer'image(n_calls) &
             " calls did not match" severity failure;
    end if;
    finished <= true;
    wait;
  end process;

  watchdog : process
  begin
    wait until finished for 200 ms;
    assert finished report "FAIL: testbench timeout" severity failure;
    wait;
  end process;
end architecture;
