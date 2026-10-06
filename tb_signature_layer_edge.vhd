-- tb_signature_layer_edge.vhd : reads sim/sig_vectors_edge.txt (written by nloc_golden.py's
-- dump_signature_vectors) and checks signature_layer.vhd matches the golden model
-- exactly on every call: same winner index, same score, same recruit decision.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;

entity tb_signature_layer_edge is
end entity;

architecture sim of tb_signature_layer_edge is
  constant PATCH : positive := 144;
  signal clk : std_logic := '0';
  signal rst : std_logic := '1';
  signal finished : boolean := false;

  signal learn     : std_logic := '0';
  signal pix_data  : std_logic_vector(7 downto 0) := (others => '0');
  signal pix_idx   : std_logic_vector(7 downto 0) := (others => '0');
  signal pix_valid : std_logic := '0';
  signal pix_last  : std_logic := '0';

  signal sig_valid, sig_new, sig_none : std_logic;
  signal sig_idx   : std_logic_vector(4 downto 0);
  signal sig_score : std_logic_vector(6 downto 0);

  signal n_pass, n_fail : integer := 0;
begin
  clk <= not clk after 5 ns when not finished;

  dut : entity work.signature_layer
    port map (clk => clk, rst => rst, learn => learn,
              pix_data => pix_data, pix_idx => pix_idx,
              pix_valid => pix_valid, pix_last => pix_last,
              sig_valid => sig_valid, sig_idx => sig_idx, sig_score => sig_score,
              sig_new => sig_new, sig_none => sig_none);

  stim : process
    file     f       : text open read_mode is "sig_vectors_edge.txt";
    variable ln       : line;
    variable n_calls   : integer;
    variable v_learn   : integer;
    variable v_idx     : integer;
    variable v_score   : integer;
    variable v_pix     : integer;
    variable good      : boolean;
    variable got_idx, got_score : integer;
  begin
    wait for 20 ns; rst <= '0'; wait for 20 ns;

    readline(f, ln); read(ln, n_calls);
    report "reading " & integer'image(n_calls) & " signature test vectors";

    for k in 0 to n_calls - 1 loop
      readline(f, ln);
      read(ln, v_learn); read(ln, v_idx); read(ln, v_score);
      if v_learn = 1 then learn <= '1'; else learn <= '0'; end if;

      for a in 0 to PATCH - 1 loop
        read(ln, v_pix);
        pix_data  <= std_logic_vector(to_unsigned(v_pix, 8));
        pix_idx   <= std_logic_vector(to_unsigned(a, 8));
        pix_valid <= '1';
        if a = PATCH - 1 then pix_last <= '1'; else pix_last <= '0'; end if;
        wait until rising_edge(clk);
        pix_valid <= '0'; pix_last <= '0';
      end loop;

      wait until rising_edge(clk) and (sig_valid = '1');

      if v_idx = -1 then
        good := sig_none = '1';
      else
        got_idx   := to_integer(unsigned(sig_idx));
        got_score := to_integer(unsigned(sig_score));
        good := (sig_none = '0') and (got_idx = v_idx) and (got_score = v_score);
      end if;

      if good then
        n_pass <= n_pass + 1;
      else
        n_fail <= n_fail + 1;
        report "FAIL call " & integer'image(k) &
               ": expected idx=" & integer'image(v_idx) &
               " score=" & integer'image(v_score) &
               " (none=" & boolean'image(v_idx = -1) & ")" severity error;
      end if;
      wait until rising_edge(clk);
    end loop;

    if n_fail = 0 then
      report "PASS: all " & integer'image(n_calls) &
             " signature-layer calls matched the golden model" severity note;
    else
      report "FAIL: " & integer'image(n_fail) & " of " & integer'image(n_calls) &
             " calls did not match" severity failure;
    end if;
    finished <= true;
    wait;
  end process;

  watchdog : process
  begin
    wait until finished for 20 ms;
    assert finished report "FAIL: testbench timeout" severity failure;
    wait;
  end process;
end architecture;
