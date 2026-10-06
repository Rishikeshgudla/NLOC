-- tb_swm_layer.vhd : reads sim/swm_vectors.txt (written by nloc_golden.py's
-- dump_swm_vectors) and checks swm_layer.vhd's final 96-value grid against
-- the golden model, for each of several images (each with 1..N landmarks).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;

entity tb_swm_layer is
end entity;

architecture sim of tb_swm_layer is
  constant N_SIG : positive := 32;
  constant NA  : positive := 3;
  constant NAZ : positive := 180;

  signal clk : std_logic := '0';
  signal rst : std_logic := '1';
  signal finished : boolean := false;

  signal swm_clear : std_logic := '0';
  signal lm_start   : std_logic := '0';
  signal lm_idx     : std_logic_vector(4 downto 0) := (others => '0');
  signal lm_score   : std_logic_vector(6 downto 0) := (others => '0');
  signal az_idx     : std_logic_vector(7 downto 0) := (others => '0');
  signal az_data    : std_logic_vector(6 downto 0) := (others => '0');
  signal az_valid   : std_logic := '0';
  signal az_last    : std_logic := '0';
  signal swm_valid  : std_logic;

  signal dump_start : std_logic := '0';
  signal dump_idx    : std_logic_vector(6 downto 0);
  signal dump_data   : std_logic_vector(6 downto 0);
  signal dump_valid  : std_logic;
  signal dump_last   : std_logic;

  signal n_pass, n_fail, n_images : integer := 0;
begin
  clk <= not clk after 5 ns when not finished;

  dut : entity work.swm_layer
    generic map (NS => N_SIG, NA => NA, NAZ => NAZ)
    port map (clk => clk, rst => rst, swm_clear => swm_clear,
              lm_start => lm_start, lm_idx => lm_idx, lm_score => lm_score,
              az_idx => az_idx, az_data => az_data, az_valid => az_valid, az_last => az_last,
              swm_valid => swm_valid,
              dump_start => dump_start, dump_idx => dump_idx, dump_data => dump_data,
              dump_valid => dump_valid, dump_last => dump_last);

  stim : process
    file     f      : text open read_mode is "swm_vectors.txt";
    variable ln      : line;
    variable n_img    : integer;
    variable n_lm     : integer;
    variable v_idx, v_score, v_az : integer;
    variable exp_grid  : integer;
    variable got_idx, got_data : integer;
    variable img_fail  : boolean;
  begin
    wait for 20 ns; rst <= '0'; wait for 20 ns;

    readline(f, ln); read(ln, n_img);
    report "reading " & integer'image(n_img) & " SWM test images";

    for im in 0 to n_img - 1 loop
      swm_clear <= '1'; wait until rising_edge(clk); swm_clear <= '0';
      wait until rising_edge(clk);

      readline(f, ln); read(ln, n_lm);
      for l in 0 to n_lm - 1 loop
        readline(f, ln);
        read(ln, v_idx); read(ln, v_score);

        lm_idx <= std_logic_vector(to_unsigned(v_idx, 5));
        lm_score <= std_logic_vector(to_unsigned(v_score, 7));
        lm_start <= '1';
        wait until rising_edge(clk);
        lm_start <= '0';

        for a in 0 to NAZ - 1 loop
          read(ln, v_az);
          az_idx  <= std_logic_vector(to_unsigned(a, 8));
          az_data <= std_logic_vector(to_unsigned(v_az, 7));
          az_valid <= '1';
          if a = NAZ - 1 then az_last <= '1'; else az_last <= '0'; end if;
          wait until rising_edge(clk);
          az_valid <= '0'; az_last <= '0';
        end loop;
        wait until rising_edge(clk) and swm_valid = '1';
        wait until rising_edge(clk);
      end loop;

      -- now dump the grid and compare against the expected 96 values on the next line
      readline(f, ln);
      dump_start <= '1';
      wait until rising_edge(clk);
      dump_start <= '0';

      img_fail := false;
      for i in 0 to N_SIG * NA - 1 loop
        wait until rising_edge(clk) and dump_valid = '1';
        read(ln, exp_grid);
        got_idx  := to_integer(unsigned(dump_idx));
        got_data := to_integer(unsigned(dump_data));
        if got_idx = i and got_data = exp_grid then
          n_pass <= n_pass + 1;
        else
          n_fail <= n_fail + 1; img_fail := true;
          report "FAIL image " & integer'image(im) & " grid[" & integer'image(i) &
                 "]: expected " & integer'image(exp_grid) & " got " & integer'image(got_data)
                 severity error;
        end if;
        if i = N_SIG * NA - 1 then
          assert dump_last = '1' report "FAIL image " & integer'image(im) &
                 ": dump_last missing" severity error;
        end if;
      end loop;
      n_images <= n_images + 1;
      wait until rising_edge(clk);
    end loop;

    if n_fail = 0 then
      report "PASS: all " & integer'image(n_img) & " images matched the golden model"
             severity note;
    else
      report "FAIL: " & integer'image(n_fail) & " grid values did not match" severity failure;
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
