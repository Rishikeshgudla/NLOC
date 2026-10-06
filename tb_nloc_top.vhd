-- tb_nloc_top.vhd : reads sim/e2e_vectors.txt (written by gen_e2e_vectors.py,
-- which uses the REAL NLOC class -- the same learn()/query() a person calls)
-- and drives nloc_top with real N-LOC packets, byte by byte, checking the
-- final Place ID against the golden model for every packet.
--
-- Landmarks are paced with a safe gap (see nloc_top.vhd's timing note): the
-- Signature Layer's compare must finish before the next landmark's bytes
-- arrive. This gap is generous, well beyond the worst case at NS=32.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;

entity tb_nloc_top is
end entity;

architecture sim of tb_nloc_top is
  constant LM_GAP_CLKS : integer := 5000;    -- >> NS*PATCH=4608 worst-case compare

  signal clk : std_logic := '0';
  signal rst : std_logic := '1';
  signal finished : boolean := false;

  signal rx_data  : std_logic_vector(7 downto 0) := (others => '0');
  signal rx_valid : std_logic := '0';

  signal result_valid, result_new, result_none, result_full : std_logic;
  signal result_place_id : std_logic_vector(4 downto 0);
  signal pkt_err : std_logic; signal pkt_err_code : std_logic_vector(2 downto 0);

  signal n_pass, n_fail : integer := 0;

  -- Latches: the pipeline can finish (pulse result_valid) WHILE stim is still
  -- busy sending later bytes of the same packet, so a plain
  -- "wait until result_valid='1'" issued afterwards would miss a pulse that
  -- already came and went. Latch it instead; stim clears it once handled.
  signal latched, latched_err : std_logic := '0';
  signal l_id : std_logic_vector(4 downto 0);
  signal l_new, l_none, l_full : std_logic;
  signal l_err_code : std_logic_vector(2 downto 0);
  signal clear_latch : std_logic := '0';
begin
  clk <= not clk after 5 ns when not finished;

  latch_proc : process (clk)
  begin
    if rising_edge(clk) then
      if rst = '1' or clear_latch = '1' then
        latched <= '0'; latched_err <= '0';
      else
        if result_valid = '1' then
          latched <= '1'; l_id <= result_place_id;
          l_new <= result_new; l_none <= result_none; l_full <= result_full;
        end if;
        if pkt_err = '1' then
          latched_err <= '1'; l_err_code <= pkt_err_code;
        end if;
      end if;
    end if;
  end process;

  dut : entity work.nloc_top
    port map (clk => clk, rst => rst, rx_data => rx_data, rx_valid => rx_valid,
              result_valid => result_valid, result_place_id => result_place_id,
              result_new => result_new, result_none => result_none, result_full => result_full,
              pkt_err => pkt_err, pkt_err_code => pkt_err_code);

  stim : process
    file     f       : text open read_mode is "e2e_vectors.txt";
    variable ln       : line;
    variable n_calls   : integer;
    variable cmd, yaw, nlm : integer;
    variable ego, pix  : integer;
    variable csum      : integer;
    variable exp_id, exp_new : integer;
    variable xacc      : std_logic_vector(7 downto 0);
    variable got_id    : integer;

    procedure send_byte(b : in std_logic_vector(7 downto 0)) is
    begin
      rx_data <= b; rx_valid <= '1';
      wait until rising_edge(clk);
      rx_valid <= '0';
      wait until rising_edge(clk);
    end procedure;

    procedure sb(v : in integer) is
      variable b : std_logic_vector(7 downto 0);
    begin
      b := std_logic_vector(to_unsigned(v, 8));
      xacc := xacc xor b;
      send_byte(b);
    end procedure;

    procedure idle(clks : in integer) is
    begin
      for i in 1 to clks loop wait until rising_edge(clk); end loop;
    end procedure;
  begin
    wait for 20 ns; rst <= '0'; wait for 20 ns;

    readline(f, ln); read(ln, n_calls);
    report "reading " & integer'image(n_calls) & " end-to-end packets";

    for c in 0 to n_calls - 1 loop
      readline(f, ln);
      read(ln, cmd); read(ln, yaw); read(ln, nlm);

      send_byte(x"A5"); xacc := x"00";
      sb(cmd); sb(yaw mod 256); sb(yaw / 256); sb(nlm);

      for l in 0 to nlm - 1 loop
        readline(f, ln);
        read(ln, ego);
        sb(ego mod 256); sb(ego / 256);
        for k in 0 to 143 loop
          read(ln, pix); sb(pix);
        end loop;
        idle(LM_GAP_CLKS);                          -- let Signature Layer finish comparing
      end loop;

      readline(f, ln); read(ln, csum);
      send_byte(std_logic_vector(to_unsigned(csum, 8)));  -- real checksum, not recomputed here

      readline(f, ln); read(ln, exp_id); read(ln, exp_new);

      wait until rising_edge(clk) and (latched = '1' or latched_err = '1');
      assert latched_err = '0'
        report "FAIL call " & integer'image(c) & ": unexpected packet error" severity failure;

      got_id := to_integer(unsigned(l_id));
      if got_id = exp_id and ((l_new = '1') = (exp_new = 1)) then
        n_pass <= n_pass + 1;
        report "PASS call " & integer'image(c) & ": place_id=" & integer'image(got_id);
      else
        n_fail <= n_fail + 1;
        report "FAIL call " & integer'image(c) & ": expected place_id=" & integer'image(exp_id)
               & " new=" & integer'image(exp_new) & ", got place_id=" & integer'image(got_id)
               severity error;
      end if;
      clear_latch <= '1';
      wait until rising_edge(clk);
      clear_latch <= '0';
      wait until rising_edge(clk);
    end loop;

    if n_fail = 0 then
      report "PASS: all " & integer'image(n_calls) & " end-to-end packets matched the golden model"
             severity note;
    else
      report "FAIL: " & integer'image(n_fail) & " of " & integer'image(n_calls) & " packets did not match"
             severity failure;
    end if;
    finished <= true;
    wait;
  end process;

  watchdog : process
  begin
    wait until finished for 500 ms;
    assert finished report "FAIL: testbench timeout" severity failure;
    wait;
  end process;
end architecture;
