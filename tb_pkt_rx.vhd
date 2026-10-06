-- tb_pkt_rx.vhd : tests pkt_rx by feeding it bytes directly (no UART, so it is fast)
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb_pkt_rx is
end entity;

architecture sim of tb_pkt_rx is
  constant TMO : positive := 5000;             -- short timeout for simulation

  signal clk : std_logic := '0';
  signal rst : std_logic := '1';
  signal finished : boolean := false;

  signal rx_data  : std_logic_vector(7 downto 0) := (others => '0');
  signal rx_valid : std_logic := '0';

  signal hdr_valid, lm_valid, pix_valid, pix_last, pkt_done, pkt_err : std_logic;
  signal hdr_cmd, pix_data, pix_idx : std_logic_vector(7 downto 0);
  signal hdr_yaw, lm_ego : std_logic_vector(8 downto 0);
  signal hdr_nlm, lm_idx : std_logic_vector(4 downto 0);
  signal err_code : std_logic_vector(2 downto 0);

  -- monitor results (cumulative)
  type int_arr is array (0 to 15) of integer;
  signal hdr_cnt, done_cnt, err_cnt, lm_cnt, pix_cnt, pix_sum, last_cnt : integer := 0;
  signal g_cmd, g_yaw, g_nlm, g_err : integer := 0;
  signal ego_arr : int_arr := (others => -1);

  -- deterministic test data
  function pix_of(l, k : integer) return integer is
  begin return (l * 37 + k * 5 + 1) mod 256; end function;
  function ego_of(l : integer) return integer is
  begin return (l * 97 + 13) mod 360; end function;
  function exp_sum(nlm : integer) return integer is
    variable s : integer := 0;
  begin
    for l in 0 to nlm - 1 loop
      for k in 0 to 143 loop s := s + pix_of(l, k); end loop;
    end loop;
    return s;
  end function;
begin
  clk <= not clk after 5 ns when not finished;

  dut : entity work.pkt_rx
    generic map (TIMEOUT_CLKS => TMO)
    port map (clk => clk, rst => rst, rx_data => rx_data, rx_valid => rx_valid,
              hdr_valid => hdr_valid, hdr_cmd => hdr_cmd, hdr_yaw => hdr_yaw, hdr_nlm => hdr_nlm,
              lm_valid => lm_valid, lm_idx => lm_idx, lm_ego => lm_ego,
              pix_valid => pix_valid, pix_data => pix_data, pix_idx => pix_idx, pix_last => pix_last,
              pkt_done => pkt_done, pkt_err => pkt_err, err_code => err_code);

  -- Monitor: records what the DUT outputs and checks pixel index order
  monitor : process (clk)
    variable exp_idx : integer := 0;
  begin
    if rising_edge(clk) then
      if hdr_valid = '1' then
        hdr_cnt <= hdr_cnt + 1;
        g_cmd <= to_integer(unsigned(hdr_cmd));
        g_yaw <= to_integer(unsigned(hdr_yaw));
        g_nlm <= to_integer(unsigned(hdr_nlm));
      end if;
      if lm_valid = '1' then
        lm_cnt <= lm_cnt + 1; exp_idx := 0;
        ego_arr(to_integer(unsigned(lm_idx))) <= to_integer(unsigned(lm_ego));
      end if;
      if pix_valid = '1' then
        assert to_integer(unsigned(pix_idx)) = exp_idx
          report "FAIL: pixel index out of order" severity failure;
        assert (pix_last = '1') = (exp_idx = 143)
          report "FAIL: pix_last wrong" severity failure;
        pix_cnt <= pix_cnt + 1;
        pix_sum <= pix_sum + to_integer(unsigned(pix_data));
        if pix_last = '1' then last_cnt <= last_cnt + 1; end if;
        exp_idx := exp_idx + 1;
      end if;
      if pkt_done = '1' then done_cnt <= done_cnt + 1; end if;
      if pkt_err  = '1' then
        err_cnt <= err_cnt + 1; g_err <= to_integer(unsigned(err_code));
      end if;
    end if;
  end process;

  -- Stimulus + checks
  stim : process
    variable xacc : std_logic_vector(7 downto 0) := x"00";
    variable h0, d0, e0, l0, p0, s0, f0 : integer;

    procedure send_byte(b : in std_logic_vector(7 downto 0)) is
    begin
      rx_data <= b; rx_valid <= '1';
      wait until rising_edge(clk);
      rx_valid <= '0';
      wait until rising_edge(clk);
      wait until rising_edge(clk);
    end procedure;

    procedure sb(v : in integer) is          -- send byte + update checksum
      variable b : std_logic_vector(7 downto 0);
    begin
      b := std_logic_vector(to_unsigned(v, 8));
      xacc := xacc xor b;
      send_byte(b);
    end procedure;

    procedure send_packet(cmd, yaw, nlm : in integer; corrupt : in boolean) is
      variable e : integer;
      variable c : std_logic_vector(7 downto 0);
    begin
      send_byte(x"A5"); xacc := x"00";
      sb(cmd); sb(yaw mod 256); sb(yaw / 256); sb(nlm);
      for l in 0 to nlm - 1 loop
        e := ego_of(l);
        sb(e mod 256); sb(e / 256);
        for k in 0 to 143 loop sb(pix_of(l, k)); end loop;
      end loop;
      c := xacc;
      if corrupt then c := c xor x"01"; end if;
      send_byte(c);
      wait for 100 ns;
    end procedure;

    procedure snapshot is
    begin
      h0 := hdr_cnt; d0 := done_cnt; e0 := err_cnt; l0 := lm_cnt;
      p0 := pix_cnt; s0 := pix_sum; f0 := last_cnt;
    end procedure;

    procedure idle(clks : in integer) is
    begin
      for i in 1 to clks loop wait until rising_edge(clk); end loop;
    end procedure;

    procedure check_good(name : in string; cmd, yaw, nlm : in integer) is
    begin
      assert hdr_cnt = h0 + 1 and g_cmd = cmd and g_yaw = yaw and g_nlm = nlm
        report "FAIL " & name & ": header wrong" severity failure;
      assert lm_cnt = l0 + nlm report "FAIL " & name & ": landmark count" severity failure;
      assert pix_cnt = p0 + 144 * nlm report "FAIL " & name & ": pixel count" severity failure;
      assert last_cnt = f0 + nlm report "FAIL " & name & ": pix_last count" severity failure;
      assert pix_sum = s0 + exp_sum(nlm) report "FAIL " & name & ": pixel values" severity failure;
      for l in 0 to nlm - 1 loop
        assert ego_arr(l) = ego_of(l) report "FAIL " & name & ": ego angle" severity failure;
      end loop;
      assert done_cnt = d0 + 1 and err_cnt = e0
        report "FAIL " & name & ": done/err flags" severity failure;
      report "PASS " & name;
    end procedure;

    procedure check_err(name : in string; code : in integer) is
    begin
      assert err_cnt = e0 + 1 and g_err = code and done_cnt = d0
        report "FAIL " & name & ": expected err " & integer'image(code) &
               ", got err_cnt+" & integer'image(err_cnt - e0) &
               " code " & integer'image(g_err) severity failure;
      report "PASS " & name;
    end procedure;
  begin
    idle(5); rst <= '0'; idle(5);

    -- T1: normal LEARN packet, 2 landmarks
    snapshot; send_packet(1, 123, 2, false);
    check_good("T1 learn 2 landmarks", 1, 123, 2);

    -- T2: junk bytes before SYNC, then max-size QUERY packet, yaw = 359
    snapshot;
    send_byte(x"00"); send_byte(x"77"); send_byte(x"FF");
    send_packet(2, 359, 16, false);
    check_good("T2 junk + 16-landmark query", 2, 359, 16);

    -- T3: bad checksum
    snapshot; send_packet(2, 10, 1, true);
    assert done_cnt = d0 report "FAIL T3: done on bad checksum" severity failure;
    check_err("T3 bad checksum", 1);

    -- T4: NLM = 17 (too many). The garbage that follows contains a fake packet
    --     that must be IGNORED because the parser drains until the line is quiet.
    snapshot;
    send_byte(x"A5"); sb(1); sb(0); sb(0); sb(17);
    send_byte(x"A5"); send_byte(x"01"); send_byte(x"00"); send_byte(x"00"); send_byte(x"01");
    idle(100);
    check_err("T4 nlm=17 then junk ignored", 2);
    idle(TMO + 100);                          -- line quiet -> parser listens again
    assert err_cnt = e0 + 1 and done_cnt = d0
      report "FAIL T4: junk after error was not ignored (extra err/done)" severity failure;

    -- T5: good packet right after the drain
    snapshot; send_packet(1, 0, 1, false);
    check_good("T5 recovers after drain", 1, 0, 1);

    -- T6: timeout in the middle of a packet
    snapshot;
    send_byte(x"A5"); sb(2); sb(50);
    idle(TMO + 100);
    check_err("T6 timeout mid-packet", 3);

    -- T7: good packet after timeout
    snapshot; send_packet(2, 200, 3, false);
    check_good("T7 recovers after timeout", 2, 200, 3);

    -- T8: unknown command
    snapshot; send_byte(x"A5"); send_byte(x"09"); idle(100);
    check_err("T8 unknown cmd", 4);
    idle(TMO + 100);

    -- T9: yaw = 400 (> 359)
    snapshot; send_byte(x"A5"); sb(1); sb(400 mod 256); sb(400 / 256); idle(100);
    check_err("T9 yaw out of range", 5);
    idle(TMO + 100);

    -- T10: still alive at the end
    snapshot; send_packet(1, 359, 1, false);
    check_good("T10 final good packet", 1, 359, 1);

    report "ALL TESTS PASSED";
    finished <= true;
    wait;
  end process;

  watchdog : process
  begin
    wait until finished for 50 ms;
    assert finished report "FAIL: testbench timeout" severity failure;
    wait;
  end process;
end architecture;
