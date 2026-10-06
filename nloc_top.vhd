-- nloc_top.vhd : M6 - top-level pipeline
--
-- Wires pkt_rx -> signature_layer -> azimuth_layer -> swm_layer -> place_layer
-- into one learn/query cycle. Byte-stream in (as if just out of uart_rx),
-- one Place ID result out per packet.
--
-- Per landmark: wait for its angle (lm_valid) and its 144 pixels (streamed
-- straight into signature_layer as they arrive); once the Signature Layer
-- has picked a match (sig_valid), start the Azimuth Layer for that landmark
-- and stream its 180-value bubble straight into the SWM accumulator. After
-- the last landmark, dump the 96-value SWM grid straight into the Place
-- Cell Layer and report its winner as the Place ID.
--
-- TIMING NOTE: this is a SEQUENTIAL design -- one landmark's Signature Layer
-- compare (up to NS*PATCH cycles) must finish before that landmark's next
-- bytes (or the next landmark's) are of any use; pix_valid pulses that
-- arrive while the Signature Layer is mid-compare are silently missed.
-- At 115200 baud this is never close (the ~173us gap between landmarks
-- vastly exceeds the ~46us worst-case compare time at NS=32). It has NOT
-- been checked at the 921600 baud being considered for the major project --
-- do not raise the baud rate without adding a busy/backpressure signal to
-- pkt_rx first.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity nloc_top is
  port (
    clk      : in  std_logic;
    rst      : in  std_logic;

    rx_data  : in  std_logic_vector(7 downto 0);
    rx_valid : in  std_logic;

    result_valid    : out std_logic;                      -- 1-clock pulse per packet
    result_place_id : out std_logic_vector(4 downto 0);
    result_new      : out std_logic;                        -- learn: recruited a new place
    result_none     : out std_logic;                        -- query: nothing stored yet
    result_full     : out std_logic;                        -- learn: no capacity left

    pkt_err      : out std_logic;                            -- passthrough: framing error
    pkt_err_code : out std_logic_vector(2 downto 0)
  );
end entity;

architecture rtl of nloc_top is
  -- pkt_rx outputs
  signal hdr_valid : std_logic; signal hdr_cmd : std_logic_vector(7 downto 0);
  signal hdr_yaw   : std_logic_vector(8 downto 0); signal hdr_nlm : std_logic_vector(4 downto 0);
  signal lm_valid  : std_logic; signal lm_idx_p : std_logic_vector(4 downto 0);
  signal lm_ego    : std_logic_vector(8 downto 0);
  signal pix_valid : std_logic; signal pix_data : std_logic_vector(7 downto 0);
  signal pix_idx   : std_logic_vector(7 downto 0); signal pix_last : std_logic;
  signal p_done    : std_logic; signal p_err : std_logic; signal p_err_code : std_logic_vector(2 downto 0);

  -- signature_layer
  signal sig_learn : std_logic := '0';
  signal sig_valid, sig_new_s, sig_none_s : std_logic;
  signal sig_idx : std_logic_vector(4 downto 0); signal sig_score : std_logic_vector(6 downto 0);

  -- azimuth_layer
  signal az_theta : std_logic_vector(8 downto 0) := (others => '0');
  signal az_start : std_logic := '0';
  signal az_idx : std_logic_vector(7 downto 0); signal az_data : std_logic_vector(6 downto 0);
  signal az_valid, az_last : std_logic;

  -- swm_layer
  signal swm_clear : std_logic := '0';
  signal lm_start  : std_logic := '0';
  signal lm_idx_s  : std_logic_vector(4 downto 0) := (others => '0');
  signal lm_score  : std_logic_vector(6 downto 0) := (others => '0');
  signal swm_valid : std_logic;
  signal dump_start : std_logic := '0';
  signal dump_idx : std_logic_vector(6 downto 0); signal dump_data : std_logic_vector(6 downto 0);
  signal dump_valid, dump_last : std_logic;

  -- place_layer
  signal place_valid, place_new_s, place_full_s, place_none_s : std_logic;
  signal place_idx : std_logic_vector(4 downto 0); signal place_sad : std_logic_vector(12 downto 0);

  type state_t is (S_IDLE, S_WAIT_EGO, S_WAIT_SIG, S_STREAM_AZ, S_DUMP);
  signal state : state_t := S_IDLE;

  signal learn_img : std_logic := '0';          -- latched mode for the whole image
  signal yaw_img    : unsigned(8 downto 0) := (others => '0');
  signal nlm_img    : integer range 0 to 31 := 0;
  signal lm_cnt     : integer range 0 to 31 := 0;
  signal theta_reg  : integer range 0 to 359 := 0;
  signal sum9       : integer range 0 to 1023;
  signal dump_issued : std_logic := '0';
begin
  pkt_err <= p_err; pkt_err_code <= p_err_code;

  u_pkt : entity work.pkt_rx
    port map (clk => clk, rst => rst, rx_data => rx_data, rx_valid => rx_valid,
              hdr_valid => hdr_valid, hdr_cmd => hdr_cmd, hdr_yaw => hdr_yaw, hdr_nlm => hdr_nlm,
              lm_valid => lm_valid, lm_idx => lm_idx_p, lm_ego => lm_ego,
              pix_valid => pix_valid, pix_data => pix_data, pix_idx => pix_idx, pix_last => pix_last,
              pkt_done => p_done, pkt_err => p_err, err_code => p_err_code);

  -- Signature Layer: pixels are always forwarded as they arrive; its own
  -- internal state machine only acts on them while idle/capturing.
  u_sig : entity work.signature_layer
    port map (clk => clk, rst => rst, learn => sig_learn,
              pix_data => pix_data, pix_idx => pix_idx, pix_valid => pix_valid, pix_last => pix_last,
              sig_valid => sig_valid, sig_idx => sig_idx, sig_score => sig_score,
              sig_new => sig_new_s, sig_none => sig_none_s);

  u_az : entity work.azimuth_layer
    port map (clk => clk, rst => rst, az_theta => az_theta, az_start => az_start,
              az_idx => az_idx, az_data => az_data, az_valid => az_valid, az_last => az_last, az_busy => open);

  u_swm : entity work.swm_layer
    port map (clk => clk, rst => rst, swm_clear => swm_clear,
              lm_start => lm_start, lm_idx => lm_idx_s, lm_score => lm_score,
              az_idx => az_idx, az_data => az_data, az_valid => az_valid, az_last => az_last,
              swm_valid => swm_valid,
              dump_start => dump_start, dump_idx => dump_idx, dump_data => dump_data,
              dump_valid => dump_valid, dump_last => dump_last);

  u_place : entity work.place_layer
    port map (clk => clk, rst => rst, learn => learn_img,
              in_data => dump_data, in_idx => dump_idx, in_valid => dump_valid, in_last => dump_last,
              place_valid => place_valid, place_idx => place_idx, place_sad => place_sad,
              place_new => place_new_s, place_full => place_full_s, place_none => place_none_s);

  sig_learn <= learn_img;

  process (clk)
  begin
    if rising_edge(clk) then
      result_valid <= '0'; swm_clear <= '0'; lm_start <= '0';
      az_start <= '0'; dump_start <= '0';

      if rst = '1' then
        state <= S_IDLE;
      else
        if p_err = '1' then
          state <= S_IDLE;                    -- abort this image on any framing error
        else
          case state is

            when S_IDLE =>
              if hdr_valid = '1' then
                if hdr_cmd = x"01" then learn_img <= '1'; else learn_img <= '0'; end if;
                yaw_img <= unsigned(hdr_yaw);
                nlm_img <= to_integer(unsigned(hdr_nlm));
                lm_cnt <= 0;
                swm_clear <= '1';
                state <= S_WAIT_EGO;
              end if;

            when S_WAIT_EGO =>
              if lm_valid = '1' then
                sum9 <= to_integer(unsigned(lm_ego)) + to_integer(yaw_img);
                state <= S_WAIT_SIG;
              end if;

            when S_WAIT_SIG =>
              if sum9 >= 360 then theta_reg <= sum9 - 360; else theta_reg <= sum9; end if;
              if sig_valid = '1' then
                lm_idx_s <= sig_idx; lm_score <= sig_score;
                lm_start <= '1';
                az_theta <= std_logic_vector(to_unsigned(theta_reg, 9));
                az_start <= '1';
                state <= S_STREAM_AZ;
              end if;

            when S_STREAM_AZ =>
              if swm_valid = '1' then
                if lm_cnt + 1 >= nlm_img then
                  state <= S_DUMP;
                else
                  lm_cnt <= lm_cnt + 1; state <= S_WAIT_EGO;
                end if;
              end if;

            when S_DUMP =>
              if dump_issued = '0' then
                dump_start <= '1'; dump_issued <= '1';
              end if;
              if place_valid = '1' then
                result_place_id <= place_idx;
                result_new  <= place_new_s;
                result_none <= place_none_s;
                result_full <= place_full_s;
                result_valid <= '1';
                dump_issued <= '0';
                state <= S_IDLE;
              end if;
          end case;
        end if;
      end if;
    end if;
  end process;
end architecture;
