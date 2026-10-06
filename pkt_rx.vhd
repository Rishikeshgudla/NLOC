-- pkt_rx.vhd : N-LOC packet parser (sits after uart_rx)
--
-- PACKET FORMAT (all multi-byte fields little-endian)
--   0xA5                 SYNC
--   CMD        1 byte    0x01 = LEARN, 0x02 = QUERY
--   YAW        2 bytes   vehicle yaw, integer degrees 0..359
--   NLM        1 byte    number of landmarks, 1..NL_MAX
--   NLM x {  EGO   2 bytes   landmark angle in the image, degrees 0..359
--            PIXELS 144 bytes 12x12 patch, 8-bit gray, row by row  }
--   CSUM       1 byte    XOR of every byte from CMD up to the last pixel
--
-- OUTPUTS are 1-clock pulses (data is valid only while its *_valid is '1'):
--   hdr_valid  after header is accepted        (hdr_cmd, hdr_yaw, hdr_nlm)
--   lm_valid   at the start of each landmark   (lm_idx, lm_ego)
--   pix_valid  for every pixel                 (pix_data, pix_idx, pix_last)
--   pkt_done   checksum OK  -> whole packet is good
--   pkt_err    something wrong -> err_code:
--                1 = checksum mismatch   2 = NLM out of range
--                3 = timeout mid-packet  4 = unknown CMD   5 = angle > 359
-- Data is streamed out while it arrives, so downstream logic must only COMMIT
-- its result when pkt_done arrives, and discard it on pkt_err.
-- After an early error (codes 2, 4, 5) the parser ignores bytes until the line
-- has been quiet for TIMEOUT_CLKS, so leftover bytes cannot fake a new packet.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity pkt_rx is
  generic (
    NL_MAX       : positive := 16;
    PATCH        : positive := 144;
    TIMEOUT_CLKS : positive := 1_000_000      -- 10 ms at 100 MHz
  );
  port (
    clk       : in  std_logic;
    rst       : in  std_logic;
    rx_data   : in  std_logic_vector(7 downto 0);
    rx_valid  : in  std_logic;

    hdr_valid : out std_logic;
    hdr_cmd   : out std_logic_vector(7 downto 0);
    hdr_yaw   : out std_logic_vector(8 downto 0);
    hdr_nlm   : out std_logic_vector(4 downto 0);

    lm_valid  : out std_logic;
    lm_idx    : out std_logic_vector(4 downto 0);
    lm_ego    : out std_logic_vector(8 downto 0);

    pix_valid : out std_logic;
    pix_data  : out std_logic_vector(7 downto 0);
    pix_idx   : out std_logic_vector(7 downto 0);
    pix_last  : out std_logic;

    pkt_done  : out std_logic;
    pkt_err   : out std_logic;
    err_code  : out std_logic_vector(2 downto 0)
  );
end entity;

architecture rtl of pkt_rx is
  type state_t is (WAIT_SYNC, GET_CMD, GET_YAW_L, GET_YAW_H, GET_NLM,
                   GET_EGO_L, GET_EGO_H, GET_PIX, GET_CSUM, DRAIN);
  signal state   : state_t := WAIT_SYNC;
  signal xor_acc : std_logic_vector(7 downto 0) := (others => '0');
  signal tmo_cnt : integer range 0 to TIMEOUT_CLKS := 0;
  signal cmd_r   : std_logic_vector(7 downto 0) := (others => '0');
  signal lo_r    : std_logic_vector(7 downto 0) := (others => '0');
  signal yaw_r   : std_logic_vector(8 downto 0) := (others => '0');
  signal nlm_r   : integer range 0 to NL_MAX := 0;
  signal lm_cnt  : integer range 0 to NL_MAX := 0;
  signal pix_cnt : integer range 0 to PATCH - 1 := 0;
begin
  process (clk)
    variable ang : unsigned(15 downto 0);
    variable n   : integer range 0 to 255;
  begin
    if rising_edge(clk) then
      hdr_valid <= '0'; lm_valid <= '0'; pix_valid <= '0';
      pix_last  <= '0'; pkt_done <= '0'; pkt_err   <= '0';

      if rst = '1' then
        state <= WAIT_SYNC; tmo_cnt <= 0; xor_acc <= (others => '0');
      else
        -- idle timeout (runs only while a packet is in progress or draining)
        if state = WAIT_SYNC or rx_valid = '1' then
          tmo_cnt <= 0;
        elsif tmo_cnt = TIMEOUT_CLKS - 1 then
          tmo_cnt <= 0; state <= WAIT_SYNC;
          if state /= DRAIN then
            pkt_err <= '1'; err_code <= "011";
          end if;
        else
          tmo_cnt <= tmo_cnt + 1;
        end if;

        if rx_valid = '1' then
          case state is

            when WAIT_SYNC =>
              xor_acc <= (others => '0');
              if rx_data = x"A5" then state <= GET_CMD; end if;

            when GET_CMD =>
              xor_acc <= xor_acc xor rx_data;
              if rx_data = x"01" or rx_data = x"02" then
                cmd_r <= rx_data; state <= GET_YAW_L;
              else
                pkt_err <= '1'; err_code <= "100"; state <= DRAIN;
              end if;

            when GET_YAW_L =>
              xor_acc <= xor_acc xor rx_data;
              lo_r <= rx_data; state <= GET_YAW_H;

            when GET_YAW_H =>
              xor_acc <= xor_acc xor rx_data;
              ang := unsigned(rx_data & lo_r);
              if ang > 359 then
                pkt_err <= '1'; err_code <= "101"; state <= DRAIN;
              else
                yaw_r <= std_logic_vector(ang(8 downto 0)); state <= GET_NLM;
              end if;

            when GET_NLM =>
              xor_acc <= xor_acc xor rx_data;
              n := to_integer(unsigned(rx_data));
              if n < 1 or n > NL_MAX then
                pkt_err <= '1'; err_code <= "010"; state <= DRAIN;
              else
                nlm_r <= n; lm_cnt <= 0;
                hdr_valid <= '1'; hdr_cmd <= cmd_r; hdr_yaw <= yaw_r;
                hdr_nlm <= rx_data(4 downto 0);
                state <= GET_EGO_L;
              end if;

            when GET_EGO_L =>
              xor_acc <= xor_acc xor rx_data;
              lo_r <= rx_data; state <= GET_EGO_H;

            when GET_EGO_H =>
              xor_acc <= xor_acc xor rx_data;
              ang := unsigned(rx_data & lo_r);
              if ang > 359 then
                pkt_err <= '1'; err_code <= "101"; state <= DRAIN;
              else
                lm_valid <= '1'; lm_ego <= std_logic_vector(ang(8 downto 0));
                lm_idx <= std_logic_vector(to_unsigned(lm_cnt, 5));
                pix_cnt <= 0; state <= GET_PIX;
              end if;

            when GET_PIX =>
              xor_acc <= xor_acc xor rx_data;
              pix_valid <= '1'; pix_data <= rx_data;
              pix_idx <= std_logic_vector(to_unsigned(pix_cnt, 8));
              if pix_cnt = PATCH - 1 then
                pix_last <= '1';
                if lm_cnt = nlm_r - 1 then
                  state <= GET_CSUM;
                else
                  lm_cnt <= lm_cnt + 1; state <= GET_EGO_L;
                end if;
              else
                pix_cnt <= pix_cnt + 1;
              end if;

            when GET_CSUM =>
              if rx_data = xor_acc then
                pkt_done <= '1';
              else
                pkt_err <= '1'; err_code <= "001";
              end if;
              state <= WAIT_SYNC;

            when DRAIN =>
              null;                       -- ignore bytes until the line goes quiet
          end case;
        end if;
      end if;
    end if;
  end process;
end architecture;
