library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity dac_every_3s is
  Port (
    clk       : in  std_logic;   -- 50 MHz, CLK_50MHZ @ C9
    btn_south : in  std_logic;   -- BTN_SOUTH @ K17 (active-high reset)

    spi_mosi  : out std_logic;   -- T4
    spi_sck   : out std_logic;   -- U16
    dac_cs    : out std_logic;   -- N8  (CS/LD, active-low)
    dac_clr   : out std_logic;   -- P8  (CLR, active-low)

    -- Disable other peripherals on shared SPI bus (avoid contention)
    spi_ss_b  : out std_logic;   -- U3  (SPI Flash CS_B) -> keep '1'
    amp_cs    : out std_logic;   -- N7  (AMP CS)         -> keep '1'
    ad_conv   : out std_logic;   -- P11 (ADC CONV)       -> keep '0'
    sf_ce0    : out std_logic    -- D16 (StrataFlash CE0)-> keep '1'
  );
end dac_every_3s;

architecture rtl of dac_every_3s is

  ------------------------------------------------------------------
  -- SPI timing: 50MHz -> 2MHz enable -> SCK = 1MHz (toggle per enable)
  ------------------------------------------------------------------
  constant TICK_DIV  : natural := 25;  -- 50MHz/25 = 2MHz enable
  constant GAP_TICKS : natural := 32;  -- small idle gap after a frame

  signal tick_cnt : unsigned(4 downto 0) := (others => '0'); -- 0..24
  signal spi_ce   : std_logic := '0';

  ------------------------------------------------------------------
  -- 3-second timer: 50MHz * 3s = 150,000,000 cycles
  ------------------------------------------------------------------
  constant SEC3_MAX : unsigned(27 downto 0) := to_unsigned(150000000-1, 28);
  signal sec3_cnt   : unsigned(27 downto 0) := (others => '0');

  signal reset    : std_logic;
  signal start_tx : std_logic := '0';
  signal init_sent: std_logic := '0';

  ------------------------------------------------------------------
  -- DAC code ramp (Channel A)
  -- START ~1.0V, STEP ~0.3V, VREF ~3.3V  -> (approx codes)
  ------------------------------------------------------------------
  constant START_CODE : natural := 1241; -- ~1.0V @ 3.3V
  constant STEP_CODE  : natural := 372;  -- ~0.3V @ 3.3V
  constant MAX_CODE   : natural := 4095;

  signal dac_code : natural range 0 to 4095 := START_CODE;

  ------------------------------------------------------------------
  -- SPI engine (Mode 0), 32-bit frame MSB-first
  ------------------------------------------------------------------
  type state_t is (IDLE, SHIFT_RISE, SHIFT_FALL, LATCH, GAP);
  signal state : state_t := IDLE;

  signal cs   : std_logic := '1';
  signal sck  : std_logic := '0';
  signal mosi : std_logic := '0';

  signal shift_reg : std_logic_vector(31 downto 0) := (others => '0');
  signal bit_idx   : integer range 0 to 31 := 31;
  signal gap_cnt   : unsigned(5 downto 0) := (others => '0');

begin

  ------------------------------------------------------------------
  -- Shared SPI bus disables (avoid contention)
  ------------------------------------------------------------------
  spi_ss_b <= '1';
  amp_cs   <= '1';
  ad_conv  <= '0';
  sf_ce0   <= '1';

  ------------------------------------------------------------------
  -- Reset + DAC CLR (active-low)
  ------------------------------------------------------------------
  reset   <= btn_south;
  dac_clr <= not reset;   -- reset үед CLR=0, хэвийн үед CLR=1

  ------------------------------------------------------------------
  -- Outputs
  ------------------------------------------------------------------
  dac_cs   <= cs;
  spi_sck  <= sck;
  spi_mosi <= mosi;

  ------------------------------------------------------------------
  -- 2MHz clock-enable generator for SPI timing
  ------------------------------------------------------------------
  process(clk)
  begin
    if rising_edge(clk) then
      if reset='1' then
        tick_cnt <= (others => '0');
        spi_ce   <= '0';
      else
        if tick_cnt = to_unsigned(TICK_DIV-1, tick_cnt'length) then
          tick_cnt <= (others => '0');
          spi_ce   <= '1';
        else
          tick_cnt <= tick_cnt + 1;
          spi_ce   <= '0';
        end if;
      end if;
    end if;
  end process;

  ------------------------------------------------------------------
  -- 3-second update: update code + request exactly one SPI transfer
  ------------------------------------------------------------------
  process(clk)
    variable tmp : integer;
  begin
    if rising_edge(clk) then
      start_tx <= '0';

      if reset='1' then
        dac_code  <= START_CODE;
        sec3_cnt  <= (others => '0');
        init_sent <= '0';
      else
        -- send once immediately after reset release
        if init_sent='0' then
          init_sent <= '1';
          start_tx  <= '1';
          sec3_cnt  <= (others => '0');
          dac_code  <= START_CODE;
        else
          if sec3_cnt = SEC3_MAX then
            sec3_cnt <= (others => '0');

            tmp := integer(dac_code) + integer(STEP_CODE);
            if tmp > integer(MAX_CODE) then
              tmp := integer(MAX_CODE);  -- saturate at full scale
            end if;
            dac_code <= natural(tmp);

            start_tx <= '1';
          else
            sec3_cnt <= sec3_cnt + 1;
          end if;
        end if;
      end if;
    end if;
  end process;

  ------------------------------------------------------------------
  -- SPI state machine
  -- Frame (UG230):
  --   [31:24] 8 dummy bits (0x00)
  --   [23:20] CMD = 0011  (write & update)
  --   [19:16] ADDR=0000   (Channel A)
  --   [15:4]  DATA[11:0]
  --   [3:0]   4 dummy bits
  ------------------------------------------------------------------
  process(clk)
    variable data12  : std_logic_vector(11 downto 0);
    variable frame32 : std_logic_vector(31 downto 0);
  begin
    if rising_edge(clk) then
      if reset='1' then
        state     <= IDLE;
        cs        <= '1';
        sck       <= '0';
        mosi      <= '0';
        shift_reg <= (others => '0');
        bit_idx   <= 31;
        gap_cnt   <= (others => '0');

      else
        case state is
          when IDLE =>
            cs   <= '1';
            sck  <= '0';
            mosi <= '0';

            if start_tx='1' then
              data12  := std_logic_vector(to_unsigned(dac_code, 12));
              frame32 := x"00" & "0011" & "0000" & data12 & "0000";
              shift_reg <= frame32;

              bit_idx <= 31;
              cs      <= '0';
              sck     <= '0';

              -- CRITICAL FIX: use frame32 MSB directly (avoid old shift_reg)
              mosi    <= frame32(31);

              state   <= SHIFT_RISE;
            end if;

          when SHIFT_RISE =>
            if spi_ce='1' then
              sck   <= '1';      -- DAC samples MOSI on rising edge
              state <= SHIFT_FALL;
            end if;

          when SHIFT_FALL =>
            if spi_ce='1' then
              sck <= '0';        -- change MOSI on falling edge (for next bit)

              if bit_idx = 0 then
                state <= LATCH;
              else
                bit_idx <= bit_idx - 1;
                mosi    <= shift_reg(bit_idx - 1);
                state   <= SHIFT_RISE;
              end if;
            end if;

          when LATCH =>
            -- DAC updates on CS rising edge
            cs      <= '1';
            sck     <= '0';
            mosi    <= '0';
            gap_cnt <= (others => '0');
            state   <= GAP;

          when GAP =>
            if spi_ce='1' then
              if gap_cnt = to_unsigned(GAP_TICKS-1, gap_cnt'length) then
                state <= IDLE;
              else
                gap_cnt <= gap_cnt + 1;
              end if;
            end if;
        end case;
      end if;
    end if;
  end process;

end rtl;
