library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
 
entity dac_every_3s is
  Port (
    clk      : in  std_logic;   -- 50 MHz
    reset_n  : in  std_logic;   -- active-low reset
 
    spi_mosi : out std_logic;   -- T4
    spi_sck  : out std_logic;   -- U16
    dac_cs   : out std_logic;   -- N8 (CS/LD)
    dac_clr  : out std_logic    -- P8 (CLR, active-low)
  );
end;
 
architecture rtl of dac_every_3s is
 
  ------------------------------------------------------------------
  -- Clean SPI timing (tick -> SCK rise/fall) : 1MHz SCK
  ------------------------------------------------------------------
  constant TICK_DIV : integer := 25;   -- 50MHz/25=2MHz tick -> SCK=1MHz
  signal tick_cnt : integer range 0 to TICK_DIV-1 := 0;
  signal tick     : std_logic := '0';
 
  type state_t is (LOAD, SHIFT_RISE, SHIFT_FALL, LATCH, GAP);
  signal state : state_t := LOAD;
 
  signal cs   : std_logic := '1';
  signal sck  : std_logic := '0';
  signal mosi : std_logic := '0';
 
  signal shift_reg : std_logic_vector(23 downto 0) := (others=>'0');
  signal bit_idx   : integer range 0 to 23 := 0;
  signal gap_cnt   : integer range 0 to 31 := 0;
 
  ------------------------------------------------------------------
  -- DAC code ramp (VREF=3.3V)
  -- START ~1.0V, STEP ~0.3V, MAX=3.3V
  ------------------------------------------------------------------
  constant START_CODE : integer := 200; -- ~0.15V
  constant STEP_CODE  : integer := 372;  -- ~0.3V
  constant MAX_CODE   : integer := 4095; -- ~3.3V (full-scale)
 
  signal dac_code : integer range 0 to 4095 := START_CODE;
 
  ------------------------------------------------------------------
  -- 3-second timer (50MHz * 3s = 150,000,000 cycles)
  ------------------------------------------------------------------
  constant SEC3_MAX : unsigned(27 downto 0) := to_unsigned(150000000-1, 28);
  signal sec3_cnt   : unsigned(27 downto 0) := (others=>'0');
  signal tick_3s    : std_logic := '0';
 
begin
  dac_clr  <= '1';
  dac_cs   <= cs;
  spi_sck  <= sck;
  spi_mosi <= mosi;
 
  ------------------------------------------------------------------
  -- 2MHz tick generator
  ------------------------------------------------------------------
  process(clk)
  begin
    if rising_edge(clk) then
      if reset_n='0' then
        tick_cnt <= 0;
        tick     <= '0';
      else
        if tick_cnt = TICK_DIV-1 then
          tick_cnt <= 0;
          tick     <= '1';
        else
          tick_cnt <= tick_cnt + 1;
          tick     <= '0';
        end if;
      end if;
    end if;
  end process;
 
  ------------------------------------------------------------------
  -- 3-second tick generator
  ------------------------------------------------------------------
  process(clk)
  begin
    if rising_edge(clk) then
      if reset_n='0' then
        sec3_cnt <= (others=>'0');
        tick_3s  <= '0';
      else
        if sec3_cnt = SEC3_MAX then
          sec3_cnt <= (others=>'0');
          tick_3s  <= '1';
        else
          sec3_cnt <= sec3_cnt + 1;
          tick_3s  <= '0';
        end if;
      end if;
    end if;
  end process;
 
  ------------------------------------------------------------------
  -- Update dac_code every 3 seconds until it reaches full-scale
  ------------------------------------------------------------------
  process(clk)
    variable next_code : integer;
  begin
    if rising_edge(clk) then
      if reset_n='0' then
        dac_code <= START_CODE;
      else
        if tick_3s='1' then
          next_code := dac_code + STEP_CODE;
          if next_code > MAX_CODE then
            next_code := MAX_CODE;
          end if;
          dac_code <= next_code;
        end if;
      end if;
    end if;
  end process;
 
  ------------------------------------------------------------------
  -- Clean SPI state machine (continuous frames, CH A = dac_code)
  ------------------------------------------------------------------
  process(clk)
    variable data12 : std_logic_vector(11 downto 0);
  begin
    if rising_edge(clk) then
      if reset_n='0' then
        state <= LOAD;
        cs    <= '1';
        sck   <= '0';
        mosi  <= '0';
        bit_idx <= 23;
        gap_cnt <= 0;
        shift_reg <= (others=>'0');
      else
        if tick='1' then
          case state is
            when LOAD =>
              data12 := std_logic_vector(to_unsigned(dac_code, 12));
              shift_reg <= "0011" & "0000" & data12 & "0000"; -- CMD=0011, CH A
              bit_idx   <= 23;
              cs        <= '0';
              sck       <= '0';
              mosi      <= shift_reg(23);
              state     <= SHIFT_RISE;
 
            when SHIFT_RISE =>
              sck <= '1';
              state <= SHIFT_FALL;
 
            when SHIFT_FALL =>
              sck <= '0';
              if bit_idx = 0 then
                state <= LATCH;
              else
                bit_idx <= bit_idx - 1;
                mosi <= shift_reg(bit_idx - 1);
                state <= SHIFT_RISE;
              end if;
 
            when LATCH =>
              cs <= '1';
              sck <= '0';
              mosi <= '0';
              gap_cnt <= 0;
              state <= GAP;
 
            when GAP =>
              if gap_cnt = 31 then
                state <= LOAD;
              else
                gap_cnt <= gap_cnt + 1;
              end if;
          end case;
        end if;
      end if;
    end if;
  end process;
 
end rtl;
 
