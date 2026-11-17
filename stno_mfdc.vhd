-- Multi-stage FIR decimation frequency-to-digital converter for STNO readout
-- Stage 1: 8-tap unweighted moving sum (decimate by 4, 200 MHz -> 50 MHz)
-- Stage 2: 7-tap weighted FIR filter (decimate by 5, 50 MHz -> 10 MHz)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library unisim;
use unisim.vcomponents.all;

entity stno_mfdc is
    generic (
        COUNTER_WIDTH : positive := 16;
        OUTPUT_WIDTH  : positive := 8
    );
    port (
        stno_clk_i   : in  std_logic;
        clk_50mhz_i  : in  std_logic;
        arst_i       : in  std_logic;
        raw_bit_o    : out std_logic;
        count_o      : out std_logic_vector(OUTPUT_WIDTH - 1 downto 0);
        sample_clk_o : out std_logic
    );
end entity stno_mfdc;

architecture rtl of stno_mfdc is

    signal stno_clk_buf   : std_logic;
    signal sample_clk_int : std_logic;
    signal dcm_locked     : std_logic;
    signal dcm_rst        : std_logic;

    component clk_wiz_200MHz
      port (
        CLK_IN1  : in  std_logic;
        CLK_OUT1 : out std_logic;
        RESET    : in  std_logic;
        LOCKED   : out std_logic
      );
    end component;

    component icon_core
      port ( CONTROL0 : inout std_logic_vector(35 downto 0) );
    end component;

    component ila_core
      port (
        CONTROL : inout std_logic_vector(35 downto 0);
        CLK     : in    std_logic;
        TRIG0   : in    std_logic_vector(8 downto 0)
      );
    end component;

    function bin_to_gray(b : unsigned) return std_logic_vector is
        variable g : unsigned(b'range);
    begin
        g(g'high) := b(g'high);
        for i in b'high - 1 downto b'low loop
            g(i) := b(i + 1) xor b(i);
        end loop;
        return std_logic_vector(g);
    end function;

    function gray_to_bin(g : std_logic_vector) return unsigned is
        variable b : unsigned(g'range);
    begin
        b(b'high) := g(b'high);
        for i in b'high - 1 downto b'low loop
            b(i) := b(i + 1) xor g(i);
        end loop;
        return b;
    end function;

    signal rst_stno_sr : std_logic_vector(1 downto 0) := (others => '1');
    signal rst_samp_sr : std_logic_vector(1 downto 0) := (others => '1');
    signal rst_stno    : std_logic := '1';
    signal rst_samp    : std_logic := '1';

    -- Asynchronous STNO-domain phase accumulator
    signal bin_cnt  : unsigned(COUNTER_WIDTH - 1 downto 0) := (others => '0');
    signal gray_cnt : std_logic_vector(COUNTER_WIDTH - 1 downto 0) := (others => '0');

    -- 3-stage Gray-code CDC synchronizer and phase differentiator
    signal gray_sync_0     : std_logic_vector(COUNTER_WIDTH - 1 downto 0) := (others => '0');
    signal gray_sync_1     : std_logic_vector(COUNTER_WIDTH - 1 downto 0) := (others => '0');
    signal gray_sync_2     : std_logic_vector(COUNTER_WIDTH - 1 downto 0) := (others => '0');
    signal sampled_bin_r   : unsigned(COUNTER_WIDTH - 1 downto 0) := (others => '0');
    signal prev_bin_sample : unsigned(COUNTER_WIDTH - 1 downto 0) := (others => '0');
    signal stno_pulse      : std_logic := '0';
    signal raw_bit_r       : std_logic := '0';

    -- Stage 1 FIR (length 8, decimate by 4)
    signal stg1_sr  : std_logic_vector(7 downto 0) := (others => '0');
    signal stg1_cnt : natural range 0 to 3 := 0;
    signal stg1_sum : unsigned(3 downto 0) := (others => '0');
    signal stg1_en  : std_logic := '0';

    -- Stage 2 FIR (length 7, decimate by 5)
    type stg2_sr_t is array (0 to 6) of unsigned(3 downto 0);
    signal stg2_sr  : stg2_sr_t := (others => (others => '0'));
    signal stg2_cnt : natural range 0 to 4 := 0;
    signal count_r  : std_logic_vector(OUTPUT_WIDTH - 1 downto 0) := (others => '0');

    -- 10 MHz oscilloscope sampling clock output (200 MHz / 20)
    signal scope_clk_reg : std_logic := '0';
    signal scope_cnt     : natural range 0 to 9 := 0;

    -- ChipScope ILA debug signals
    signal control_bus   : std_logic_vector(35 downto 0);
    signal ila_data_9bit : std_logic_vector(8 downto 0);

    attribute ASYNC_REG : string;
    attribute ASYNC_REG of gray_sync_0 : signal is "TRUE";
    attribute ASYNC_REG of gray_sync_1 : signal is "TRUE";
    attribute ASYNC_REG of gray_sync_2 : signal is "TRUE";

begin

    ibufg_stno : IBUFG port map ( I => stno_clk_i, O => stno_clk_buf );

    raw_bit_o    <= raw_bit_r;
    count_o      <= count_r;
    sample_clk_o <= scope_clk_reg;
    dcm_rst      <= not arst_i;

    clk_wiz_inst : clk_wiz_200MHz
      port map (
        CLK_IN1  => clk_50mhz_i,
        CLK_OUT1 => sample_clk_int,
        RESET    => dcm_rst,
        LOCKED   => dcm_locked
      );

    -- Async assert, sync deassert reset synchronizers
    process(stno_clk_buf, arst_i)
    begin
        if arst_i = '0' then
            rst_stno_sr <= (others => '1');
        elsif rising_edge(stno_clk_buf) then
            rst_stno_sr <= rst_stno_sr(0) & '0';
        end if;
    end process;
    rst_stno <= rst_stno_sr(1);

    process(sample_clk_int, arst_i, dcm_locked)
    begin
        if arst_i = '0' or dcm_locked = '0' then
            rst_samp_sr <= (others => '1');
        elsif rising_edge(sample_clk_int) then
            rst_samp_sr <= rst_samp_sr(0) & '0';
        end if;
    end process;
    rst_samp <= rst_samp_sr(1);

    -- Free-running STNO cycle counter
    process(stno_clk_buf)
    begin
        if rising_edge(stno_clk_buf) then
            if rst_stno = '1' then
                bin_cnt  <= (others => '0');
                gray_cnt <= (others => '0');
            else
                bin_cnt  <= bin_cnt + 1;
                gray_cnt <= bin_to_gray(bin_cnt);
            end if;
        end if;
    end process;

    -- 200 MHz phase differencing and two-stage decimation pipeline
    process(sample_clk_int)
        variable sum_v       : unsigned(3 downto 0);
        variable stg2_calc_v : unsigned(8 downto 0);
    begin
        if rising_edge(sample_clk_int) then
            if rst_samp = '1' then
                gray_sync_0     <= (others => '0');
                gray_sync_1     <= (others => '0');
                gray_sync_2     <= (others => '0');
                sampled_bin_r   <= (others => '0');
                prev_bin_sample <= (others => '0');
                stno_pulse      <= '0';
                raw_bit_r       <= '0';
                stg1_sr         <= (others => '0');
                stg1_cnt        <= 0;
                stg1_sum        <= (others => '0');
                stg1_en         <= '0';
                stg2_sr         <= (others => (others => '0'));
                stg2_cnt        <= 0;
                count_r         <= (others => '0');
            else
                -- CDC synchronization and single-cycle phase advancement check
                gray_sync_0   <= gray_cnt;
                gray_sync_1   <= gray_sync_0;
                gray_sync_2   <= gray_sync_1;
                sampled_bin_r <= gray_to_bin(gray_sync_2);
                
                if sampled_bin_r /= prev_bin_sample then
                    stno_pulse <= '1';
                    raw_bit_r  <= '1';
                else
                    stno_pulse <= '0';
                    raw_bit_r  <= '0';
                end if;
                prev_bin_sample <= sampled_bin_r;

                -- Stage 1 FIR: 8-bit unweighted moving sum (200 MHz -> 50 MHz)
                stg1_sr <= stno_pulse & stg1_sr(7 downto 1);
                
                if stg1_cnt = 3 then
                    stg1_cnt <= 0;
                    
                    sum_v := (others => '0');
                    for i in 0 to 7 loop
                        if stg1_sr(i) = '1' then
                            sum_v := sum_v + 1;
                        end if;
                    end loop;
                    
                    stg1_sum <= sum_v;
                    stg1_en  <= '1';
                else
                    stg1_cnt <= stg1_cnt + 1;
                    stg1_en  <= '0';
                end if;

                -- Stage 2 FIR: 7-tap weighted filter [1, 3, 7, 10, 7, 3, 1] (50 MHz -> 10 MHz)
                if stg1_en = '1' then
                    stg2_sr(0) <= stg1_sum;
                    stg2_sr(1) <= stg2_sr(0);
                    stg2_sr(2) <= stg2_sr(1);
                    stg2_sr(3) <= stg2_sr(2);
                    stg2_sr(4) <= stg2_sr(3);
                    stg2_sr(5) <= stg2_sr(4);
                    stg2_sr(6) <= stg2_sr(5);

                    if stg2_cnt = 4 then
                        stg2_cnt <= 0;

                        stg2_calc_v := resize(stg2_sr(0), 9) +
                                       resize(stg2_sr(1) * 3, 9) +
                                       resize(stg2_sr(2) * 7, 9) +
                                       resize(stg2_sr(3) * 10, 9) +
                                       resize(stg2_sr(4) * 7, 9) +
                                       resize(stg2_sr(5) * 3, 9) +
                                       resize(stg2_sr(6), 9);

                        -- Saturate at 255 to prevent 8-bit wrap-around
                        if stg2_calc_v > 255 then
                            count_r <= (others => '1');
                        else
                            count_r <= std_logic_vector(resize(stg2_calc_v, OUTPUT_WIDTH));
                        end if;
                    else
                        stg2_cnt <= stg2_cnt + 1;
                    end if;
                end if;

            end if;
        end if;
    end process;

    -- Generate 10 MHz clock strobe for oscilloscope acquisition
    process(sample_clk_int)
    begin
        if rising_edge(sample_clk_int) then
            if rst_samp = '1' then
                scope_cnt     <= 0;
                scope_clk_reg <= '0';
            else
                if scope_cnt = 9 then
                    scope_cnt     <= 0;
                    scope_clk_reg <= not scope_clk_reg;
                else
                    scope_cnt <= scope_cnt + 1;
                end if;
            end if;
        end if;
    end process;

    icon_inst : icon_core port map ( CONTROL0 => control_bus );
    ila_data_9bit <= raw_bit_r & count_r;
    ila_inst : ila_core
    port map (
        CONTROL => control_bus,
        CLK     => sample_clk_int,
        TRIG0   => ila_data_9bit
    );

end architecture rtl;
