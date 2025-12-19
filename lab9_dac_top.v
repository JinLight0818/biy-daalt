//============================================================
// lab9_dac_top.v
// Spartan-3E Starter Kit – Chapter 9 DAC (LTC2624) demo
//
// Controls LTC2624 quad 12-bit DAC over shared SPI bus.
//  - SW[3:0] set upper nibble of 12-bit DAC code (D[11:8])
//  - BTN_EAST toggles lower byte D[7:0] between 0x00 and 0xFF
//  - BTN_WEST cycles channel A->B->C->D
//  - BTN_SOUTH is reset (clears DAC via DAC_CLR, resets UI state)
//
// SPI mode: CPOL=0, CPHA=0 (data valid before rising edge, captured on rising).
// Frame format (32 bits, MSB first):
//   [31:24] don't care, [23:20] command, [19:16] address, [15:4] data[11:0], [3:0] don't care
// Command used: 0b0011 = "Write and Update" (update output on DAC_CS rising edge)
//============================================================
`timescale 1ns/1ps

module lab9_dac_top (
    input  wire        CLK_50MHZ,

    input  wire [3:0]  SW,
    input  wire        BTN_SOUTH,
    input  wire        BTN_WEST,
    input  wire        BTN_EAST,

    output wire [7:0]  LED,

    // Shared SPI bus
    input  wire        SPI_MISO,
    output wire        SPI_MOSI,
    output wire        SPI_SCK,

    // DAC control
    output wire        DAC_CS,
    output wire        DAC_CLR,

    // Other devices on shared bus (force disable values to avoid contention)
    output wire        SPI_SS_B,   // SPI Flash select (active low) -> keep High to disable
    output wire        AMP_CS,     // AMP select (active low) -> keep High to disable
    output wire        AD_CONV,    // ADC convert control -> keep Low to disable transactions
    output wire        SF_CE0      // StrataFlash CE (active low) -> keep High to disable
);

    // -------------------------
    // Disable other SPI devices
    // -------------------------
    assign SPI_SS_B = 1'b1;
    assign AMP_CS   = 1'b1;
    assign AD_CONV  = 1'b0;
    assign SF_CE0   = 1'b1;

    // -------------------------
    // Reset + debounced buttons
    // -------------------------
    wire reset = BTN_SOUTH;

    wire btn_west_pulse;
    wire btn_east_pulse;

    debounce #(
        .COUNT_MAX(20'd999_999)   // ~20ms at 50MHz
    ) u_db_west (
        .clk(CLK_50MHZ),
        .reset(reset),
        .noisy(BTN_WEST),
        .clean(),
        .rising_pulse(btn_west_pulse)
    );

    debounce #(
        .COUNT_MAX(20'd999_999)   // ~20ms at 50MHz
    ) u_db_east (
        .clk(CLK_50MHZ),
        .reset(reset),
        .noisy(BTN_EAST),
        .clean(),
        .rising_pulse(btn_east_pulse)
    );

    // -------------------------
    // Simple UI state
    // -------------------------
    reg [1:0] chan_sel;        // 0=A, 1=B, 2=C, 3=D
    reg       lo_ff;           // 0 -> low byte 0x00, 1 -> low byte 0xFF

    always @(posedge CLK_50MHZ) begin
        if (reset) begin
            chan_sel <= 2'd0;
            lo_ff    <= 1'b0;
        end else begin
            if (btn_west_pulse) begin
                chan_sel <= chan_sel + 2'd1;
            end
            if (btn_east_pulse) begin
                lo_ff <= ~lo_ff;
            end
        end
    end

    // 12-bit DAC code assembled from switches + lo_ff
    wire [11:0] dac_data = { SW[3:0], (lo_ff ? 8'hFF : 8'h00) };

    // Address mapping (common LTC2624 convention)
    wire [3:0] dac_addr =
        (chan_sel == 2'd0) ? 4'b0000 :
        (chan_sel == 2'd1) ? 4'b0001 :
        (chan_sel == 2'd2) ? 4'b0010 :
                             4'b0011;

    localparam [3:0] CMD_WRITE_UPDATE = 4'b0011;

    // Build 32-bit SPI frame (MSB first)
    wire [31:0] dac_frame = { 8'h00, CMD_WRITE_UPDATE, dac_addr, dac_data, 4'h0 };

    // -------------------------
    // DAC_CLR (active-low async reset to DAC)
    // Hold low while reset button is pressed
    // -------------------------
    assign DAC_CLR = ~reset;

    // -------------------------
    // SPI transaction control
    // -------------------------
    wire        spi_busy;
    wire        spi_done;
    reg         spi_start;
    reg  [31:0] frame_reg;

    // Detect changes and push an update when SPI is idle
    reg  [3:0]  sw_prev;
    reg  [1:0]  chan_prev;
    reg         lo_prev;
    reg         update_pending;

    always @(posedge CLK_50MHZ) begin
        spi_start <= 1'b0;

        if (reset) begin
            sw_prev        <= 4'd0;
            chan_prev      <= 2'd0;
            lo_prev        <= 1'b0;
            update_pending <= 1'b1;     // send initial value after reset
            frame_reg      <= 32'd0;
        end else begin
            // Track changes
            if (SW != sw_prev) begin
                sw_prev        <= SW;
                update_pending <= 1'b1;
            end
            if (chan_sel != chan_prev) begin
                chan_prev      <= chan_sel;
                update_pending <= 1'b1;
            end
            if (lo_ff != lo_prev) begin
                lo_prev        <= lo_ff;
                update_pending <= 1'b1;
            end

            // Fire a transaction when idle
            if (!spi_busy && update_pending) begin
                frame_reg      <= dac_frame;
                spi_start      <= 1'b1;
                update_pending <= 1'b0;
            end

            // If changes happen during a transfer, update_pending will be re-set above
        end
    end

    // SPI master instance (mode 0)
    spi_master_ltc2624 #(
        .SCK_DIV(25) // 50MHz/(2*25)=1MHz SCK
    ) u_spi (
        .clk(CLK_50MHZ),
        .reset(reset),
        .start(spi_start),
        .tx_data(frame_reg),
        .miso(SPI_MISO),
        .rx_data(),
        .busy(spi_busy),
        .done(spi_done),
        .sck(SPI_SCK),
        .mosi(SPI_MOSI),
        .cs_n(DAC_CS)
    );

    // -------------------------
    // LEDs (debug)
    // LED[1:0] = channel, LED2 = lo_ff, LED3 = spi_busy, LED[7:4]=SW
    // -------------------------
    assign LED[1:0] = chan_sel;
    assign LED[2]   = lo_ff;
    assign LED[3]   = spi_busy;
    assign LED[7:4] = SW;

endmodule
