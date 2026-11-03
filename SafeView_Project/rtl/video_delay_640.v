// =============================================================================
// Module : video_delay_640
// Function : Parameterized pixel-path delay aligner for image carousel design.
//            Shifts I_hs, I_vs, I_de through a STAGES-deep shift register and
//            taps at three points to produce the read-enable, data-capture, and
//            output-sync phases required by the upstream FIFO read path.
//
// Tap relationships (preserved from baseline video_delay):
//   O_read_en  = I_de delayed (STAGES-3) cycles   -- FIFO read enable
//   data cap   = I_de delayed (STAGES-2) cycles   -- capture condition for I_data
//   O_hs/O_vs/O_de = delayed (STAGES-1) cycles   -- output sync aligned with O_data
//
// When STAGES=21, this module is cycle-accurate equivalent to the baseline
// video_delay module (which uses hard-coded 21-bit shift registers with taps
// at [18], [19], [20]).
//
// Usage: Two instances are used in parallel (A path = old display buffer,
//        B path = transition target new image) to guarantee strict pixel
//        alignment between the two data streams throughout the pipeline.
//
// Constraint: STAGES must be >= 3 for all tap indices to be valid.
//
// Reset style note: This module uses synchronous active-high reset (I_rst
//   sampled on posedge I_clk), which differs from the baseline video_delay
//   that uses asynchronous active-high reset. This is the project convention
//   for the image carousel design.
//
// Language: Verilog-2001 (no SystemVerilog constructs)
// =============================================================================

module video_delay_640 #(
    parameter DATA_WIDTH = 24,
    parameter STAGES     = 21          // STAGES=21 => cycle-accurate with baseline video_delay
)(
    input  wire                  I_clk,
    input  wire                  I_rst,          // Active-high synchronous reset
    output wire                  O_read_en,      // = I_de delayed (STAGES-3) cycles
    input  wire [DATA_WIDTH-1:0] I_data,         // Pixel data from read FIFO
    input  wire                  I_hs,
    input  wire                  I_vs,
    input  wire                  I_de,
    output wire                  O_hs,           // = I_hs delayed (STAGES-1) cycles
    output wire                  O_vs,           // = I_vs delayed (STAGES-1) cycles
    output wire                  O_de,           // = I_de delayed (STAGES-1) cycles
    output wire [DATA_WIDTH-1:0] O_data          // Pixel data aligned with O_de
);

    // -------------------------------------------------------------------------
    // Shift registers for sync signal delay
    // -------------------------------------------------------------------------
    reg [STAGES-1:0] hs_d;
    reg [STAGES-1:0] vs_d;
    reg [STAGES-1:0] de_d;

    // -------------------------------------------------------------------------
    // Registered pixel data output
    // -------------------------------------------------------------------------
    reg [DATA_WIDTH-1:0] O_data_r;

    // -------------------------------------------------------------------------
    // Tap assignments (parameterized)
    //
    //   STAGES=21 mapping:
    //     STAGES-3 = 18  =>  O_read_en   (FIFO read enable, 2 cycles early)
    //     STAGES-2 = 19  =>  data capture condition
    //     STAGES-1 = 20  =>  O_hs, O_vs, O_de (output sync)
    // -------------------------------------------------------------------------
    assign O_read_en = de_d[STAGES-3];
    assign O_hs      = hs_d[STAGES-1];
    assign O_vs      = vs_d[STAGES-1];
    assign O_de      = de_d[STAGES-1];
    assign O_data    = O_data_r;

    // -------------------------------------------------------------------------
    // Pixel data capture and blanking
    //
    // Data is captured one cycle before the output tap (STAGES-2 vs STAGES-1),
    // matching the baseline pipeline phase. During blanking (de_d[STAGES-2]==0),
    // the output is forced to zero.
    // -------------------------------------------------------------------------
    always @(posedge I_clk) begin
        if (I_rst) begin
            O_data_r <= {DATA_WIDTH{1'b0}};
        end else if (de_d[STAGES-2]) begin
            O_data_r <= I_data;
        end else begin
            O_data_r <= {DATA_WIDTH{1'b0}};
        end
    end

    // -------------------------------------------------------------------------
    // Sync signal shift registers
    //
    // Each clock cycle, new input is shifted into bit [0], and all existing
    // bits shift up by one position. The oldest value appears at [STAGES-1].
    // -------------------------------------------------------------------------
    always @(posedge I_clk) begin
        if (I_rst) begin
            hs_d <= {STAGES{1'b0}};
            vs_d <= {STAGES{1'b0}};
            de_d <= {STAGES{1'b0}};
        end else begin
            hs_d <= {hs_d[STAGES-2:0], I_hs};
            vs_d <= {vs_d[STAGES-2:0], I_vs};
            de_d <= {de_d[STAGES-2:0], I_de};
        end
    end

endmodule
