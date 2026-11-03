//==============================================================================
// Module: transition_mixer
// Description: Transition blending stage for image carousel.
//              Wraps alpha_mixer to handle three transition modes:
//              - Mode 1 (Fade): Full-screen alpha blending using I_alpha.
//              - Mode 2 (Slide Right): New image (B) slides in from right.
//              - Mode 3 (Slide Left): New image (B) slides in from left.
//              - Mode 0 / I_dual_en=0: Bit-exact pass-through of A.
//
// Column Logic: Tracks X coordinate (x_cnt) using I_de to determine
//               which pixels in a slide transition should show image B.
//               x_cnt resets to 0 on the rising edge of I_de (start of line).
//
// Bit-Exact Pass-through: When I_dual_en is low, blend_en to alpha_mixer
//                         is forced low, ensuring I_a passes through with
//                         zero error (critical for "no flicker" acceptance).
//==============================================================================

module transition_mixer #(
    parameter DATA_WIDTH = 24,
    parameter H_ACTIVE   = 640
)(
    input  wire                  I_pix_clk,
    input  wire                  I_rst,        // Active-high synchronous reset
    input  wire [DATA_WIDTH-1:0] I_a,          // Old image (A path)
    input  wire [DATA_WIDTH-1:0] I_b,          // New image (B path)
    input  wire                  I_hs,
    input  wire                  I_vs,
    input  wire                  I_de,         // Aligned with I_a/I_b
    input  wire                  I_dual_en,    // Transition in progress
    input  wire [7:0]            I_alpha,      // Fade coefficient (mode 1)
    input  wire [1:0]            I_mode,       // 0=None, 1=Fade, 2=Right, 3=Left
    input  wire [10:0]           I_off,        // Slide offset 0..H_ACTIVE
    output wire [DATA_WIDTH-1:0] O_y,
    output wire                  O_hs,
    output wire                  O_vs,
    output wire                  O_de
);

    // 11-bit constant for parameter width alignment
    localparam [10:0] H_ACTIVE_11 = H_ACTIVE;

    // 1. Column coordinate generation
    //
    // x_cnt is reset to 0 during blanking (I_de==0) and incremented once per
    // active pixel. The pixel data I_a/I_b and the mixer inputs are registered
    // on the same posedge, so the value of x_cnt observed during an active
    // cycle is exactly the running column index of that pixel (0..H_ACTIVE-1).
    //
    // IMPORTANT: x_cnt must be reset during blanking, NOT on the de rising
    // edge. Resetting on the rising edge leaves a stale value in the first
    // active cycle and lags the pixel stream by one column, shifting the
    // slide boundary by one pixel.
    reg [9:0]  x_cnt;      // 0..H_ACTIVE-1

    always @(posedge I_pix_clk) begin
        if (I_rst) begin
            x_cnt <= 10'd0;
        end else if (I_de) begin
            x_cnt <= x_cnt + 10'd1;
        end else begin
            x_cnt <= 10'd0;
        end
    end

    // 2. Slide mode column switching (combinational)
    wire [10:0] x_ext   = {1'b0, x_cnt};
    wire [10:0] thr     = H_ACTIVE_11 - I_off;             // Threshold for mode 2
    wire        hit_r   = (x_ext >= thr);                  // Right slide hit
    wire        hit_l   = (x_ext <  I_off);                // Left slide hit
    wire        slide_hit = (I_mode == 2'd2) ? hit_r :
                            (I_mode == 2'd3) ? hit_l : 1'b0;

    // 3. Unified alpha_mixer inputs (combinational)
    wire        blend_en  = I_dual_en & ((I_mode == 2'd1) | slide_hit);
    wire [7:0]  mix_alpha = (I_mode == 2'd1) ? I_alpha : 8'hFF;

    // 4. alpha_mixer instantiation
    alpha_mixer #(.DATA_WIDTH(DATA_WIDTH)) u_alpha_mixer (
        .I_clk      (I_pix_clk),
        .I_rst      (I_rst),
        .I_a        (I_a),
        .I_b        (I_b),
        .I_alpha    (mix_alpha),
        .I_blend_en (blend_en),
        .I_hs       (I_hs),
        .I_vs       (I_vs),
        .I_de       (I_de),
        .O_y        (O_y),
        .O_hs       (O_hs),
        .O_vs       (O_vs),
        .O_de       (O_de)
    );

endmodule
