//==============================================================================
// Module: alpha_mixer
// Description: Generic 2:1 Alpha Blending Mixer
//              Used for image transition effects and OSD layer overlay
//
// Formula (per 8-bit channel):
//   O_y_ch = blend_en ? ((I_a_ch * (256 - alpha) + I_b_ch * alpha) >> 8) : I_a_ch
//
// Special Cases:
//   - blend_en == 0: bit-exact bypass to I_a (no multiplier involved)
//   - alpha == 0:    output equals I_a (mathematically exact)
//   - alpha == 255:  output equals I_b (explicit special case to avoid rounding error)
//   - O_de == 0:     output forced to 0 (blanking period)
//
// Pipeline Latency: 1 clock cycle (all outputs registered)
// Sideband Signals: O_hs/O_vs/O_de have same 1-cycle delay as O_y
//
// Ports:
//   I_clk       - Clock input
//   I_rst       - Synchronous active-high reset
//   I_a         - Background source (24-bit RGB, output when alpha==0)
//   I_b         - Foreground source (24-bit RGB, output when alpha==255)
//   I_alpha     - Blend coefficient (8-bit, 0-255)
//   I_blend_en  - Blend enable (0 = bypass I_a, 1 = blend)
//   I_hs        - Horizontal sync input
//   I_vs        - Vertical sync input
//   I_de        - Data enable input
//   O_y         - Blended output (24-bit RGB)
//   O_hs        - Horizontal sync output (1-cycle delayed)
//   O_vs        - Vertical sync output (1-cycle delayed)
//   O_de        - Data enable output (1-cycle delayed)
//==============================================================================

module alpha_mixer #(
    parameter DATA_WIDTH = 24
)(
    input  wire                  I_clk,
    input  wire                  I_rst,
    input  wire [DATA_WIDTH-1:0] I_a,
    input  wire [DATA_WIDTH-1:0] I_b,
    input  wire [7:0]            I_alpha,
    input  wire                  I_blend_en,
    input  wire                  I_hs,
    input  wire                  I_vs,
    input  wire                  I_de,
    output reg  [DATA_WIDTH-1:0] O_y,
    output reg                   O_hs,
    output reg                   O_vs,
    output reg                   O_de
);

    // Function: blend_channel
    // Single-channel alpha blend with special case handling
    // Inputs:
    //   a_ch     - Background channel (8-bit)
    //   b_ch     - Foreground channel (8-bit)
    //   alpha    - Blend coefficient (8-bit)
    //   blend_en - Blend enable
    // Output:
    //   Blended channel value (8-bit)
    function [7:0] blend_channel;
        input [7:0] a_ch;
        input [7:0] b_ch;
        input [7:0] alpha;
        input       blend_en;

        reg [8:0]  inv_alpha;
        reg [16:0] prod_a;
        reg [16:0] prod_b;
        reg [17:0] sum;

        begin
            if (!blend_en) begin
                // Bypass: no blending, output background
                blend_channel = a_ch;
            end else if (alpha == 8'hFF) begin
                // Special case: alpha=255, output foreground exactly
                blend_channel = b_ch;
            end else begin
                // Normal blend: (a * (256-alpha) + b * alpha) >> 8
                inv_alpha = 9'd256 - {1'b0, alpha};
                prod_a = {1'b0, a_ch} * inv_alpha;
                prod_b = {1'b0, b_ch} * {1'b0, alpha};
                sum = {1'b0, prod_a} + {1'b0, prod_b};
                blend_channel = sum[15:8];
            end
        end
    endfunction

    // Pipeline stage: register all outputs with 1-cycle latency
    // O_y and O_hs/O_vs/O_de are all registered from the same clock edge,
    // so O_de(t+1) = I_de(t) and O_y(t+1) is gated by I_de(t).
    // This guarantees: on any cycle where O_de==0, O_y==0 (same cycle).
    always @(posedge I_clk) begin
        if (I_rst) begin
            O_y  <= {DATA_WIDTH{1'b0}};
            O_hs <= 1'b0;
            O_vs <= 1'b0;
            O_de <= 1'b0;
        end else begin
            // Sideband signals: 1-cycle delay, same pipeline as O_y
            O_hs <= I_hs;
            O_vs <= I_vs;
            O_de <= I_de;

            // Blend output gated by I_de (same cycle as data inputs)
            // When I_de==0 this cycle, O_y registers 0 next cycle,
            // matching O_de which also registers 0 next cycle.
            if (I_de) begin
                O_y[23:16] <= blend_channel(I_a[23:16], I_b[23:16], I_alpha, I_blend_en);
                O_y[15:8]  <= blend_channel(I_a[15:8],  I_b[15:8],  I_alpha, I_blend_en);
                O_y[7:0]   <= blend_channel(I_a[7:0],   I_b[7:0],   I_alpha, I_blend_en);
            end else begin
                // Blanking: output zero when de is low
                O_y <= {DATA_WIDTH{1'b0}};
            end
        end
    end

endmodule
