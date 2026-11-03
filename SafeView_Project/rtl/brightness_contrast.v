//==============================================================================
// Module: brightness_contrast
// Per-pixel brightness / contrast adjustment.
//   brightness : add a signed offset to each RGB channel.
//   contrast   : (channel - 128) * factor + 128, factor = num/16.
// All results are clamped to [0, 255].
// One-clock pipeline; hs/vs/de are delayed by one cycle to match the data.
//==============================================================================

module brightness_contrast #(
    parameter DATA_WIDTH = 24
)(
    input  wire                  I_clk,
    input  wire                  I_rst,
    input  wire [DATA_WIDTH-1:0] I_rgb,
    input  wire                  I_hs,
    input  wire                  I_vs,
    input  wire                  I_de,
    input  wire [1:0]            I_bright,    // brightness level 0..3
    input  wire [1:0]            I_contrast,  // contrast level 0..3
    output reg  [DATA_WIDTH-1:0] O_rgb,
    output reg                   O_hs,
    output reg                   O_vs,
    output reg                   O_de
);

    function [7:0] adj;
        input [7:0] x;
        input [1:0] bsel;
        input [1:0] csel;
        reg signed [15:0] centered;
        reg signed [15:0] scaled;
        reg signed [15:0] summed;
        reg [3:0]  cnum;
        reg signed [15:0] boff;
        begin
            case (bsel)
                2'd0: boff = -40;
                2'd1: boff =   0;
                2'd2: boff =  40;
                default: boff = 80;
            endcase

            case (csel)
                2'd0: cnum = 12;    // 0.75x
                2'd1: cnum = 16;    // 1.00x
                2'd2: cnum = 20;    // 1.25x
                default: cnum = 24; // 1.50x
            endcase

            centered = $signed({8'd0, x}) - 16'sd128;
            scaled   = centered * $signed({12'd0, cnum});
            summed   = (scaled >>> 4) + 16'sd128 + boff;

            if (summed > 16'sd255)
                adj = 8'd255;
            else if (summed < 16'sd0)
                adj = 8'd0;
            else
                adj = summed[7:0];
        end
    endfunction

    wire [7:0] r_adj = adj(I_rgb[23:16], I_bright, I_contrast);
    wire [7:0] g_adj = adj(I_rgb[15:8],  I_bright, I_contrast);
    wire [7:0] b_adj = adj(I_rgb[7:0],   I_bright, I_contrast);

    always @(posedge I_clk) begin
        if (I_rst) begin
            O_rgb <= {DATA_WIDTH{1'b0}};
            O_hs  <= 1'b0;
            O_vs  <= 1'b0;
            O_de  <= 1'b0;
        end else begin
            O_rgb <= {r_adj, g_adj, b_adj};
            O_hs  <= I_hs;
            O_vs  <= I_vs;
            O_de  <= I_de;
        end
    end

endmodule