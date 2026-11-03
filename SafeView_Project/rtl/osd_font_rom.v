`timescale 1ns/1ps
// ============================================================
// Module : osd_font_rom
// Function: OSD font bitmap ROM (block RAM, synchronous read)
// Device  : Anlogic EG4S20BG256
// Address : {glyph_idx[6:0], row[3:0]}, 2048 entries total
// Data    : 16-bit per row, bit15 = leftmost pixel, 1 = foreground
// ============================================================

module osd_font_rom (
    input  wire        I_clk,     // pixel clock 25MHz (video_clk)
    input  wire [11:0] I_addr,    // {glyph_idx[6:0], row[3:0]}, 2048 entries
    output reg  [15:0] O_row      // 16-bit row bitmap; bit15 = leftmost pixel; 1 = foreground
);

    reg [15:0] mem [0:2047]; /* fehdl force_ram=1, ram_style="bram" */

    initial begin
`include "../doc/convert/osd_font_data.vh"
    end

    always @(posedge I_clk) O_row <= mem[I_addr[10:0]];

endmodule