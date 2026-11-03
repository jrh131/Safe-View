`timescale 1ns/1ps
// ============================================================
// Module : osd_logo_rom
// Function: OSD logo bitmap ROM (block RAM, synchronous read)
// Device  : Anlogic EG4S20BG256
// Content : 80 rows x 8 words of 16 bit = 640 entries (128x80 logo)
// Address : y * (LOGO_W/16) + x_word
// Data    : 16-bit per word, bit15 = leftmost pixel, 1 = foreground
// ============================================================

module osd_logo_rom #(
    parameter LOGO_W = 128,       // logo width in pixels (must be multiple of 16)
    parameter LOGO_H = 80         // logo height in pixels
)(
    input  wire        I_clk,     // pixel clock 25MHz (video_clk)
    input  wire [12:0] I_addr,    // y * (LOGO_W/16) + x_word, 0..639 valid
    output reg  [15:0] O_bits     // 16-pixel word; bit15 = leftmost pixel; 1 = foreground
);

    reg [15:0] mem [0:639]; /* fehdl force_ram=1, ram_style="bram" */

    initial begin
`include "../doc/convert/osd_logo_data.vh"
    end

    always @(posedge I_clk) O_bits <= mem[I_addr[9:0]];

endmodule