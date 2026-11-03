`timescale 1ns/1ps
// ============================================================
// Module : osd_text_rom
// Function: OSD banner slot-to-glyph index ROM (synchronous read, 1-cycle latency)
// Device  : Anlogic EG4S20BG256 (case-inferred ROM)
// Address : 11-bit slot index, 128 entries
// Data    : 8-bit glyph index per slot
// ============================================================

module osd_text_rom (
    input  wire        I_clk,     // pixel clock 25MHz (video_clk)
    input  wire [10:0] I_addr,    // slot linear index 0..127
    output wire [7:0]  O_glyph    // glyph index for this slot
);

    reg [7:0] O_glyph_r;

    always @(posedge I_clk) begin
        case (I_addr)
`include "../doc/convert/osd_text_data.vh"
            default: O_glyph_r <= 8'd64;
        endcase
    end

    assign O_glyph = O_glyph_r;

endmodule
