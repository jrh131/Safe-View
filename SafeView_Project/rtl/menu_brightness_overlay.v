`timescale 1ns/1ps

// Brightness + contrast scaler plus an on-screen configuration menu.
// The image is scaled first; menu pixels remain at fixed brightness.
module menu_brightness_overlay (
    input  wire        I_clk,
    input  wire        I_rst,
    input  wire [23:0] I_rgb,
    input  wire        I_hs,
    input  wire        I_vs,
    input  wire        I_de,
    input  wire        I_menu_enable,
    input  wire [2:0]  I_cursor,
    input  wire [1:0]  I_period,
    input  wire [3:0]  I_volume,
    input  wire [3:0]  I_brightness,
    input  wire [3:0]  I_contrast,
    input  wire        I_auto_on,
    output reg  [23:0] O_rgb,
    output reg         O_hs,
    output reg         O_vs,
    output reg         O_de
);

reg [10:0] x;
reg [9:0] y;
reg de_prev;
reg [23:0] rgb_scaled;
reg [23:0] rgb_final;
reg menu_pixel;
reg text_pixel;
reg bar_pixel;
reg cursor_pixel;
reg [23:0] menu_color;
reg [2:0] row_sel;
reg [4:0] char_col;
reg [2:0] glyph_row;
reg [2:0] glyph_col;
reg [7:0] char_code;
reg [4:0] glyph_bits;
reg [15:0] r_mul, g_mul, b_mul;
reg signed [15:0] r_cont, g_cont, b_cont;
reg signed [15:0] r_sum, g_sum, b_sum;
reg [10:0] row_y0;

function [7:0] label_char;
    input [2:0] item;
    input [4:0] col;
    begin
        label_char = 8'h20;
        case (item)
            3'd0: case(col) 0:label_char="T";1:label_char="I";2:label_char="M";3:label_char="E"; default:label_char=8'h20; endcase
            3'd1: case(col) 0:label_char="V";1:label_char="O";2:label_char="L"; default:label_char=8'h20; endcase
            3'd2: case(col) 0:label_char="B";1:label_char="R";2:label_char="I";3:label_char="G";4:label_char="H";5:label_char="T"; default:label_char=8'h20; endcase
            3'd3: case(col) 0:label_char="C";1:label_char="O";2:label_char="N";3:label_char="T"; default:label_char=8'h20; endcase
            default: case(col) 0:label_char="E";1:label_char="X";2:label_char="I";3:label_char="T"; default:label_char=8'h20; endcase
        endcase
    end
endfunction

function [4:0] font5x7;
    input [7:0] ch;
    input [2:0] rr;
    begin
        font5x7 = 5'b00000;
        case (ch)
            "A": case(rr) 0:font5x7=5'b01110;1:font5x7=5'b10001;2:font5x7=5'b10001;3:font5x7=5'b11111;4:font5x7=5'b10001;5:font5x7=5'b10001;6:font5x7=5'b10001;endcase
            "B": case(rr) 0:font5x7=5'b11110;1:font5x7=5'b10001;2:font5x7=5'b10001;3:font5x7=5'b11110;4:font5x7=5'b10001;5:font5x7=5'b10001;6:font5x7=5'b11110;endcase
            "C": case(rr) 0:font5x7=5'b01110;1:font5x7=5'b10001;2:font5x7=5'b10000;3:font5x7=5'b10000;4:font5x7=5'b10001;5:font5x7=5'b10001;6:font5x7=5'b01110;endcase
            "D": case(rr) 0:font5x7=5'b11110;1:font5x7=5'b10001;2:font5x7=5'b10001;3:font5x7=5'b10001;4:font5x7=5'b10001;5:font5x7=5'b10001;6:font5x7=5'b11110;endcase
            "E": case(rr) 0:font5x7=5'b11111;1:font5x7=5'b10000;2:font5x7=5'b10000;3:font5x7=5'b11110;4:font5x7=5'b10000;5:font5x7=5'b10000;6:font5x7=5'b11111;endcase
            "G": case(rr) 0:font5x7=5'b01110;1:font5x7=5'b10001;2:font5x7=5'b10000;3:font5x7=5'b10111;4:font5x7=5'b10001;5:font5x7=5'b10001;6:font5x7=5'b01110;endcase
            "H": case(rr) 0:font5x7=5'b10001;1:font5x7=5'b10001;2:font5x7=5'b10001;3:font5x7=5'b11111;4:font5x7=5'b10001;5:font5x7=5'b10001;6:font5x7=5'b10001;endcase
            "I": case(rr) 0:font5x7=5'b11111;1:font5x7=5'b00100;2:font5x7=5'b00100;3:font5x7=5'b00100;4:font5x7=5'b00100;5:font5x7=5'b00100;6:font5x7=5'b11111;endcase
            "L": case(rr) 0:font5x7=5'b10000;1:font5x7=5'b10000;2:font5x7=5'b10000;3:font5x7=5'b10000;4:font5x7=5'b10000;5:font5x7=5'b10000;6:font5x7=5'b11111;endcase
            "M": case(rr) 0:font5x7=5'b10001;1:font5x7=5'b11011;2:font5x7=5'b10101;3:font5x7=5'b10101;4:font5x7=5'b10001;5:font5x7=5'b10001;6:font5x7=5'b10001;endcase
            "N": case(rr) 0:font5x7=5'b10001;1:font5x7=5'b11001;2:font5x7=5'b10101;3:font5x7=5'b10011;4:font5x7=5'b10001;5:font5x7=5'b10001;6:font5x7=5'b10001;endcase
            "O": case(rr) 0:font5x7=5'b01110;1:font5x7=5'b10001;2:font5x7=5'b10001;3:font5x7=5'b10001;4:font5x7=5'b10001;5:font5x7=5'b10001;6:font5x7=5'b01110;endcase
            "R": case(rr) 0:font5x7=5'b11110;1:font5x7=5'b10001;2:font5x7=5'b10001;3:font5x7=5'b11110;4:font5x7=5'b10100;5:font5x7=5'b10010;6:font5x7=5'b10001;endcase
            "T": case(rr) 0:font5x7=5'b11111;1:font5x7=5'b00100;2:font5x7=5'b00100;3:font5x7=5'b00100;4:font5x7=5'b00100;5:font5x7=5'b00100;6:font5x7=5'b00100;endcase
            "U": case(rr) 0:font5x7=5'b10001;1:font5x7=5'b10001;2:font5x7=5'b10001;3:font5x7=5'b10001;4:font5x7=5'b10001;5:font5x7=5'b10001;6:font5x7=5'b01110;endcase
            "V": case(rr) 0:font5x7=5'b10001;1:font5x7=5'b10001;2:font5x7=5'b10001;3:font5x7=5'b10001;4:font5x7=5'b10001;5:font5x7=5'b01010;6:font5x7=5'b00100;endcase
            "X": case(rr) 0:font5x7=5'b10001;1:font5x7=5'b10001;2:font5x7=5'b01010;3:font5x7=5'b00100;4:font5x7=5'b01010;5:font5x7=5'b10001;6:font5x7=5'b10001;endcase
            default: font5x7=5'b00000;
        endcase
    end
endfunction

always @(*) begin
    r_mul = I_rgb[23:16] * I_brightness;
    g_mul = I_rgb[15:8]  * I_brightness;
    b_mul = I_rgb[7:0]   * I_brightness;
    if (I_brightness == 4'd15)
        rgb_scaled = I_rgb;
    else
        rgb_scaled = {r_mul[11:4], g_mul[11:4], b_mul[11:4]};

    r_cont = ($signed({1'b0, rgb_scaled[23:16]}) - 10'sd128) * $signed({1'b0, I_contrast});
    g_cont = ($signed({1'b0, rgb_scaled[15:8]})  - 10'sd128) * $signed({1'b0, I_contrast});
    b_cont = ($signed({1'b0, rgb_scaled[7:0]})   - 10'sd128) * $signed({1'b0, I_contrast});
    r_sum = (r_cont >>> 4) + 16'sd128;
    g_sum = (g_cont >>> 4) + 16'sd128;
    b_sum = (b_cont >>> 4) + 16'sd128;
    if (I_contrast == 4'd15)
        rgb_final = rgb_scaled;
    else
        rgb_final = { r_sum[7:0], g_sum[7:0], b_sum[7:0] };

    menu_pixel = I_menu_enable && I_de && x>=11'd130 && x<11'd510 && y>=10'd60 && y<10'd360;
    text_pixel = 1'b0;
    bar_pixel = 1'b0;
    cursor_pixel = 1'b0;
    menu_color = 24'h102040;
    row_sel = 3'd7;
    row_y0 = 11'd0;

    if (y>=10'd115 && y<10'd155) begin row_sel=3'd0; row_y0=11'd115; end
    else if (y>=10'd160 && y<10'd200) begin row_sel=3'd1; row_y0=11'd160; end
    else if (y>=10'd205 && y<10'd245) begin row_sel=3'd2; row_y0=11'd205; end
    else if (y>=10'd250 && y<10'd290) begin row_sel=3'd3; row_y0=11'd250; end
    else if (y>=10'd300 && y<10'd340) begin row_sel=3'd4; row_y0=11'd300; end

    char_col = 5'd0;
    glyph_row = 3'd0;
    glyph_col = 3'd0;
    char_code = 8'h20;
    glyph_bits = 5'd0;
    if (row_sel < 3'd5 && x>=11'd160 && x<11'd256 &&
        y>=row_y0+11'd12 && y<row_y0+11'd26) begin
        char_col = (x-11'd160)/12;
        glyph_col = ((x-11'd160)%12) >> 1;
        glyph_row = (y-row_y0-11'd12) >> 1;
        char_code = label_char(row_sel,char_col);
        glyph_bits = font5x7(char_code,glyph_row);
        if (glyph_col < 3'd5)
            text_pixel = glyph_bits[4-glyph_col];
    end

    // Large MENU title.
    if (x>=11'd278 && x<11'd326 && y>=10'd78 && y<10'd92) begin
        char_col = (x-11'd278)/12;
        glyph_col = ((x-11'd278)%12) >> 1;
        glyph_row = (y-10'd78) >> 1;
        char_code = label_char(3'd0,char_col); // overwritten below to spell MENU
        case (char_col)
            0: char_code="M"; 1:char_code="E"; 2:char_code="N"; default:char_code="U";
        endcase
        glyph_bits = font5x7(char_code,glyph_row);
        if (glyph_col < 3'd5) text_pixel = glyph_bits[4-glyph_col];
    end

    if (row_sel < 3'd5 && row_sel==I_cursor &&
        (x==11'd145 || x==11'd146 || x==11'd493 || x==11'd494 ||
         y==row_y0 || y==row_y0+11'd39))
        cursor_pixel = 1'b1;

    if (row_sel==3'd0 && x>=11'd310 && x<11'd446 && y>=row_y0+11'd12 && y<row_y0+11'd27)
        bar_pixel = ((x-11'd310)/34 <= I_period);
    else if (row_sel==3'd1 && x>=11'd310 && x<11'd445 && y>=row_y0+11'd12 && y<row_y0+11'd27)
        bar_pixel = (x-11'd310 < I_volume*9);
    else if (row_sel==3'd2 && x>=11'd310 && x<11'd445 && y>=row_y0+11'd12 && y<row_y0+11'd27)
        bar_pixel = (x-11'd310 < I_brightness*9);
    else if (row_sel==3'd3 && x>=11'd310 && x<11'd445 && y>=row_y0+11'd12 && y<row_y0+11'd27)
        bar_pixel = (x-11'd310 < I_contrast*9);

    if (menu_pixel) begin
        if (cursor_pixel) menu_color = 24'hFFFF00;
        else if (text_pixel) menu_color = 24'hFFFFFF;
        else if (bar_pixel) menu_color = 24'h20D080;
        else if (x==11'd130 || x==11'd509 || y==10'd60 || y==10'd359) menu_color = 24'h40A0FF;
        else menu_color = 24'h102040;
    end

    // Auto-play status lamp in the title area.
    if (menu_pixel && x>=11'd460 && x<11'd480 && y>=10'd78 && y<10'd98)
        menu_color = I_auto_on ? 24'h00FF40 : 24'h802020;
end

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        x<=0; y<=0; de_prev<=0;
        O_rgb<=0; O_hs<=0; O_vs<=0; O_de<=0;
    end else begin
        de_prev <= I_de;
        if (I_de) begin
            if (!de_prev) x<=0;
            else x<=x+1'b1;
        end else begin
            x<=0;
            if (de_prev) begin
                if (y==10'd479) y<=0;
                else y<=y+1'b1;
            end
        end
        O_hs <= I_hs;
        O_vs <= I_vs;
        O_de <= I_de;
        if (!I_de) O_rgb <= 24'd0;
        else if (menu_pixel) O_rgb <= menu_color;
        else O_rgb <= rgb_final;
    end
end

endmodule