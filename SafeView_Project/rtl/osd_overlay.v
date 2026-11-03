`timescale 1ns / 1ps
//============================================================================
// Module : osd_overlay
// Purpose : OSD overlay engine for image-carousel display.
//           Blends three layers onto the frame-buffer base image (L0):
//             L1 - Dynamic text layer (scrolling banner + uptime timestamp)
//             L2 - Static logo bitmap
//           Text layer (L1) has highest priority over logo layer (L2).
//           Each layer uses alpha blending via cascaded alpha_mixer instances.
//
// Pipeline accounting (per pixel x0, steady-state):
//
//   t=0 : I_img/I_hs/I_vs/I_de present for pixel x0; pos_x = x0.
//         Combinational: region flags, ROM addresses, timestamp glyph.
//         text_rom_addr_c and logo_rom_addr_c driven directly to ROMs.
//
//   RE1 : ROMs internally capture their I_addr (x0's addresses).
//         Register stage 1: in_banner_d1, in_ts_d1, in_logo_d1,
//           bit_in_unit_d1, glyph_ts_d1, row_d1.
//         Shift register: img_d1 <= I_img(x0), hs_d1, vs_d1, de_d1.
//
//   t=1 : Text ROM output glyph_banner = glyph(x0).
//         Logo ROM output logo_rom_bits = bitmap(x0).
//         Compute: logo_bit_raw = in_logo_d1 ? logo_rom_bits[15-bit_d1] : 0
//
//   RE2 : Register stage 2: in_banner_d2, in_ts_d2, in_logo_d2,
//           bit_in_unit_d2, glyph_ts_d2, glyph_banner_d2, row_d2, logo_bit_d2.
//         Shift register: img_d2 <= img_d1, hs_d2, vs_d2, de_d2.
//
//   t=2 : img_d2 = I(x0), logo_bit_d2 = logo(x0), de_d2 = de(x0).
//         All mixer1 inputs valid for x0.
//         Compute: glyph_sel, font_rom_addr_c (directly to font ROM).
//
//   RE3 : Mixer1 captures t=2 inputs -> output at t=3.
//         Font ROM internally captures font_rom_addr_c (x0's font addr).
//         Register stage 3: bit_in_unit_d3, in_logo_d3.
//
//   t=3 : Font ROM output font_row = bitmap(x0).
//         Mixer1 output mix1_rgb = logo blend(x0).
//         Compute: txt_bit = font_row[15 - bit_in_unit_d3] (combinational, x0).
//         txt_bit is fed to mixer2 as-is: mixer2's own internal register
//         samples it at RE4, so an extra pre-register would shift the text
//         layer by one pixel relative to the image. Do NOT pre-register it.
//
//   RE4 : Mixer2 captures t=3 inputs -> output at t=4.
//         Register stage 4: in_logo_d4, txt_bit_d4 (for layer_hit).
//
//   t=4 : O_rgb = final pixel(x0). O_hs/O_vs/O_de = sync(x0).
//         O_layer_hit = {in_logo_d4, txt_bit_d4}.
//
//   OSD_PIPE = 2 (shift) + 1 (mixer1) + 1 (mixer2) = 4 cycles.
//
// Layer compositing order (back to front):
//   L0 (image) -> mixer1 blends L2 (logo) -> mixer2 blends L1 (text).
//   When txt_bit=1, COL_TXT dominates. When logo_bit=1 and txt_bit=0,
//   COL_LOGO dominates.
//
// Region definitions (default parameters, 640x480):
//   Banner    : Y in [56, 71],  full X range, 16-row glyph band, scrolling.
//   Timestamp : Y in [434, 449], X in [16, 143], 8 glyphs "HH:MM:SS".
//   Logo      : Y in [24, 103], X in [496, 623], 128x80 bitmap.
//   Banner and timestamp Y-ranges do not overlap, so no priority conflict.
//
// Sub-modules instantiated (all pre-existing, not modified here):
//   alpha_mixer     x2  (logo blend, text blend)
//   osd_scroll_ctrl x1  (scroll phase + BCD uptime)
//   osd_text_rom    x1  (banner slot -> glyph index)
//   osd_font_rom    x1  (glyph + row -> 16-bit pixel row)
//   osd_logo_rom    x1  (logo bitmap)
//============================================================================

module osd_overlay #(
    parameter H_ACTIVE    = 640,
    parameter V_ACTIVE    = 480,
    parameter DATA_WIDTH  = 24,
    parameter OSD_PIPE    = 4,           // Output latency: 2 shift + 2 mixer = 4
    parameter BANNER_Y0   = 56,          // Banner start row (16-row band)
    parameter BANNER_W    = 2048,        // Scroll strip width (must be power of 2)
    parameter SCROLL_STEP = 2,           // px/frame, passed to osd_scroll_ctrl
    parameter TICK_DIV    = 25000000,    // 1s @25MHz, passed to osd_scroll_ctrl
    parameter TEXT_UNITS  = 128,         // Banner strip glyph count (BANNER_W/16)
    parameter LOGO_X0     = 496,
    parameter LOGO_Y0     = 24,
    parameter LOGO_W      = 128,         // Must be multiple of 16
    parameter LOGO_H      = 80,
    parameter TS_X0       = 16,
    parameter TS_Y0       = 434,
    parameter PARAM_X0    = 496,       // Brightness/contrast info X origin
    parameter PARAM_Y0    = 434,       // Brightness/contrast info Y origin
    parameter COL_TXT     = 24'hFFFFFF,  // Banner text foreground colour
    parameter COL_TS      = 24'hFFFFFF,  // Timestamp & brightness/contrast foreground colour
    parameter COL_LOGO    = 24'h1BA0E2   // Logo foreground colour
)(
    input  wire                  I_pix_clk,
    input  wire                  I_rst,              // Active-high synchronous reset
    input  wire [DATA_WIDTH-1:0] I_img,              // Frame-buffer pixel (L0)
    input  wire                  I_hs,
    input  wire                  I_vs,
    input  wire                  I_de,
    input  wire [7:0]            I_alpha_txt,        // Text layer alpha
    input  wire [7:0]            I_alpha_logo,       // Logo layer alpha
    input  wire [3:0]            I_bright,           // Brightness level 1..15 (15=full)
    input  wire [3:0]            I_contrast,         // Contrast level 1..15 (15=neutral)
    input  wire                  I_hide,             // 1=隐藏全部 OSD（告警期间）
    output wire [DATA_WIDTH-1:0] O_rgb,
    output wire                  O_hs,
    output wire                  O_vs,
    output wire                  O_de,
    output wire [1:0]            O_layer_hit         // [1]=logo hit, [0]=text hit
);

    //========================================================================
    // Derived local parameters
    //========================================================================

    // Banner address width: BANNER_W must be a power of 2.
    // Default BANNER_W=1024 -> BAW=10.
    // Truncating a sum to BAW bits is equivalent to modulo BANNER_W only
    // when BANNER_W is a power of 2 (required constraint on the parameter).
    localparam BAW = (BANNER_W <= 2)    ? 1  :
                     (BANNER_W <= 4)    ? 2  :
                     (BANNER_W <= 8)    ? 3  :
                     (BANNER_W <= 16)   ? 4  :
                     (BANNER_W <= 32)   ? 5  :
                     (BANNER_W <= 64)   ? 6  :
                     (BANNER_W <= 128)  ? 7  :
                     (BANNER_W <= 256)  ? 8  :
                     (BANNER_W <= 512)  ? 9  :
                     (BANNER_W <= 1024) ? 10 : 11;

    // Logo stride in 16-pixel words expressed as a shift amount.
    // Default LOGO_W=128 -> LOGO_W_SHIFT=3 (stride = 8 words per row).
    localparam LOGO_W_SHIFT = (LOGO_W <= 16)   ? 0 :
                              (LOGO_W <= 32)   ? 1 :
                              (LOGO_W <= 64)   ? 2 :
                              (LOGO_W <= 128)  ? 3 :
                              (LOGO_W <= 256)  ? 4 :
                              (LOGO_W <= 512)  ? 5 :
                              (LOGO_W <= 1024) ? 6 : 7;

    // Region SPANS (measured from the region origin, not from screen origin).
    // logo_xoff_raw = pos_x - LOGO_X0 already has its origin removed, so the
    // comparison must be against LOGO_W / LOGO_H themselves. Comparing against
    // (LOGO_X0 + LOGO_W) would make the region far too wide and corrupt the
    // in_logo hit flag.
    localparam [10:0] LOGO_X_SPAN = LOGO_W;         // 128 px
    localparam [9:0]  LOGO_Y_SPAN = LOGO_H;         // 80  px
    localparam [10:0] TS_X_SPAN   = 128;            // 8 glyphs * 16 px
    localparam [10:0] PARAM_X_SPAN = 144;           // 9 glyphs * 16 px

    //========================================================================
    // Coordinate generation
    //========================================================================

    reg        de_r;
    reg        vs_r;
    reg [9:0]  pos_x;     // column: 0 .. H_ACTIVE-1
    reg [9:0]  pos_y;     // row:    0 .. V_ACTIVE-1

    wire de_falling = de_r & ~I_de;   // end of active line
    wire vs_falling = vs_r & ~I_vs;   // frame boundary

    always @(posedge I_pix_clk) begin
        if (I_rst) begin
            de_r  <= 1'b0;
            vs_r  <= 1'b0;
            pos_x <= 10'd0;
            pos_y <= 10'd0;
        end else begin
            de_r <= I_de;
            vs_r <= I_vs;

            if (~de_r & I_de)
                pos_x <= 10'd0;
            else if (I_de)
                pos_x <= pos_x + 10'd1;

            if (vs_falling)
                pos_y <= 10'd0;
            else if (de_falling)
                pos_y <= pos_y + 10'd1;
        end
    end

    //========================================================================
    // Input delay shift register (2-stage)
    // img_d2/hs_d2/vs_d2/de_d2 carry the pixel from 2 cycles ago,
    // aligning with the ROM look-up results at the mixer1 input (t=2).
    //========================================================================

    reg [DATA_WIDTH-1:0] img_d1, img_d2;
    reg                  hs_d1,  hs_d2;
    reg                  vs_d1,  vs_d2;
    reg                  de_d1,  de_d2;

    always @(posedge I_pix_clk) begin
        if (I_rst) begin
            img_d1 <= {DATA_WIDTH{1'b0}};
            img_d2 <= {DATA_WIDTH{1'b0}};
            hs_d1  <= 1'b0;  hs_d2 <= 1'b0;
            vs_d1  <= 1'b0;  vs_d2 <= 1'b0;
            de_d1  <= 1'b0;  de_d2 <= 1'b0;
        end else begin
            img_d1 <= I_img;
            img_d2 <= img_d1;
            hs_d1  <= I_hs;
            hs_d2  <= hs_d1;
            vs_d1  <= I_vs;
            vs_d2  <= vs_d1;
            de_d1  <= I_de;
            de_d2  <= de_d1;
        end
    end

    //========================================================================
    // Scroll controller (external sub-module)
    //========================================================================

    wire [10:0] scroll_phase;
    wire [3:0] hh_t, hh_u, mm_t, mm_u, ss_t, ss_u;
    wire       sec_tick;

    osd_scroll_ctrl #(
        .BANNER_W    (BANNER_W),
        .SCROLL_STEP (SCROLL_STEP),
        .TICK_DIV    (TICK_DIV)
    ) u_scroll (
        .I_pix_clk      (I_pix_clk),
        .I_rst          (I_rst),
        .I_vs           (I_vs),
        .O_scroll_phase (scroll_phase),
        .O_hh_t         (hh_t),
        .O_hh_u         (hh_u),
        .O_mm_t         (mm_t),
        .O_mm_u         (mm_u),
        .O_ss_t         (ss_t),
        .O_ss_u         (ss_u),
        .O_sec_tick     (sec_tick)
    );

    //========================================================================
    // t=0: Combinational region detection and ROM address computation
    //========================================================================

    // ---- Banner (scrolling text) region ----
    wire [9:0] row_banner_raw = pos_y - BANNER_Y0;
    wire       in_banner      = (row_banner_raw < 10'd16);

    // Banner column: (scroll_phase + pos_x) truncated to BAW bits.
    // This truncation is equivalent to modulo BANNER_W when BANNER_W is a
    // power of 2 (which is a required constraint on the parameter).
    wire [BAW-1:0] banner_col  = scroll_phase[BAW-1:0] + pos_x;
    wire [BAW-5:0] banner_unit = banner_col[BAW-1:4];

    // Text ROM address: slot index zero-extended to 11 bits.
    // Clamped to 0 outside banner to avoid x-propagation in simulation.
    wire [10:0] text_rom_addr_c = in_banner
        ? {{(11 - BAW + 4){1'b0}}, banner_unit}
        : 11'd0;

    // ---- Timestamp region ----
    wire [9:0]  row_ts_raw = pos_y - TS_Y0;
    wire [10:0] col_ts_raw = pos_x - TS_X0;
    wire        in_ts      = (row_ts_raw < 10'd16) & (col_ts_raw < TS_X_SPAN);

    wire [2:0]  ts_unit    = col_ts_raw[6:4];

    // Timestamp glyph index (combinational, no ROM look-up).
    // Font mapping: ASCII 0x20 -> glyph 64.  '0'..'9' -> 80..89, ':' -> 90.
    reg [7:0] glyph_ts;
    always @(*) begin
        case (ts_unit)
            3'd0:    glyph_ts = 8'd80 + {4'b0000, hh_t};
            3'd1:    glyph_ts = 8'd80 + {4'b0000, hh_u};
            3'd2:    glyph_ts = 8'd90;
            3'd3:    glyph_ts = 8'd80 + {4'b0000, mm_t};
            3'd4:    glyph_ts = 8'd80 + {4'b0000, mm_u};
            3'd5:    glyph_ts = 8'd90;
            3'd6:    glyph_ts = 8'd80 + {4'b0000, ss_t};
            3'd7:    glyph_ts = 8'd80 + {4'b0000, ss_u};
            default: glyph_ts = 8'd64;
        endcase
    end

    // ---- Brightness/contrast info region (bottom-right corner) ----
    wire [9:0]  row_param_raw = pos_y - PARAM_Y0;
    wire [10:0] col_param_raw = pos_x - PARAM_X0;
    wire        in_param      = (row_param_raw < 10'd16) & (col_param_raw < PARAM_X_SPAN);

    wire [3:0]  param_unit    = col_param_raw[7:4];

    // 亮度/对比度 4-bit（1~15）→ 0~100 近似百分比（显示用，非精确）
    function [6:0] level_to_percent;
        input [3:0] lv;
        begin
            case (lv)
                4'd15:   level_to_percent = 7'd100;
                4'd14:   level_to_percent = 7'd93;
                4'd13:   level_to_percent = 7'd86;
                4'd12:   level_to_percent = 7'd79;
                4'd11:   level_to_percent = 7'd71;
                4'd10:   level_to_percent = 7'd64;
                4'd9:    level_to_percent = 7'd57;
                4'd8:    level_to_percent = 7'd50;
                4'd7:    level_to_percent = 7'd43;
                4'd6:    level_to_percent = 7'd36;
                4'd5:    level_to_percent = 7'd29;
                4'd4:    level_to_percent = 7'd21;
                4'd3:    level_to_percent = 7'd14;
                4'd2:    level_to_percent = 7'd7;
                default: level_to_percent = 7'd0;   // 1 -> 0
            endcase
        end
    endfunction

    wire [6:0] bright_pct   = level_to_percent(I_bright);
    wire [6:0] contrast_pct = level_to_percent(I_contrast);

    // 0~100 拆分为百位/十位/个位（BCD 显示）
    wire [3:0] bright_h   = (bright_pct >= 7'd100) ? 4'd1 : 4'd0;
    wire [3:0] bright_t   = (bright_pct / 7'd10) % 7'd10;
    wire [3:0] bright_u   = bright_pct % 7'd10;
    wire [3:0] contrast_h = (contrast_pct >= 7'd100) ? 4'd1 : 4'd0;
    wire [3:0] contrast_t = (contrast_pct / 7'd10) % 7'd10;
    wire [3:0] contrast_u = contrast_pct % 7'd10;

    // Info line glyphs: "B<整数> C<整数>"（0~100）.
    // Font map: 'B'->98, 'C'->99, '0'..'9'->80..89, space->64.
    reg [7:0] glyph_param;
    always @(*) begin
        case (param_unit)
            4'd0:    glyph_param = 8'd98;                             // 'B'
            4'd1:    glyph_param = 8'd80 + {4'b0000, bright_h};       // 百位
            4'd2:    glyph_param = 8'd80 + {4'b0000, bright_t};       // 十位
            4'd3:    glyph_param = 8'd80 + {4'b0000, bright_u};       // 个位
            4'd4:    glyph_param = 8'd64;                             // ' '
            4'd5:    glyph_param = 8'd99;                             // 'C'
            4'd6:    glyph_param = 8'd80 + {4'b0000, contrast_h};     // 百位
            4'd7:    glyph_param = 8'd80 + {4'b0000, contrast_t};     // 十位
            4'd8:    glyph_param = 8'd80 + {4'b0000, contrast_u};     // 个位
            default: glyph_param = 8'd64;                             // ' '
        endcase
    end

    // ---- Logo region ----
    wire [10:0] logo_xoff_raw = pos_x - LOGO_X0;
    wire [9:0]  logo_yoff_raw = pos_y - LOGO_Y0;
    wire        in_logo       = (logo_xoff_raw < LOGO_X_SPAN) &
                                (logo_yoff_raw < LOGO_Y_SPAN);

    wire [10:0] logo_xoff = in_logo ? logo_xoff_raw : 11'd0;
    wire [9:0]  logo_yoff = in_logo ? logo_yoff_raw : 10'd0;
    wire [3:0]  logo_xbit = logo_xoff[3:0];
    wire [6:0]  logo_xw   = logo_xoff[10:4];

    // Logo ROM address: row * stride + word_index.
    // stride = LOGO_W / 16 = 1 << LOGO_W_SHIFT.
    // Clamped to 0 outside logo region to prevent x-propagation.
    wire [12:0] logo_rom_addr_c = in_logo
        ? (({3'd0, logo_yoff} << LOGO_W_SHIFT) + {6'd0, logo_xw})
        : 13'd0;

    // 文本图层专用字内位索引（banner 与 timestamp 区域互不重叠）。
    // 滚动字幕随滚动偏移：(scroll_phase + pos_x) % 16
    // 计时固定格(TS_X0 为 16 的倍数)：pos_x % 16
    // 注意：Logo 图层的字内位另用 logo_xbit（经 logo_xbit_d1 单独流水），
    // 不得复用此信号 —— banner 满宽(全 X)与 logo(Y 56~71 重叠 16 行)存在重叠区，
    // 若共用会把 scroll_phase 偏移混入 logo，导致图上沿 16 行呈周期性分段晃动。
    wire [3:0] bit_in_unit = in_banner ? banner_col[3:0] : pos_x[3:0];

    // Font ROM row selector (shared by banner, timestamp and info)
    wire [3:0] row_sel = in_ts    ? row_ts_raw[3:0]    :
                         in_param ? row_param_raw[3:0] :
                         row_banner_raw[3:0];

    //========================================================================
    // ROM instances
    // Addresses connected directly (combinational). Each ROM internally
    // captures I_addr at the posedge; output is valid one cycle later.
    // So: address driven at t=N, ROM output valid at t=N+1.
    //========================================================================

    // Text ROM: banner slot -> glyph index.
    // I_addr = text_rom_addr_c (driven at t=0 for x0).
    // O_glyph valid at t=1 for x0.
    // font_rom_addr_c is declared here (not at its assignment site) because
    // the font ROM instance below uses it before that assignment appears.
    wire [11:0] font_rom_addr_c;
    wire [7:0] glyph_banner;
    osd_text_rom u_text_rom (
        .I_clk   (I_pix_clk),
        .I_addr  (text_rom_addr_c),
        .O_glyph (glyph_banner)
    );

    // Logo ROM: logo bitmap.
    // I_addr = logo_rom_addr_c (driven at t=0 for x0).
    // O_bits valid at t=1 for x0.
    wire [15:0] logo_rom_bits;
    osd_logo_rom #(
        .LOGO_W (LOGO_W),
        .LOGO_H (LOGO_H)
    ) u_logo_rom (
        .I_clk  (I_pix_clk),
        .I_addr (logo_rom_addr_c),
        .O_bits (logo_rom_bits)
    );

    // Font ROM: glyph bitmap.
    // I_addr = font_rom_addr_c (driven at t=2 for x0).
    // O_row valid at t=3 for x0.
    wire [15:0] font_row;
    osd_font_rom u_font_rom (
        .I_clk  (I_pix_clk),
        .I_addr (font_rom_addr_c),
        .O_row  (font_row)
    );

    //========================================================================
    // RE1 stage: Register t=0 flags and coordinates for use at t=1.
    // (pos_x advances to x0+1 after RE1.)
    //========================================================================

    reg        in_banner_d1;
    reg        in_ts_d1;
    reg        in_param_d1;
    reg        in_logo_d1;
    reg [3:0]  bit_in_unit_d1;
    reg [3:0]  logo_xbit_d1;
    reg [7:0]  glyph_ts_d1;
    reg [7:0]  glyph_param_d1;
    reg [3:0]  row_d1;

    always @(posedge I_pix_clk) begin
        if (I_rst) begin
            in_banner_d1   <= 1'b0;
            in_ts_d1       <= 1'b0;
            in_param_d1    <= 1'b0;
            in_logo_d1     <= 1'b0;
            bit_in_unit_d1 <= 4'd0;
            logo_xbit_d1   <= 4'd0;
            glyph_ts_d1    <= 8'd0;
            glyph_param_d1 <= 8'd0;
            row_d1         <= 4'd0;
        end else begin
            in_banner_d1   <= in_banner;
            in_ts_d1       <= in_ts;
            in_param_d1    <= in_param;
            in_logo_d1     <= in_logo;
            bit_in_unit_d1 <= bit_in_unit;
            logo_xbit_d1   <= logo_xbit;
            glyph_ts_d1    <= glyph_ts;
            glyph_param_d1 <= glyph_param;
            row_d1         <= row_sel;
        end
    end

    //========================================================================
    // t=1: ROM outputs valid for x0.
    //   glyph_banner = text ROM output for x0 (addr captured at RE1).
    //   logo_rom_bits = logo ROM output for x0 (addr captured at RE1).
    //   Compute logo_bit_raw from ROM output and d1 flags (all for x0).
    //========================================================================

    // Logo bit selection: bit15 is the leftmost pixel in the 16-px word.
    // Force to 0 outside the logo region so the logo never bleeds out.
    wire logo_bit_raw = in_logo_d1
        ? logo_rom_bits[4'd15 - {1'b0, logo_xbit_d1}]
        : 1'b0;

    //========================================================================
    // RE2 stage: Register t=1 values for use at t=2.
    //   All d2 signals correspond to pixel x0.
    //========================================================================

    reg        in_banner_d2;
    reg        in_ts_d2;
    reg        in_param_d2;
    reg        in_logo_d2;
    reg [3:0]  bit_in_unit_d2;
    reg [7:0]  glyph_ts_d2;
    reg [7:0]  glyph_param_d2;
    reg [7:0]  glyph_banner_d2;
    reg [3:0]  row_d2;
    reg        logo_bit_d2;

    always @(posedge I_pix_clk) begin
        if (I_rst) begin
            in_banner_d2   <= 1'b0;
            in_ts_d2       <= 1'b0;
            in_param_d2    <= 1'b0;
            in_logo_d2     <= 1'b0;
            bit_in_unit_d2 <= 4'd0;
            glyph_ts_d2    <= 8'd0;
            glyph_param_d2 <= 8'd0;
            glyph_banner_d2 <= 8'd0;
            row_d2         <= 4'd0;
            logo_bit_d2    <= 1'b0;
        end else begin
            in_banner_d2    <= in_banner_d1;
            in_ts_d2        <= in_ts_d1;
            in_param_d2     <= in_param_d1;
            in_logo_d2      <= in_logo_d1;
            bit_in_unit_d2  <= bit_in_unit_d1;
            glyph_ts_d2     <= glyph_ts_d1;
            glyph_param_d2  <= glyph_param_d1;
            glyph_banner_d2 <= glyph_banner;     // text ROM output for x0
            row_d2          <= row_d1;
            logo_bit_d2     <= logo_bit_raw;     // logo bit for x0
        end
    end

    //========================================================================
    // t=2: Mixer1 inputs valid for x0.  Compute font ROM address.
    //
    //   img_d2      = I_img(x0)   via 2-stage shift.
    //   logo_bit_d2 = logo(x0)    via ROM(t=1) + RE2.
    //   hs_d2/vs_d2/de_d2 = sync(x0) via 2-stage shift.
    //
    //   glyph_sel and font_rom_addr_c computed combinationally for x0,
    //   driven directly to font ROM (captured at RE3, output at t=3).
    //========================================================================

    // Glyph arbitration: timestamp takes priority over banner.
    // When neither region is active, select space glyph (8'd64) so the
    // font ROM returns an all-zero bitmap and txt_bit falls to 0.
    wire [7:0] glyph_sel = in_ts_d2     ? glyph_ts_d2     :
                           in_param_d2  ? glyph_param_d2  :
                           in_banner_d2 ? glyph_banner_d2 :
                           8'd64;

    // Font ROM address = {glyph_index[6:0], row[3:0]}
    assign font_rom_addr_c = {glyph_sel[6:0], row_d2};

    //========================================================================
    // Mixer 1 (Logo blend): inputs at t=2, captured at RE3, output at t=3.
    //   I_a       = img_d2         (background = frame-buffer pixel x0)
    //   I_b       = COL_LOGO       (foreground colour)
    //   I_blend_en= logo_bit_d2    (1 = logo pixel, 0 = background)
    //   hs/vs/de  = hs_d2/vs_d2/de_d2 (sync for pixel x0)
    //========================================================================

    wire [DATA_WIDTH-1:0] mix1_rgb;
    wire                  mix1_hs, mix1_vs, mix1_de;

    alpha_mixer #(
        .DATA_WIDTH (DATA_WIDTH)
    ) u_mix_logo (
        .I_clk      (I_pix_clk),
        .I_rst      (I_rst),
        .I_a        (img_d2),
        .I_b        (COL_LOGO),
        .I_alpha    (I_alpha_logo),
        .I_blend_en (logo_bit_d2 & ~I_hide),
        .I_hs       (hs_d2),
        .I_vs       (vs_d2),
        .I_de       (de_d2),
        .O_y        (mix1_rgb),
        .O_hs       (mix1_hs),
        .O_vs       (mix1_vs),
        .O_de       (mix1_de)
    );

    //========================================================================
    // RE3 stage: Register for t=3 use.
    //   Mixer1 captures t=2 inputs -> output at t=3.
    //   Font ROM captures font_rom_addr_c -> output at t=3.
    //========================================================================

    reg [3:0] bit_in_unit_d3;
    reg       in_logo_d3;
    reg       ts_param_d3;

    always @(posedge I_pix_clk) begin
        if (I_rst) begin
            bit_in_unit_d3 <= 4'd0;
            in_logo_d3     <= 1'b0;
            ts_param_d3    <= 1'b0;
        end else begin
            bit_in_unit_d3 <= bit_in_unit_d2;
            in_logo_d3     <= in_logo_d2;
            ts_param_d3    <= in_ts_d2 | in_param_d2;
        end
    end

    //========================================================================
    // t=3: Font ROM output valid for x0.  Mixer1 output valid for x0.
    //   font_row   = font ROM bitmap for x0's glyph/row.
    //   mix1_rgb   = logo-blended pixel for x0.
    //   txt_bit    = font_row[15 - bit_in_unit_d3] (text pixel for x0).
    //
    //   txt_bit is fed combinationally to mixer2.  The "registering" happens
    //   implicitly inside mixer2's internal register (captured at RE4).
    //   This gives OSD_PIPE = 4 as specified.
    //========================================================================

    // Text pixel bit from font bitmap.  bit_in_unit_d3 carries x0's
    // sub-pixel position (delayed through RE1->RE2->RE3).
    wire txt_bit = font_row[4'd15 - {1'b0, bit_in_unit_d3}];

    //========================================================================
    // Mixer 2 (Text blend): inputs at t=3, captured at RE4, output at t=4.
    //   I_a       = mix1_rgb       (logo-blended background from mixer1)
    //   I_b       = COL_TXT        (text foreground colour)
    //   I_blend_en= txt_bit        (combinational text pixel; mixer2's own
    //                               internal register samples it at RE4, so no
    //                               extra pre-register is needed - adding one
    //                               would misalign the text layer by 1 pixel)
    //   hs/vs/de  = mix1_hs/vs/de  (sync from mixer1, same pixel x0)
    //
    // Total pipeline: input(t=0) -> shift(2) -> mixer1(t=3) -> mixer2(t=4)
    //   O_hs/O_vs/O_de delayed by OSD_PIPE = 4 relative to input.
    //========================================================================

    wire [DATA_WIDTH-1:0] txt_color = ts_param_d3 ? COL_TS : COL_TXT;

    wire [DATA_WIDTH-1:0] mix2_rgb;
    wire                  mix2_hs, mix2_vs, mix2_de;

    alpha_mixer #(
        .DATA_WIDTH (DATA_WIDTH)
    ) u_mix_text (
        .I_clk      (I_pix_clk),
        .I_rst      (I_rst),
        .I_a        (mix1_rgb),
        .I_b        (txt_color),
        .I_alpha    (I_alpha_txt),
        .I_blend_en (txt_bit & ~I_hide),
        .I_hs       (mix1_hs),
        .I_vs       (mix1_vs),
        .I_de       (mix1_de),
        .O_y        (mix2_rgb),
        .O_hs       (mix2_hs),
        .O_vs       (mix2_vs),
        .O_de       (mix2_de)
    );

    //========================================================================
    // RE4 stage: Register for t=4 output.
    //   Mixer2 captures t=3 inputs -> output at t=4.
    //   Layer-hit indicators registered for alignment with output pixel.
    //========================================================================

    reg       in_logo_d4;
    reg       txt_bit_d4;

    always @(posedge I_pix_clk) begin
        if (I_rst) begin
            in_logo_d4 <= 1'b0;
            txt_bit_d4 <= 1'b0;
        end else begin
            in_logo_d4 <= in_logo_d3;
            txt_bit_d4 <= txt_bit;      // combinational t=3 value -> valid at t=4
        end
    end

    //========================================================================
    // Output assignment (t=4)
    //========================================================================

    assign O_rgb = mix2_rgb;
    assign O_hs  = mix2_hs;
    assign O_vs  = mix2_vs;
    assign O_de  = mix2_de;

    // Debug layer-hit indicator, aligned with output pixel at t=4.
    //   bit 0 = text layer hit  (txt_bit for x0, registered at RE4)
    //   bit 1 = logo region hit (in_logo for x0, delayed RE1->RE2->RE3->RE4)
    assign O_layer_hit = {in_logo_d4, txt_bit_d4};

endmodule
