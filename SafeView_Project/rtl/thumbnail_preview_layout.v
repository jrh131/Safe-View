`timescale 1ns/1ps

// Automatic thumbnail storage and 640x480 preview compositor.
// Four 80x60 RGB332 thumbnails are written while BMP files are read.
// Short key1 selects the next image; long key1 selects single/2x2 layout.
module thumbnail_preview_layout (
    input  wire        I_wr_clk,
    input  wire        I_wr_en,
    input  wire [14:0] I_wr_addr,
    input  wire [7:0]  I_wr_data,

    input  wire        I_pix_clk,
    input  wire        I_rst,
    input  wire        I_thumb_ready,
    input  wire [1:0]  I_layout_mode,
    input  wire [2:0]  I_selected,
    input  wire [23:0] I_rgb,
    input  wire        I_hs,
    input  wire        I_vs,
    input  wire        I_de,
    output reg  [23:0] O_rgb,
    output reg         O_hs,
    output reg         O_vs,
    output reg         O_de
);

wire [7:0] q0, q1, q2, q3, q4;
reg [14:0] rd_addr;
reg [2:0] rd_bank_d;
reg [7:0] thumb_q;

wire [2:0] wr_bank = I_wr_addr[14:12];

// Five 4096x8 true dual-port blocks. Port A is TF clock/write,
// port B is pixel clock/read.
EG_LOGIC_BRAM #(
    .DATA_WIDTH_A(8), .ADDR_WIDTH_A(12), .DATA_DEPTH_A(4096),
    .DATA_WIDTH_B(8), .ADDR_WIDTH_B(12), .DATA_DEPTH_B(4096),
    .MODE("DP"), .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
    .IMPLEMENT("32K"), .INIT_FILE("NONE"), .FILL_ALL("NONE")
) u_thumb_ram0 (
    .doa(), .addra(I_wr_addr[11:0]), .clka(I_wr_clk), .dia(I_wr_data),
    .cea(1'b1), .ocea(1'b0), .wea(I_wr_en && wr_bank==3'd0), .rsta(1'b0), .bea(1'b0),
    .dob(q0), .addrb(rd_addr[11:0]), .clkb(I_pix_clk), .dib(8'd0),
    .ceb(1'b1), .oceb(1'b0), .web(1'b0), .rstb(1'b0), .beb(1'b0)
);
EG_LOGIC_BRAM #(
    .DATA_WIDTH_A(8), .ADDR_WIDTH_A(12), .DATA_DEPTH_A(4096),
    .DATA_WIDTH_B(8), .ADDR_WIDTH_B(12), .DATA_DEPTH_B(4096),
    .MODE("DP"), .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
    .IMPLEMENT("32K"), .INIT_FILE("NONE"), .FILL_ALL("NONE")
) u_thumb_ram1 (
    .doa(), .addra(I_wr_addr[11:0]), .clka(I_wr_clk), .dia(I_wr_data),
    .cea(1'b1), .ocea(1'b0), .wea(I_wr_en && wr_bank==3'd1), .rsta(1'b0), .bea(1'b0),
    .dob(q1), .addrb(rd_addr[11:0]), .clkb(I_pix_clk), .dib(8'd0),
    .ceb(1'b1), .oceb(1'b0), .web(1'b0), .rstb(1'b0), .beb(1'b0)
);
EG_LOGIC_BRAM #(
    .DATA_WIDTH_A(8), .ADDR_WIDTH_A(12), .DATA_DEPTH_A(4096),
    .DATA_WIDTH_B(8), .ADDR_WIDTH_B(12), .DATA_DEPTH_B(4096),
    .MODE("DP"), .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
    .IMPLEMENT("32K"), .INIT_FILE("NONE"), .FILL_ALL("NONE")
) u_thumb_ram2 (
    .doa(), .addra(I_wr_addr[11:0]), .clka(I_wr_clk), .dia(I_wr_data),
    .cea(1'b1), .ocea(1'b0), .wea(I_wr_en && wr_bank==3'd2), .rsta(1'b0), .bea(1'b0),
    .dob(q2), .addrb(rd_addr[11:0]), .clkb(I_pix_clk), .dib(8'd0),
    .ceb(1'b1), .oceb(1'b0), .web(1'b0), .rstb(1'b0), .beb(1'b0)
);
EG_LOGIC_BRAM #(
    .DATA_WIDTH_A(8), .ADDR_WIDTH_A(12), .DATA_DEPTH_A(4096),
    .DATA_WIDTH_B(8), .ADDR_WIDTH_B(12), .DATA_DEPTH_B(4096),
    .MODE("DP"), .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
    .IMPLEMENT("32K"), .INIT_FILE("NONE"), .FILL_ALL("NONE")
) u_thumb_ram3 (
    .doa(), .addra(I_wr_addr[11:0]), .clka(I_wr_clk), .dia(I_wr_data),
    .cea(1'b1), .ocea(1'b0), .wea(I_wr_en && wr_bank==3'd3), .rsta(1'b0), .bea(1'b0),
    .dob(q3), .addrb(rd_addr[11:0]), .clkb(I_pix_clk), .dib(8'd0),
    .ceb(1'b1), .oceb(1'b0), .web(1'b0), .rstb(1'b0), .beb(1'b0)
);
EG_LOGIC_BRAM #(
    .DATA_WIDTH_A(8), .ADDR_WIDTH_A(12), .DATA_DEPTH_A(4096),
    .DATA_WIDTH_B(8), .ADDR_WIDTH_B(12), .DATA_DEPTH_B(4096),
    .MODE("DP"), .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
    .IMPLEMENT("32K"), .INIT_FILE("NONE"), .FILL_ALL("NONE")
) u_thumb_ram4 (
    .doa(), .addra(I_wr_addr[11:0]), .clka(I_wr_clk), .dia(I_wr_data),
    .cea(1'b1), .ocea(1'b0), .wea(I_wr_en && wr_bank==3'd4), .rsta(1'b0), .bea(1'b0),
    .dob(q4), .addrb(rd_addr[11:0]), .clkb(I_pix_clk), .dib(8'd0),
    .ceb(1'b1), .oceb(1'b0), .web(1'b0), .rstb(1'b0), .beb(1'b0)
);

reg [10:0] x;
reg [9:0]  y;
reg        de_prev;
reg        de_d;
reg        hs_d;
reg        vs_d;
reg [23:0] rgb_d;
reg        use_thumb_d;
reg        border_d;
reg [1:0]  thumb_idx_d;

reg [1:0] idx_c;
reg [6:0] tx_c;
reg [5:0] ty_c;
reg       use_c;
reg       border_c;
reg [14:0] addr_c;

function [14:0] thumb_base;
    input [1:0] idx;
    begin
        case (idx)
            2'd0: thumb_base = 15'd0;
            2'd1: thumb_base = 15'd4800;
            2'd2: thumb_base = 15'd9600;
            default: thumb_base = 15'd14400;
        endcase
    end
endfunction

always @(*) begin
    idx_c = 2'd0;
    tx_c = 7'd0;
    ty_c = 6'd0;
    use_c = 1'b0;
    border_c = 1'b0;

    // Mode 0: unobstructed full image; mode 1: bottom thumbnail strip;
    // mode 2: four-window overview.
    if (I_thumb_ready && I_de && (I_layout_mode == 2'd2)) begin
        idx_c = {y >= 10'd240, x >= 11'd320};
        tx_c = ((x >= 11'd320) ? (x - 11'd320) : x) >> 2;
        ty_c = ((y >= 10'd240) ? (y - 10'd240) : y) >> 2;
        use_c = 1'b1;
        // tx_c/ty_c are local coordinates within the selected 320x240
        // quadrant, scaled down by four. One thumbnail pixel therefore
        // produces a four-pixel-wide highlight border.
        border_c = (idx_c == I_selected) &&
                   ((tx_c < 7'd1) || (tx_c >= 7'd79) ||
                    (ty_c < 6'd1) || (ty_c >= 6'd59));
    end else if (I_thumb_ready && I_de && (I_layout_mode == 2'd1) &&
                 (y >= 10'd416) && (y < 10'd476) &&
                 (x >= 11'd160) && (x < 11'd480)) begin
        idx_c = (x - 11'd160) >> 6;
        // Correct 80-pixel slot selection (division by 80 via ranges).
        if (x < 11'd240) begin idx_c=2'd0; tx_c=x-11'd160; end
        else if (x < 11'd320) begin idx_c=2'd1; tx_c=x-11'd240; end
        else if (x < 11'd400) begin idx_c=2'd2; tx_c=x-11'd320; end
        else begin idx_c=2'd3; tx_c=x-11'd400; end
        ty_c = y - 10'd416;
        use_c = 1'b1;
        border_c = (idx_c == I_selected) &&
                   ((tx_c < 7'd2) || (tx_c >= 7'd78) ||
                    (ty_c < 6'd2) || (ty_c >= 6'd58));
    end

    addr_c = thumb_base(idx_c) + (ty_c * 15'd80) + tx_c;
end

// Select the BRAM whose 4K address page contains the requested byte.
// Keeping this mux combinational avoids adding an extra pixel of latency
// after the synchronous BRAM read.
always @(*) begin
    case (rd_bank_d)
        3'd0: thumb_q = q0;
        3'd1: thumb_q = q1;
        3'd2: thumb_q = q2;
        3'd3: thumb_q = q3;
        default: thumb_q = q4;
    endcase
end

always @(posedge I_pix_clk or posedge I_rst) begin
    if (I_rst) begin
        x <= 11'd0;
        y <= 10'd0;
        de_prev <= 1'b0;
        rd_addr <= 15'd0;
        rd_bank_d <= 3'd0;
        de_d <= 1'b0;
        hs_d <= 1'b0;
        vs_d <= 1'b0;
        rgb_d <= 24'd0;
        use_thumb_d <= 1'b0;
        border_d <= 1'b0;
        thumb_idx_d <= 2'd0;
        O_rgb <= 24'd0;
        O_hs <= 1'b0;
        O_vs <= 1'b0;
        O_de <= 1'b0;
    end else begin
        de_prev <= I_de;
        // Count coordinates from active-video line starts. This is
        // independent of the HS/VS polarity used by the HDMI timing core.
        if (I_de) begin
            if (!de_prev)
                x <= 11'd0;
            else
                x <= x + 11'd1;
        end else begin
            x <= 11'd0;
            if (de_prev) begin
                if (y == 10'd479)
                    y <= 10'd0;
                else
                    y <= y + 10'd1;
            end
        end

        rd_addr <= addr_c;
        rd_bank_d <= addr_c[14:12];
        de_d <= I_de;
        hs_d <= I_hs;
        vs_d <= I_vs;
        rgb_d <= I_rgb;
        use_thumb_d <= use_c;
        border_d <= border_c;
        thumb_idx_d <= idx_c;

        O_de <= de_d;
        O_hs <= hs_d;
        O_vs <= vs_d;
        if (!de_d)
            O_rgb <= 24'd0;
        else if (border_d)
            O_rgb <= 24'hFFFF00;
        else if (use_thumb_d)
            O_rgb <= {thumb_q[7:5],thumb_q[7:5],thumb_q[7:6],
                      thumb_q[4:2],thumb_q[4:2],thumb_q[4:3],
                      thumb_q[1:0],thumb_q[1:0],thumb_q[1:0],thumb_q[1:0]};
        else
            O_rgb <= rgb_d;
    end
end

endmodule
