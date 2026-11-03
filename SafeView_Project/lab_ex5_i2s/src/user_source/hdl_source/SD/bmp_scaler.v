// ============================================================================
// bmp_scaler.v — 最近邻缩放（纯流式，仅缩小 + 等大）
// ============================================================================
// 把任意分辨率（宽 640~2048、高 480~2048）的 24bit 无压缩 BMP 像素流，
// 缩放到固定 640x480，并保持"自底向上"的到达顺序不变。
//
// 下游 frame_fifo_write 使用 WRITE_V_FLIP=1 且 I_write_len 固定为 307200，
// 依赖"自底向上"的 640x480 输入流；本模块输出恰好满足这两点，因此
// 显示 / 转场 / 告警 / OSD / 音频等 SDRAM 之后的逻辑完全不受影响。
//
// 缩放算法：最近邻（nearest-neighbor），源驱动 DDA + 半偏移（+0.5）。
// 水平方向逐像素判选目标列；垂直方向在"行尾"统一推进下一行的判选，
// 行内用一个 row_en 寄存器保持当前行的判选结果，避免行首/行尾相位错位
// （否则会在等倍/半倍等场景多输出或漏输出一行，导致写 SDRAM 长度失配）。
// 因为仅处理 src_w>=640 且 src_h>=480 的缩小/等大场景，每行/每列最多
// 映射一个目标，故无需任何行缓冲，纯流式。
//
// 时序：frame_start 有效时用 src_w/src_h 初始化 DDA（此时 BMP 头已解析完毕，
// 源宽高已稳定）；之后每个 src_valid 逐像素做水平 DDA 判选。
// ============================================================================

module bmp_scaler #(
    parameter integer TARGET_W = 640,
    parameter integer TARGET_H = 480
)(
    input  wire         clk,
    input  wire         rst,
    input  wire         frame_start,   // 一帧源图数据开始（load_data_go 单拍脉冲）
    input  wire [11:0]  src_w,         // 源宽（640~2048）
    input  wire [11:0]  src_h,         // 源高（480~2048）
    input  wire         src_valid,     // 源像素有效（= bmp 每像素写使能）
    input  wire [23:0]  src_data,      // 源像素，B:G:R 各 8bit
    output reg          out_valid,
    output reg  [23:0]  out_data
);

localparam [12:0] TW = TARGET_W;
localparam [12:0] TH = TARGET_H;

reg [11:0] sx;      // 源列 0..src_w-1
reg [12:0] h_err;   // 水平 DDA 累加器（< src_w）
reg [12:0] v_err;   // 垂直 DDA 累加器（< src_h）
reg        row_en;  // 当前源行是否被垂直选中从而需要输出

// 水平 DDA：hsel 判选当前源像素是否映射到某一目标列
wire [12:0] h_add      = h_err + TW;
wire        hsel       = (h_add >= {1'b0, src_w});
wire [12:0] h_err_next = hsel ? (h_add - {1'b0, src_w}) : h_add;

// 垂直 DDA：在行尾用"当前行判选结果 row_en"推进累加器，得到下一行的
// v_err 与 row_en（严格镜像水平 DDA 的逐项递推，避免相位错位/多行）。
wire [12:0] v_add      = v_err + TH;
wire [12:0] v_err_next = row_en ? (v_add - {1'b0, src_h}) : v_add;
wire        row_en_next = (v_err_next + TH >= {1'b0, src_h});

always @(posedge clk or posedge rst) begin
    if (rst) begin
        sx        <= 12'd0;
        h_err     <= 13'd0;
        v_err     <= 13'd0;
        row_en    <= 1'b0;
        out_valid <= 1'b0;
        out_data  <= 24'd0;
    end else if (frame_start) begin
        // BMP 头已解析完毕，src_w/src_h 此刻有效；初始化 DDA（半偏移）
        sx        <= 12'd0;
        h_err     <= {1'b0, src_w[11:1]};
        v_err     <= {1'b0, src_h[11:1]};
        row_en    <= ({1'b0, src_h[11:1]} + TH >= {1'b0, src_h});
        out_valid <= 1'b0;
    end else begin
        out_valid <= 1'b0;
        if (src_valid) begin
            // 当前源像素：水平选中 且 所在行垂直选中时输出
            if (hsel && row_en) begin
                out_valid <= 1'b1;
                out_data  <= src_data;
            end

            // 列 / 行推进
            if (sx == (src_w - 12'd1)) begin
                // 行尾：水平累加器复位半偏移；垂直 DDA 推进到下一行
                sx     <= 12'd0;
                h_err  <= {1'b0, src_w[11:1]};
                v_err  <= v_err_next;
                row_en <= row_en_next;
            end else begin
                sx    <= sx + 12'd1;
                h_err <= h_err_next;
            end
        end
    end
end

endmodule