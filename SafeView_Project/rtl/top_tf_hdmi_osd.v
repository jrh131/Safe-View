// ============================================================================
// Top-Level Module: top_tf_hdmi_osd.v
// ============================================================================
// 本模块是基线顶层 top_tf_hdmi_audio.v 的扩展副本，包含两部分功能：
//   扩展一：OSD 图层叠加与滚动字幕
//   扩展二：双流读取 + 转场特效引擎
//
// 基线文件：lab_ex4_tf/src/user_source/hdl_source/top_tf_hdmi_audio.v
// 基线状态：已验证可上板工作
//
// SDRAM 物理区域划分（三区域轮转，保证读写不冲突）：
//   REGION 0 : 基地址 0       (21'd0)      -- 640x480 = 307200 像素
//   REGION 1 : 基地址 307200  (21'd307200) -- 640x480 = 307200 像素
//   REGION 2 : 基地址 614400  (21'd614400) -- 640x480 = 307200 像素
// 三个区域始终互不相同：当前显示区域 A、转场新图区域 B、loader 写入区域 W。
//
// 转场期间写保护机制：
//   transition_ctrl 维护 disp_reg / wr_reg / spare_reg 三区域轮转不变式，
//   保证 O_wr_addr 始终指向第三个空闲区域，不会写入正在被读取的 A 或 B
//   区域。loader（sd_card_bmp）的写目标地址由 transition_ctrl.O_wr_addr
//   接管，通过 frame_read_write_dual 的 I_wr_addr 端口传入，替代基线
//   的 write_addr_index 查表方式。
//
// 主要改动（相对于基线 top_tf_hdmi_audio.v）：
// 1. frame_read_write 替换为 frame_read_write_dual（双读通道 A/B）
// 2. 双路 video_delay_640 分别接收 rd_data_a / rd_data_b 的像素数据
// 3. alpha_mixer 替换为 transition_mixer（支持 fade / slide-right / slide-left）
// 4. 新增 transition_ctrl 实例（三区域轮转 + 转场系数生成 + CDC 同步）
// 5. OSD 叠加引擎 osd_overlay（滚动字幕 + Logo 叠加）
// 6. video_rgb_to_axis_640x480 输入源改为 OSD 输出
//
// 语言标准：Verilog-2001（纯 Verilog，无 SystemVerilog 扩展）
// ============================================================================

module top(
    input                       clk,
    input                       rst_n,
    input                       key1,           // 手动下一张
    input                       key2,           // 自动播放 开/关 + 长按切周期
    input                       key3,           // 手动上一张 + 菜单上一选项 + 长按上一布局模式
    input  [3:0]                sw,             // SW[0]=火灾告警使能 SW[1]=音频波形 SW[2]=震动告警使能 SW[3]=特效选择
    input                       quake_in,       // 震动传感器SW-420 (J1 Pin1 = D14)
    input                       fire_in,        // 烟雾传感器 DO (J2 Pin1 = P4, 丝印 GPIOB_0)

    output [5:0]                seg_sel,
    output [7:0]                seg_data,

    // HDMI TMDS
    output                      HDMI_CLK_P,
    output                      HDMI_D2_P,
    output                      HDMI_D1_P,
    output                      HDMI_D0_P,

    // HDMI DDC
    output                      HDMI_DDC_SCL,
    inout                       HDMI_DDC_SDA,

    // TF card SPI
    output                      sd_ncs,
    output                      sd_dclk,
    output                      sd_mosi,
    input                       sd_miso
);

parameter MEM_DATA_BITS = 32;
parameter ADDR_BITS     = 21;
parameter BUSRT_BITS    = 10;
parameter FRAME_PIXELS  = 24'd307200;   // 640*480

wire Sdr_init_done;
wire Sdr_init_ref_vld;
wire Sdr_busy;

wire sd_card_clk;
wire ext_mem_clk;
wire ext_mem_clk_sft;
wire video_clk;
wire hdmi_5x_clk;

wire        display_valid;

wire [3:0]  state_code;

wire [2:0]  img_count;
wire [1:0]  auto_period_sel;    // 0:0.5s 1:1s 2:2s 3:4s
wire        auto_play_on;
wire [3:0]  brightness_level;   // 统一亮度（仅由菜单 BRIGHT 控制，4-bit：1~15，15=满亮度）
wire [3:0]  contrast_level;     // 统一对比度（仅由菜单 CONT 控制，4-bit：1~15，15=中性）
wire [6:0]  seg_data_curimg;   // 数码管：当前图片编号 0~5
wire [6:0]  seg_data_imgcnt;   // 数码管：图片总数 0~6
wire [6:0]  seg_data_period;   // 数码管：轮播周期档位 0~3（0.5/1/2/4s）
wire [6:0]  seg_data_layout;   // 数码管：缩略图布局 0~2（单图/底部条/2x2）
wire [6:0]  seg_data_vol;      // 数码管：音量 0~15
wire [6:0]  seg_data_bright;   // 数码管：亮度 0~15
wire [1:0]  fx_sel;            // SW4 特效选择（供转场引擎）

wire        video_read_req;
wire        video_read_req_ack;
wire        video_read_en;

wire        sd_card_write_en;
wire [31:0] sd_card_write_data;
wire        sd_card_write_req;
wire        sd_card_write_req_ack;
wire        frame_write_finish;
reg         frame_write_toggle_mem;



wire App_rd_en;
wire [ADDR_BITS-1:0] App_rd_addr;
wire Sdr_rd_en;
wire [MEM_DATA_BITS-1:0] Sdr_rd_dout;
wire App_wr_en;
wire [ADDR_BITS-1:0] App_wr_addr;
wire [MEM_DATA_BITS-1:0] App_wr_din;
wire [3:0] App_wr_dm;

// 转场引擎相关线网（由 frame_read_write_dual / transition_ctrl 驱动）
wire [31:0] rd_data_a;
wire [31:0] rd_data_b;
wire        empty_a;
wire        empty_b;
wire        dual_en;
wire [7:0]  trans_alpha;
wire [1:0]  trans_mode;
wire [10:0] trans_off;
wire        commit_toggle;
wire [20:0] rd_addr_a;
wire [20:0] rd_addr_b;
wire [20:0] wr_addr;
wire        trans_cfg_toggle;

// 告警旁路切图的等效读地址/使能/握手线网。
// 必须在 frame_read_write_dual 实例之前用正确位宽显式声明，
// 否则端口连接处会对未声明标识符生成 1 位隐式网，把 21 位读地址截断成 1 位。
wire [20:0] rd_addr_a_eff;
wire [20:0] rd_addr_b_eff;
wire        dual_en_eff;
wire        cfg_toggle_eff;

wire hs_0;
wire vs_0;
wire de_0;

// HDMI 1.4b 相关
wire        axis_s_user;
wire        axis_s_valid;
wire        axis_s_last;
wire [23:0] axis_s_data;
wire        axis_s_ready;

wire        edid_trig;
wire        edid_valid;
wire [7:0]  edid_data;

wire [9:0]  tmds_ch0_data;
wire [9:0]  tmds_ch1_data;
wire [9:0]  tmds_ch2_data;
wire [9:0]  tmds_clk_data;

// HDMI 音频链路
wire        audio_pll_lock;
wire        audio_mclk;
wire        audio_i2s_bclk;
wire        audio_i2s_lrck;
wire        audio_i2s_dout;
wire        audio_valid;
wire [23:0] audio_left_data;
wire [23:0] audio_right_data;
wire        acr_valid;
wire [19:0] acr_cts;
wire [19:0] acr_n;

// 统一复位：TF 图像链路 + HDMI 链路
wire sys_pll_lock;
wire video_pll_lock;
wire rst_all;

reg [19:0] rst_cnt = 20'd0;
reg rst_all_reg = 1'b1;

// OSD 扩展：A/B 双路延迟信号
wire [23:0] img_a;
wire [23:0] img_b;
wire        hs_a;
wire        vs_a;
wire        de_a;
wire        hs_b;
wire        vs_b;
wire        de_b;

// OSD 扩展：转场混合器输出（本阶段为 A 路直通）
wire [23:0] img_blend;
wire        hs_m;
wire        vs_m;
wire        de_m;

// OSD 扩展：OSD 叠加引擎输入输出
wire [23:0] osd_img_in;
wire [23:0] osd_rgb;
wire        osd_hs;
wire        osd_vs;
wire        osd_de;
wire [1:0]  osd_layer_hit;

// 缩略图预览输出
wire [23:0] preview_rgb;
wire        preview_hs;
wire        preview_vs;
wire        preview_de;

// 音频波形叠加输出（sw[1] 控制唤出/隐藏）
wire [23:0] wave_rgb;
wire        wave_hs;
wire        wave_vs;
wire        wave_de;

// 缩略图/音频切换：sd_card_bmp 输出的缩略图与图片索引信号
wire [2:0]  current_image_index;
wire        thumbnail_ready;
wire [1:0]  layout_mode;
wire        thumb_write_en;
wire [14:0] thumb_write_addr;
wire [7:0]  thumb_write_data;

// 菜单控制与统一音量/亮度（由 sd_card_bmp 输出）
wire        menu_enable;
wire [2:0]  menu_cursor;
wire [3:0]  volume_level;
wire        auto_play_status;

// 菜单叠加输出（最上层，绘制在波形之上）
wire [23:0] menu_rgb;
wire        menu_hs;
wire        menu_vs;
wire        menu_de;

// 地震告警频闪输出（quake_alarm 在 video_clk 域，替换为橙频闪）
wire [23:0] alarm_rgb;
wire        alarm_hs;
wire        alarm_vs;
wire        alarm_de;
wire        quake_alarm;
wire        alert_disp;       // 延时对齐后的显示告警（橙闪+音频），video_clk 域
wire        quake_ready;      // sd_card_clk 域：地震告警图(第5张,index4)已载入 REGION_4
wire        fire_ready;       // sd_card_clk 域：火灾告警图(第6张,index5)已载入 REGION_5

// 火灾告警输出（fire_alarm 在 video_clk 域，红色频闪，与地震告警同模式切图）
wire [23:0] fire_rgb;
wire        fire_hs;
wire        fire_vs;
wire        fire_de;
wire        fire_alarm;       // 火灾告警原始电平（驱动切图 mux / OSD 隐藏 / 按键冻结）
wire        fire_alert;       // 火灾告警延时对齐电平（驱动红闪 + 音频）

// 音频流播：TF 卡音频 → 异步 FIFO → 播放器 跨时钟域连线
wire        audio_fifo_we;       // sd_card_clk 域：流播器写使能
wire [15:0] audio_fifo_data;     // sd_card_clk 域：流播器写入 16bit 样本
wire [9:0]  audio_fifo_wrusedw;  // sd_card_clk 域：FIFO 写域使用量（流播器门限）
wire [15:0] audio_stream_dout;   // audio_mclk 域：FIFO 队首 16bit 样本
wire        audio_stream_valid;  // audio_mclk 域：FIFO 非空（SHOW_AHEAD）
wire        audio_stream_re;     // audio_mclk 域：播放器读使能（弹出队首）

// ============================================================================
// 复位生成：PLL 双锁 + 稳定 20ms 后释放，释放后忽略 PLL 抖动
// ============================================================================
always @(posedge clk) begin
    if (!rst_n) begin
        rst_cnt <= 20'd0;
        rst_all_reg <= 1'b1;
    end else if (!rst_all_reg) begin
        // 一旦复位释放，忽略后续微小的 PLL 抖动，防止系统在使用时随机复位
        rst_all_reg <= 1'b0;
    end else if (sys_pll_lock && video_pll_lock) begin
        // 等待 PLL 连续稳定 20ms @ 50MHz
        if (rst_cnt < 20'd1_000_000) begin
            rst_cnt <= rst_cnt + 1'b1;
            rst_all_reg <= 1'b1;
        end else begin
            rst_all_reg <= 1'b0;
        end
    end else begin
        rst_cnt <= 20'd0;
        rst_all_reg <= 1'b1;
    end
end

assign rst_all = rst_all_reg;

// 音频链路独立复位：在图像复位释放的基础上，额外等待音频 PLL 锁定。
// 若音频 PLL 因故未锁，音频链路保持复位，但不能拖累图像主链路卡死在复位。
wire rst_audio;
assign rst_audio = rst_all | ~audio_pll_lock;

// ============================================================================
// 时钟生成：sys_pll 和 video_pll
// ============================================================================
sys_pll sys_pll_m0(
    .refclk     (clk),
    .reset      (1'b0),
    .extlock    (sys_pll_lock),
    .clk0_out   (sd_card_clk),
    .clk1_out   (ext_mem_clk),
    .clk2_out   (ext_mem_clk_sft)
);

video_pll video_pll_m0(
    .refclk     (clk),
    .reset      (1'b0),
    .extlock    (video_pll_lock),
    .clk0_out   (video_clk),
    .clk1_out   (hdmi_5x_clk)
);

// 音频主时钟 PLL：只取 12.288MHz audio_mclk，供 I2S 测试音使用
PLL_HDMI_AUDIO u_audio_pll(
    .refclk     (clk),
    .reset      (1'b0),
    .extlock    (audio_pll_lock),
    .clk0_out   (),
    .clk1_out   (),
    .clk2_out   (audio_mclk)
);

// ============================================================================
// TF 卡多图扫描与缓存（双缓冲）
// ============================================================================
// mem_clk 域把 write_finish 单拍转成 toggle，供 sd_card_clk 域可靠同步
always @(posedge ext_mem_clk or posedge rst_all) begin
    if (rst_all)
        frame_write_toggle_mem <= 1'b0;
    else if (frame_write_finish)
        frame_write_toggle_mem <= ~frame_write_toggle_mem;
end

sd_card_bmp #(
    .CLK_FREQ_HZ       (100_000_000),
    .SCAN_START_SECTOR (32'd0),
    .SCAN_MAX_SECTOR   (32'd131071),
    .SCAN_TARGET_COUNT (3'd6)
) sd_card_bmp_m0(
    .clk               (sd_card_clk),
    .rst               (rst_all),
    .key_next          (key1),
    .key_auto          (key2),
    .key_prev          (key3),
    .state_code        (state_code),
    .img_count         (img_count),
    .period_sel_out    (auto_period_sel),
    .auto_play_out     (auto_play_on),
    .bmp_width         (16'd640),
    .bmp_height        (16'd480),
    .display_valid     (display_valid),

    .current_image_index(current_image_index),
    .thumbnail_ready    (thumbnail_ready),
    .layout_mode        (layout_mode),
    .thumb_write_en     (thumb_write_en),
    .thumb_write_addr   (thumb_write_addr),
    .thumb_write_data   (thumb_write_data),
    .menu_enable        (menu_enable),
    .menu_cursor        (menu_cursor),
    .volume_level       (volume_level),
    .brightness_level   (brightness_level),
    .contrast_level     (contrast_level),
    .auto_play_status   (auto_play_status),
    .quake_ready        (quake_ready),
    .fire_ready         (fire_ready),
    .alarm_freeze       (alarm_freeze_s2),

    .write_finish_toggle(frame_write_toggle_mem),
    .commit_toggle      (commit_toggle),
    .wr_addr            (wr_addr),

    .write_req         (sd_card_write_req),
    .write_req_ack     (sd_card_write_req_ack),
    .write_en          (sd_card_write_en),
    .write_data        (sd_card_write_data),
    .SD_nCS            (sd_ncs),
    .SD_DCLK           (sd_dclk),
    .SD_MOSI           (sd_mosi),
    .SD_MISO           (sd_miso),

    .audio_fifo_we     (audio_fifo_we),
    .audio_fifo_data   (audio_fifo_data),
    .audio_fifo_wrusedw(audio_fifo_wrusedw)
);

// ============================================================================
// 数码管显示：6 个数码管分别显示项目核心运行参数（左→右）
//   0=当前图片编号 1=图片总数 2=轮播周期档位 3=缩略图布局 4=音量 5=亮度
// ============================================================================
seg_decoder seg_curimg_m0(
    .bin_data          ({1'b0, current_image_index}),
    .seg_data          (seg_data_curimg)
);

seg_decoder seg_imgcnt_m0(
    .bin_data          ({1'b0, img_count}),
    .seg_data          (seg_data_imgcnt)
);

seg_decoder seg_period_m0(
    .bin_data          ({2'b00, auto_period_sel}),
    .seg_data          (seg_data_period)
);

seg_decoder seg_layout_m0(
    .bin_data          ({2'b00, layout_mode}),
    .seg_data          (seg_data_layout)
);

seg_decoder seg_vol_m0(
    .bin_data          (volume_level),
    .seg_data          (seg_data_vol)
);

seg_decoder seg_bright_m0(
    .bin_data          (brightness_level),
    .seg_data          (seg_data_bright)
);

assign fx_sel = {1'b0, sw[3]};                        // SW4 : 0=淡入淡出, 1=滑动

seg_scan seg_scan_m0(
    .clk               (clk),
    .rst_n             (rst_n),
    .seg_sel           (seg_sel),
    .seg_data          (seg_data),
    .seg_data_0        ({1'b1, seg_data_curimg}),
    .seg_data_1        ({1'b1, seg_data_imgcnt}),
    .seg_data_2        ({1'b1, seg_data_period}),
    .seg_data_3        ({1'b1, seg_data_layout}),
    .seg_data_4        ({1'b1, seg_data_vol}),
    .seg_data_5        ({1'b1, seg_data_bright})
);

// ============================================================================
// 视频时序生成与帧缓存读取
// ============================================================================
video_timing_data video_timing_data_m0(
    .video_clk         (video_clk),
    .rst               (rst_all),
    .read_req          (video_read_req),
    .read_req_ack      (video_read_req_ack),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0)
);

frame_read_write_dual #(
    .MEM_DATA_BITS (32),
    .ADDR_BITS     (21),
    .BURST_BITS    (9),
    .BURST_SIZE    (256),
    .FIFO_DEPTH    (512),
    .WRITE_V_FLIP  (1),
    .FRAME_WIDTH   (640),
    .FRAME_HEIGHT  (480)
) u_frame_rw_dual (
    .I_rst            (rst_all),
    .I_mem_clk        (ext_mem_clk),
    .I_sdr_init_done  (Sdr_init_done),
    .I_sdr_init_ref_vld(Sdr_init_ref_vld),
    .I_sdr_busy       (Sdr_busy),

    .O_app_rd_en      (App_rd_en),
    .O_app_rd_addr    (App_rd_addr),
    .I_sdr_rd_en      (Sdr_rd_en),
    .I_sdr_rd_dout    (Sdr_rd_dout),

    .O_app_wr_en      (App_wr_en),
    .O_app_wr_addr    (App_wr_addr),
    .O_app_wr_din     (App_wr_din),
    .O_app_wr_dm      (App_wr_dm),

    .I_read_clk       (video_clk),
    .I_read_req       (video_read_req),
    .O_read_req_ack   (video_read_req_ack),
    .O_read_finish    (),
    .I_read_len       (FRAME_PIXELS[20:0]),
    .I_rd_addr_a      (rd_addr_a_eff),
    .I_rd_addr_b      (rd_addr_b_eff),
    .I_dual_en        (dual_en_eff),
    .I_cfg_toggle     (cfg_toggle_eff),
    .I_read_en        (video_read_en),
    .O_rd_data_a      (rd_data_a),
    .O_rd_data_b      (rd_data_b),
    .O_empty_a        (empty_a),
    .O_empty_b        (empty_b),

    .I_write_clk      (sd_card_clk),
    .I_write_req      (sd_card_write_req),
    .O_write_req_ack  (sd_card_write_req_ack),
    .O_write_finish   (frame_write_finish),
    .I_wr_addr        (wr_addr),
    .I_write_len      (FRAME_PIXELS[20:0]),
    .I_write_en       (sd_card_write_en),
    .I_write_data     (sd_card_write_data)
);

sdram U3(
    .Clk               (ext_mem_clk),
    .Clk_sft           (ext_mem_clk_sft),
    .Rst               (rst_all),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),
    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_dm         (App_wr_dm),
    .App_wr_din        (App_wr_din),
    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout)
);

// ============================================================================
// [改动 1] 视频延迟：A/B 双路（替换基线的 video_delay video_delay_m0）
// ============================================================================
// A/B 两路参数与结构完全相同（均 STAGES=21），保证 A/B 像素严格对齐。
// A 路数据源：frame_read_write_dual 的 rd_data_a（区域 A 像素）
// B 路数据源：frame_read_write_dual 的 rd_data_b（区域 B 像素）
//
// A 路：当前显示主通道
video_delay_640 #(
    .DATA_WIDTH (24),
    .STAGES     (21)
) u_delay_a (
    .I_clk      (video_clk),
    .I_rst      (rst_all),
    .O_read_en  (video_read_en),
    .I_data     (rd_data_a[31:8]),
    .I_hs       (hs_0),
    .I_vs       (vs_0),
    .I_de       (de_0),
    .O_hs       (hs_a),
    .O_vs       (vs_a),
    .O_de       (de_a),
    .O_data     (img_a)
);

// B 路：转场新图通道（与 A 路参数完全一致，STAGES=21）
video_delay_640 #(
    .DATA_WIDTH (24),
    .STAGES     (21)
) u_delay_b (
    .I_clk      (video_clk),
    .I_rst      (rst_all),
    .O_read_en  (),
    .I_data     (rd_data_b[31:8]),
    .I_hs       (hs_0),
    .I_vs       (vs_0),
    .I_de       (de_0),
    .O_hs       (hs_b),
    .O_vs       (vs_b),
    .O_de       (de_b),
    .O_data     (img_b)
);

// ============================================================================
// [改动 2] 转场混合器（支持 fade / slide-right / slide-left 三种模式）
// ============================================================================
// transition_mixer 内部例化 alpha_mixer，根据 I_mode 和 I_off 自动选择：
//   Mode 0 / I_dual_en=0 : bit-exact 直通 A 路（无转场）
//   Mode 1                : 全屏 alpha 混合（fade）
//   Mode 2                : B 路从右侧滑入（slide-right）
//   Mode 3                : B 路从左侧滑入（slide-left）
transition_mixer #(
    .DATA_WIDTH (24),
    .H_ACTIVE   (640)
) u_mix_transition (
    .I_pix_clk  (video_clk),
    .I_rst      (rst_all),
    .I_a        (img_a),
    .I_b        (img_b),
    .I_hs       (hs_a),
    .I_vs       (vs_a),
    .I_de       (de_a),
    .I_dual_en  (dual_en_eff),
    .I_alpha    (trans_alpha),
    .I_mode     (trans_mode),
    .I_off      (trans_off),
    .O_y        (img_blend),
    .O_hs       (hs_m),
    .O_vs       (vs_m),
    .O_de       (de_m)
);

// ============================================================================
// [改动 3] 转场控制器（三区域轮转 + 转场系数生成 + CDC 同步）
// ============================================================================
// transition_ctrl 负责：
//   1. 三区域轮转：保证 loader 写入区域始终不同于 A/B 读取区域
//   2. 转场系数生成：每帧更新 alpha（fade）和 off（slide 位移）
//   3. CDC 同步：检测 sd_card_bmp 的 display_valid / disp_buf_idx 提交事件
//   4. 帧边界锁定：所有影响显示的输出仅在 vs 下降沿更新，防止撕裂
transition_ctrl #(
    .ADDR_BITS     (21),
    .REGION_0_ADDR (21'd0),
    .REGION_1_ADDR (21'd307200),
    .REGION_2_ADDR (21'd614400),
    .REGION_3_ADDR (21'd921600),
    .REGION_4_ADDR (21'd1228800),
    .REGION_5_ADDR (21'd1536000),
    .N_FRAMES      (32),
    .H_ACTIVE      (640)
) u_transition_ctrl (
    .I_pix_clk        (video_clk),
    .I_rst            (rst_all),
    .I_vs             (vs_0),
    .I_commit_toggle  (commit_toggle),
    .I_img_idx        (current_image_index),
    .I_fx_sel         (fx_sel),
    .I_force_quake    (1'b0),
    .O_dual_en        (dual_en),
    .O_alpha          (trans_alpha),
    .O_mode           (trans_mode),
    .O_off            (trans_off),
    .O_transition_busy(),
    .O_rd_addr_a      (rd_addr_a),
    .O_rd_addr_b      (rd_addr_b),
    .O_cfg_toggle     (trans_cfg_toggle)
);

// ============================================================================
// CDC 同步：sd_card_clk(50MHz) 域控制信号 → video_clk(25MHz) 域（2 级 DFF 消亚稳态）
// 这些多比特信号原为 sd_card_bmp 输出、直接被各叠加层采样，未做跨时钟域同步，
// 亚稳态采样会导致叠加层偶发错位像素。此处统一打 2 拍，供下游 OSD/缩略图/菜单使用。
// ============================================================================
reg  [3:0] brightness_s1, brightness_s2;
reg  [3:0] contrast_s1,   contrast_s2;
reg  [2:0] menu_cursor_s1, menu_cursor_s2;
reg        menu_enable_s1, menu_enable_s2;
reg  [1:0] auto_period_s1, auto_period_s2;
reg  [3:0] volume_s1,      volume_s2;
reg        auto_status_s1, auto_status_s2;
reg  [1:0] layout_mode_s1, layout_mode_s2;
reg  [2:0] image_index_s1, image_index_s2;
reg        thumb_ready_s1, thumb_ready_s2;
reg        quake_ready_s1, quake_ready_s2;

always @(posedge video_clk) begin
    if (rst_all) begin
        brightness_s2  <= 4'd15;   // 满亮度（直通）
        contrast_s2    <= 4'd15;   // 中性（直通）
        menu_cursor_s2 <= 3'd0;
        menu_enable_s2 <= 1'b0;
        auto_period_s2 <= 2'd0;
        volume_s2      <= 4'd0;
        auto_status_s2 <= 1'b0;
        layout_mode_s2 <= 2'd0;
        image_index_s2 <= 3'd0;
        thumb_ready_s2 <= 1'b0;
        quake_ready_s2 <= 1'b0;
    end else begin
        brightness_s1  <= brightness_level;
        contrast_s1    <= contrast_level;
        menu_cursor_s1 <= menu_cursor;
        menu_enable_s1 <= menu_enable;
        auto_period_s1 <= auto_period_sel;
        volume_s1      <= volume_level;
        auto_status_s1 <= auto_play_status;
        layout_mode_s1 <= layout_mode;
        image_index_s1 <= current_image_index;
        thumb_ready_s1 <= thumbnail_ready;
        quake_ready_s1 <= quake_ready;

        brightness_s2  <= brightness_s1;
        contrast_s2    <= contrast_s1;
        menu_cursor_s2 <= menu_cursor_s1;
        menu_enable_s2 <= menu_enable_s1;
        auto_period_s2 <= auto_period_s1;
        volume_s2      <= volume_s1;
        auto_status_s2 <= auto_status_s1;
        layout_mode_s2 <= layout_mode_s1;
        image_index_s2 <= image_index_s1;
        thumb_ready_s2 <= thumb_ready_s1;
        quake_ready_s2 <= quake_ready_s1;
    end
end

// 告警电平跨时钟域：video_clk → sd_card_clk（冻结按键/自动轮播）。
// 告警期间的叠加层隐藏 / 强制切图控制均在 video_clk 域。
reg alarm_freeze_s1, alarm_freeze_s2;
always @(posedge sd_card_clk) begin
    if (rst_all) begin
        alarm_freeze_s1 <= 1'b0;
        alarm_freeze_s2 <= 1'b0;
    end else begin
        alarm_freeze_s1 <= quake_alarm | fire_alarm;
        alarm_freeze_s2 <= alarm_freeze_s1;
    end
end

// ============================================================================
// 告警切图（旁路转场引擎）：quake_alarm / fire_alarm 直接 mux 读地址到对应
// 告警图区域，上升/下降沿各翻转一次独立 cfg_toggle，与正常转场 cfg_toggle
// 异或合并，驱动读引擎重新锁存地址。告警解除后自动回归转场引擎输出的正常地址。
// 火灾告警优先于地震告警（两者同时触发时显示火灾告警图）。
// ============================================================================
localparam [20:0] QUAKE_REGION = 21'd1228800;  // 第5张图（index 4）地震告警图
localparam [20:0] FIRE_REGION  = 21'd1536000;  // 第6张图（index 5）火灾告警图

reg        qk_alarm_d;
reg        qk_cfg_toggle;
reg        fire_alarm_d;
reg        fire_cfg_toggle;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        qk_alarm_d      <= 1'b0;
        qk_cfg_toggle   <= 1'b0;
        fire_alarm_d    <= 1'b0;
        fire_cfg_toggle <= 1'b0;
    end else begin
        qk_alarm_d   <= quake_alarm;
        fire_alarm_d <= fire_alarm;
        if (quake_alarm != qk_alarm_d)
            qk_cfg_toggle <= ~qk_cfg_toggle;
        if (fire_alarm != fire_alarm_d)
            fire_cfg_toggle <= ~fire_cfg_toggle;
    end
end

wire alarm_disp_active = fire_alarm | quake_alarm;

assign rd_addr_a_eff  = fire_alarm  ? FIRE_REGION  :
                        quake_alarm ? QUAKE_REGION : rd_addr_a;
assign rd_addr_b_eff  = fire_alarm  ? FIRE_REGION  :
                        quake_alarm ? QUAKE_REGION : rd_addr_b;
assign dual_en_eff    = alarm_disp_active ? 1'b0 : dual_en;
assign cfg_toggle_eff = trans_cfg_toggle ^ qk_cfg_toggle ^ fire_cfg_toggle;

wire [1:0]  layout_mode_eff = alarm_disp_active ? 2'd0 : layout_mode_s2;
wire        wave_show       = sw[1] & ~alarm_disp_active;
wire        menu_enable_eff = menu_enable_s2 & ~alarm_disp_active;

// 临时诊断：锁存告警触发 quake_alarm，触发一次即恒为 1（直到复位），
// 消除 3 秒告警窗口内肉眼读数码管的时序歧义。定位后请删除。
reg quake_sticky;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        quake_sticky <= 1'b0;
    else if (quake_alarm)
        quake_sticky <= 1'b1;
end

// ============================================================================
// [改动 4] OSD 叠加引擎
// ============================================================================
// OSD 输入：首图提交前黑屏（基线的 display_valid 门控），提交后显示转场混合器输出
assign osd_img_in = display_valid ? img_blend : 24'd0;

osd_overlay #(
    .H_ACTIVE     (640),
    .V_ACTIVE     (480),
    .DATA_WIDTH   (24),
    .OSD_PIPE     (4),
    .BANNER_Y0    (56),
    .BANNER_W     (2048),
    .SCROLL_STEP  (2),
    .TICK_DIV     (25000000),
    .TEXT_UNITS   (128),
    .LOGO_X0      (496),
    .LOGO_Y0      (24),
    .LOGO_W       (128),
    .LOGO_H       (80),
    .TS_X0        (16),
    .TS_Y0        (434),
    .COL_TXT      (24'h000000),
    .COL_TS       (24'hFFFFFF),
    .COL_LOGO     (24'h1BA0E2)
) u_osd_overlay (
    .I_pix_clk    (video_clk),
    .I_rst        (rst_all),
    .I_img        (osd_img_in),
    .I_hs         (hs_m),
    .I_vs         (vs_m),
    .I_de         (de_m),
    .I_alpha_txt  (8'hFF),
    .I_alpha_logo (8'hFF),
    .I_bright     (brightness_s2),
    .I_contrast   (contrast_s2),
    .I_hide       (alarm_disp_active),
    .O_rgb        (osd_rgb),
    .O_hs         (osd_hs),
    .O_vs         (osd_vs),
    .O_de         (osd_de),
    .O_layer_hit  (osd_layer_hit)
);

// ============================================================================
// 缩略图预览与 2x2 布局（OSD 输出后叠加；key1 长按切换 单图/底部条/2x2）
// ============================================================================
thumbnail_preview_layout u_thumbnail_preview_layout (
    .I_wr_clk      (sd_card_clk),
    .I_wr_en       (thumb_write_en),
    .I_wr_addr     (thumb_write_addr),
    .I_wr_data     (thumb_write_data),
    .I_pix_clk     (video_clk),
    .I_rst         (rst_all),
    .I_thumb_ready (thumb_ready_s2),
    .I_layout_mode (layout_mode_eff),
    .I_selected    (image_index_s2),
    .I_rgb         (osd_rgb),
    .I_hs          (osd_hs),
    .I_vs          (osd_vs),
    .I_de          (osd_de),
    .O_rgb         (preview_rgb),
    .O_hs          (preview_hs),
    .O_vs          (preview_vs),
    .O_de          (preview_de)
);

// ============================================================================
// 音频 FFT 频谱叠加（sw[1] 拨上显示、拨下隐藏）
// ============================================================================
// 取数点：I2S_receiver 输出的左声道数据（已在 video_clk 域），取其高 8 位
// 有效样本（即 8-bit 有符号源 PCM 数据）。频谱叠加在缩略图预览之后。
audio_spectrum u_audio_spectrum (
    .I_clk         (video_clk),
    .I_rst         (rst_all),
    .I_show        (wave_show),
    .I_audio_valid (audio_valid),
    .I_audio_sample(audio_left_data[22:15]),
    .I_rgb         (preview_rgb),
    .I_hs          (preview_hs),
    .I_vs          (preview_vs),
    .I_de          (preview_de),
    .O_rgb         (wave_rgb),
    .O_hs          (wave_hs),
    .O_vs          (wave_vs),
    .O_de          (wave_de)
);

// ============================================================================
// 菜单叠加 + 统一亮度/对比度缩放（最上层，绘制在波形之上）
// ============================================================================
menu_brightness_overlay u_menu_brightness_overlay (
    .I_clk         (video_clk),
    .I_rst         (rst_all),
    .I_rgb         (wave_rgb),
    .I_hs          (wave_hs),
    .I_vs          (wave_vs),
    .I_de          (wave_de),
    .I_menu_enable (menu_enable_eff),
    .I_cursor      (menu_cursor_s2),
    .I_period      (auto_period_s2),
    .I_volume      (volume_s2),
    .I_brightness  (brightness_s2),
    .I_contrast    (contrast_s2),
    .I_auto_on     (auto_status_s2),
    .O_rgb         (menu_rgb),
    .O_hs          (menu_hs),
    .O_vs          (menu_vs),
    .O_de          (menu_de)
);

// ============================================================================
// 地震告警：震动触发检测 + 全屏红/橙频闪（在菜单叠加之后、AXIS 编码之前）
// ============================================================================
quake_alarm #(
    .CLK_MHZ     (25)
) u_quake_alarm (
    .clk         (video_clk),
    .rst         (rst_all),
    .quake_in    (quake_in),
    .enable      (sw[2]),
    .rgb_i       (menu_rgb),
    .hs_i        (menu_hs),
    .vs_i        (menu_vs),
    .de_i        (menu_de),
    .rgb_o       (alarm_rgb),
    .hs_o        (alarm_hs),
    .vs_o        (alarm_vs),
    .de_o        (alarm_de),
    .alarm_o     (quake_alarm),
    .alert_o     (alert_disp)
);

// ============================================================================
// 火灾告警：火焰触发检测 + 告警切图 + 红色频闪 + 警报（与地震告警同模式）
// ============================================================================
fire_alarm #(
    .CLK_MHZ     (25),
    .ACTIVE_HIGH (0)
) u_fire_alarm (
    .clk         (video_clk),
    .rst         (rst_all),
    .fire_in     (fire_in),
    .enable      (sw[0]),
    .rgb_i       (alarm_rgb),
    .hs_i        (alarm_hs),
    .vs_i        (alarm_vs),
    .de_i        (alarm_de),
    .rgb_o       (fire_rgb),
    .hs_o        (fire_hs),
    .vs_o        (fire_vs),
    .de_o        (fire_de),
    .alarm_o     (fire_alarm),
    .alert_o     (fire_alert)
);

// ============================================================================
// [改动 4] RGB/DE 转 AXIS 视频（输入源改为火灾告警频闪输出，含地震与火灾两级叠加）
// ============================================================================
video_rgb_to_axis_640x480 u_video_rgb_to_axis_640x480(
    .I_clk         (video_clk),
    .I_rst         (rst_all),
    .I_vs          (fire_vs),
    .I_de          (fire_de),
    .I_rgb         (fire_rgb),
    .O_video_user  (axis_s_user),
    .O_video_valid (axis_s_valid),
    .O_video_last  (axis_s_last),
    .O_video_data  (axis_s_data)
);

// ============================================================================
// 音频链路：图片切换联动 PCM 语音 → I2S 接收 → ARC 计算 → HDMI 发射核
// ============================================================================
// TF 卡音频跨时钟域缓冲：写侧 sd_card_clk（audio_tf_stream 填充），读侧 audio_mclk（播放器读取）
audio_async_fifo u_audio_async_fifo (
    .rst       (rst_all),
    .clkw      (sd_card_clk),
    .clkr      (audio_mclk),
    .we        (audio_fifo_we),
    .di        (audio_fifo_data),
    .re        (audio_stream_re),
    .dout      (audio_stream_dout),
    .valid     (audio_stream_valid),
    .full_flag (),
    .empty_flag(),
    .wrusedw   (audio_fifo_wrusedw),
    .rdusedw   ()
);

image_linked_audio_pcm_i2s u_image_linked_audio_pcm_i2s (
    .I_mclk         (audio_mclk),
    .I_rst          (rst_audio),
    .I_image_valid  (display_valid),
    .I_image_index  (current_image_index),
    .I_volume_level (volume_level),
    .I_alarm        (alert_disp | fire_alert),
    .I_stream_dout  (audio_stream_dout),
    .I_stream_valid (audio_stream_valid),
    .O_stream_re    (audio_stream_re),
    .O_i2s_BCLK     (audio_i2s_bclk),
    .O_i2s_LRCK     (audio_i2s_lrck),
    .O_i2s_DOUT     (audio_i2s_dout)
);

I2S_receiver u_I2S_receiver(
    .I_clk              (video_clk),
    .I_rst              (rst_audio),
    .I_i2s_BCLK         (audio_i2s_bclk),
    .I_i2s_LRCK         (audio_i2s_lrck),
    .I_i2s_DOUT         (audio_i2s_dout),
    .O_audio_valid      (audio_valid),
    .O_audio_left_data  (audio_left_data),
    .O_audio_right_data (audio_right_data)
);

audio_arc_calculate #(
    .ACR_N         (6144)
) u_audio_arc_calculate (
    .I_clk         (video_clk),
    .I_rst         (rst_audio),
    .I_audio_valid (audio_valid),
    .O_acr_valid   (acr_valid),
    .O_acr_cts     (acr_cts),
    .O_acr_n       (acr_n)
);

// ============================================================================
// HDMI 1.4b 发射
// ============================================================================
// 上电后自动打一拍，触发一次 EDID 读取
startup_pulse #(
    .CNT_MAX(20'd100000)
) u_startup_pulse (
    .I_clk   (video_clk),
    .I_rst   (rst_all),
    .O_pulse (edid_trig)
);

hdmi_1_4b_transmitter_core_wrapper #(
    .DEVICE                 ( "EG"       ),
    .HTOTAL                 ( 800        ),
    .HSA                    ( 96         ),
    .HFP                    ( 16         ),
    .HBP                    ( 48         ),
    .HACTIVE                ( 640        ),
    .VTOTAL                 ( 525        ),
    .VSA                    ( 2          ),
    .VFP                    ( 10         ),
    .VBP                    ( 33         ),
    .VACTIVE                ( 480        ),
    .VIDEO_VIC              ( 1          ),
    .VIDEO_TPG              ( "Disable"  ),
    .VIDEO_FORMAT           ( "RGB"      ),
    .AUDIO_SAMPLE_RATE      ( "48K"      ),
    .IIC_SCL_DIV            ( 250        )
) u_hdmi_1_4b_transmitter_core_wrapper(
    .I_pixel_clk        (video_clk),
    .I_rst              (rst_all),
    .I_edid_read_trig   (edid_trig),
    .O_edid_read_valid  (edid_valid),
    .O_edid_read_data   (edid_data),

    .I_axis_s_user      (axis_s_user),
    .I_axis_s_valid     (axis_s_valid),
    .I_axis_s_last      (axis_s_last),
    .I_axis_s_data      (axis_s_data),
    .O_axis_s_ready     (axis_s_ready),

    .I_audio_valid      (audio_valid),
    .I_audio_left_data  (audio_left_data),
    .I_audio_right_data (audio_right_data),
    .I_acr_valid        (acr_valid),
    .I_acr_cts          (acr_cts),
    .I_acr_n            (acr_n),

    .O_video_locked     (),
    .O_ddc_scl          (HDMI_DDC_SCL),
    .IO_ddc_sda         (HDMI_DDC_SDA),

    .O_ch0_tmds_data    (tmds_ch0_data),
    .O_ch1_tmds_data    (tmds_ch1_data),
    .O_ch2_tmds_data    (tmds_ch2_data),
    .O_clk_tmds_data    (tmds_clk_data)
);

hdmi_phy_wrapper #(
    .DEVICE ( "EG" )
) u_hdmi2phy_wrapper(
    .I_pixel_clk        (video_clk),
    .I_serial_clk       (hdmi_5x_clk),
    .I_rst              (rst_all),
    .I_tmds_channel_0   (tmds_ch0_data),
    .I_tmds_channel_1   (tmds_ch1_data),
    .I_tmds_channel_2   (tmds_ch2_data),
    .I_tmds_channel_clk (tmds_clk_data),
    .O_tmds_ch0_p       (HDMI_D0_P),
    .O_tmds_ch1_p       (HDMI_D1_P),
    .O_tmds_ch2_p       (HDMI_D2_P),
    .O_tmds_clk_p       (HDMI_CLK_P)
);

endmodule

// ============================================================================
// quake_alarm —— 地震告警：震动触发检测 + 告警电平输出
// ----------------------------------------------------------------------------
// 内联在本文件末尾（与 top 同文件），避免额外源文件在 TD 工程管理中被遗漏。
// 时钟域：clk（video_clk 25MHz）；quake_in 为异步输入，模块内 2 级同步消亚稳态。
// 复位：rst 高有效（与 rst_all 一致）。
//   1. 震动检测：100ms 窗口内 >=3 个上升沿脉冲 -> 触发告警
//   2. 告警保持与解除：触发后锁存告警，不会自动停止，
//      仅当 enable=0（拨码开关下拉）时手动关停解除
//   3. 视频透传 + 告警时在告警图上叠橙色呼吸闪烁（源已由顶层 mux 切到 REGION_4 地震告警图）
//   4. 黑屏对齐：alarm 触发/解除后，视频先黑屏 ALIGN_FRAMES 帧，等告警图经 SDRAM
//      读引擎切换稳定后，再同时点亮告警图 + 橙闪 + 音频，保证三者同现同消
//   5. 输出 alarm_o（原始告警，驱动瞬间切图 / OSD 隐藏 / 按键冻结）与
//      alert_o（延时对齐后的显示告警，驱动橙闪 / 音频）
// 语言标准：Verilog-2001
// ============================================================================
module quake_alarm #(
    parameter CLK_MHZ     = 25,          // clk 频率（MHz）
    parameter WIN_MS      = 100,         // 计数窗口 100ms
    parameter PULSE_MIN   = 3,           // 窗口内 >=3 个脉冲触发
    parameter CYCLE_MS    = 1000,        // 满周期（最低->最高->最低）时长 ms
    parameter AMIN_ALPHA  = 60,          // 最低不透明度（>0，保证仍有橙色叠层且图像可见）
    parameter AMAX_ALPHA  = 220,         // 最高不透明度（<255，保证下方图像仍可见）
    parameter FLASH_COLOR = 24'hFF8000,  // 叠层颜色（橙色）
    parameter ALIGN_FRAMES = 3           // 告警图切换稳定所需帧数（黑屏对齐窗口）
)(
    input         clk, rst,
    input         quake_in,              // SW-420 数字输出（异步）
    input         enable,                // 拨码开关使能：1=启用告警，0=禁用并立即解除
    input  [23:0] rgb_i,                 // 上游像素（顶层已 mux 到地震告警图 REGION_4）
    input         hs_i, vs_i, de_i,
    output [23:0] rgb_o,
    output        hs_o, vs_o, de_o,
    output        alarm_o,               // 原始告警电平（驱动瞬间切图 / OSD 隐藏 / 按键冻结）
    output        alert_o                // 延时对齐后的告警电平（驱动橙闪 / 音频）
);

  // ---- 计时上限 ----
  localparam integer WIN_CNT    = WIN_MS     * CLK_MHZ * 1000;   // 2.5M @25MHz
  localparam integer CYCLE_CLKS = CYCLE_MS   * CLK_MHZ * 1000;   // 满周期时钟数
  localparam integer DELTA      = AMAX_ALPHA - AMIN_ALPHA;        // 不透明度行程
  localparam integer RAMP_STEP  = CYCLE_CLKS / (2 * DELTA);       // 每 1 LSB 所需时钟数

  // 叠层 RGB 分量
  localparam [7:0] FLASH_R = FLASH_COLOR[23:16];
  localparam [7:0] FLASH_G = FLASH_COLOR[15:8];
  localparam [7:0] FLASH_B = FLASH_COLOR[7:0];

  // ---- 异步输入 2 级同步 + 上升沿 ----
  reg q1, q2, q2_d;
  always @(posedge clk or posedge rst) begin
    if (rst) {q2, q1} <= 2'b00;
    else     {q2, q1} <= {q1, quake_in};
  end
  always @(posedge clk or posedge rst) begin
    if (rst) q2_d <= 1'b0;
    else     q2_d <= q2;
  end
  wire rise = q2 & ~q2_d;

  // ---- 100ms 窗口脉冲计数 ----
  reg [31:0] win_cnt;
  reg [1:0]  pulse_cnt;
  reg        det;
  wire       win_end = (win_cnt == WIN_CNT - 1'b1);

  always @(posedge clk or posedge rst) begin
    if (rst) begin
      win_cnt   <= 32'd0;
      pulse_cnt <= 2'd0;
      det       <= 1'b0;
    end else begin
      det <= 1'b0;
      if (win_end) begin
        det       <= (pulse_cnt >= PULSE_MIN[1:0]);
        pulse_cnt <= 2'd0;
        win_cnt   <= 32'd0;
      end else begin
        win_cnt <= win_cnt + 1'b1;
        if (rise && (pulse_cnt < PULSE_MIN[1:0]))
          pulse_cnt <= pulse_cnt + 1'b1;
      end
    end
  end

  // ---- 告警状态：触发置位并锁存，不会自动解除，仅 enable=0（拨码开关下拉）手动关停 ----
  reg        alarm;
  always @(posedge clk or posedge rst) begin
    if (rst) begin
      alarm   <= 1'b0;
    end else if (!enable) begin
      // 拨码开关禁用（下拉）：手动解除告警
      alarm   <= 1'b0;
    end else if (det) begin
      // 触发置位，之后锁存，直到拨码开关下拉才会清零
      alarm   <= 1'b1;
    end
  end

  // ---- 不透明度 alpha：线性三角波（AMIN_ALPHA <-> AMAX_ALPHA），每步仅 ±1 LSB ----
  reg [7:0]  alpha;
  reg        ramp_up;
  reg [31:0] ramp_cnt;
  always @(posedge clk or posedge rst) begin
    if (rst) begin
      alpha    <= AMIN_ALPHA[7:0];
      ramp_up  <= 1'b1;
      ramp_cnt <= 32'd0;
    end else begin
      if (ramp_cnt >= RAMP_STEP - 1'b1) begin
        ramp_cnt <= 32'd0;
        if (ramp_up) begin
          if (alpha >= AMAX_ALPHA[7:0]) begin
            ramp_up <= 1'b0;
            alpha   <= alpha - 1'b1;
          end else begin
            alpha   <= alpha + 1'b1;
          end
        end else begin
          if (alpha <= AMIN_ALPHA[7:0]) begin
            ramp_up <= 1'b1;
            alpha   <= alpha + 1'b1;
          end else begin
            alpha   <= alpha - 1'b1;
          end
        end
      end else begin
        ramp_cnt <= ramp_cnt + 1'b1;
      end
    end
  end

  // ---- alpha 混合：叠层橙色 * alpha + 原图（告警图） * (255-alpha)，右移 8 位 ----
  wire [7:0]  ia     = 8'd255 - alpha;
  wire [15:0] mix_rf = FLASH_R * alpha;
  wire [15:0] mix_ri = rgb_i[23:16] * ia;
  wire [15:0] mix_gf = FLASH_G * alpha;
  wire [15:0] mix_gi = rgb_i[15:8]  * ia;
  wire [15:0] mix_bf = FLASH_B * alpha;
  wire [15:0] mix_bi = rgb_i[7:0]   * ia;
  wire [16:0] acc_r  = {1'b0, mix_rf} + {1'b0, mix_ri};
  wire [16:0] acc_g  = {1'b0, mix_gf} + {1'b0, mix_gi};
  wire [16:0] acc_b  = {1'b0, mix_bf} + {1'b0, mix_bi};
  wire [7:0]  r_bl   = acc_r[15:8];
  wire [7:0]  g_bl   = acc_g[15:8];
  wire [7:0]  b_bl   = acc_b[15:8];

  // ---- 黑屏对齐：alarm 经 ALIGN_FRAMES 帧延时得到 alert，alarm^alert 期间黑屏 ----
  reg vs_d1;
  always @(posedge clk or posedge rst) begin
    if (rst) vs_d1 <= 1'b0;
    else     vs_d1 <= vs_i;
  end
  wire frame_tick = vs_d1 & ~vs_i;   // vs 下降沿 = 帧起点

  reg [ALIGN_FRAMES:0] alert_pipe;
  always @(posedge clk or posedge rst) begin
    if (rst) alert_pipe <= {(ALIGN_FRAMES+1){1'b0}};
    else if (frame_tick) alert_pipe <= {alert_pipe[ALIGN_FRAMES-1:0], alarm};
  end
  wire alert = alert_pipe[ALIGN_FRAMES];
  wire black = alarm ^ alert;

  // ---- 视频：黑屏期输出 0；否则告警对齐后叠橙色呼吸闪烁，非告警透传 ----
  assign hs_o = hs_i;
  assign vs_o = vs_i;
  assign de_o = de_i;
  assign rgb_o = black ? 24'd0 : ((de_i && alert) ? {r_bl, g_bl, b_bl} : rgb_i);

  assign alarm_o = alarm;
  assign alert_o = alert;

endmodule

// ============================================================================
// fire_alarm —— 火灾告警：火焰触发检测 + 告警切图 + 红色呼吸闪烁 + 警报
// ----------------------------------------------------------------------------
// 参考 quake_alarm，采用完全一样的"闪烁 + 切图 + 警报"模式：
//   1. 火焰检测：火灾是持续状态，检测采用"有效电平持续去抖"而非脉冲计数，
//      默认高电平有效（ACTIVE_HIGH=1）。当前显示图为火灾告警图（REGION_5）。
//   2. 告警保持与解除：触发后锁存告警，不会自动停止，
//      仅当 enable=0（拨码开关 sw[0] 下拉）时手动关停解除
//   3. 视频透传 + 告警时在告警图上叠红色呼吸闪烁（源已由顶层 mux 切到 REGION_5 火灾告警图）
//   4. 黑屏对齐：alarm 触发/解除后，视频先黑屏 ALIGN_FRAMES 帧，等告警图经 SDRAM
//      读引擎切换稳定后，再同时点亮告警图 + 红闪 + 音频，保证三者同现同消
//   5. 输出 alarm_o（原始告警，驱动切图 mux / OSD 隐藏 / 按键冻结）与
//      alert_o（延时对齐后的显示告警，驱动红闪 / 音频）
// 时钟域：clk（video_clk 25MHz）；fire_in 为异步输入，模块内 2 级同步。
// 语言标准：Verilog-2001
// ============================================================================
module fire_alarm #(
    parameter CLK_MHZ      = 25,          // clk 频率（MHz）
    parameter DEBOUNCE_MS  = 100,         // 有效电平持续去抖时长
    parameter CYCLE_MS     = 1000,        // 红色呼吸满周期（最低->最高->最低）
    parameter AMIN_ALPHA   = 60,          // 最低不透明度（>0，保证图像可见）
    parameter AMAX_ALPHA   = 220,         // 最高不透明度（<255，保证图像可见）
    parameter FLASH_COLOR  = 24'hFF0000,  // 叠层颜色（纯红）
    parameter ACTIVE_HIGH  = 1,           // 1=火警输出高电平有效，0=低电平有效
    parameter ALIGN_FRAMES = 3            // 告警图切换稳定所需帧数（黑屏对齐窗口）
)(
    input         clk, rst,
    input         fire_in,               // 火焰传感器数字输出（异步）
    input         enable,                // 拨码开关 sw[0]：1=使能，0=下拉手动关停
    input  [23:0] rgb_i,                 // 上游像素（顶层已 mux 到火灾告警图）
    input         hs_i, vs_i, de_i,
    output [23:0] rgb_o,
    output        hs_o, vs_o, de_o,
    output        alarm_o,               // 原始告警电平（驱动切图 mux / OSD 隐藏 / 按键冻结）
    output        alert_o                // 延时对齐后的告警电平（驱动红闪 / 音频）
);

  // ---- 计时上限 ----
  localparam integer DEBOUNCE_CNT = DEBOUNCE_MS * CLK_MHZ * 1000;  // 2.5M @25MHz
  localparam integer CYCLE_CLKS   = CYCLE_MS   * CLK_MHZ * 1000;   // 满周期时钟数
  localparam integer DELTA        = AMAX_ALPHA - AMIN_ALPHA;        // 不透明度行程
  localparam integer RAMP_STEP    = CYCLE_CLKS / (2 * DELTA);       // 每 1 LSB 所需时钟数

  // 叠层 RGB 分量
  localparam [7:0] FLASH_R = FLASH_COLOR[23:16];
  localparam [7:0] FLASH_G = FLASH_COLOR[15:8];
  localparam [7:0] FLASH_B = FLASH_COLOR[7:0];

  // ---- 异步输入 2 级同步 ----
  reg q1, q2;
  always @(posedge clk or posedge rst) begin
    if (rst) {q2, q1} <= 2'b00;
    else     {q2, q1} <= {q1, fire_in};
  end

  // ---- 有效电平持续去抖检测：火情是持续状态，等效于连续检测到火焰 ----
  wire fire_active = ACTIVE_HIGH ? q2 : ~q2;
  reg [31:0] deb_cnt;
  reg        det;
  always @(posedge clk or posedge rst) begin
    if (rst) begin
      deb_cnt <= 32'd0;
      det     <= 1'b0;
    end else begin
      det <= 1'b0;
      if (fire_active) begin
        if (deb_cnt >= DEBOUNCE_CNT - 1'b1) begin
          det     <= 1'b1;                  // 去抖饱和后保持触发
          deb_cnt <= DEBOUNCE_CNT - 1'b1;   // 饱和，避免翻转时溢出
        end else begin
          deb_cnt <= deb_cnt + 1'b1;
        end
      end else begin
        deb_cnt <= 32'd0;
      end
    end
  end

  // ---- 告警状态：触发置位并锁存，仅 enable=0（拨码下拉）手动关停 ----
  reg alarm;
  always @(posedge clk or posedge rst) begin
    if (rst) begin
      alarm <= 1'b0;
    end else if (!enable) begin
      alarm <= 1'b0;                       // 拨码开关下拉：手动解除告警
    end else if (det) begin
      alarm <= 1'b1;                       // 触发置位，之后锁存
    end
  end

  // ---- 红色呼吸 alpha 三角波（AMIN_ALPHA <-> AMAX_ALPHA），每步仅 ±1 LSB ----
  reg [7:0]  alpha;
  reg        ramp_up;
  reg [31:0] ramp_cnt;
  always @(posedge clk or posedge rst) begin
    if (rst) begin
      alpha    <= AMIN_ALPHA[7:0];
      ramp_up  <= 1'b1;
      ramp_cnt <= 32'd0;
    end else begin
      if (ramp_cnt >= RAMP_STEP - 1'b1) begin
        ramp_cnt <= 32'd0;
        if (ramp_up) begin
          if (alpha >= AMAX_ALPHA[7:0]) begin
            ramp_up <= 1'b0;
            alpha   <= alpha - 1'b1;
          end else begin
            alpha   <= alpha + 1'b1;
          end
        end else begin
          if (alpha <= AMIN_ALPHA[7:0]) begin
            ramp_up <= 1'b1;
            alpha   <= alpha + 1'b1;
          end else begin
            alpha   <= alpha - 1'b1;
          end
        end
      end else begin
        ramp_cnt <= ramp_cnt + 1'b1;
      end
    end
  end

  // ---- 红色 alpha 混合：叠层红色 * alpha + 原图 * (255-alpha)，右移 8 位 ----
  wire [7:0]  ia     = 8'd255 - alpha;
  wire [15:0] mix_rf = FLASH_R * alpha;
  wire [15:0] mix_ri = rgb_i[23:16] * ia;
  wire [15:0] mix_gf = FLASH_G * alpha;
  wire [15:0] mix_gi = rgb_i[15:8]  * ia;
  wire [15:0] mix_bf = FLASH_B * alpha;
  wire [15:0] mix_bi = rgb_i[7:0]   * ia;
  wire [16:0] acc_r  = {1'b0, mix_rf} + {1'b0, mix_ri};
  wire [16:0] acc_g  = {1'b0, mix_gf} + {1'b0, mix_gi};
  wire [16:0] acc_b  = {1'b0, mix_bf} + {1'b0, mix_bi};
  wire [7:0]  r_bl   = acc_r[15:8];
  wire [7:0]  g_bl   = acc_g[15:8];
  wire [7:0]  b_bl   = acc_b[15:8];

  // ---- 黑屏对齐：alarm 经 ALIGN_FRAMES 帧延时得到 alert，alarm^alert 期间黑屏 ----
  reg vs_d1;
  always @(posedge clk or posedge rst) begin
    if (rst) vs_d1 <= 1'b0;
    else     vs_d1 <= vs_i;
  end
  wire frame_tick = vs_d1 & ~vs_i;   // vs 下降沿 = 帧起点

  reg [ALIGN_FRAMES:0] alert_pipe;
  always @(posedge clk or posedge rst) begin
    if (rst) alert_pipe <= {(ALIGN_FRAMES+1){1'b0}};
    else if (frame_tick) alert_pipe <= {alert_pipe[ALIGN_FRAMES-1:0], alarm};
  end
  wire alert = alert_pipe[ALIGN_FRAMES];
  wire black = alarm ^ alert;

  // ---- 视频：黑屏期输出 0；否则告警对齐后叠红色呼吸闪烁，非告警透传 ----
  assign hs_o = hs_i;
  assign vs_o = vs_i;
  assign de_o = de_i;
  assign rgb_o = black ? 24'd0 : ((de_i && alert) ? {r_bl, g_bl, b_bl} : rgb_i);

  assign alarm_o = alarm;
  assign alert_o = alert;

endmodule

// ============================================================================
// 8-bit x 1024 异步 FIFO，用于 TF 卡音频流播的跨时钟域缓冲。
// 内嵌于此文件末尾，避免 TD 工程需要额外登记源文件而触发 black box。
// 写侧：sd_card_clk（audio_tf_stream 流播器填充）
// 读侧：audio_mclk（image_linked_audio_pcm_i2s 播放器读取）
// 复用工程内 anlogic 软 FIFO wfifo_32_32_512（见 IP/afifo_16_32_256.v），
// 通过参数改写为 8 bit 宽、10 bit 地址 = 1024 深。SHOW_AHEAD 模式：dout 常显队首，
// valid 即 !empty_flag，读侧按 re 单拍弹出。
// ============================================================================
module audio_async_fifo (
    input            rst,        // 异步复位，高有效
    input            clkw,       // 写时钟（sd_card_clk）
    input            clkr,       // 读时钟（audio_mclk）
    input            we,
    input  [15:0]    di,
    input            re,
    output [15:0]    dout,
    output           valid,      // = !empty（SHOW_AHEAD 模式）
    output           full_flag,
    output           empty_flag,
    output [9:0]     wrusedw,    // 写域已写入数量（供流播器做填充门限）
    output [9:0]     rdusedw     // 读域可读数量
);

wfifo_32_32_512 #(
    .DATA_WIDTH_W  (16),
    .DATA_WIDTH_R  (16),
    .ADDR_WIDTH_W  (10),
    .ADDR_WIDTH_R  (10),
    .AL_FULL_NUM   (1000),
    .AL_EMPTY_NUM  (4),
    .SHOW_AHEAD_EN (1'b1),
    .OUTREG_EN     ("NOREG")
) u_fifo (
    .rst       (rst),
    .clkw      (clkw),
    .clkr      (clkr),
    .we        (we),
    .di        (di),
    .re        (re),
    .dout      (dout),
    .valid     (valid),
    .full_flag (full_flag),
    .empty_flag(empty_flag),
    .afull     (),
    .aempty    (),
    .wrusedw   (wrusedw),
    .rdusedw   (rdusedw)
);

endmodule
