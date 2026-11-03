module sd_card_bmp #(
    parameter integer CLK_FREQ_HZ       = 100_000_000,
    parameter [31:0]  SCAN_START_SECTOR = 32'd0,
    parameter [31:0]  SCAN_MAX_SECTOR   = 32'd131071,
    parameter [2:0]   SCAN_TARGET_COUNT = 3'd6,      // 上限由 SDRAM 容量决定：2M 字 / 307200 字每张 ≈ 6 张
    parameter integer LOAD_TIMEOUT_MS   = 5000        // 单张图读取超时（毫秒）
)(
    input                       clk,
    input                       rst,
    input                       key_next,
    input                       key_auto,
    input                       key_prev,         // key3：短按=上一张；菜单内短按=上一选项；长按=上一布局模式
    input                       alarm_freeze,     // 地震/火灾告警中：冻结按键与自动轮播（sd_card_clk 域）
    output [3:0]                state_code,
    output [2:0]                img_count,        // 已扫描到的合法 BMP 张数（0~6），用于屏幕自检指示
    output [1:0]                period_sel_out,   // 轮播周期档位 0:0.5s 1:1s 2:2s 3:4s
    output                      auto_play_out,    // 自动轮播开关状态
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,
    output reg                  display_valid,

    input                       write_finish_toggle,
    output reg                  commit_toggle,     // 每切一次图翻转一次，供转场引擎 CDC 检测提交
    output [20:0]               wr_addr,          // 当前装载图片的固定 SDRAM 区域基地址（=load_idx*307200）
    output wire [2:0]           current_image_index, // 当前真正显示中的图片编号
    output reg                  thumbnail_ready,     // 缩略图全部预加载完成
    output reg [1:0]            layout_mode,         // 0:单图 1:底部条 2:2x2
    output reg                  thumb_write_en,
    output reg [14:0]           thumb_write_addr,
    output reg [7:0]            thumb_write_data,

    output reg                  menu_enable,
    output reg [2:0]            menu_cursor,
    output reg [3:0]            volume_level,
    output reg [3:0]            brightness_level,
    output reg [3:0]            contrast_level,
    output reg                  auto_play_status,
    output                      quake_ready,      // 地震告警图(第5张,index4)已载入 REGION_4
    output                      fire_ready,       // 火灾告警图(第6张,index5)已载入 REGION_5

    output                      write_req,
    input                       write_req_ack,
    output                      write_en,
    output [31:0]               write_data,
    output                      SD_nCS,
    output                      SD_DCLK,
    output                      SD_MOSI,
    input                       SD_MISO,

    // 音频流播异步 FIFO 写侧（连到 top 层 audio_async_fifo，写时钟 sd_card_clk）
    output                      audio_fifo_we,
    output [15:0]               audio_fifo_data,
    input  [9:0]                audio_fifo_wrusedw
);

wire key_next_press;
wire key_next_long;
wire key_auto_short;
wire key_auto_long;
wire key_prev_press;
wire key_prev_long;

wire             bmp_sd_sec_read;
wire [31:0]      bmp_sd_sec_read_addr;
wire             audio_sd_sec_read;
wire [31:0]      audio_sd_sec_read_addr;
wire             sd_sec_read;
wire [31:0]      sd_sec_read_addr;
wire [7:0]       sd_sec_read_data;
wire             sd_sec_read_data_valid;
wire             sd_sec_read_end;
wire [31:0]      bpb_first_data;
wire             bpb_valid;
wire             bmp_data_wr_en;
wire [23:0]      bmp_data;
wire             load_data_go;
wire [31:0]      src_width;
wire [31:0]      src_height;
wire             scale_valid;
wire [23:0]      scale_data;

wire             sd_init_done;
wire             bmp_ready;
wire             scan_done;
wire             scan_found_valid;
wire [31:0]      scan_found_sector;
wire [2:0]       scan_found_total;

reg              scan_start_pulse;
reg              load_start_pulse;
reg [31:0]       load_sector;
reg              scan_kicked;
reg              first_image_committed;
reg              auto_play_en;
reg [31:0]       auto_cnt;
reg [1:0]        period_sel;          // 0:0.5s 1:1s 2:2s 3:4s
reg [2:0]        img_found_count;
reg [2:0]        img_idx;              // 当前真正显示中的图片编号
reg [2:0]        load_idx;             // 当前正在预载写入的图片编号
reg [31:0]       img_sector0;
reg [31:0]       img_sector1;
reg [31:0]       img_sector2;
reg [31:0]       img_sector3;
reg [31:0]       img_sector4;
reg [31:0]       img_sector5;
reg              load_busy;
reg [31:0]       load_timeout_cnt;
reg              load_abort;

// 预加载相关
reg              preload_done;
reg              preload_load;
reg [2:0]        preload_idx;

// 菜单长按 V O L / B R I G H T 项连续递减
reg              adjust_repeat_active;
reg [31:0]       adjust_repeat_cnt;

reg [9:0]        thumb_src_x;
reg [8:0]        thumb_src_y;
reg [14:0]       thumb_row_base;

// bmp_ready 先表示“源文件读取/送 FIFO 完成”；真正切显示要等 write_finish_toggle 同步后
reg              source_done_seen;

// 同步 mem_clk 域的 write_finish_toggle
reg [2:0]        wrfin_tgl_sync;
wire             write_finish_pulse;

wire auto_tick;
wire [2:0] next_from_current;
wire [2:0] prev_from_current;

assign write_en       = scale_valid;
assign write_data     = {scale_data[23:16], scale_data[15:8], scale_data[7:0], 8'b0};
assign current_image_index = img_idx;
assign period_sel_out = period_sel;
assign auto_play_out  = auto_play_en;
assign img_count      = img_found_count;
assign write_finish_pulse = wrfin_tgl_sync[2] ^ wrfin_tgl_sync[1];
assign wr_addr        = img_region_base(load_idx);

// SD 读请求 mux：预载完成前 bmp_read 独占总线；预载完成后（运行时无图片 SD 读）
// 交给音频流播器 audio_tf_stream 复用总线，读取 ttsmaker 音频文件。
assign sd_sec_read      = preload_done ? audio_sd_sec_read      : bmp_sd_sec_read;
assign sd_sec_read_addr = preload_done ? audio_sd_sec_read_addr : bmp_sd_sec_read_addr;

// 轮播周期可选：长按 key2 在 0.5 s / 1 s / 2 s / 4 s 之间切换
function [31:0] period_ticks;
    input [1:0] sel;
    begin
        case (sel)
            2'd0:    period_ticks = CLK_FREQ_HZ / 2;
            2'd1:    period_ticks = CLK_FREQ_HZ;
            2'd2:    period_ticks = CLK_FREQ_HZ * 2;
            default: period_ticks = CLK_FREQ_HZ * 4;
        endcase
    end
endfunction

wire [31:0] auto_period = period_ticks(period_sel);
assign auto_tick  = (auto_cnt == (auto_period - 32'd1));

// 第 5 张（index 4）固定作为地震告警图，第 6 张（index 5）固定作为火灾告警图，
// 轮播只显示 0~3（disp_count，共 4 张普通图）。
// 实际扫描到 6 张时普通图为 0~3；不足时相应缩减，缺告警图则无法触发对应告警。
wire [2:0] disp_count = (img_found_count > 3'd4) ? 3'd4 : img_found_count;
assign quake_ready = preload_done && (img_found_count >= 3'd5);
assign fire_ready  = preload_done && (img_found_count == 3'd6);

assign next_from_current = next_index_limited(img_idx, disp_count);
assign prev_from_current = prev_index_limited(img_idx, disp_count);

// 懒加载：当前已完整写入 SDRAM 的可轮播图片数。
wire [2:0] loaded_count = preload_done ? disp_count :
                          ((preload_idx > disp_count) ? disp_count : preload_idx);
wire       next_ready   = (next_from_current < loaded_count);
wire       prev_ready   = (prev_from_current < loaded_count);

// key1：短按 = 手动下一张；长按（≥1s）= 切换缩略图布局（单图/底部条/2x2）
key_short_long #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .LONG_MS     (1000)
) u_key_next (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_next),
    .short_pulse(key_next_press),
    .long_pulse (key_next_long)
);

// key2：短按 = 开/关自动轮播；长按（按住 ≥0.8 s）= 打开/关闭菜单
key_press_short_long #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (20),
    .LONG_MS     (800)
) u_key_auto (
    .clk         (clk),
    .rst         (rst),
    .button_in   (key_auto),
    .short_pulse (key_auto_short),
    .long_pulse  (key_auto_long)
);

// key3：短按 = 手动上一张；长按（≥0.8s）= 反向切换缩略图布局。
// 与 key2 使用同一按键模块及参数（20ms 消抖 + 800ms 长按），保证相同的极短反应延迟。
key_press_short_long #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (20),
    .LONG_MS     (800)
) u_key_prev (
    .clk         (clk),
    .rst         (rst),
    .button_in   (key_prev),
    .short_pulse (key_prev_press),
    .long_pulse  (key_prev_long)
);

function [2:0] next_index_limited;
    input [2:0] cur;
    input [2:0] count;
    begin
        if (count <= 3'd1)
            next_index_limited = 3'd0;
        else if (cur >= (count - 3'd1))
            next_index_limited = 3'd0;
        else
            next_index_limited = cur + 3'd1;
    end
endfunction

function [2:0] prev_index_limited;
    input [2:0] cur;
    input [2:0] count;
    begin
        if (count <= 3'd1)
            prev_index_limited = 3'd0;
        else if (cur == 3'd0)
            prev_index_limited = count - 3'd1;
        else
            prev_index_limited = cur - 3'd1;
    end
endfunction

function [31:0] sector_lut;
    input [2:0] idx;
    begin
        case (idx)
            3'd0: sector_lut = img_sector0;
            3'd1: sector_lut = img_sector1;
            3'd2: sector_lut = img_sector2;
            3'd3: sector_lut = img_sector3;
            3'd4: sector_lut = img_sector4;
            3'd5: sector_lut = img_sector5;
            default: sector_lut = img_sector0;
        endcase
    end
endfunction

function [20:0] img_region_base;
    input [2:0] idx;
    begin
        case (idx)
            3'd0: img_region_base = 21'd0;
            3'd1: img_region_base = 21'd307200;
            3'd2: img_region_base = 21'd614400;
            3'd3: img_region_base = 21'd921600;
            3'd4: img_region_base = 21'd1228800;
            3'd5: img_region_base = 21'd1536000;
            // 防御性修复：非法索引不再回退到 REGION_0（否则会覆盖首图），
            // 改为指向 2M 字地址空间的最后一个字，落在第 6 张图之后的无用区，
            // 即便误写也只污染空白区，不会破坏任何已显示的图片。
            default: img_region_base = 21'h1FFFFF;
        endcase
    end
endfunction

always @(posedge clk or posedge rst) begin
    if (rst) begin
        wrfin_tgl_sync        <= 3'b000;
        scan_start_pulse      <= 1'b0;
        load_start_pulse      <= 1'b0;
        load_sector           <= 32'd0;
        scan_kicked           <= 1'b0;
        first_image_committed <= 1'b0;
        auto_play_en          <= 1'b0;
        auto_cnt              <= 32'd0;
        img_found_count       <= 3'd0;
        img_idx               <= 3'd0;
        load_idx              <= 3'd0;
        img_sector0           <= 32'd0;
        img_sector1           <= 32'd0;
        img_sector2           <= 32'd0;
        img_sector3           <= 32'd0;
        img_sector4           <= 32'd0;
        img_sector5           <= 32'd0;
        load_busy             <= 1'b0;
        source_done_seen      <= 1'b0;
        display_valid         <= 1'b0;
        commit_toggle         <= 1'b0;
        load_timeout_cnt      <= 32'd0;
        load_abort            <= 1'b0;
        period_sel            <= 2'd1;
        preload_done          <= 1'b0;
        preload_load          <= 1'b0;
        preload_idx           <= 3'd0;
        thumbnail_ready       <= 1'b0;
        layout_mode           <= 2'd0;
        thumb_src_x           <= 10'd0;
        thumb_src_y           <= 9'd0;
        thumb_row_base        <= 15'd4720;
        thumb_write_en        <= 1'b0;
        thumb_write_addr      <= 15'd0;
        thumb_write_data      <= 8'd0;
        menu_enable           <= 1'b0;
        menu_cursor           <= 3'd0;
        volume_level          <= 4'd12;
        brightness_level      <= 4'd15;
        contrast_level        <= 4'd15;
        auto_play_status      <= 1'b0;
        adjust_repeat_active  <= 1'b0;
        adjust_repeat_cnt     <= 32'd0;
    end else begin
        wrfin_tgl_sync   <= {wrfin_tgl_sync[1:0], write_finish_toggle};
        scan_start_pulse <= 1'b0;
        load_start_pulse <= 1'b0;
        load_abort       <= 1'b0;
        thumb_write_en   <= 1'b0;

        // 长按 key1 循环切换缩略图布局（单图 → 底部条 → 2x2 → 单图）；菜单内长按 key1 = 光标上移
        if (key_next_long && !alarm_freeze && thumbnail_ready && !menu_enable) begin
            if (layout_mode == 2'd2)
                layout_mode <= 2'd0;
            else
                layout_mode <= layout_mode + 2'd1;
        end

        // 长按 key3 反向循环切换缩略图布局（单图 → 2x2 → 底部条 → 单图），与 key1 相反
        if (key_prev_long && !alarm_freeze && thumbnail_ready && !menu_enable) begin
            if (layout_mode == 2'd0)
                layout_mode <= 2'd2;
            else
                layout_mode <= layout_mode - 2'd1;
        end

        // ---- 菜单状态机：长按 key2 开/关菜单；菜单内 key1 上下移动，key2 短按调节/长按递减 ----
        auto_play_status <= auto_play_en;

        // 长按递减的连续重复（按住每 150ms 减一档）
        if (key_auto && !alarm_freeze) begin
            adjust_repeat_active <= 1'b0;
            adjust_repeat_cnt    <= 32'd0;
        end else if (adjust_repeat_active) begin
            if (adjust_repeat_cnt >= (CLK_FREQ_HZ/1000)*150 - 1) begin
                adjust_repeat_cnt <= 32'd0;
                if (menu_enable && menu_cursor == 3'd1 && volume_level > 4'd0)
                    volume_level <= volume_level - 4'd1;
                if (menu_enable && menu_cursor == 3'd2 && brightness_level > 4'd1)
                    brightness_level <= brightness_level - 4'd1;
                if (menu_enable && menu_cursor == 3'd3 && contrast_level > 4'd1)
                    contrast_level <= contrast_level - 4'd1;
            end else begin
                adjust_repeat_cnt <= adjust_repeat_cnt + 32'd1;
            end
        end

        // 长按 key2：关闭态→打开菜单；VOL/BRIGHT/CONT 项→递减；其它项→关闭菜单
        if (key_auto_long && !alarm_freeze) begin
            if (!menu_enable) begin
                menu_enable <= 1'b1;
                menu_cursor <= 3'd0;
            end else if (menu_cursor == 3'd1) begin
                if (volume_level > 4'd0) volume_level <= volume_level - 4'd1;
                adjust_repeat_active <= 1'b1;
                adjust_repeat_cnt    <= 32'd0;
            end else if (menu_cursor == 3'd2) begin
                if (brightness_level > 4'd1) brightness_level <= brightness_level - 4'd1;
                adjust_repeat_active <= 1'b1;
                adjust_repeat_cnt    <= 32'd0;
            end else if (menu_cursor == 3'd3) begin
                if (contrast_level > 4'd1) contrast_level <= contrast_level - 4'd1;
                adjust_repeat_active <= 1'b1;
                adjust_repeat_cnt    <= 32'd0;
            end else begin
                menu_enable <= 1'b0;
            end
        end

        // 菜单光标导航与参数调节（TIME/VOL/BRIGHT/CONT/EXIT）
        if (menu_enable && !alarm_freeze) begin
            if (key_next_press) begin
                if (menu_cursor == 3'd4)
                    menu_cursor <= 3'd0;
                else
                    menu_cursor <= menu_cursor + 3'd1;
            end
            if (key_next_long) begin
                if (menu_cursor == 3'd0)
                    menu_cursor <= 3'd4;
                else
                    menu_cursor <= menu_cursor - 3'd1;
            end
            if (key_prev_press) begin
                if (menu_cursor == 3'd0)
                    menu_cursor <= 3'd4;
                else
                    menu_cursor <= menu_cursor - 3'd1;
            end
            if (key_auto_short) begin
                case (menu_cursor)
                    3'd0: begin
                        period_sel <= period_sel + 2'd1;   // 0.5/1/2/4s 循环
                        auto_cnt   <= 32'd0;
                    end
                    3'd1: begin
                        if (volume_level == 4'd15) volume_level <= 4'd0;
                        else                       volume_level <= volume_level + 4'd1;
                    end
                    3'd2: begin
                        if (brightness_level == 4'd15) brightness_level <= 4'd1;
                        else                           brightness_level <= brightness_level + 4'd1;
                    end
                    3'd3: begin
                        if (contrast_level == 4'd15) contrast_level <= 4'd1;
                        else                         contrast_level <= contrast_level + 4'd1;
                    end
                    default: menu_enable <= 1'b0;   // EXIT
                endcase
            end
        end

        // 从每张 640x480 BMP 生成一张 80x60 RGB332 缩略图。
        // BMP 像素自底向上到达，故 thumb_row_base 向下递减。
        if (load_start_pulse) begin
            thumb_src_x <= 10'd0;
            thumb_src_y <= 9'd0;
            case (load_idx)
                3'd0: thumb_row_base <= 15'd4720;
                3'd1: thumb_row_base <= 15'd9520;
                3'd2: thumb_row_base <= 15'd14320;
                3'd3: thumb_row_base <= 15'd19120;
                3'd4: thumb_row_base <= 15'd23920;
                default: thumb_row_base <= 15'd28720;
            endcase
        end else if (scale_valid) begin
            if ((thumb_src_x[2:0] == 3'b000) &&
                (thumb_src_y[2:0] == 3'b000)) begin
                thumb_write_en   <= 1'b1;
                thumb_write_addr <= thumb_row_base + (thumb_src_x >> 3);
                thumb_write_data <= {scale_data[23:21], scale_data[15:13], scale_data[7:6]};
            end

            if (thumb_src_x == 10'd639) begin
                thumb_src_x <= 10'd0;
                if (thumb_src_y < 9'd479)
                    thumb_src_y <= thumb_src_y + 9'd1;
                if ((thumb_src_y[2:0] == 3'b111) && (thumb_src_y < 9'd479))
                    thumb_row_base <= thumb_row_base - 15'd80;
            end else begin
                thumb_src_x <= thumb_src_x + 10'd1;
            end
        end

        if (!sd_init_done) begin
            scan_kicked           <= 1'b0;
            first_image_committed <= 1'b0;
            auto_play_en          <= 1'b0;
            auto_cnt              <= 32'd0;
            img_found_count       <= 3'd0;
            img_idx               <= 3'd0;
            load_idx              <= 3'd0;
            img_sector0           <= 32'd0;
            img_sector1           <= 32'd0;
            img_sector2           <= 32'd0;
            img_sector3           <= 32'd0;
            img_sector4           <= 32'd0;
            img_sector5           <= 32'd0;
            load_busy             <= 1'b0;
            source_done_seen      <= 1'b0;
            display_valid         <= 1'b0;
            commit_toggle         <= 1'b0;
            load_timeout_cnt      <= 32'd0;
            preload_done          <= 1'b0;
            preload_load          <= 1'b0;
            preload_idx           <= 3'd0;
            thumbnail_ready       <= 1'b0;
            layout_mode           <= 2'd0;
            menu_enable           <= 1'b0;
        end else begin
            // 扫描阶段缓存各张图的起始 sector
            if (scan_found_valid) begin
                case (img_found_count)
                    3'd0: img_sector0 <= scan_found_sector;
                    3'd1: img_sector1 <= scan_found_sector;
                    3'd2: img_sector2 <= scan_found_sector;
                    3'd3: img_sector3 <= scan_found_sector;
                    3'd4: img_sector4 <= scan_found_sector;
                    3'd5: img_sector5 <= scan_found_sector;
                    default: ;
                endcase

                if (img_found_count < SCAN_TARGET_COUNT)
                    img_found_count <= img_found_count + 3'd1;
            end

            // 记住 bmp_read 已经把源图送完 FIFO，但还不能确认写入，得等整帧写完
            if (load_busy && bmp_ready)
                source_done_seen <= 1'b1;

            // 只有真正收到 write_finish_toggle 脉冲，才确认一张图完全写入 SDRAM
            if (load_busy && source_done_seen && write_finish_pulse) begin
                load_busy        <= 1'b0;
                source_done_seen <= 1'b0;
                load_timeout_cnt <= 32'd0;
                if (preload_load) begin
                    preload_load <= 1'b0;
                    if (preload_idx == 3'd0) begin
                        // 懒加载：首图写入完成即立刻提交显示，不等待其余图片预载
                        img_idx               <= 3'd0;
                        display_valid         <= 1'b1;
                        first_image_committed <= 1'b1;
                        commit_toggle         <= ~commit_toggle;
                        if (img_found_count > 3'd1)
                            preload_idx <= preload_idx + 3'd1;
                        else begin
                            preload_done    <= 1'b1;
                            thumbnail_ready <= 1'b1;
                            preload_idx     <= 3'd0;
                        end
                    end else if ((preload_idx + 3'd1 >= img_found_count) ||
                                 (preload_idx == SCAN_TARGET_COUNT - 3'd1)) begin
                        preload_done    <= 1'b1;
                        thumbnail_ready <= 1'b1;
                        preload_idx     <= 3'd0;
                    end else begin
                        preload_idx <= preload_idx + 3'd1;
                    end
                end
            end
            // 预加载阶段读图超时：删坏图（前移、末条清零、总数减一），保持区域与逻辑索引对齐。
            // 首图 / 运行时切图不再存在 SD 读，故超时仅发生在预加载阶段。
            else if (load_busy) begin
                if (load_timeout_cnt >= (CLK_FREQ_HZ / 1000) * LOAD_TIMEOUT_MS) begin
                    load_busy        <= 1'b0;
                    source_done_seen <= 1'b0;
                    load_timeout_cnt <= 32'd0;
                    load_abort       <= 1'b1;

                    if (preload_load) begin
                        preload_load <= 1'b0;
                        // 若删的正是预加载最后一张，则预加载直接结束
                        if (preload_idx >= (img_found_count - 3'd1)) begin
                            preload_done    <= 1'b1;
                            thumbnail_ready <= 1'b1;
                        end
                        // 从 img_sector0..5 中删除 preload_idx 处条目：前移、末条清零、总数减一
                        case (preload_idx)
                            3'd0: begin img_sector0 <= img_sector1; img_sector1 <= img_sector2; img_sector2 <= img_sector3; img_sector3 <= img_sector4; img_sector4 <= img_sector5; img_sector5 <= 32'd0; end
                            3'd1: begin img_sector1 <= img_sector2; img_sector2 <= img_sector3; img_sector3 <= img_sector4; img_sector4 <= img_sector5; img_sector5 <= 32'd0; end
                            3'd2: begin img_sector2 <= img_sector3; img_sector3 <= img_sector4; img_sector4 <= img_sector5; img_sector5 <= 32'd0; end
                            3'd3: begin img_sector3 <= img_sector4; img_sector4 <= img_sector5; img_sector5 <= 32'd0; end
                            3'd4: begin img_sector4 <= img_sector5; img_sector5 <= 32'd0; end
                            default: begin img_sector5 <= 32'd0; end
                        endcase
                        if (img_found_count > 3'd0)
                            img_found_count <= img_found_count - 3'd1;
                    end
                end else begin
                    load_timeout_cnt <= load_timeout_cnt + 32'd1;
                end
            end
            else begin
                load_timeout_cnt <= 32'd0;
            end

            // 上电后自动发起一次 BMP 列表扫描（张数上限由 SDRAM 容量决定）
            if (!scan_kicked && bmp_ready) begin
                scan_start_pulse      <= 1'b1;
                scan_kicked           <= 1'b1;
                first_image_committed <= 1'b0;
                auto_play_en          <= 1'b0;
                auto_cnt              <= 32'd0;
                img_found_count       <= 3'd0;
                img_idx               <= 3'd0;
                load_idx              <= 3'd0;
                img_sector0           <= 32'd0;
                img_sector1           <= 32'd0;
                img_sector2           <= 32'd0;
                img_sector3           <= 32'd0;
                img_sector4           <= 32'd0;
                img_sector5           <= 32'd0;
                display_valid         <= 1'b0;
                commit_toggle         <= 1'b0;
                load_busy             <= 1'b0;
                source_done_seen      <= 1'b0;
                preload_done          <= 1'b0;
                preload_load          <= 1'b0;
                preload_idx           <= 3'd0;
                thumbnail_ready       <= 1'b0;
            end else begin
                // 短按 key2：开 / 关自动轮播（菜单打开时由菜单状态机处理，不响应）
                if (!menu_enable && key_auto_short && !alarm_freeze && scan_done && (disp_count > 3'd1)) begin
                    auto_play_en <= ~auto_play_en;
                    auto_cnt     <= 32'd0;
                end

                // 手动下一张（key1 短按）：纯指针切换，无 SD 读，极低延迟
                if (!menu_enable && key_next_press && !alarm_freeze && scan_done && display_valid &&
                    first_image_committed && (disp_count > 3'd1) && next_ready) begin
                    img_idx       <= next_from_current;
                    commit_toggle <= ~commit_toggle;
                    auto_cnt      <= 32'd0;
                end

                // 手动上一张（key3 短按）：与 key1 方向相反
                if (!menu_enable && key_prev_press && !alarm_freeze && scan_done && display_valid &&
                    first_image_committed && (disp_count > 3'd1) && prev_ready) begin
                    img_idx       <= prev_from_current;
                    commit_toggle <= ~commit_toggle;
                    auto_cnt      <= 32'd0;
                end

                // 自动轮播：持续计时，到点切换下一张（菜单打开时暂停）
                if (!menu_enable && scan_done && auto_play_en && !alarm_freeze && display_valid &&
                    first_image_committed && (disp_count > 3'd1)) begin
                    if (auto_tick) begin
                        // 懒加载：目标图尚未预载完成时跳过本次切换，直接回零重新计时
                        if (next_ready) begin
                            img_idx       <= next_from_current;
                            commit_toggle <= ~commit_toggle;
                        end
                        auto_cnt <= 32'd0;
                    end else begin
                        auto_cnt <= auto_cnt + 32'd1;
                    end
                end else begin
                    auto_cnt <= 32'd0;
                end

                // 扫描完成：顺序把每张图全量读入其固定 SDRAM 区域（同时生成缩略图）。
                // 预载完成后导航即为纯指针切换，不再有任何运行时 SD 读。
                if (scan_done && !preload_done && bmp_ready && !load_busy &&
                    (img_found_count != 3'd0) &&
                    (preload_idx < img_found_count) &&        // 防御性修复：索引必须落在已扫描到的图内
                    (preload_idx < SCAN_TARGET_COUNT)) begin // 且不得超出固定区域数量
                    load_idx         <= preload_idx;
                    load_sector      <= sector_lut(preload_idx);
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    preload_load     <= 1'b1;
                    source_done_seen <= 1'b0;
                    auto_cnt         <= 32'd0;
                end
            end
        end
    end
end

bmp_read bmp_read_m0(
    .clk                    (clk),
    .rst                    (rst),
    .ready                  (bmp_ready),

    .scan_start             (scan_start_pulse),
    .scan_start_sector      (SCAN_START_SECTOR),
    .scan_max_sector        (SCAN_MAX_SECTOR),
    .scan_target_count      (SCAN_TARGET_COUNT),
    .scan_done              (scan_done),
    .scan_found_valid       (scan_found_valid),
    .scan_found_sector      (scan_found_sector),
    .scan_found_total       (scan_found_total),

    .load_start             (load_start_pulse),
    .load_abort             (load_abort),
    .load_sector            (load_sector),

    .sd_init_done           (sd_init_done),
    .state_code             (state_code),
    .bmp_width              (bmp_width),
    .bmp_height             (bmp_height),
    .write_req              (write_req),
    .write_req_ack          (write_req_ack),
    .sd_sec_read            (bmp_sd_sec_read),
    .sd_sec_read_addr       (bmp_sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .bmp_data_wr_en         (bmp_data_wr_en),
    .bmp_data               (bmp_data),
    .load_data_go           (load_data_go),
    .src_width              (src_width),
    .src_height             (src_height),
    .bpb_first_data         (bpb_first_data),
    .bpb_valid              (bpb_valid)
);

// 最近邻缩放：把任意分辨率（640~2048 x 480~2048）像素流缩放到固定 640x480，
// 保持自底向上顺序不变；SDRAM 之后的显示/转场/告警/OSD 逻辑完全不受影响。
bmp_scaler #(
    .TARGET_W (640),
    .TARGET_H (480)
) u_bmp_scaler (
    .clk         (clk),
    .rst         (rst),
    .frame_start (load_data_go),
    .src_w       (src_width[11:0]),
    .src_h       (src_height[11:0]),
    .src_valid   (bmp_data_wr_en),
    .src_data    (bmp_data),
    .out_valid   (scale_valid),
    .out_data    (scale_data)
);

sd_card_top sd_card_top_m0(
    .clk                    (clk),
    .rst                    (rst),
    .SD_nCS                 (SD_nCS),
    .SD_DCLK                (SD_DCLK),
    .SD_MOSI                (SD_MOSI),
    .SD_MISO                (SD_MISO),
    .sd_init_done           (sd_init_done),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .sd_sec_write           (1'b0),
    .sd_sec_write_addr      (32'd0),
    .sd_sec_write_data      (),
    .sd_sec_write_data_req  (),
    .sd_sec_write_end       ()
);

// 音频流播（ttsmaker 第3张图）：预载完成后复用已空闲的 SD 总线，循环把音频写入异步 FIFO。
audio_tf_stream u_audio_tf_stream (
    .clk               (clk),
    .rst               (rst),
    .start             (preload_done),
    .bpb_valid         (bpb_valid),
    .bpb_first_data    (bpb_first_data),
    .image_index       (img_idx),
    .sd_sec_read       (audio_sd_sec_read),
    .sd_sec_read_addr  (audio_sd_sec_read_addr),
    .sd_sec_read_data  (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end   (sd_sec_read_end),
    .fifo_wrusedw      (audio_fifo_wrusedw),
    .fifo_we           (audio_fifo_we),
    .fifo_data         (audio_fifo_data)
);

endmodule

// ============================================================================
// 短按 / 长按判别按键模块（key1 用，无独立消抖，长按阈值 1s）
// ============================================================================
module key_short_long #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer LONG_MS = 1000
)(
    input  wire clk,
    input  wire rst,
    input  wire button_in,
    output reg  short_pulse,
    output reg  long_pulse
);
localparam integer LONG_CYCLES = (CLK_FREQ_HZ / 1000) * LONG_MS;
reg sync0, sync1;
reg [31:0] hold_cnt;
reg pressed;
reg long_sent;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        sync0 <= 1'b1;
        sync1 <= 1'b1;
        hold_cnt <= 32'd0;
        pressed <= 1'b0;
        long_sent <= 1'b0;
        short_pulse <= 1'b0;
        long_pulse <= 1'b0;
    end else begin
        sync0 <= button_in;
        sync1 <= sync0;
        short_pulse <= 1'b0;
        long_pulse <= 1'b0;

        if (!sync1) begin
            if (!pressed) begin
                pressed <= 1'b1;
                hold_cnt <= 32'd0;
                long_sent <= 1'b0;
            end else if (!long_sent) begin
                if (hold_cnt >= LONG_CYCLES-1) begin
                    long_pulse <= 1'b1;
                    long_sent <= 1'b1;
                end else begin
                    hold_cnt <= hold_cnt + 32'd1;
                end
            end
        end else if (pressed) begin
            if (!long_sent && (hold_cnt > (CLK_FREQ_HZ/50)))
                short_pulse <= 1'b1;
            pressed <= 1'b0;
            hold_cnt <= 32'd0;
            long_sent <= 1'b0;
        end
    end
end
endmodule

// ============================================================================
// 短按 / 长按判别按键模块（key2/key3 用，带 20ms 消抖，长按阈值 800ms）
//   短按：按下后松开（未达到长按阈值）时，在松手当拍给出 short_pulse
//   长按：按住达到 LONG_MS 时立刻给出 long_pulse（一次按住只给一次）
// ============================================================================
module key_press_short_long #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer DEBOUNCE_MS = 20,
    parameter integer LONG_MS     = 800
)(
    input  wire clk,
    input  wire rst,
    input  wire button_in,      // 默认：松开=1，按下=0
    output reg  short_pulse,
    output reg  long_pulse
);

localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;
localparam integer LONG_CYCLES     = (CLK_FREQ_HZ / 1000) * LONG_MS;

reg         button_sync0;
reg         button_sync1;
reg         button_stable;
reg [31:0]  deb_cnt;
reg [31:0]  hold_cnt;
reg         long_fired;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        button_sync0 <= 1'b1;
        button_sync1 <= 1'b1;
        button_stable <= 1'b1;
        deb_cnt      <= 32'd0;
        hold_cnt     <= 32'd0;
        long_fired   <= 1'b0;
        short_pulse  <= 1'b0;
        long_pulse   <= 1'b0;
    end else begin
        button_sync0 <= button_in;
        button_sync1 <= button_sync0;
        short_pulse  <= 1'b0;
        long_pulse   <= 1'b0;

        // 20 ms 消抖
        if (button_sync1 == button_stable) begin
            deb_cnt <= 32'd0;
        end else if (deb_cnt >= DEBOUNCE_CYCLES - 1) begin
            button_stable <= button_sync1;
            deb_cnt       <= 32'd0;
        end else begin
            deb_cnt <= deb_cnt + 32'd1;
        end

        // 按住计时：到阈值发一次长按；松手时若未触发过长按则算短按
        if (!button_stable) begin
            if (hold_cnt < LONG_CYCLES)
                hold_cnt <= hold_cnt + 32'd1;

            if (!long_fired && (hold_cnt >= LONG_CYCLES - 1)) begin
                long_fired <= 1'b1;
                long_pulse <= 1'b1;
            end
        end else begin
            if ((hold_cnt != 32'd0) && !long_fired)
                short_pulse <= 1'b1;
            hold_cnt   <= 32'd0;
            long_fired <= 1'b0;
        end
    end
end

endmodule

// ============================================================================
// TF 卡 ttsmaker 音频流播（写侧，sd_card_clk 域）。
// 内嵌于此文件末尾，避免 TD 工程需要额外登记源文件而触发 black box。
//
// 功能：
//   1. 预载完成（preload_done=start=1，此时 SD 总线空闲）后，从 FAT 数据区起点
//      bpb_first_data 起做一次性“TSA1”魔数扇区扫描，定位 ttsmaker 音频文件起始扇区
//      与采样数（文件头 8 字节 = 4 字节魔数 "TSA1" + 4 字节小端采样数）。
//   2. 定位成功后进入“循环填充”状态机：只要异步 FIFO 有空间（wrusedw<=门限），就
//      从 SD 卡按扇区读取音频数据写入 FIFO；播到文件末尾后回绕到文件头，持续循环。
//      这样 FIFO 内始终是“音频文件 + 文件尾回绕到文件头”的连续样本流。
//   3. “每两次播放间隔 5 秒”由读侧播放器 image_linked_audio_pcm_i2s 负责：它播满
//      整段采样数后插入 5 秒静音；本模块只负责无脑保满 FIFO。
// ============================================================================
module audio_tf_stream #(
    parameter [31:0] SCAN_MAX_SECTOR = 32'd131071,   // 定位扫描上限
    parameter [15:0] BLANK_STOP      = 16'd256,      // 连续空白扇区判定为数据区尾部，提前收尾
    parameter [9:0]  FILL_THRESH     = 10'd480       // FIFO 写域使用量 <= 此时才允许再读一扇区
)(
    input            clk,
    input            rst,
    input            start,             // = preload_done，SD 总线已空闲且 FAT 参数有效
    input            bpb_valid,         // FAT 数据区参数是否有效
    input  [31:0]    bpb_first_data,    // FAT 数据区起始扇区
    input  [2:0]     image_index,       // 当前显示图片索引（3'd2=idx2→file1，3'd3=idx3→file2）

    // SD 扇区读（本模块作为请求方，经上层 mux 接入 sd_card_top）
    output reg       sd_sec_read,
    output reg [31:0] sd_sec_read_addr,
    input  [7:0]     sd_sec_read_data,
    input            sd_sec_read_data_valid,
    input            sd_sec_read_end,

    // 异步 FIFO 写侧
    input  [9:0]     fifo_wrusedw,
    output           fifo_we,
    output [15:0]    fifo_data
);

localparam [2:0] ST_IDLE      = 3'd0;
localparam [2:0] ST_LOC       = 3'd1;   // 定位：逐扇区扫描魔数
localparam [2:0] ST_FILL_WAIT = 3'd2;   // 流播：等 FIFO 有空位
localparam [2:0] ST_FILL      = 3'd3;   // 流播：读一扇区并写 FIFO

reg [2:0]  state;
reg [8:0]  byte_cnt;         // 当前扇区内字节位置 0..511
reg [7:0]  hdr0, hdr1, hdr2, hdr3;
reg [7:0]  hdr4, hdr5, hdr6, hdr7;
reg [31:0] scan_sector;      // 定位阶段当前扫描扇区
reg        sector_any_other; // 定位阶段当前扇区是否含非 0x00/0xFF 数据
reg [15:0] scan_blank_run;   // 定位阶段连续空白扇区计数
reg [31:0] ts_start1;        // file1(TSA1) 起始扇区
reg [31:0] ts_start2;        // file2(TSA2) 起始扇区
reg [19:0] ts_samples1;      // file1 采样数（小端）
reg [19:0] ts_samples2;      // file2 采样数
reg        located1;
reg        located2;
reg [31:0] ts_start;         // 当前活动文件起始扇区
reg [19:0] ts_samples;       // 当前活动文件采样数
reg [2:0]  sel_index;        // 当前活动文件对应的图片索引
reg [19:0] audio_cnt;        // 本轮已写入的音频采样数
reg [9:0]  sector_idx;       // 流播阶段文件内扇区序号（0..N-1）
reg [31:0] cur_sector;       // 流播阶段当前文件绝对扇区
reg        located;
reg [7:0]  sample_lo;        // 16bit 样本低字节锁存

wire magic1 = (hdr0 == 8'h54) && (hdr1 == 8'h53) && (hdr2 == 8'h41) && (hdr3 == 8'h31);  // TSA1 → idx2
wire magic2 = (hdr0 == 8'h54) && (hdr1 == 8'h53) && (hdr2 == 8'h41) && (hdr3 == 8'h32);  // TSA2 → idx3
wire [19:0] hdr_samples = {hdr6[3:0], hdr5, hdr4};  // 低 20 位 = 采样数
wire header_zone = (sector_idx == 10'd0) && (byte_cnt < 9'd8);
wire audio_ok    = (audio_cnt < ts_samples);
wire [31:0] target_start   = (image_index == 3'd3) ? ts_start2   : ts_start1;
wire [19:0] target_samples = (image_index == 3'd3) ? ts_samples2 : ts_samples1;

// 流播写 FIFO：16bit 小端样本，偶数字节=低字节锁存，奇数字节=高字节拼完整样本写 FIFO
assign fifo_data = {sd_sec_read_data, sample_lo};   // {高字节, 低字节}
assign fifo_we   = (state == ST_FILL) && sd_sec_read_data_valid && audio_ok && !header_zone && byte_cnt[0];

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state           <= ST_IDLE;
        sd_sec_read     <= 1'b0;
        sd_sec_read_addr<= 32'd0;
        byte_cnt        <= 9'd0;
        hdr0 <= 8'd0; hdr1 <= 8'd0; hdr2 <= 8'd0; hdr3 <= 8'd0;
        hdr4 <= 8'd0; hdr5 <= 8'd0; hdr6 <= 8'd0; hdr7 <= 8'd0;
        scan_sector     <= 32'd0;
        sector_any_other<= 1'b0;
        scan_blank_run  <= 16'd0;
        ts_start1       <= 32'd0;
        ts_start2       <= 32'd0;
        ts_samples1     <= 20'd0;
        ts_samples2     <= 20'd0;
        located1        <= 1'b0;
        located2        <= 1'b0;
        ts_start        <= 32'd0;
        ts_samples      <= 20'd0;
        sel_index       <= 3'd7;
        audio_cnt       <= 20'd0;
        sample_lo       <= 8'd0;
        sector_idx      <= 10'd0;
        cur_sector      <= 32'd0;
        located         <= 1'b0;
    end else begin
        case (state)
            ST_IDLE: begin
                sd_sec_read <= 1'b0;
                byte_cnt    <= 9'd0;
                sector_any_other <= 1'b0;
                scan_blank_run   <= 16'd0;
                if (start && !located) begin
                    scan_sector      <= bpb_valid ? bpb_first_data : 32'd0;
                    sd_sec_read_addr <= bpb_valid ? bpb_first_data : 32'd0;
                    state            <= ST_LOC;
                end
            end

            ST_LOC: begin
                sd_sec_read <= 1'b1;
                if (sd_sec_read_data_valid) begin
                    byte_cnt <= byte_cnt + 9'd1;
                    case (byte_cnt)
                        9'd0: hdr0 <= sd_sec_read_data;
                        9'd1: hdr1 <= sd_sec_read_data;
                        9'd2: hdr2 <= sd_sec_read_data;
                        9'd3: hdr3 <= sd_sec_read_data;
                        9'd4: hdr4 <= sd_sec_read_data;
                        9'd5: hdr5 <= sd_sec_read_data;
                        9'd6: hdr6 <= sd_sec_read_data;
                        9'd7: hdr7 <= sd_sec_read_data;
                        default: ;
                    endcase
                    if ((sd_sec_read_data != 8'd0) && (sd_sec_read_data != 8'hFF))
                        sector_any_other <= 1'b1;
                end

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    byte_cnt    <= 9'd0;
                    if (magic1 && (hdr_samples != 20'd0) && !located1) begin
                        located1    <= 1'b1;
                        ts_start1   <= scan_sector;
                        ts_samples1 <= hdr_samples;
                        if (located2) begin
                            located <= 1'b1;
                            state   <= ST_FILL_WAIT;
                        end else begin
                            sector_any_other <= 1'b0;
                            scan_blank_run   <= 16'd0;
                            scan_sector      <= scan_sector + 32'd1;
                            sd_sec_read_addr <= scan_sector + 32'd1;
                            state            <= ST_LOC;
                        end
                    end else if (magic2 && (hdr_samples != 20'd0) && !located2) begin
                        located2    <= 1'b1;
                        ts_start2   <= scan_sector;
                        ts_samples2 <= hdr_samples;
                        if (located1) begin
                            located <= 1'b1;
                            state   <= ST_FILL_WAIT;
                        end else begin
                            sector_any_other <= 1'b0;
                            scan_blank_run   <= 16'd0;
                            scan_sector      <= scan_sector + 32'd1;
                            sd_sec_read_addr <= scan_sector + 32'd1;
                            state            <= ST_LOC;
                        end
                    end else if (!sector_any_other) begin
                        if (scan_blank_run >= (BLANK_STOP - 16'd1)) begin
                            state <= ST_IDLE;
                        end else begin
                            scan_blank_run   <= scan_blank_run + 16'd1;
                            scan_sector      <= scan_sector + 32'd1;
                            sd_sec_read_addr <= scan_sector + 32'd1;
                            state            <= ST_LOC;
                        end
                    end else begin
                        sector_any_other <= 1'b0;
                        scan_blank_run   <= 16'd0;
                        if (scan_sector >= SCAN_MAX_SECTOR) begin
                            state <= ST_IDLE;
                        end else begin
                            scan_sector      <= scan_sector + 32'd1;
                            sd_sec_read_addr <= scan_sector + 32'd1;
                            state            <= ST_LOC;
                        end
                    end
                end
            end

            ST_FILL_WAIT: begin
                sd_sec_read <= 1'b0;
                byte_cnt    <= 9'd0;
                // 初次进入(sel_index 非法)或切图触发：加载目标文件参数，本拍不填充
                if (image_index != sel_index) begin
                    sel_index  <= image_index;
                    ts_start   <= target_start;
                    ts_samples <= target_samples;
                    cur_sector <= target_start;
                    sector_idx <= 10'd0;
                    audio_cnt  <= 20'd0;
                end else if (fifo_wrusedw <= FILL_THRESH) begin
                    sd_sec_read_addr <= cur_sector;
                    state            <= ST_FILL;
                end
            end

            ST_FILL: begin
                sd_sec_read <= 1'b1;
                if (sd_sec_read_data_valid) begin
                    byte_cnt <= byte_cnt + 9'd1;
                    if (audio_ok && !header_zone) begin
                        if (byte_cnt[0])          // 高字节：拼完整样本写 FIFO，计数+1
                            audio_cnt <= audio_cnt + 20'd1;
                        else                       // 低字节：锁存
                            sample_lo <= sd_sec_read_data;
                    end
                end

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    byte_cnt    <= 9'd0;
                    if (audio_cnt >= ts_samples) begin
                        audio_cnt <= 20'd0;
                        sector_idx <= 10'd0;
                        cur_sector <= ts_start;
                        state      <= ST_FILL_WAIT;
                    end else begin
                        sector_idx  <= sector_idx + 10'd1;
                        cur_sector  <= cur_sector + 32'd1;
                        state       <= ST_FILL_WAIT;
                    end
                end
            end

            default: begin
                state <= ST_IDLE;
            end
        endcase
    end
end

endmodule