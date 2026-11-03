`timescale 1ns/1ps

// Four-image linked PCM player.
// ROM clips: signed 8-bit mono PCM at 4 kHz, replayed at 48 kHz by holding
// each source sample for twelve HDMI audio frames (idx0/1/3/4/5).
// Stream clip (idx2): signed 16-bit mono PCM at 16 kHz, held for three frames.
module image_linked_audio_pcm_i2s (
    input  wire       I_mclk,
    input  wire       I_rst,
    input  wire       I_image_valid,
    input  wire [2:0] I_image_index,
    input  wire [3:0] I_volume_level,
    input  wire       I_alarm,          // 地震告警（video_clk 域，本模块内 2 级同步）
    input  wire [15:0] I_stream_dout,   // 异步 FIFO 读数据（audio_mclk 域，16bit 样本）
    input  wire       I_stream_valid,   // 异步 FIFO 非空（SHOW_AHEAD 模式 = !empty_flag）
    output reg        O_stream_re,      // 异步 FIFO 读使能（每个源样本一拍）
    output reg        O_i2s_BCLK,
    output reg        O_i2s_LRCK,
    output reg        O_i2s_DOUT
);

localparam integer AUDIO_DEPTH = 30240;
// ttsmaker 流播源：idx2=ttsmaker_audio.bin(TSA1,634752样本)，idx3=ttsmaker_audio2.bin(TSA2,358272样本)
localparam integer STREAM_SAMPLE_COUNT = 634752;
localparam [19:0]  STREAM_CNT_MAX      = 20'd634751;  // = STREAM_SAMPLE_COUNT - 1 (idx2)
localparam [19:0]  STREAM_CNT_MAX_2    = 20'd358271;  // = 358272 - 1 (idx3)
localparam [17:0]  GAP_FRAME_COUNT     = 18'd240000;  // 5 s @ 48 kHz
localparam [17:0]  GAP_CNT_MAX         = 18'd239999;  // = GAP_FRAME_COUNT - 1
localparam [14:0] START_0 = 15'd0;
localparam [14:0] END_0   = 15'd6048;
localparam [14:0] START_1 = 15'd6048;
localparam [14:0] END_1   = 15'd12192;
localparam [14:0] START_2 = 15'd12192;
localparam [14:0] END_2   = 15'd18240;
localparam [14:0] START_3 = 15'd18240;
localparam [14:0] END_3   = 15'd30240;

function [14:0] clip_start;
    input [2:0] idx;
    begin
        case (idx)
            3'd0: clip_start = START_0;
            3'd1: clip_start = START_1;
            3'd2: clip_start = START_2;
            3'd3: clip_start = START_3;
            3'd4: clip_start = START_0;
            3'd5: clip_start = START_1;
            default: clip_start = START_0;
        endcase
    end
endfunction

function [14:0] clip_end;
    input [2:0] idx;
    begin
        case (idx)
            3'd0: clip_end = END_0;
            3'd1: clip_end = END_1;
            3'd2: clip_end = END_2;
            3'd3: clip_end = END_3;
            3'd4: clip_end = END_0;
            3'd5: clip_end = END_1;
            default: clip_end = END_0;
        endcase
    end
endfunction

reg       S_valid_sync_1;
reg       S_valid_sync_2;
reg [2:0] S_index_sync_1;
reg [2:0] S_index_sync_2;
reg [2:0] S_active_index;
reg       S_have_image;
reg       S_playing;
reg [14:0] S_rom_addr;
reg [14:0] S_rom_end;
reg [3:0] S_upsample_cnt;
reg [5:0] S_bit_cnt;
reg [63:0] S_shift_reg;
reg signed [23:0] S_sample_word;
reg [2:0] S_bank_sel_q;
reg signed [7:0] S_rom_q;
// 流播状态（idx2/idx3 专属）
reg        S_stream_gap;            // 1 = 正在 5 秒间隔
reg [19:0] S_stream_cnt;            // 本轮已播源样本计数
reg [17:0] S_gap_cnt;               // 间隔内已过 48kHz 帧数
reg signed [15:0] S_stream_q;       // 当前流播源样本（FIFO 读数据寄存器，16bit）
reg [3:0] S_volume_sync_1;
reg [3:0] S_volume_sync_2;
reg       S_alarm_sync_1;
reg       S_alarm_sync_2;
// 告警音 DDS 状态
reg [7:0]  S_tone_div;
reg [31:0] S_tone_phase;
reg [31:0] S_tone_freq;
reg        S_tone_up;
reg [15:0] S_tone_on_cnt;
reg        S_tone_on;
reg signed [23:0] S_tone_smp;
wire [7:0] W_bank_q0;
wire [7:0] W_bank_q1;
wire [7:0] W_bank_q2;
wire [7:0] W_bank_q3;
wire [7:0] W_bank_q4;
wire [7:0] W_bank_q5;
wire [7:0] W_bank_q6;
wire [7:0] W_bank_q7;
wire stream_mode = (S_active_index == 3'd2) || (S_active_index == 3'd3);   // idx2/idx3 用 TF 流播（16bit 源）
wire [19:0] W_stream_cnt_max = (S_active_index == 3'd3) ? STREAM_CNT_MAX_2 : STREAM_CNT_MAX;
// ROM 8bit 样本放大到满量程附近（符号扩展后算术左移 15 位，约 -6dBFS，与告警/ROM 播放器一致）
wire signed [23:0] W_rom_pcm24    = {{16{S_rom_q[7]}}, S_rom_q} <<< 15;
// 流播 16bit 样本左对齐到 24bit 满量程：{样本, 低8位补0} 等价于左移 8 位
wire signed [23:0] W_stream_pcm24 = {S_stream_q, 8'd0};
wire signed [23:0] W_pcm24_full   = stream_mode ? W_stream_pcm24 : W_rom_pcm24;
wire signed [27:0] W_pcm_scaled = W_pcm24_full * $signed({1'b0, S_volume_sync_2});
wire signed [23:0] W_pcm24 = (S_volume_sync_2 == 4'd15) ?
                             W_pcm24_full : (W_pcm_scaled >>> 4);
// 流播间隔期间静音
wire signed [23:0] W_pcm24_play = (stream_mode && S_stream_gap) ? 24'sd0 : W_pcm24;

// ---- 地震告警音 DDS 常量与输出 ----
localparam [31:0] TONE_FMIN     = 32'd89478485;   // 1.0kHz @48k
localparam [31:0] TONE_FMAX     = 32'd125269879;  // 1.4kHz @48k
localparam [15:0] TONE_ON_CNT   = 16'd4800;       // 100ms @48k
localparam signed [23:0] TONE_AMP = 24'sd3500000;

wire tone_smp_en = (S_tone_div == 8'd255);        // 每 256 mclk 一拍 = 48kHz

// 告警时用方波覆盖 PCM，否则沿用原 PCM 样本
wire signed [23:0] W_smp_cur = S_alarm_sync_2 ? S_tone_smp : S_sample_word;
wire signed [23:0] W_smp_new = S_alarm_sync_2 ? S_tone_smp : W_pcm24_play;

// Eight explicit 32-Kbit ROM blocks. Each block stores 4096 x 8-bit samples.
// INIT_FILE is handled by the Anlogic primitive and is embedded in bitstream.
EG_LOGIC_BRAM #(
    .DATA_WIDTH_A(8), .ADDR_WIDTH_A(12), .DATA_DEPTH_A(4096),
    .DATA_WIDTH_B(8), .ADDR_WIDTH_B(12), .DATA_DEPTH_B(4096),
    .MODE("SP"), .REGMODE_A("NOREG"), .IMPLEMENT("32K"),
    .INIT_FILE("audio/image_audio_bank0.mif"), .FILL_ALL("NONE")
) u_audio_rom_0 (
    .doa(W_bank_q0), .addra(S_rom_addr[11:0]), .clka(I_mclk),
    .dia(8'd0), .cea(1'b1), .ocea(1'b0), .wea(1'b0), .rsta(1'b0), .bea(1'b0),
    .dob(), .addrb(12'd0), .clkb(1'b0), .dib(8'd0), .ceb(1'b0),
    .oceb(1'b0), .web(1'b0), .rstb(1'b0), .beb(1'b0)
);
EG_LOGIC_BRAM #(
    .DATA_WIDTH_A(8), .ADDR_WIDTH_A(12), .DATA_DEPTH_A(4096),
    .DATA_WIDTH_B(8), .ADDR_WIDTH_B(12), .DATA_DEPTH_B(4096),
    .MODE("SP"), .REGMODE_A("NOREG"), .IMPLEMENT("32K"),
    .INIT_FILE("audio/image_audio_bank1.mif"), .FILL_ALL("NONE")
) u_audio_rom_1 (
    .doa(W_bank_q1), .addra(S_rom_addr[11:0]), .clka(I_mclk),
    .dia(8'd0), .cea(1'b1), .ocea(1'b0), .wea(1'b0), .rsta(1'b0), .bea(1'b0),
    .dob(), .addrb(12'd0), .clkb(1'b0), .dib(8'd0), .ceb(1'b0),
    .oceb(1'b0), .web(1'b0), .rstb(1'b0), .beb(1'b0)
);
EG_LOGIC_BRAM #(
    .DATA_WIDTH_A(8), .ADDR_WIDTH_A(12), .DATA_DEPTH_A(4096),
    .DATA_WIDTH_B(8), .ADDR_WIDTH_B(12), .DATA_DEPTH_B(4096),
    .MODE("SP"), .REGMODE_A("NOREG"), .IMPLEMENT("32K"),
    .INIT_FILE("audio/image_audio_bank2.mif"), .FILL_ALL("NONE")
) u_audio_rom_2 (
    .doa(W_bank_q2), .addra(S_rom_addr[11:0]), .clka(I_mclk),
    .dia(8'd0), .cea(1'b1), .ocea(1'b0), .wea(1'b0), .rsta(1'b0), .bea(1'b0),
    .dob(), .addrb(12'd0), .clkb(1'b0), .dib(8'd0), .ceb(1'b0),
    .oceb(1'b0), .web(1'b0), .rstb(1'b0), .beb(1'b0)
);
EG_LOGIC_BRAM #(
    .DATA_WIDTH_A(8), .ADDR_WIDTH_A(12), .DATA_DEPTH_A(4096),
    .DATA_WIDTH_B(8), .ADDR_WIDTH_B(12), .DATA_DEPTH_B(4096),
    .MODE("SP"), .REGMODE_A("NOREG"), .IMPLEMENT("32K"),
    .INIT_FILE("audio/image_audio_bank3.mif"), .FILL_ALL("NONE")
) u_audio_rom_3 (
    .doa(W_bank_q3), .addra(S_rom_addr[11:0]), .clka(I_mclk),
    .dia(8'd0), .cea(1'b1), .ocea(1'b0), .wea(1'b0), .rsta(1'b0), .bea(1'b0),
    .dob(), .addrb(12'd0), .clkb(1'b0), .dib(8'd0), .ceb(1'b0),
    .oceb(1'b0), .web(1'b0), .rstb(1'b0), .beb(1'b0)
);
EG_LOGIC_BRAM #(
    .DATA_WIDTH_A(8), .ADDR_WIDTH_A(12), .DATA_DEPTH_A(4096),
    .DATA_WIDTH_B(8), .ADDR_WIDTH_B(12), .DATA_DEPTH_B(4096),
    .MODE("SP"), .REGMODE_A("NOREG"), .IMPLEMENT("32K"),
    .INIT_FILE("audio/image_audio_bank4.mif"), .FILL_ALL("NONE")
) u_audio_rom_4 (
    .doa(W_bank_q4), .addra(S_rom_addr[11:0]), .clka(I_mclk),
    .dia(8'd0), .cea(1'b1), .ocea(1'b0), .wea(1'b0), .rsta(1'b0), .bea(1'b0),
    .dob(), .addrb(12'd0), .clkb(1'b0), .dib(8'd0), .ceb(1'b0),
    .oceb(1'b0), .web(1'b0), .rstb(1'b0), .beb(1'b0)
);
EG_LOGIC_BRAM #(
    .DATA_WIDTH_A(8), .ADDR_WIDTH_A(12), .DATA_DEPTH_A(4096),
    .DATA_WIDTH_B(8), .ADDR_WIDTH_B(12), .DATA_DEPTH_B(4096),
    .MODE("SP"), .REGMODE_A("NOREG"), .IMPLEMENT("32K"),
    .INIT_FILE("audio/image_audio_bank5.mif"), .FILL_ALL("NONE")
) u_audio_rom_5 (
    .doa(W_bank_q5), .addra(S_rom_addr[11:0]), .clka(I_mclk),
    .dia(8'd0), .cea(1'b1), .ocea(1'b0), .wea(1'b0), .rsta(1'b0), .bea(1'b0),
    .dob(), .addrb(12'd0), .clkb(1'b0), .dib(8'd0), .ceb(1'b0),
    .oceb(1'b0), .web(1'b0), .rstb(1'b0), .beb(1'b0)
);
EG_LOGIC_BRAM #(
    .DATA_WIDTH_A(8), .ADDR_WIDTH_A(12), .DATA_DEPTH_A(4096),
    .DATA_WIDTH_B(8), .ADDR_WIDTH_B(12), .DATA_DEPTH_B(4096),
    .MODE("SP"), .REGMODE_A("NOREG"), .IMPLEMENT("32K"),
    .INIT_FILE("audio/image_audio_bank6.mif"), .FILL_ALL("NONE")
) u_audio_rom_6 (
    .doa(W_bank_q6), .addra(S_rom_addr[11:0]), .clka(I_mclk),
    .dia(8'd0), .cea(1'b1), .ocea(1'b0), .wea(1'b0), .rsta(1'b0), .bea(1'b0),
    .dob(), .addrb(12'd0), .clkb(1'b0), .dib(8'd0), .ceb(1'b0),
    .oceb(1'b0), .web(1'b0), .rstb(1'b0), .beb(1'b0)
);
EG_LOGIC_BRAM #(
    .DATA_WIDTH_A(8), .ADDR_WIDTH_A(12), .DATA_DEPTH_A(4096),
    .DATA_WIDTH_B(8), .ADDR_WIDTH_B(12), .DATA_DEPTH_B(4096),
    .MODE("SP"), .REGMODE_A("NOREG"), .IMPLEMENT("32K"),
    .INIT_FILE("audio/image_audio_bank7.mif"), .FILL_ALL("NONE")
) u_audio_rom_7 (
    .doa(W_bank_q7), .addra(S_rom_addr[11:0]), .clka(I_mclk),
    .dia(8'd0), .cea(1'b1), .ocea(1'b0), .wea(1'b0), .rsta(1'b0), .bea(1'b0),
    .dob(), .addrb(12'd0), .clkb(1'b0), .dib(8'd0), .ceb(1'b0),
    .oceb(1'b0), .web(1'b0), .rstb(1'b0), .beb(1'b0)
);

always @(posedge I_mclk) begin
    S_bank_sel_q <= S_rom_addr[14:12];
    case (S_bank_sel_q)
        3'd0: S_rom_q <= W_bank_q0;
        3'd1: S_rom_q <= W_bank_q1;
        3'd2: S_rom_q <= W_bank_q2;
        3'd3: S_rom_q <= W_bank_q3;
        3'd4: S_rom_q <= W_bank_q4;
        3'd5: S_rom_q <= W_bank_q5;
        3'd6: S_rom_q <= W_bank_q6;
        default: S_rom_q <= W_bank_q7;
    endcase
end

// 流播源样本：持续锁存 FIFO 队首数据（SHOW_AHEAD，re 弹出后下一拍更新）
always @(posedge I_mclk or posedge I_rst) begin
    if (I_rst)
        S_stream_q <= 16'sd0;
    else if (I_stream_valid)
        S_stream_q <= I_stream_dout;
end

always @(posedge I_mclk or posedge I_rst) begin
    if (I_rst) begin
        S_valid_sync_1 <= 1'b0;
        S_valid_sync_2 <= 1'b0;
        S_index_sync_1 <= 3'd0;
        S_index_sync_2 <= 3'd0;
        S_volume_sync_1 <= 4'd15;
        S_volume_sync_2 <= 4'd15;
        S_alarm_sync_1  <= 1'b0;
        S_alarm_sync_2  <= 1'b0;
    end else begin
        S_valid_sync_1 <= I_image_valid;
        S_valid_sync_2 <= S_valid_sync_1;
        S_index_sync_1 <= I_image_index;
        S_index_sync_2 <= S_index_sync_1;
        S_volume_sync_1 <= I_volume_level;
        S_volume_sync_2 <= S_volume_sync_1;
        S_alarm_sync_1  <= I_alarm;
        S_alarm_sync_2  <= S_alarm_sync_1;
    end
end

// ---- 告警音 DDS：方波 + 三角扫频(1.0k<->1.4k) + 100ms 通断 ----
always @(posedge I_mclk or posedge I_rst) begin
    if (I_rst) begin
        S_tone_div     <= 8'd0;
        S_tone_phase   <= 32'd0;
        S_tone_freq    <= TONE_FMIN;
        S_tone_up      <= 1'b1;
        S_tone_on_cnt  <= 16'd0;
        S_tone_on      <= 1'b0;
        S_tone_smp     <= 24'sd0;
    end else begin
        S_tone_div <= S_tone_div + 8'd1;
        if (tone_smp_en) begin
            if (S_tone_up) begin
                if (S_tone_freq >= TONE_FMAX) begin
                    S_tone_freq <= TONE_FMAX;
                    S_tone_up   <= 1'b0;
                end else
                    S_tone_freq <= S_tone_freq + 32'd900;
            end else begin
                if (S_tone_freq <= TONE_FMIN) begin
                    S_tone_freq <= TONE_FMIN;
                    S_tone_up   <= 1'b1;
                end else
                    S_tone_freq <= S_tone_freq - 32'd900;
            end
            if (S_tone_on_cnt >= TONE_ON_CNT - 16'd1) begin
                S_tone_on_cnt <= 16'd0;
                S_tone_on     <= ~S_tone_on;
            end else
                S_tone_on_cnt <= S_tone_on_cnt + 16'd1;
            S_tone_phase <= S_tone_phase + S_tone_freq;
            S_tone_smp   <= (S_alarm_sync_2 && S_tone_on) ?
                            (S_tone_phase[31] ? TONE_AMP : -TONE_AMP) : 24'sd0;
        end
    end
end

always @(posedge I_mclk or posedge I_rst) begin
    if (I_rst) begin
        O_i2s_BCLK     <= 1'b0;
        O_i2s_LRCK     <= 1'b0;
        O_i2s_DOUT     <= 1'b0;
        O_stream_re    <= 1'b0;
        S_active_index <= 3'd0;
        S_have_image   <= 1'b0;
        S_playing      <= 1'b0;
        S_rom_addr     <= 15'd0;
        S_rom_end      <= 15'd0;
        S_upsample_cnt <= 4'd0;
        S_bit_cnt      <= 6'd0;
        S_shift_reg    <= 64'd0;
        S_sample_word  <= 24'sd0;
        S_stream_gap   <= 1'b0;
        S_stream_cnt   <= 20'd0;
        S_gap_cnt      <= 18'd0;
    end else begin
        O_i2s_BCLK <= ~O_i2s_BCLK;
        O_stream_re <= 1'b0;   // 默认不弹 FIFO，仅在流播推进拍置 1

        if (O_i2s_BCLK == 1'b1) begin
            O_i2s_DOUT  <= S_shift_reg[63];
            S_shift_reg <= {S_shift_reg[62:0], 1'b0};

            if (S_bit_cnt == 6'd63) begin
                S_bit_cnt <= 6'd0;
                if (O_i2s_LRCK == 1'b0) begin
                    O_i2s_LRCK  <= 1'b1;
                    S_shift_reg <= {W_smp_cur, 40'd0};
                end else begin
                    O_i2s_LRCK <= 1'b0;

                    if (!S_valid_sync_2) begin
                        S_have_image   <= 1'b0;
                        S_playing      <= 1'b0;
                        S_sample_word  <= 24'sd0;
                        S_upsample_cnt <= 4'd0;
                        S_shift_reg    <= (S_alarm_sync_2 ? {S_tone_smp, 40'd0} : 64'd0);
                    end else if (!S_have_image ||
                                 (S_index_sync_2 != S_active_index)) begin
                        S_have_image   <= 1'b1;
                        S_active_index <= S_index_sync_2;
                        S_rom_addr     <= clip_start(S_index_sync_2);
                        S_rom_end      <= clip_end(S_index_sync_2);
                        S_upsample_cnt <= 4'd0;
                        S_sample_word  <= 24'sd0;
                        S_shift_reg    <= (S_alarm_sync_2 ? {S_tone_smp, 40'd0} : 64'd0);
                        // 按图索音：idx2=TF流播(TSA1) / idx3=TF流播(TSA2) / 其余(idx0,1,4,5)=静音
                        if ((S_index_sync_2 == 3'd2) || (S_index_sync_2 == 3'd3)) begin
                            S_playing    <= 1'b1;
                            S_stream_gap <= 1'b0;
                            S_stream_cnt <= 20'd0;
                            S_gap_cnt    <= 18'd0;
                        end else begin
                            S_playing    <= 1'b0;
                        end
                    end else if (S_playing) begin
                        S_sample_word <= W_pcm24_play;
                        S_shift_reg   <= {W_smp_new, 40'd0};

                        // 5 秒间隔：按 48kHz 帧计数（每帧一次），满 5s 回播
                        if (stream_mode && S_stream_gap) begin
                            if (S_gap_cnt >= GAP_CNT_MAX) begin
                                S_gap_cnt    <= 18'd0;
                                S_stream_gap <= 1'b0;
                                S_stream_cnt <= 20'd0;
                            end else begin
                                S_gap_cnt <= S_gap_cnt + 18'd1;
                            end
                        end

                        if (S_upsample_cnt == 4'd2) begin
                            S_upsample_cnt <= 4'd0;
                            if (stream_mode) begin
                                // 每 3 帧推进一个源样本；播满整段进入 5 秒间隔
                                if (!S_stream_gap) begin
                                    if (S_stream_cnt >= W_stream_cnt_max) begin
                                        S_stream_gap <= 1'b1;
                                        S_gap_cnt    <= 18'd0;
                                    end else begin
                                        S_stream_cnt <= S_stream_cnt + 20'd1;
                                        O_stream_re  <= 1'b1;   // 弹出 FIFO 下一个样本
                                    end
                                end
                            end else begin
                                // ROM clip3（idx3）原逻辑
                                if ((S_rom_addr + 15'd1) >= S_rom_end) begin
                                    S_playing     <= 1'b0;
                                    S_sample_word <= 24'sd0;
                                end else begin
                                    S_rom_addr <= S_rom_addr + 15'd1;
                                end
                            end
                        end else begin
                            S_upsample_cnt <= S_upsample_cnt + 4'd1;
                        end
                    end else begin
                        S_sample_word <= 24'sd0;
                        S_shift_reg   <= (S_alarm_sync_2 ? {S_tone_smp, 40'd0} : 64'd0);
                    end
                end
            end else begin
                S_bit_cnt <= S_bit_cnt + 6'd1;
            end
        end
    end
end

endmodule
