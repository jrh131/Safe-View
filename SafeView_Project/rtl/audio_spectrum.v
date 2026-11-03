`timescale 1ns / 1ps
//============================================================================
// Module : audio_spectrum
// Purpose : Real-time 256-point FFT spectrum analyzer overlay for the OSD.
//           Replaces the time-domain oscilloscope (audio_waveform) with a
//           frequency-domain bar display: 64 log-spaced bars, rainbow colour
//           gradient (red = bass ... blue = treble).
//
// Data path:
//   I_audio_sample[7:0] is the signed instantaneous PCM amplitude (already in
//   the video pixel-clock domain). Each I_audio_valid pulse is captured into a
//   256-entry working RAM in bit-reversed order. Once 256 samples are ready a
//   radix-2 (DIT) FFT runs in-place on that RAM (natural-order output), then a
//   127-bin magnitude pass groups |Re|+|Im| into 64 log-spaced bars. The bar
//   levels feed a frame-rate attack/decay smoother, and the result is drawn as
//   coloured bars inside a rectangular region.
//
// Timing:
//   Everything runs in the pixel clock domain (I_clk). Capture is 48 kHz
//   (~5.3 ms / frame), the FFT body is ~5120 clocks (~204 us @ 25 MHz, for the
//   5-cycle pipelined butterfly with synchronous-read RAM) and the magnitude
//   pass ~130 clocks, so a full spectrum refresh comfortably fits into a video
//   frame. The output register applies a constant 1-cycle latency to the video
//   path, identical to the module it replaces.
//
// Resources:
//   Working RAM is synchronous-read block RAM; twiddle / palette / bin-map are
//   small distributed LUT ROMs; the four 24x16 butterfly products are inferred
//   as DSP multipliers.
//============================================================================

module audio_spectrum #(
    parameter H_ACTIVE    = 640,
    parameter V_ACTIVE    = 480,
    parameter DATA_WIDTH  = 24,
    parameter SPEC_X0     = 16,          // spectrum region X origin
    parameter SPEC_Y0     = 224,         // spectrum region Y origin
    parameter SPEC_W      = 512,         // region width  (power of 2)
    parameter SPEC_H      = 256,         // region height (even)
    parameter FFT_N       = 256,         // FFT length
    parameter FFT_LOG     = 8,           // log2(FFT_N)
    parameter NBAR        = 64           // number of spectrum bars
)(
    input  wire                  I_clk,
    input  wire                  I_rst,
    input  wire                  I_show,          // dip-switch: 1=show, 0=hide
    input  wire                  I_audio_valid,   // pulse on a new PCM sample
    input  wire [7:0]            I_audio_sample,  // signed 8-bit amplitude
    input  wire [DATA_WIDTH-1:0] I_rgb,
    input  wire                  I_hs,
    input  wire                  I_vs,
    input  wire                  I_de,
    output reg  [DATA_WIDTH-1:0] O_rgb,
    output reg                   O_hs,
    output reg                   O_vs,
    output reg                   O_de
);

    // Bar geometry: SPEC_W / NBAR must be a power of two (8 here).
    localparam NBAR_LOG = 6;                      // log2(64)
    localparam BAR_W    = SPEC_W / NBAR;          // 8 px per bar
    localparam [23:0] DECAY_STEP  = 24'd512;      // bar fall per video frame

    // Internal FFT data width (headroom: 8-bit input, no per-stage scaling).
    localparam F_DATA = 24;

    // Phases of the main sequencer.
    localparam PH_CAP = 2'd0;
    localparam PH_FFT = 2'd1;
    localparam PH_MAG = 2'd2;

    //========================================================================
    // Log2 helpers used by the display.
    //========================================================================
    function [4:0] msb_index;
        input [23:0] v;
        integer i;
        reg found;
        begin
            found = 0;
            msb_index = 5'd0;
            for (i = 23; i >= 0; i = i - 1) begin
                if (!found && v[i]) begin
                    msb_index = i[4:0];
                    found = 1;
                end
            end
        end
    endfunction

    function [7:0] log_height;
        input [23:0] v;
        reg [4:0] p;
        reg [1:0] fr;
        reg [10:0] h;
        reg [23:0] vs;
        begin
            p = msb_index(v);
            if (p >= 5'd2) begin
                vs = v >> (p - 5'd2);
                fr = vs[1:0];
            end else if (p == 5'd1)
                fr = {v[0], 1'b0};
            else
                fr = 2'b00;
            h = p * 24 + fr * 6;
            log_height = (h > 11'd255) ? 8'd255 : h[7:0];
        end
    endfunction

    //========================================================================
    // Look-up memories (initialised from generated data files).
    //========================================================================
    reg signed [15:0] tw_cos  [0:127];
    reg signed [15:0] tw_sin  [0:127];
    reg [7:0]         bin_bar [0:127];
    reg [23:0]        palette [0:63];
    initial begin
`include "../doc/convert/fft_twiddle_256.vh"
    end
    initial begin
`include "../doc/convert/fft_bin_to_bar_256.vh"
    end
    initial begin
`include "../doc/convert/fft_palette_64.vh"
    end

    // Bar levels and smoothing.
    reg [23:0] bar_val  [0:NBAR-1];   // current spectrum (one per FFT refresh)
    reg [23:0] disp_val [0:NBAR-1];   // attack/decay smoothed level

    //========================================================================
    // Working RAM (FFT in-place; complex = wre + j*wim)
    //   Implemented as synchronous-read block RAM (simple dual port: one
    //   shared write port + one shared read port). The original combinational
    //   accesses (3 reads + 1 write) made TD expand this into a multi-port
    //   distributed RAM, costing thousands of LUTs and congesting placement.
    //   The 1-cycle read latency is absorbed by the 5-cycle butterfly and the
    //   pipelined magnitude pass below.
    //========================================================================
    reg [F_DATA-1:0] wre [0:FFT_N-1]; /* fehdl force_ram=1, ram_style="bram" */
    reg [F_DATA-1:0] wim [0:FFT_N-1]; /* fehdl force_ram=1, ram_style="bram" */

    reg [FFT_LOG-1:0] mem_waddr;
    reg               mem_we;
    reg [F_DATA-1:0]  mem_wd_re;
    reg [F_DATA-1:0]  mem_wd_im;

    wire [FFT_LOG-1:0] mem_raddr;      // shared synchronous read-port address
    reg  [F_DATA-1:0]  mem_rd_re;
    reg  [F_DATA-1:0]  mem_rd_im;

    always @(posedge I_clk) begin
        if (mem_we) begin
            wre[mem_waddr] <= mem_wd_re;
            wim[mem_waddr] <= mem_wd_im;
        end
        mem_rd_re <= wre[mem_raddr];
        mem_rd_im <= wim[mem_raddr];
    end

    //========================================================================
    // Main sequencer: CAP -> FFT -> MAG -> CAP ...
    //========================================================================
    reg [1:0]        phase;
    reg [FFT_LOG-1:0] cap_ptr;
    reg              fft_done_r;
    reg              mag_done_r;

    wire cap_we   = (phase == PH_CAP) & I_audio_valid;
    wire cap_full = cap_we & (cap_ptr == (FFT_N-1));

    always @(posedge I_clk) begin
        if (I_rst) begin
            phase <= PH_CAP;
        end else begin
            case (phase)
                PH_CAP: if (cap_full)   phase <= PH_FFT;
                PH_FFT: if (fft_done_r) phase <= PH_MAG;
                PH_MAG: if (mag_done_r) phase <= PH_CAP;
                default:                phase <= PH_CAP;
            endcase
        end
    end

    always @(posedge I_clk) begin
        if (I_rst)
            cap_ptr <= {FFT_LOG{1'b0}};
        else if (cap_we)
            cap_ptr <= cap_ptr + 1'b1;
    end

    //========================================================================
    // FFT butterfly address / twiddle generation (combinational).
    //   stage s, butterfly counter bcnt, DIT decimation with bit-reversed
    //   input so the output comes out in natural bin order.
    //   The working RAM now has a synchronous read (1-cycle latency), so the
    //   butterfly is pipelined over 5 cycles
    //   (ADDR -> RDA -> RDB -> SUM -> DIFF). This keeps the RAM-read ->
    //   multiplier -> adder path split across two registered stages and never
    //   closes a full loop inside a single clock period.
    //========================================================================
    reg  [2:0] stage;
    reg  [6:0] bcnt;
    reg  [2:0] bf_st;                 // 0=ADDR 1=RDA 2=RDB 3=SUM 4=DIFF

    localparam BF_ADDR = 3'd0;        // issue read of addr_a; latch twiddle/addr
    localparam BF_RDA  = 3'd1;        // latch a operand; issue read of addr_b
    localparam BF_RDB  = 3'd2;        // latch b operand
    localparam BF_SUM  = 3'd3;        // write sum to addr_a; latch diff
    localparam BF_DIFF = 3'd4;        // write diff to addr_b; advance stage/bcnt

    reg signed [F_DATA-1:0] lat_a_re; // operands latched in the READ stage
    reg signed [F_DATA-1:0] lat_a_im;
    reg signed [F_DATA-1:0] lat_b_re;
    reg signed [F_DATA-1:0] lat_b_im;
    reg signed [15:0]       lat_tw_c;
    reg signed [15:0]       lat_tw_s;
    reg [FFT_LOG-1:0]       lat_addr_a;
    reg [FFT_LOG-1:0]       lat_addr_b;
    reg signed [F_DATA-1:0] diff_re;  // latched difference (written second)
    reg signed [F_DATA-1:0] diff_im;

    wire [7:0] half    = (1 << stage);                    // half-span = 2^stage
    wire [6:0] jpos    = bcnt & (half - 1'b1);
    wire [7:0] grp     = {1'b0, bcnt} >> stage;
    wire [3:0] shift_a = {1'b0, stage} + 1'b1;            // 1..8 (wide enough)
    wire [7:0] addr_a  = (grp << shift_a) | {1'b0, jpos};
    wire [7:0] addr_b  = addr_a + half;
    wire [6:0] tw_idx  = jpos << (7 - stage);             // twiddle index k

    wire signed [15:0] tw_c = tw_cos[tw_idx];
    wire signed [15:0] tw_s = tw_sin[tw_idx];

    // Complex multiply b * W, W = cos - j*sin (Q1.15), on latched operands.
    wire signed [39:0] m_rr  = lat_b_re * lat_tw_c;
    wire signed [39:0] m_ii  = lat_b_im * lat_tw_s;
    wire signed [39:0] m_ri  = lat_b_re * lat_tw_s;
    wire signed [39:0] m_ir  = lat_b_im * lat_tw_c;
    wire signed [39:0] tre40 = m_rr - m_ii;               // Re{b*W}
    wire signed [39:0] tim40 = m_ri + m_ir;               // Im{b*W}

    wire signed [F_DATA-1:0] tre = tre40 >>> 15;          // drop Q1.15 scale
    wire signed [F_DATA-1:0] tim = tim40 >>> 15;

    wire signed [F_DATA:0] sre = {lat_a_re[F_DATA-1], lat_a_re} + {tre[F_DATA-1], tre};
    wire signed [F_DATA:0] sim = {lat_a_im[F_DATA-1], lat_a_im} + {tim[F_DATA-1], tim};
    wire signed [F_DATA:0] dre = {lat_a_re[F_DATA-1], lat_a_re} - {tre[F_DATA-1], tre};
    wire signed [F_DATA:0] dim = {lat_a_im[F_DATA-1], lat_a_im} - {tim[F_DATA-1], tim};

    always @(posedge I_clk) begin
        if (I_rst || (phase != PH_FFT)) begin
            stage      <= 3'd0;
            bcnt       <= 7'd0;
            bf_st      <= BF_ADDR;
            fft_done_r <= 1'b0;
            lat_a_re   <= {F_DATA{1'b0}};
            lat_a_im   <= {F_DATA{1'b0}};
            lat_b_re   <= {F_DATA{1'b0}};
            lat_b_im   <= {F_DATA{1'b0}};
            lat_tw_c   <= 16'sd0;
            lat_tw_s   <= 16'sd0;
            lat_addr_a <= {FFT_LOG{1'b0}};
            lat_addr_b <= {FFT_LOG{1'b0}};
            diff_re    <= {F_DATA{1'b0}};
            diff_im    <= {F_DATA{1'b0}};
        end else if (fft_done_r) begin
            fft_done_r <= 1'b1;                 // hold until phase handoff
        end else begin
            case (bf_st)
                BF_ADDR: begin                   // latch twiddle + both addresses
                    lat_tw_c   <= tw_c;
                    lat_tw_s   <= tw_s;
                    lat_addr_a <= addr_a[FFT_LOG-1:0];
                    lat_addr_b <= addr_b[FFT_LOG-1:0];
                    bf_st      <= BF_RDA;        // addr_a read was issued this cycle
                end
                BF_RDA: begin                    // latch operand a (= wre[addr_a])
                    lat_a_re   <= mem_rd_re;
                    lat_a_im   <= mem_rd_im;
                    bf_st      <= BF_RDB;        // addr_b read was issued this cycle
                end
                BF_RDB: begin                    // latch operand b (= wre[addr_b])
                    lat_b_re   <= mem_rd_re;
                    lat_b_im   <= mem_rd_im;
                    bf_st      <= BF_SUM;
                end
                BF_SUM: begin                    // write sum to addr_a, latch diff
                    diff_re    <= dre[F_DATA-1:0];
                    diff_im    <= dim[F_DATA-1:0];
                    bf_st      <= BF_DIFF;
                end
                default: begin                   // BF_DIFF : write diff, advance
                    bf_st <= BF_ADDR;
                    if (bcnt == 7'd127) begin
                        bcnt <= 7'd0;
                        if (stage == 3'd7) begin
                            stage      <= 3'd0;
                            fft_done_r <= 1'b1;
                        end else begin
                            stage <= stage + 1'b1;
                        end
                    end else begin
                        bcnt <= bcnt + 1'b1;
                    end
                end
            endcase
        end
    end

    wire fft_we = (phase == PH_FFT) & ~fft_done_r &
                  ((bf_st == BF_SUM) | (bf_st == BF_DIFF));
    wire [FFT_LOG-1:0] fft_waddr = (bf_st == BF_SUM) ? lat_addr_a : lat_addr_b;
    wire [F_DATA-1:0]  fft_wd_re = (bf_st == BF_SUM) ? sre[F_DATA-1:0] : diff_re;
    wire [F_DATA-1:0]  fft_wd_im = (bf_st == BF_SUM) ? sim[F_DATA-1:0] : diff_im;

    //========================================================================
    // Capture write port (bit-reversed addressing).
    //========================================================================
    wire [FFT_LOG-1:0] cap_waddr = {cap_ptr[0], cap_ptr[1], cap_ptr[2], cap_ptr[3],
                                    cap_ptr[4], cap_ptr[5], cap_ptr[6], cap_ptr[7]};
    wire [F_DATA-1:0]  cap_wd_re = {{(F_DATA-8){I_audio_sample[7]}}, I_audio_sample};
    wire [F_DATA-1:0]  cap_wd_im = {F_DATA{1'b0}};

    always @* begin
        if (cap_we) begin
            mem_we    = 1'b1;
            mem_waddr = cap_waddr;
            mem_wd_re = cap_wd_re;
            mem_wd_im = cap_wd_im;
        end else if (fft_we) begin
            mem_we    = 1'b1;
            mem_waddr = fft_waddr;
            mem_wd_re = fft_wd_re;
            mem_wd_im = fft_wd_im;
        end else begin
            mem_we    = 1'b0;
            mem_waddr = {FFT_LOG{1'b0}};
            mem_wd_re = {F_DATA{1'b0}};
            mem_wd_im = {F_DATA{1'b0}};
        end
    end

    // Shared synchronous read-port address (sampled by the block RAM):
    //   - FFT ADDR state reads addr_a, RDA state reads addr_b;
    //   - magnitude pass issues one bin (mag_bin) per cycle.
    assign mem_raddr =
        (phase == PH_FFT && !fft_done_r) ?
            ((bf_st == BF_ADDR) ? addr_a :
             (bf_st == BF_RDA)  ? addr_b : {FFT_LOG{1'b0}}) :
        (phase == PH_MAG) ? mag_bin :
        {FFT_LOG{1'b0}};

    //========================================================================
    // Magnitude scan: for bins 1..127 compute |Re|+|Im| and accumulate into
    // the matching log-spaced bar.
    //========================================================================
    reg [7:0]  mag_bin;        // bin being issued to the read port this cycle
    reg [7:0]  mag_bin_q;      // bin whose data is currently on mem_rd_*
    reg        finalize;
    reg [NBAR_LOG-1:0] cur_bar;
    reg [23:0] cur_sum;

    wire signed [F_DATA-1:0] mre = mem_rd_re;
    wire signed [F_DATA-1:0] mim = mem_rd_im;
    wire [F_DATA-1:0] abs_re    = mre[F_DATA-1] ? (~mre + 1'b1) : mre;
    wire [F_DATA-1:0] abs_im    = mim[F_DATA-1] ? (~mim + 1'b1) : mim;
    wire [F_DATA-1:0] mag       = abs_re + abs_im;   // < 2^15 in practice
    wire [7:0]        bar_d     = bin_bar[mag_bin_q[6:0]];

    always @(posedge I_clk) begin
        if (I_rst) begin
            mag_bin    <= 8'd0;
            mag_bin_q  <= 8'd0;
            finalize   <= 1'b0;
            cur_bar    <= {NBAR_LOG{1'b0}};
            cur_sum    <= 24'd0;
            mag_done_r <= 1'b0;
        end else if (phase != PH_MAG) begin
            mag_bin    <= 8'd0;
            mag_bin_q  <= 8'd0;
            finalize   <= 1'b0;
            cur_bar    <= {NBAR_LOG{1'b0}};
            cur_sum    <= 24'd0;
            mag_done_r <= 1'b0;
        end else begin
            mag_done_r <= 1'b0;
            mag_bin_q  <= mag_bin;

            if (finalize) begin
                bar_val[cur_bar] <= cur_sum;
                mag_done_r       <= 1'b1;
                finalize         <= 1'b0;
                mag_bin          <= 8'd0;
                mag_bin_q        <= 8'd0;
            end else begin
                // Advance the issued read address (0..127 then hold).
                if (mag_bin != 8'd127)
                    mag_bin <= mag_bin + 1'b1;

                // Accumulate the bin whose data is now valid (= mag_bin_q).
                if (mag_bin_q == 8'd0) begin
                    ;                              // DC bin: ignore
                end else if (mag_bin_q == 8'd1) begin
                    cur_bar <= bar_d[5:0];
                    cur_sum <= mag;
                end else if (bar_d[5:0] != cur_bar) begin
                    bar_val[cur_bar] <= cur_sum;
                    cur_bar          <= bar_d[5:0];
                    cur_sum          <= mag;
                end else begin
                    cur_sum <= cur_sum + mag;
                end

                if (mag_bin_q == 8'd127)
                    finalize <= 1'b1;
            end
        end
    end

    //========================================================================
    // Frame-rate attack / decay (scans 64 bars / frame).
    //========================================================================
    reg [NBAR_LOG-1:0] dptr;
    reg                decaying;

    always @(posedge I_clk) begin
        if (I_rst) begin
            dptr     <= {NBAR_LOG{1'b0}};
            decaying <= 1'b0;
        end else if (vs_falling) begin
            dptr     <= {NBAR_LOG{1'b0}};
            decaying <= 1'b1;
        end else if (decaying) begin
            if (bar_val[dptr] > disp_val[dptr])
                disp_val[dptr] <= bar_val[dptr];
            else if (disp_val[dptr] > DECAY_STEP)
                disp_val[dptr] <= disp_val[dptr] - DECAY_STEP;
            else
                disp_val[dptr] <= 24'd0;

            if (dptr == (NBAR-1))
                decaying <= 1'b0;
            else
                dptr <= dptr + 1'b1;
        end
    end

    //========================================================================
    // Pixel coordinate generation (aligned with the incoming video stream).
    //========================================================================
    reg        de_r;
    reg        vs_r;
    reg [9:0]  pos_x;
    reg [9:0]  pos_y;

    wire de_falling = de_r & ~I_de;
    wire vs_falling = vs_r & ~I_vs;

    always @(posedge I_clk) begin
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
    // Region / bar rendering.
    //========================================================================
    wire in_region = I_show &
                     (pos_x >= SPEC_X0) & (pos_x < (SPEC_X0 + SPEC_W)) &
                     (pos_y >= SPEC_Y0) & (pos_y < (SPEC_Y0 + SPEC_H));

    wire [9:0] xoff = pos_x - SPEC_X0;
    wire [7:0] yoff = pos_y - SPEC_Y0;
    wire [NBAR_LOG-1:0] bar_x = xoff[9:3];

    wire [23:0] dv = disp_val[bar_x];
    wire [7:0]  hgt  = log_height(dv);

    wire [8:0] body_top = (SPEC_H - {1'b0, hgt});
    wire [8:0] yoff9    = {1'b0, yoff};

    wire hit_body = in_region & (yoff9 >= body_top);
    wire [23:0] bar_color = palette[bar_x];

    //========================================================================
    // Output register: constant 1-cycle latency for the whole video path.
    //========================================================================
    always @(posedge I_clk) begin
        if (I_rst) begin
            O_rgb <= {DATA_WIDTH{1'b0}};
            O_hs  <= 1'b0;
            O_vs  <= 1'b0;
            O_de  <= 1'b0;
        end else begin
            if (hit_body)
                O_rgb <= bar_color;
            else
                O_rgb <= I_rgb;
            O_hs  <= I_hs;
            O_vs  <= I_vs;
            O_de  <= I_de;
        end
    end

endmodule