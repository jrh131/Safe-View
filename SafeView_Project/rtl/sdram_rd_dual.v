`timescale 1ns/1ps
//============================================================================
// Module : sdram_rd_dual
// Description : Dual-stream SDRAM read engine for image transition effects.
//
// This module extends the baseline single-stream read engine
// (frame_fifo_read) to support reading two frame buffers (A and B)
// simultaneously within one frame period. It issues bursts in an
// A, B, A, B, ... interleaved pattern so that both frame FIFOs receive
// data in lockstep, enabling pixel-aligned cross-fade or wipe effects.
//
// == Intra-frame burst sequence ==
// Bursts alternate between stream A and stream B: A, B, A, B, ...
// Each burst transfers BURST_SIZE words (default 256).
// Each stream reads read_len words total.
//   N_BURST_PER_STREAM = read_len / BURST_SIZE  (e.g. 307200/256 = 1200)
//   Total bursts per frame = 2 * N_BURST_PER_STREAM (dual mode)
//                         = N_BURST_PER_STREAM      (single mode, A only)
//
// == Two-stream pixel lockstep -- physical basis ==
// This module exists because the transition effect requires reading the
// same pixel index from both frames in the same cycle. Lockstep is
// guaranteed by three properties:
//   1. Both read FIFOs share one O_fifo_aclr (cleared together);
//   2. The consumer uses one read_en to pop one word from each FIFO;
//   3. Each FIFO receives the same number of BURST_SIZE blocks in order.
// The design does NOT depend on the exact latency between O_app_rd_en
// and I_sdr_rd_en. It only depends on "each O_app_rd_en produces
// exactly one burst of BURST_SIZE I_sdr_rd_en returns, in order" --
// the same assumption the baseline frame_fifo_read + rfifo already
// relied upon implicitly.
//
// == Return-count demux ==
// A counter (ret_cnt) tracks I_sdr_rd_en pulses within each burst
// (0 .. BURST_SIZE-1). A selector (ret_sel) routes each return to
// FIFO A (ret_sel=0) or FIFO B (ret_sel=1). ret_sel flips after
// every complete BURST_SIZE return block.
//
// CRITICAL: ret_cnt and ret_sel are aligned to stream A in S_ACK
// (the frame start). This is safe because the FSM can only reach
// S_ACK after every return of the previous burst has drained --
// rd_burst_finish requires rd_delay==10 to elapse after the last
// O_app_rd_en, which is the same drain guarantee the baseline
// relies on. In addition O_fifo_aclr is asserted during S_ACK, so
// both FIFOs are being flushed and any theoretical late return is
// discarded rather than mis-routed.
//
// NOTE: do NOT gate this reset with an "O_app_rd_en && first" flag.
// During a burst O_app_rd_en is high for BURST_SIZE consecutive
// cycles, so such a flag would hold the counter at zero for the
// whole first burst and de-align the demux by the return-pipeline
// depth.
//
// == Single-stream compatibility (I_dual_en latched as 0) ==
// When dual_en_cur == 0:
//   - burst_sel stays 0 (only stream A is read);
//   - ret_sel stays 0 (all returns route to FIFO A);
//   - Total bursts = N_BURST_PER_STREAM (same as baseline).
// Behavior is functionally identical to the baseline frame_fifo_read.
//
// == BURST_SIZE = 256 ==
// The baseline frame_fifo_read declares BURST_SIZE default = 128,
// but the upper-level frame_read_write instantiates it with the
// module-level default BURST_SIZE = 256 (no explicit override).
// This module sets BURST_SIZE default = 256 to match the actual
// value used in the system and avoid any burst-count mismatch.
//
// == Configuration capture via cfg_toggle handshake ==
// I_rd_addr_a, I_rd_addr_b, and I_dual_en are multi-bit signals from
// the pixel clock domain. They are captured into mem_clk-domain _lat
// registers on the synchronized I_cfg_toggle edge (cfg_t2 != cfg_t1),
// using a valid-flag / multi-cycle CDC handshake: the source toggles
// O_cfg_toggle in the same cycle it changes the config and holds it
// stable afterwards, so by the time the toggle edge is detected the
// multi-bit data has settled and can be sampled safely. The read
// cursors then load from these _lat registers once per frame in S_IDLE.
// cfg_pending remains as a debug/observability flag only.
//
// == Verilog-2001, synthesizable ==
// No always_ff, logic, typedef, interface, int, initial, $display,
// division, or modulo operators. Async active-high reset.
//============================================================================

module sdram_rd_dual #(
    parameter MEM_DATA_BITS = 32,
    parameter ADDR_BITS     = 21,
    parameter BURST_BITS    = 9,
    parameter BURST_SIZE    = 256,
    parameter FIFO_DEPTH    = 512
)(
    input  wire                    I_mem_clk,
    input  wire                    I_rst,                 // Active-high async reset
    input  wire                    I_sdr_init_done,
    input  wire                    I_sdr_init_ref_vld,
    input  wire                    I_sdr_busy,
    input  wire                    I_wrfifo_urgent,       // Write FIFO near-full: yield next read burst
    input  wire                    I_sdr_rd_en,           // SDRAM read-data-valid return

    // SDRAM user interface
    output wire                    O_app_rd_en,
    output wire [ADDR_BITS-1:0]    O_app_rd_addr,
    input  wire                    I_app_wr_busy,         // Write channel busy (write priority)
    output wire                    O_rd_busy,             // Read burst in progress

    // Dual FIFO write-side control (FIFOs instantiated externally)
    output wire                    O_fifo_aclr,           // Clears both FIFO A and FIFO B
    output wire                    O_fifo_we_a,           // FIFO A write enable
    output wire                    O_fifo_we_b,           // FIFO B write enable
    input  wire [9:0]              I_used_a,              // FIFO A write-side occupancy
    input  wire [9:0]              I_used_b,              // FIFO B write-side occupancy

    // Read request side (pixel domain)
    input  wire                    I_read_req,            // Frame read request, held until ack
    output wire                    O_read_req_ack,
    output wire                    O_read_finish,
    input  wire [ADDR_BITS-1:0]    I_read_len,
    input  wire [ADDR_BITS-1:0]    I_rd_addr_a,           // Stream A physical base address
    input  wire [ADDR_BITS-1:0]    I_rd_addr_b,           // Stream B physical base address
    input  wire                    I_dual_en,             // Dual-stream enable
    input  wire                    I_cfg_toggle           // Config change flag (edge = new config)
);

    //------------------------------------------------------------------------
    // FSM state encoding (identical to baseline frame_fifo_read)
    //------------------------------------------------------------------------
    localparam S_IDLE           = 4'd0;
    localparam S_ACK            = 4'd1;
    localparam S_CHECK_FIFO     = 4'd2;
    localparam S_READ_BURST     = 4'd3;
    localparam S_READ_BURST_END = 4'd4;
    localparam S_END            = 4'd5;

    //------------------------------------------------------------------------
    // Local constants
    //------------------------------------------------------------------------
    localparam [255:0] ONE  = 256'd1;
    localparam [255:0] ZERO = 256'd0;

    //------------------------------------------------------------------------
    // Internal registers -- request pipeline (baseline: 3-stage sync)
    //------------------------------------------------------------------------
    reg                      read_req_d0, read_req_d1, read_req_d2;
    reg [ADDR_BITS-1:0]      read_len_d0, read_len_d1, read_len_latch;
    reg [ADDR_BITS-1:0]      read_cnt;

    //------------------------------------------------------------------------
    // Internal registers -- burst control (baseline timing preserved)
    //------------------------------------------------------------------------
    reg [3:0]                state;
    reg [ADDR_BITS-1:0]      App_rd_addr_r;
    reg [BURST_BITS-1:0]     burst_cnt;
    reg                      App_rd_en_r, App_rd_en_d0;
    reg [3:0]                rd_delay;
    reg                      read_req_ack_r;

    //------------------------------------------------------------------------
    // Internal registers -- dual-stream address cursors and burst selector
    //------------------------------------------------------------------------
    reg [ADDR_BITS-1:0]      addr_a_cur, addr_b_cur;
    reg                      burst_sel;
    reg                      dual_en_cur;   // frame-locked dual-enable for this read

    //------------------------------------------------------------------------
    // Internal registers -- return-count demux
    //------------------------------------------------------------------------
    reg [BURST_BITS-1:0]     ret_cnt;
    reg                      ret_sel;

    //------------------------------------------------------------------------
    // FIFO asynchronous clear (registered, baseline behaviour)
    //------------------------------------------------------------------------
    reg                      fifo_aclr_r;

    //------------------------------------------------------------------------
    // Internal registers -- configuration CDC and latch
    //------------------------------------------------------------------------
    reg                      cfg_t0, cfg_t1, cfg_t2;
    reg                      cfg_pending;
    reg                      dual_en_lat;
    reg [ADDR_BITS-1:0]      rd_addr_a_lat, rd_addr_b_lat;

    //------------------------------------------------------------------------
    // Combinational assignments
    //------------------------------------------------------------------------

    // Frame-end limit: in dual mode, total words read = 2 * read_len
    // (because bursts alternate between A and B streams).
    // Use left-shift by 1 (multiply by 2) to avoid division/modulo.
    wire [ADDR_BITS-1:0] read_cnt_limit = dual_en_cur ?
        {read_len_latch[ADDR_BITS-2:0], 1'b0} :  // 2 * read_len
        read_len_latch;                           // 1 * read_len (single mode)

    // SDRAM user-interface outputs (baseline-equivalent)
    assign O_app_rd_addr = App_rd_addr_r;
    assign O_app_rd_en   = App_rd_en_d0;
    assign O_rd_busy     = (state == S_READ_BURST);

    // FIFO clear: registered output driven by the FSM (baseline behaviour).
    // The same signal clears both FIFO A and FIFO B.
    assign O_fifo_aclr   = fifo_aclr_r;

    // Read request outputs
    assign O_read_req_ack = read_req_ack_r;
    assign O_read_finish  = (state == S_END) ? 1'b1 : 1'b0;

    // Burst-finish detection (identical to baseline):
    //   rd_vld: burst_cnt has reached BURST_SIZE (all addresses issued)
    //   rd_delay: counts to 10 after last rd_en to allow returns to drain
    wire rd_vld        = (state == S_READ_BURST) &&
                         (burst_cnt >= BURST_SIZE[BURST_BITS-1:0]);
    wire rd_burst_finish = rd_vld && (rd_delay == 4'd10);

    // FIFO space check: select the target FIFO for the NEXT burst
    wire [9:0] next_used = burst_sel ? I_used_b : I_used_a;

    // Return demux outputs: route I_sdr_rd_en returns to the correct FIFO
    assign O_fifo_we_a = I_sdr_rd_en & ~ret_sel;
    assign O_fifo_we_b = I_sdr_rd_en &  ret_sel;

    //------------------------------------------------------------------------
    // Request pipeline registers (baseline 3-stage sync, unchanged)
    //------------------------------------------------------------------------
    always @(posedge I_mem_clk or posedge I_rst) begin
        if (I_rst == 1'b1) begin
            read_req_d0 <= 1'b0;
            read_req_d1 <= 1'b0;
            read_req_d2 <= 1'b0;
            read_len_d0 <= ZERO[ADDR_BITS-1:0];
            read_len_d1 <= ZERO[ADDR_BITS-1:0];
        end
        else begin
            read_req_d0 <= I_read_req;
            read_req_d1 <= read_req_d0;
            read_req_d2 <= read_req_d1;
            read_len_d0 <= I_read_len;
            read_len_d1 <= read_len_d0;
        end
    end

    //------------------------------------------------------------------------
    // rd_delay counter (baseline timing, unchanged)
    // Counts from 0 to 10 after last O_app_rd_en of a burst.
    // rd_burst_finish fires when rd_delay == 10, ensuring all BURST_SIZE
    // returns have been absorbed before the FSM advances.
    //------------------------------------------------------------------------
    always @(posedge I_mem_clk or posedge I_rst) begin
        if (I_rst == 1'b1)
            rd_delay <= 4'd0;
        else if (O_app_rd_en)
            rd_delay <= 4'd0;
        else if (rd_delay < 4'd10)
            rd_delay <= rd_delay + 1'b1;
    end

    //------------------------------------------------------------------------
    // Burst counter, address generation and App_rd_en generation
    //
    // Structure and assignment style are IDENTICAL to the baseline
    // frame_fifo_read: ONE always block, ALL non-blocking assignments.
    //
    // Do not split this into several blocks and do not mix blocking
    // assignments in. burst_cnt is both updated here and read by the
    // App_rd_en_d0 comparison; a blocking update would make the value
    // observed in simulation differ from the value synthesis infers for
    // the flop, producing a simulation/synthesis mismatch and a wrong
    // number of read-enable pulses per burst.
    //
    // Address generation differs from the baseline in ONE way: the
    // baseline reads one contiguous range so successive bursts could
    // simply keep auto-incrementing, whereas the dual engine alternates
    // between two streams and must reload the start address of the
    // target stream at every burst start. The reload therefore happens
    // in S_CHECK_FIFO (which is entered before every burst, including
    // the first one after S_ACK), selecting the cursor named by
    // burst_sel. Inside a burst the baseline auto-increment is kept.
    //------------------------------------------------------------------------
    always @(posedge I_mem_clk or posedge I_rst) begin
        if (I_rst == 1'b1) begin
            burst_cnt     <= ZERO[BURST_BITS-1:0];
            App_rd_addr_r <= ZERO[ADDR_BITS-1:0];
            App_rd_en_d0  <= 1'b0;
        end
        else begin
            // ---- burst word counter (baseline) ----
            if (state == S_CHECK_FIFO)
                burst_cnt <= ZERO[BURST_BITS-1:0];
            else if (O_app_rd_en)
                burst_cnt <= burst_cnt + 1'b1;
            else
                burst_cnt <= burst_cnt;

            // ---- read address (dual: reload per burst) ----
            if (state == S_CHECK_FIFO)
                App_rd_addr_r <= burst_sel ? addr_b_cur : addr_a_cur;
            else if (O_app_rd_en)
                App_rd_addr_r <= App_rd_addr_r + 1'b1;
            else
                App_rd_addr_r <= App_rd_addr_r;

            // ---- read enable (baseline) ----
            if (App_rd_en_r && burst_cnt + O_app_rd_en < BURST_SIZE[BURST_BITS-1:0])
                App_rd_en_d0 <= 1'b1;
            else
                App_rd_en_d0 <= 1'b0;
        end
    end

    //------------------------------------------------------------------------
    // Return-count demux
    //
    // Routes each I_sdr_rd_en return pulse to FIFO A or FIFO B.
    // ret_cnt counts returns within a burst (0..BURST_SIZE-1).
    // ret_sel selects the target FIFO; flips after each complete burst.
    //
    // Alignment happens in S_ACK (frame start). This is safe and must NOT
    // be replaced by an "O_app_rd_en && first" flag: O_app_rd_en stays high
    // for BURST_SIZE consecutive cycles, so such a flag would hold the
    // counter at zero for the whole first burst and de-align the demux by
    // the return pipeline depth.
    //------------------------------------------------------------------------
    always @(posedge I_mem_clk or posedge I_rst) begin
        if (I_rst == 1'b1) begin
            ret_cnt <= {BURST_BITS{1'b0}};
            ret_sel <= 1'b0;
        end
        else if (state == S_ACK) begin
            // Frame start: align the demux to stream A. All returns of the
            // previous burst have drained (rd_burst_finish waited for
            // rd_delay==10 after the last O_app_rd_en) and O_fifo_aclr is
            // flushing both FIFOs, so no late return can be mis-routed.
            ret_cnt <= {BURST_BITS{1'b0}};
            ret_sel <= 1'b0;
        end
        else if (I_sdr_rd_en) begin
            if (ret_cnt == BURST_SIZE[BURST_BITS-1:0] - 1'b1) begin
                // Last return of this burst: wrap counter, flip selector
                ret_cnt <= {BURST_BITS{1'b0}};
                ret_sel <= dual_en_cur ? ~ret_sel : 1'b0;
            end
            else begin
                ret_cnt <= ret_cnt + 1'b1;
            end
        end
    end

    //------------------------------------------------------------------------
    // Configuration CDC (I_cfg_toggle + I_rd_addr_a/b + I_dual_en cross from
    // the pixel domain)
    //
    // I_rd_addr_a / I_rd_addr_b / I_dual_en are multi-bit signals from the
    // pixel clock domain. Sampling them directly (as S_ACK used to do) can
    // latch a half-old / half-new address and misroute the SDRAM burst,
    // leaving persistent "stain" pixels. The source (transition_ctrl)
    // toggles O_cfg_toggle in the same cycle it changes these signals and
    // holds them stable afterwards, so the single-bit toggle is the valid
    // flag for a valid-flag / multi-cycle CDC handshake.
    //
    // I_cfg_toggle is synchronised through cfg_t0/t1/t2. On the synchronised
    // edge (cfg_t2 != cfg_t1) the multi-bit data has long settled, so it is
    // safe to capture I_rd_addr_a/b and I_dual_en into _lat registers here.
    // The read cursors then load from these _lat registers once per frame in
    // S_IDLE, never re-sampling the raw inputs mid-frame. cfg_pending remains
    // as an observation flag (set on the edge, cleared in S_ACK).
    //------------------------------------------------------------------------
    always @(posedge I_mem_clk or posedge I_rst) begin
        if (I_rst == 1'b1) begin
            cfg_t0        <= 1'b0;
            cfg_t1        <= 1'b0;
            cfg_t2        <= 1'b0;
            cfg_pending   <= 1'b1;   // Force first-frame latch
            dual_en_lat   <= 1'b0;
            rd_addr_a_lat <= {ADDR_BITS{1'b0}};
            rd_addr_b_lat <= {ADDR_BITS{1'b0}};
        end
        else begin
            cfg_t0 <= I_cfg_toggle;
            cfg_t1 <= cfg_t0;
            cfg_t2 <= cfg_t1;
            if (cfg_t2 != cfg_t1)
                cfg_pending <= 1'b1;
            else if (state == S_ACK)
                cfg_pending <= 1'b0;

            // Multi-cycle capture: on the synchronised toggle edge the
            // multi-bit config has settled, so a direct sample is safe.
            if (cfg_t2 != cfg_t1) begin
                dual_en_lat   <= I_dual_en;
                rd_addr_a_lat <= I_rd_addr_a;
                rd_addr_b_lat <= I_rd_addr_b;
            end
        end
    end

    //------------------------------------------------------------------------
    // Main FSM + address cursor management
    //
    // State transitions are identical to baseline frame_fifo_read except:
    //   - S_CHECK_FIFO uses next_used (burst_sel-selected FIFO) for space
    //   - S_READ_BURST updates dual address cursors and flips burst_sel
    //   - S_IDLE loads the read cursors from the handshake-captured config
    //------------------------------------------------------------------------
    always @(posedge I_mem_clk or posedge I_rst) begin
        if (I_rst == 1'b1) begin
            state         <= S_IDLE;
            read_len_latch <= ZERO[ADDR_BITS-1:0];
            App_rd_en_r   <= 1'b0;
            read_cnt      <= ZERO[ADDR_BITS-1:0];
            read_req_ack_r <= 1'b0;
            fifo_aclr_r   <= 1'b0;
            burst_sel     <= 1'b0;
            addr_a_cur    <= ZERO[ADDR_BITS-1:0];
            addr_b_cur    <= ZERO[ADDR_BITS-1:0];
            dual_en_cur   <= 1'b0;
        end
        else begin
            case (state)
                //----------------------------------------------------------
                // S_IDLE: wait for synchronized read request
                //----------------------------------------------------------
                S_IDLE: begin
                    if (read_req_d2 == 1'b1 && I_sdr_init_done) begin
                        state <= S_ACK;
                        // Lock the frame's read cursors from the handshake-
                        // synchronised config exactly once. This used to live
                        // in S_ACK (which spans several cycles); sampling once
                        // here means a config change arriving mid-frame can no
                        // longer retrigger a cursor reload.
                        addr_a_cur <= rd_addr_a_lat;
                        addr_b_cur <= rd_addr_b_lat;
                        dual_en_cur <= dual_en_lat;
                        burst_sel  <= 1'b0;
                    end
                    read_req_ack_r <= 1'b0;
                end

                //----------------------------------------------------------
                // S_ACK: frame start -- clear FIFOs, latch read_len, reset
                //        the read-progress counter.
                //
                // Note: the multi-bit config (I_rd_addr_a/b, I_dual_en) is
                // NOT sampled here anymore. It is captured by the cfg_toggle
                // handshake (see Configuration CDC block) and the read cursors
                // were already loaded from the captured _lat registers once in
                // S_IDLE. Sampling raw inputs here for several cycles was the
                // source of the persistent "stain" mis-routing.
                //----------------------------------------------------------
                S_ACK: begin
                    if (read_req_d2 == 1'b0) begin
                        // read_req de-asserted: move to first burst
                        state        <= S_CHECK_FIFO;
                        read_req_ack_r <= 1'b0;
                        fifo_aclr_r  <= 1'b0;
                    end
                    else begin
                        // Hold in S_ACK while read_req is asserted
                        read_req_ack_r <= 1'b1;
                        fifo_aclr_r  <= 1'b1;
                        read_len_latch <= read_len_d1;
                    end
                    // Reset read progress
                    read_cnt <= ZERO[ADDR_BITS-1:0];
                end

                //----------------------------------------------------------
                // S_CHECK_FIFO: wait for target FIFO space and write
                //               channel availability (write priority).
                //
                // Space check uses next_used which selects the FIFO
                // matching the next burst's target stream (burst_sel).
                // Write-priority semantics (~I_app_wr_busy) preserved
                // from baseline -- never weakened.
                //----------------------------------------------------------
                S_CHECK_FIFO: begin
                    if (read_req_d2 == 1'b1)
                        state <= S_ACK;
                    // 刷新(ref_vld)有效期间不发起新读突发，让读请求绕开 REF 窗口，
                    // 避免 REF 抢占读总线造成读流空洞、FIFO 瞬时读空吐出全 1 白点。
                    else if (next_used < (FIFO_DEPTH - BURST_SIZE) && ~I_app_wr_busy && ~I_wrfifo_urgent && ~I_sdr_init_ref_vld) begin
                        state       <= S_READ_BURST;
                        App_rd_en_r <= 1'b1;
                    end
                end

                //----------------------------------------------------------
                // S_READ_BURST: issue BURST_SIZE read addresses, then
                //               wait for all returns to drain (rd_delay).
                //
                // On burst finish:
                //   - Advance the active stream's address cursor
                //   - Flip burst_sel in dual mode (stay on A in single)
                //   - Increment read_cnt by BURST_SIZE
                //----------------------------------------------------------
                S_READ_BURST: begin
                    if (rd_burst_finish == 1'b1) begin
                        App_rd_en_r   <= 1'b0;
                        state         <= S_READ_BURST_END;
                        read_cnt      <= read_cnt + BURST_SIZE[ADDR_BITS-1:0];

                        // Advance the target stream's address cursor
                        if (burst_sel == 1'b0)
                            addr_a_cur <= addr_a_cur + BURST_SIZE[ADDR_BITS-1:0];
                        else
                            addr_b_cur <= addr_b_cur + BURST_SIZE[ADDR_BITS-1:0];

                        // Flip burst selector for next burst (dual mode only)
                        burst_sel <= dual_en_cur ? ~burst_sel : 1'b0;
                    end
                end

                //----------------------------------------------------------
                // S_READ_BURST_END: decide whether to continue bursting
                //                   or finish the frame.
                //
                // If a new read_req arrives, restart (S_ACK).
                // If read_cnt < read_cnt_limit, more bursts needed.
                // Otherwise, frame is complete.
                //
                // Dual-mode note: read_cnt accumulates BURST_SIZE per
                // burst. In dual mode, total bursts = 2 * N_BURST_PER_STREAM,
                // so read_cnt reaches 2 * read_len. The read_cnt_limit
                // wire handles this (1x in single mode, 2x in dual mode).
                //----------------------------------------------------------
                S_READ_BURST_END: begin
                    if (read_req_d2 == 1'b1)
                        state <= S_ACK;
                    else if (read_cnt < read_cnt_limit)
                        state <= S_CHECK_FIFO;
                    else
                        state <= S_END;
                end

                //----------------------------------------------------------
                // S_END: frame complete, return to idle
                //----------------------------------------------------------
                S_END: begin
                    state <= S_IDLE;
                end

                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule
