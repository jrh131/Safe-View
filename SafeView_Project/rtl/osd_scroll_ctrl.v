//============================================================================
// Module : osd_scroll_ctrl
// Purpose : Dynamic coordinate generator for image-carousel OSD overlay.
//           Produces:
//             (1) Smooth-scrolling banner phase  O_scroll_phase [0, BANNER_W)
//             (2) Uptime timestamp BCD digits   HH:MM:SS (power-on origin)
//             (3) One-clock second pulse         O_sec_tick
//
// Parameters:
//   BANNER_W    - Scroll strip total width in pixels, including trailing
//                 blank region. Must fit in 10 bits (<= 1024 for default
//                 10-bit O_scroll_phase).
//   SCROLL_STEP - Horizontal displacement per video frame in pixels.
//                 Typical range 1..4. Must satisfy SCROLL_STEP <= BANNER_W.
//   TICK_DIV    - Number of pixel-clock periods per second.
//                 Default 25_000_000 corresponds to a 25 MHz pixel clock.
//
// Output / Update Timing:
//   O_scroll_phase and all BCD digit outputs are updated exclusively on
//   the falling edge of I_vs (detected as vs_d & ~I_vs). This guarantees
//   that coordinates and digits remain constant within each video frame,
//   preventing mid-frame tearing or digit flicker.
//   O_sec_tick pulses high for exactly one I_pix_clk cycle when the
//   internal prescaler wraps. It is NOT gated to frame boundaries.
//
// Reset:
//   I_rst is active-high SYNCHRONOUS reset. All registers clear to zero.
//============================================================================

module osd_scroll_ctrl #(
    parameter BANNER_W    = 1024,
    parameter SCROLL_STEP = 2,
    parameter TICK_DIV    = 25000000
)(
    input  wire        I_pix_clk,
    input  wire        I_rst,
    input  wire        I_vs,
    output reg  [10:0] O_scroll_phase,
    output reg  [3:0]  O_hh_t,
    output reg  [3:0]  O_hh_u,
    output reg  [3:0]  O_mm_t,
    output reg  [3:0]  O_mm_u,
    output reg  [3:0]  O_ss_t,
    output reg  [3:0]  O_ss_u,
    output reg         O_sec_tick
);

    //------------------------------------------------------------------------
    // Local parameters
    //------------------------------------------------------------------------
    localparam DIV_W = (TICK_DIV <= 2)          ? 1  :
                       (TICK_DIV <= 4)          ? 2  :
                       (TICK_DIV <= 8)          ? 3  :
                       (TICK_DIV <= 16)         ? 4  :
                       (TICK_DIV <= 32)         ? 5  :
                       (TICK_DIV <= 64)         ? 6  :
                       (TICK_DIV <= 128)        ? 7  :
                       (TICK_DIV <= 256)        ? 8  :
                       (TICK_DIV <= 512)        ? 9  :
                       (TICK_DIV <= 1024)       ? 10 :
                       (TICK_DIV <= 2048)       ? 11 :
                       (TICK_DIV <= 4096)       ? 12 :
                       (TICK_DIV <= 8192)       ? 13 :
                       (TICK_DIV <= 16384)      ? 14 :
                       (TICK_DIV <= 32768)      ? 15 :
                       (TICK_DIV <= 65536)      ? 16 :
                       (TICK_DIV <= 131072)     ? 17 :
                       (TICK_DIV <= 262144)     ? 18 :
                       (TICK_DIV <= 524288)     ? 19 :
                       (TICK_DIV <= 1048576)    ? 20 :
                       (TICK_DIV <= 2097152)    ? 21 :
                       (TICK_DIV <= 4194304)    ? 22 :
                       (TICK_DIV <= 8388608)    ? 23 :
                       (TICK_DIV <= 16777216)   ? 24 :
                       (TICK_DIV <= 33554432)   ? 25 :
                       (TICK_DIV <= 67108864)   ? 26 :
                       (TICK_DIV <= 134217728)  ? 27 :
                       (TICK_DIV <= 268435456)  ? 28 :
                       (TICK_DIV <= 536870912)  ? 29 :
                       (TICK_DIV <= 1073741824) ? 30 : 31;

    //------------------------------------------------------------------------
    // Internal registers
    //------------------------------------------------------------------------

    // Second-prescaler counter: counts 0 .. TICK_DIV-1
    reg [DIV_W-1:0] sec_div_cnt;

    // BCD time-keeping counters (internal, free-running)
    reg [3:0] ss_u;   // seconds  ones:  0..9
    reg [3:0] ss_t;   // seconds  tens:  0..5
    reg [3:0] mm_u;   // minutes  ones:  0..9
    reg [3:0] mm_t;   // minutes  tens:  0..5
    reg [3:0] hh_u;   // hours    ones:  0..9
    reg [3:0] hh_t;   // hours    tens:  0..2

    // Vertical-sync delay register for falling-edge detection
    reg vs_d;

    //------------------------------------------------------------------------
    // Frame-start detection (falling edge of I_vs)
    //------------------------------------------------------------------------
    wire frame_tick = vs_d & ~I_vs;

    //------------------------------------------------------------------------
    // Width-normalised copies of the width-related parameters.
    // Verilog-2001 does not allow a part-select on a plain parameter, so the
    // arithmetic below uses these localparams instead.
    //------------------------------------------------------------------------
    localparam [10:0] STEP_10   = SCROLL_STEP;         // SCROLL_STEP as 11 bits
    localparam [10:0] REMAIN_10 = BANNER_W - SCROLL_STEP;

    //------------------------------------------------------------------------
    // Cascaded BCD carry signals (active when second tick fires)
    //------------------------------------------------------------------------

    // Seconds-ones carry: rolls 9 -> 0, propagates to seconds-tens
    wire ss_u_carry = (ss_u == 4'd9);

    // Seconds-tens carry: rolls 5 -> 0, propagates to minutes-ones
    wire ss_t_carry = ss_u_carry & (ss_t == 4'd5);

    // Minutes-ones carry: rolls 9 -> 0, propagates to minutes-tens
    wire mm_u_carry = ss_t_carry & (mm_u == 4'd9);

    // Minutes-tens carry: rolls 5 -> 0, propagates to hours-ones
    wire mm_t_carry = mm_u_carry & (mm_t == 4'd5);

    // Hours-ones carry: rolls 9 -> 0, propagates to hours-tens
    wire hh_u_carry = mm_t_carry & (hh_u == 4'd9);

    // Hours wrap: 24-hour cycle. At 23:59:59 the state is hh_t=2, hh_u=3,
    // so the wrap is detected at hh = 23 (NOT at hh = 24, which is unreachable
    // and would otherwise display a spurious 24:00:00 hour).
    wire hh_wrap    = mm_t_carry & (hh_t == 4'd2) & (hh_u == 4'd3);

    //------------------------------------------------------------------------
    // Main sequential logic (synchronous reset, positive-edge clock)
    //------------------------------------------------------------------------
    always @(posedge I_pix_clk) begin
        if (I_rst) begin
            // Synchronous reset: all outputs and internals to zero
            sec_div_cnt    <= {DIV_W{1'b0}};
            O_sec_tick     <= 1'b0;
            ss_u           <= 4'd0;
            ss_t           <= 4'd0;
            mm_u           <= 4'd0;
            mm_t           <= 4'd0;
            hh_u           <= 4'd0;
            hh_t           <= 4'd0;
            O_scroll_phase <= 11'd0;
            O_hh_t         <= 4'd0;
            O_hh_u         <= 4'd0;
            O_mm_t         <= 4'd0;
            O_mm_u         <= 4'd0;
            O_ss_t         <= 4'd0;
            O_ss_u         <= 4'd0;
            vs_d           <= 1'b0;
        end else begin
            //------------------------------------------------------------
            // I_vs delay for edge detection (always shifts)
            //------------------------------------------------------------
            vs_d <= I_vs;

            //------------------------------------------------------------
            // Second-tick prescaler
            //------------------------------------------------------------
            if (sec_div_cnt == (TICK_DIV - 1)) begin
                sec_div_cnt <= {DIV_W{1'b0}};
                O_sec_tick  <= 1'b1;
            end else begin
                sec_div_cnt <= sec_div_cnt + 1'b1;
                O_sec_tick  <= 1'b0;
            end

            //------------------------------------------------------------
            // BCD counter chain (updates every second tick)
            //------------------------------------------------------------
            if (O_sec_tick) begin
                // -- seconds ones --
                if (ss_u_carry)
                    ss_u <= 4'd0;
                else
                    ss_u <= ss_u + 1'b1;

                // -- seconds tens --
                if (ss_u_carry) begin
                    if (ss_t_carry)
                        ss_t <= 4'd0;
                    else
                        ss_t <= ss_t + 1'b1;
                end

                // -- minutes ones --
                if (ss_t_carry) begin
                    if (mm_u_carry)
                        mm_u <= 4'd0;
                    else
                        mm_u <= mm_u + 1'b1;
                end

                // -- minutes tens --
                if (mm_u_carry) begin
                    if (mm_t_carry)
                        mm_t <= 4'd0;
                    else
                        mm_t <= mm_t + 1'b1;
                end

                // -- hours (ones and tens together, 24-hour wrap) --
                if (mm_t_carry) begin
                    if (hh_wrap) begin
                        hh_t <= 4'd0;
                        hh_u <= 4'd0;
                    end else if (hh_u_carry) begin
                        hh_t <= hh_t + 1'b1;
                        hh_u <= 4'd0;
                    end else begin
                        hh_u <= hh_u + 1'b1;
                    end
                end
            end

            //------------------------------------------------------------
            // Frame-rate updates (scroll phase + digit output latch)
            // Triggered on I_vs falling edge only.
            //------------------------------------------------------------
            if (frame_tick) begin
                // Scroll phase: increment by SCROLL_STEP with modular wrap.
                // Text scrolls from right to left (enters from the right edge).
                // phase >= (BANNER_W - SCROLL_STEP)  =>  phase - (BANNER_W - SCROLL_STEP)
                //                                     =>  phase + SCROLL_STEP - BANNER_W
                // so it stays in range [0, BANNER_W - 1].
                if (O_scroll_phase >= REMAIN_10)
                    O_scroll_phase <= O_scroll_phase - REMAIN_10;
                else
                    O_scroll_phase <= O_scroll_phase + STEP_10;

                // Latch BCD digits to output registers (flicker-free)
                O_hh_t <= hh_t;
                O_hh_u <= hh_u;
                O_mm_t <= mm_t;
                O_mm_u <= mm_u;
                O_ss_t <= ss_t;
                O_ss_u <= ss_u;
            end
        end
    end

endmodule
