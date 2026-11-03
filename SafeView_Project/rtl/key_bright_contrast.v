//==============================================================================
// Module: key_bright_contrast
// KEY3 short/long press adjusts the selected brightness or contrast level.
//   SW3 (sel) : 0 = brightness, 1 = contrast
//   short press : selected level +1 (wrap 0..3)
//   long press  : selected level -1 (wrap 0..3)
//==============================================================================

module key_bright_contrast #(
    parameter integer CLK_FREQ_HZ = 25_000_000,
    parameter integer DEBOUNCE_MS = 20,
    parameter integer LONG_MS     = 800
)(
    input  wire       clk,
    input  wire       rst,          // active-high reset
    input  wire       key_in,       // released = 1, pressed = 0
    input  wire       sel,          // 0 = brightness, 1 = contrast
    output reg  [1:0] O_bright,
    output reg  [1:0] O_contrast
);

    localparam integer DEB_CYC  = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;
    localparam integer LONG_CYC = (CLK_FREQ_HZ / 1000) * LONG_MS;

    reg        sync0;
    reg        sync1;
    reg        stable;
    reg [31:0] deb_cnt;
    reg [31:0] hold_cnt;
    reg        long_fired;

    wire press_now =  stable && (hold_cnt != 32'd0) && !long_fired;
    wire long_now  = !stable && !long_fired && (hold_cnt >= (LONG_CYC - 1));

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            sync0      <= 1'b1;
            sync1      <= 1'b1;
            stable     <= 1'b1;
            deb_cnt    <= 32'd0;
            hold_cnt   <= 32'd0;
            long_fired <= 1'b0;
            O_bright   <= 2'd1;   // neutral-ish default (level 1)
            O_contrast <= 2'd1;
        end else begin
            sync0 <= key_in;
            sync1 <= sync0;

            if (sync1 == stable) begin
                deb_cnt <= 32'd0;
            end else if (deb_cnt >= (DEB_CYC - 1)) begin
                stable  <= sync1;
                deb_cnt <= 32'd0;
            end else begin
                deb_cnt <= deb_cnt + 32'd1;
            end

            if (!stable) begin
                if (hold_cnt < LONG_CYC)
                    hold_cnt <= hold_cnt + 32'd1;
                if (!long_fired && (hold_cnt >= (LONG_CYC - 1)))
                    long_fired <= 1'b1;
            end else begin
                hold_cnt   <= 32'd0;
                long_fired <= 1'b0;
            end

            if (long_now) begin
                if (sel) begin
                    O_contrast <= (O_contrast == 2'd0) ? 2'd3 : (O_contrast - 2'd1);
                end else begin
                    O_bright <= (O_bright == 2'd0) ? 2'd3 : (O_bright - 2'd1);
                end
            end else if (press_now) begin
                if (sel) begin
                    O_contrast <= (O_contrast == 2'd3) ? 2'd0 : (O_contrast + 2'd1);
                end else begin
                    O_bright <= (O_bright == 2'd3) ? 2'd0 : (O_bright + 2'd1);
                end
            end
        end
    end

endmodule