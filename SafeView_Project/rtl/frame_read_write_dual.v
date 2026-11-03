// ============================================================================
// Module : frame_read_write_dual
// File     : rtl/frame_read_write_dual.v
// Language : Verilog-2001 (synthesizable)
// ============================================================================
//
// Description
// -----------
// Dual-read variant of the baseline frame_read_write module.
// This is a pure wiring/instantiation module -- it contains NO functional
// logic of its own (no counters, FSMs, comparators, or other behavioral
// statements).  All behavior is delegated to the four submodules below.
//
// Differences from baseline frame_read_write (3 changes):
//   1. Read engine replaced:  frame_fifo_read  -->  sdram_rd_dual
//      The new read engine supports simultaneous dual-address reads,
//      producing two independent data streams from two SDRAM regions.
//   2. Read FIFO count doubled: 1 --> 2 (A/B channels)
//      Two rfifo_32_32_512 instances (read_buf_a, read_buf_b) provide
//      independent pixel outputs O_rd_data_a / O_rd_data_b.
//   3. Write base address changed to explicit input:
//      Baseline used write_addr_index to select among 4 pre-configured
//      base addresses.  This variant fixes write_addr_index = 2'b00 and
//      connects write_addr_0 to the explicit input port I_wr_addr,
//      giving the upper layer (transition engine) full control over the
//      write target address.  This is functionally equivalent when only
//      address slot 0 is used.
//
// Lock-step precondition for dual read FIFOs:
//   Both rfifo instances (read_buf_a, read_buf_b) share the SAME
//   read_fifo_aclr (async clear) and I_read_en (read enable).
//   This is the physical prerequisite for pixel-level lock-step
//   operation between channels A and B.  They MUST NOT be separated.
//
// ============================================================================

module frame_read_write_dual #(
    parameter MEM_DATA_BITS = 32,
    parameter READ_DATA_BITS  = 32,
    parameter WRITE_DATA_BITS = 32,
    parameter ADDR_BITS     = 21,
    parameter BURST_BITS    = 9,
    parameter BURST_SIZE    = 256,
    parameter FIFO_DEPTH    = 512,
    parameter WRITE_V_FLIP  = 1,
    parameter FRAME_WIDTH   = 640,
    parameter FRAME_HEIGHT  = 480
)(
    input  wire                     I_rst,
    input  wire                     I_mem_clk,
    input  wire                     I_sdr_init_done,
    input  wire                     I_sdr_init_ref_vld,
    input  wire                     I_sdr_busy,

    // SDRAM read interface
    output wire                     O_app_rd_en,
    output wire [ADDR_BITS-1:0]     O_app_rd_addr,
    input  wire                     I_sdr_rd_en,
    input  wire [MEM_DATA_BITS-1:0] I_sdr_rd_dout,

    // SDRAM write interface
    output wire                     O_app_wr_en,
    output wire [ADDR_BITS-1:0]     O_app_wr_addr,
    output wire [MEM_DATA_BITS-1:0] O_app_wr_din,
    output wire [3:0]               O_app_wr_dm,

    // Read side (pixel domain)
    input  wire                     I_read_clk,
    input  wire                     I_read_req,
    output wire                     O_read_req_ack,
    output wire                     O_read_finish,
    input  wire [ADDR_BITS-1:0]     I_read_len,
    input  wire [ADDR_BITS-1:0]     I_rd_addr_a,
    input  wire [ADDR_BITS-1:0]     I_rd_addr_b,
    input  wire                     I_dual_en,
    input  wire                     I_cfg_toggle,
    input  wire                     I_read_en,
    output wire [READ_DATA_BITS-1:0]  O_rd_data_a,
    output wire [READ_DATA_BITS-1:0]  O_rd_data_b,
    output wire                     O_empty_a,
    output wire                     O_empty_b,

    // Write side (TF domain)
    input  wire                     I_write_clk,
    input  wire                     I_write_req,
    output wire                     O_write_req_ack,
    output wire                     O_write_finish,
    input  wire [ADDR_BITS-1:0]     I_wr_addr,
    input  wire [ADDR_BITS-1:0]     I_write_len,
    input  wire                     I_write_en,
    input  wire [WRITE_DATA_BITS-1:0] I_write_data
);

    // ========================================================================
    // Internal wires
    // ========================================================================

    // Write FIFO occupancy (read-side word count, mem_clk domain)
    wire [BURST_BITS-1:0] wrusedw_d;

    // Write FIFO near-full flag (wfifo_32_32_512 is 512 deep, full at 511).
    // During lazy background preload the display keeps reading SDRAM while the
    // SD card keeps filling this FIFO. Without backpressure the FIFO can hit
    // full and silently drop writes (wr_en_s = (!full_flag) & we), which
    // corrupts pixels ("stains") and can hang a load until timeout. When the
    // FIFO has got far past one burst, force the read engine to yield its next
    // burst so the write engine drains it before overflow.
    wire wrfifo_urgent;
    assign wrfifo_urgent = (wrusedw_d >= 9'd384);

    // Busy flags exchanged between read and write engines
    wire O_wr_busy;
    wire O_rd_busy;

    // Async-clear signals generated by sub-engines for their respective FIFOs
    wire write_fifo_aclr;
    wire read_fifo_aclr;

    // Busy aliases forwarded across submodules
    wire App_wr_busy = O_wr_busy;
    wire App_rd_busy = O_rd_busy;

    // Write data-mask: always zero (all 4 byte lanes active)
    assign O_app_wr_dm = 4'b0000;

    // Dual read FIFO write-enable and occupancy
    wire O_fifo_we_a;
    wire O_fifo_we_b;
    wire [9:0] used_a;
    wire [9:0] used_b;

    // ========================================================================
    // 1. Write FIFO  (wfifo_32_32_512)
    //    Cross-clock-domain buffer: write_clk -> mem_clk
    //    Identical to baseline frame_read_write instantiation.
    // ========================================================================

    wfifo_32_32_512 write_buf (
        .clkr       (I_mem_clk          ),
        .clkw       (I_write_clk        ),
        .rst        (write_fifo_aclr    ),
        .we         (I_write_en         ),
        .re         (O_app_wr_en        ),
        .di         (I_write_data       ),
        .empty_flag (                   ),
        .full_flag  (                   ),
        .wrusedw    (                   ),
        .rdusedw    (wrusedw_d          ),
        .dout       (O_app_wr_din       )
    );

    // ========================================================================
    // 2. Frame FIFO Write controller  (frame_fifo_write)
    //    Manages SDRAM write bursts and write FIFO flow control.
    //    Difference from baseline: write_addr_0 is connected to the
    //    explicit I_wr_addr input, and write_addr_index is fixed at 2'b00.
    //    This is functionally equivalent to selecting slot 0 in the
    //    baseline's 4-slot address table, giving the upper layer
    //    (transition engine) direct control over the write base address.
    // ========================================================================

    frame_fifo_write #(
        .MEM_DATA_BITS (MEM_DATA_BITS),
        .ADDR_BITS     (ADDR_BITS),
        .BURST_BITS    (BURST_BITS),
        .BURST_SIZE    (BURST_SIZE),
        .WRITE_V_FLIP  (WRITE_V_FLIP),
        .FRAME_WIDTH   (FRAME_WIDTH),
        .FRAME_HEIGHT  (FRAME_HEIGHT)
    ) frame_fifo_write_m0 (
        .rst            (I_rst              ),
        .mem_clk        (I_mem_clk          ),
        .Sdr_init_done  (I_sdr_init_done    ),
        .Sdr_init_ref_vld(I_sdr_init_ref_vld),
        .Sdr_busy       (I_sdr_busy         ),
        .App_rd_busy    (App_rd_busy        ),
        .O_wr_busy      (O_wr_busy          ),
        .App_wr_en      (O_app_wr_en        ),
        .App_wr_addr    (O_app_wr_addr      ),
        .write_req      (I_write_req        ),
        .write_req_ack  (O_write_req_ack    ),
        .write_finish   (O_write_finish     ),
        .write_addr_0   (I_wr_addr          ),
        .write_addr_1   ({ADDR_BITS{1'b0}}  ),
        .write_addr_2   ({ADDR_BITS{1'b0}}  ),
        .write_addr_3   ({ADDR_BITS{1'b0}}  ),
        .write_addr_index(2'b00             ),
        .write_len      (I_write_len        ),
        .fifo_aclr      (write_fifo_aclr    ),
        .rdusedw        (wrusedw_d          )
    );

    // ========================================================================
    // 3a. Read FIFO A  (rfifo_32_32_512)
    //     Cross-clock-domain buffer: mem_clk -> read_clk, channel A.
    //     Shares read_fifo_aclr and I_read_en with channel B --
    //     this is the physical lock-step prerequisite for dual-channel
    //     pixel output.  DO NOT separate these shared signals.
    // ========================================================================

    rfifo_32_32_512 read_buf_a (
        .clkr       (I_read_clk       ),
        .clkw       (I_mem_clk        ),
        .rst        (read_fifo_aclr   ),   // shared with B -- lock-step
        .we         (O_fifo_we_a      ),
        .re         (I_read_en        ),   // shared with B -- lock-step
        .di         (I_sdr_rd_dout    ),
        .empty_flag (O_empty_a        ),
        .full_flag  (                 ),
        .wrusedw    (used_a           ),
        .rdusedw    (                 ),
        .dout       (O_rd_data_a      )
    );

    // ========================================================================
    // 3b. Read FIFO B  (rfifo_32_32_512)
    //     Cross-clock-domain buffer: mem_clk -> read_clk, channel B.
    //     Shares read_fifo_aclr and I_read_en with channel A --
    //     this is the physical lock-step prerequisite for dual-channel
    //     pixel output.  DO NOT separate these shared signals.
    // ========================================================================

    rfifo_32_32_512 read_buf_b (
        .clkr       (I_read_clk       ),
        .clkw       (I_mem_clk        ),
        .rst        (read_fifo_aclr   ),   // shared with A -- lock-step
        .we         (O_fifo_we_b      ),
        .re         (I_read_en        ),   // shared with A -- lock-step
        .di         (I_sdr_rd_dout    ),
        .empty_flag (O_empty_b        ),
        .full_flag  (                 ),
        .wrusedw    (used_b           ),
        .rdusedw    (                 ),
        .dout       (O_rd_data_b      )
    );

    // ========================================================================
    // 4. Dual-read SDRAM engine  (sdram_rd_dual)
    //     Replaces baseline frame_fifo_read.  Supports two simultaneous
    //     read addresses (I_rd_addr_a / I_rd_addr_b) when I_dual_en is
    //     asserted, or single-address reads when I_dual_en is deasserted.
    //     Drives write-enable signals O_fifo_we_a / O_fifo_we_b to the
    //     two read FIFOs independently.
    // ========================================================================

    sdram_rd_dual #(
        .MEM_DATA_BITS (MEM_DATA_BITS),
        .ADDR_BITS     (ADDR_BITS),
        .BURST_BITS    (BURST_BITS),
        .BURST_SIZE    (BURST_SIZE),
        .FIFO_DEPTH    (FIFO_DEPTH)
    ) u_rd_dual (
        .I_mem_clk        (I_mem_clk          ),
        .I_rst            (I_rst              ),
        .I_sdr_init_done  (I_sdr_init_done    ),
        .I_sdr_init_ref_vld(I_sdr_init_ref_vld),
        .I_sdr_busy       (I_sdr_busy         ),
        .I_wrfifo_urgent  (wrfifo_urgent      ),
        .I_sdr_rd_en      (I_sdr_rd_en        ),
        .O_app_rd_en      (O_app_rd_en        ),
        .O_app_rd_addr    (O_app_rd_addr      ),
        .I_app_wr_busy    (App_wr_busy        ),
        .O_rd_busy        (O_rd_busy          ),
        .O_fifo_aclr      (read_fifo_aclr     ),
        .O_fifo_we_a      (O_fifo_we_a        ),
        .O_fifo_we_b      (O_fifo_we_b        ),
        .I_used_a         (used_a             ),
        .I_used_b         (used_b             ),
        .I_read_req       (I_read_req         ),
        .O_read_req_ack   (O_read_req_ack     ),
        .O_read_finish    (O_read_finish      ),
        .I_read_len       (I_read_len         ),
        .I_rd_addr_a      (I_rd_addr_a        ),
        .I_rd_addr_b      (I_rd_addr_b        ),
        .I_dual_en        (I_dual_en          ),
        .I_cfg_toggle     (I_cfg_toggle       )
    );

endmodule
