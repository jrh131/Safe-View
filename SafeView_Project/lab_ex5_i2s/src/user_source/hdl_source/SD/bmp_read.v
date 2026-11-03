module bmp_read(
    input                       clk,
    input                       rst,
    output                      ready,

    // 上电扫描：从 scan_start_sector 开始，顺序寻找前 scan_target_count 张 BMP
    input                       scan_start,
    input  [31:0]               scan_start_sector,
    input  [31:0]               scan_max_sector,
    input  [2:0]                scan_target_count,
    output reg                  scan_done,
    output reg                  scan_found_valid,
    output reg [31:0]           scan_found_sector,
    output reg [2:0]            scan_found_total,

    // 按指定扇区加载一张图到 SDRAM
    input                       load_start,
    input                       load_abort,   // 上层超时后强制中止本次装载
    input  [31:0]               load_sector,

    input                       sd_init_done,
    output reg [3:0]            state_code,
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,

    output reg                  write_req,
    input                       write_req_ack,

    output reg                  sd_sec_read,
    output reg [31:0]           sd_sec_read_addr,
    input  [7:0]                sd_sec_read_data,
    input                       sd_sec_read_data_valid,
    input                       sd_sec_read_end,

    output reg                  bmp_data_wr_en,
    output reg [23:0]           bmp_data,
    output reg                  load_data_go,   // BMP 头解析完毕、像素数据阶段开始的单拍脉冲
    output [31:0]               src_width,      // 解析出的源图宽度（load_data_go 起有效）
    output [31:0]               src_height,     // 解析出的源图高度（load_data_go 起有效）
    output [31:0]               bpb_first_data, // FAT 数据区起始扇区（供音频流播定位）
    output                      bpb_valid       // FAT 数据区参数是否有效
);

localparam ST_IDLE      = 3'd0;
localparam ST_SCAN      = 3'd1;
localparam ST_LOAD_HDR  = 3'd2;
localparam ST_LOAD_WAIT = 3'd3;
localparam ST_LOAD_DATA = 3'd4;
localparam ST_FAT_MBR   = 3'd5;
localparam ST_FAT_BPB   = 3'd6;

// 扫描提前终止阈值：连续 EMPTY_STOP 个“空白”扇区（全 0x00 或全 0xFF）视为文件尾部。
// 阈值必须大于 FAT 单文件簇尾 slack 上限（64KB 簇 = 128 扇区，本工程最坏 slack 约 119 扇区），
// 否则会把图片之间的零填充区误判成文件尾，导致后面的图丢失。256 扇区(128KB)既安全又几乎
// 不增加上电时间；当实际图片数少于 scan_target_count 时，仍能远早于“扫完整个 64MB”就收尾。
localparam [15:0] EMPTY_STOP = 16'd256;

reg [2:0]  state;
reg [9:0]  rd_cnt;

reg [7:0]  header_0;
reg [7:0]  header_1;
reg [31:0] file_len;
reg [31:0] pixel_offset;
reg [31:0] width;
reg [31:0] height;
reg [15:0] bit_count;
reg [31:0] compression;

reg [31:0] scan_sector;
reg [31:0] load_sector_latched;
reg [31:0] bmp_len_cnt;
reg [1:0]  bmp_byte_idx;

// 扫描阶段空白/文件尾部检测
reg        sector_any_other;   // 当前扫描扇区是否含非 0x00/0xFF 的真实数据字节
reg [15:0] scan_blank_run;     // 连续空白扇区计数

// FAT 前置解析：MBR 分区 LBA 与 BPB 数据区参数
reg [31:0] partition_lba;
reg [7:0]  mbr_type;
reg [31:0] mbr_lba;
reg [15:0] bps;
reg [7:0]  spc;
reg [15:0] bpb_reserved;
reg [7:0]  bpb_num_fat;
reg [15:0] bpb_root_ent;
reg [15:0] bpb_fat16;
reg [31:0] bpb_fat32;

wire header_match;
wire scan_match;
wire bmp_data_valid;
wire [31:0] file_sector_count;
wire [31:0] next_scan_sector_if_match;
wire [31:0] next_scan_sector_if_miss;

assign ready = (state == ST_IDLE);
// 放宽匹配：接受任意分辨率的 24bit 无压缩 BMP（宽 640~2048、高 480~2048、正高度）。
// 正高度 = 自底向上存储；负高度(top-down)不支持，直接判为不匹配。
// 缩放器 bmp_scaler 仅处理缩小/等大（源 >= 640x480），故下限定为 640/480。
assign header_match = (header_0 == "B") &&
                      (header_1 == "M") &&
                      (width  >= 32'd640) && (width  <= 32'd2048) &&
                      (height >= 32'd480) && (height <= 32'd2048) &&
                      (height[31]     == 1'b0) &&
                      (bit_count    == 16'd24) &&
                      (compression  == 32'd0);
// width/height 在 header_match 中已限定 <=2048（高 20 位必为 0），故截取低 12 位即可完整
// 覆盖合法范围；当高位置位时 header_match 必为假、scan_match 随之必为假，截断不影响功能判定。
// 将 32x32 乘法降为 12x12，消除单周期乘加链（2x MULT18 + 7 ADDER）造成的 setup 违例。
wire [23:0] width_h  = {12'd0, width[11:0]};
wire [23:0] height_h = {12'd0, height[11:0]};
wire [31:0] bmp_file_len = 32'd54 + width_h * height_h * 32'd3;
// 仅扫描阶段用更严格的 scan_match：额外要求 file_len 精确等于 54 + width*height*3，
// 以剔除“已删除文件残留”的幽灵图，同时兼容不同分辨率的合法 BMP。
assign scan_match = header_match && (file_len == bmp_file_len);

assign src_width  = width;
assign src_height = height;
assign bmp_data_valid = (sd_sec_read_data_valid == 1'b1) &&
                        (bmp_len_cnt >= pixel_offset) &&
                        (bmp_len_cnt <  file_len);
assign file_sector_count = (file_len == 32'd0) ? 32'd1 : ((file_len + 32'd511) >> 9);
// FAT 文件按“簇”(spc 扇区/簇)对齐存储：一张图占用完整簇，相邻文件的下一张起始扇区
// = 当前扇区 + 按簇向上取整的扇区数。按簇对齐跳过可以直接命中下一张图头部，避免扫到
// 本张图簇尾的零填充区(slack)——否则这些全 0/0xFF 扇区会被下方的空白尾检测误判为
// “文件尾部”而提前终止扫描，导致后面的图(如第 4 张)被丢掉。
wire [31:0] spc32                = {24'd0, spc};
wire [31:0] file_cluster_sectors  = (file_sector_count + spc32 - 32'd1) & ~(spc32 - 32'd1);
wire [31:0] file_skip_sectors     = bpb_valid ? file_cluster_sectors : file_sector_count;
assign next_scan_sector_if_match = scan_sector + file_skip_sectors;
assign next_scan_sector_if_miss  = scan_sector + 32'd1;

// 数据区起点计算：跳过 MBR/FAT/根目录这类高扇区号的死扫描。
// FAT16：根目录位于 FAT 之后的固定区域；FAT32：根目录在数据区簇链内（root_ent=0）。
wire       bpb_is_fat16        = (bpb_fat16 != 16'd0);
wire [31:0] bpb_fat_size       = bpb_is_fat16 ? {16'd0, bpb_fat16} : bpb_fat32;
wire [31:0] bpb_root_dir_sectors = bpb_is_fat16 ?
                                   (({16'd0, bpb_root_ent} * 32'd32 + 32'd511) >> 9) :
                                   32'd0;
assign bpb_first_data = partition_lba + {16'd0, bpb_reserved}
                         + ({24'd0, bpb_num_fat} * bpb_fat_size)
                         + bpb_root_dir_sectors;
assign bpb_valid       = (bps == 16'd512) && (spc != 8'd0) &&
                         (bpb_num_fat != 8'd0) && (bpb_fat_size != 32'd0);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        rd_cnt <= 10'd0;
    end else if ((state == ST_SCAN) || (state == ST_LOAD_HDR) ||
                 (state == ST_FAT_MBR) || (state == ST_FAT_BPB)) begin
        if (sd_sec_read_data_valid)
            rd_cnt <= rd_cnt + 10'd1;
        else if (sd_sec_read_end)
            rd_cnt <= 10'd0;
    end else begin
        rd_cnt <= 10'd0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        header_0     <= 8'd0;
        header_1     <= 8'd0;
        file_len     <= 32'd0;
        pixel_offset <= 32'd54;
        width        <= 32'd0;
        height       <= 32'd0;
        bit_count    <= 16'd0;
        compression  <= 32'd0;
    end else if (((state == ST_SCAN) || (state == ST_LOAD_HDR)) && sd_sec_read_data_valid) begin
        case (rd_cnt)
            10'd0 : header_0 <= sd_sec_read_data;
            10'd1 : header_1 <= sd_sec_read_data;

            10'd2 : file_len[7:0] <= sd_sec_read_data;
            10'd3 : file_len[15:8] <= sd_sec_read_data;
            10'd4 : file_len[23:16] <= sd_sec_read_data;
            10'd5 : file_len[31:24] <= sd_sec_read_data;

            10'd10: pixel_offset[7:0] <= sd_sec_read_data;
            10'd11: pixel_offset[15:8] <= sd_sec_read_data;
            10'd12: pixel_offset[23:16] <= sd_sec_read_data;
            10'd13: pixel_offset[31:24] <= sd_sec_read_data;

            10'd18: width[7:0] <= sd_sec_read_data;
            10'd19: width[15:8] <= sd_sec_read_data;
            10'd20: width[23:16] <= sd_sec_read_data;
            10'd21: width[31:24] <= sd_sec_read_data;

            10'd22: height[7:0] <= sd_sec_read_data;
            10'd23: height[15:8] <= sd_sec_read_data;
            10'd24: height[23:16] <= sd_sec_read_data;
            10'd25: height[31:24] <= sd_sec_read_data;

            10'd28: bit_count[7:0] <= sd_sec_read_data;
            10'd29: bit_count[15:8] <= sd_sec_read_data;

            10'd30: compression[7:0] <= sd_sec_read_data;
            10'd31: compression[15:8] <= sd_sec_read_data;
            10'd32: compression[23:16] <= sd_sec_read_data;
            10'd33: compression[31:24] <= sd_sec_read_data;
            default: ;
        endcase
    end
end

// FAT 前置解析字节收集：MBR(分区类型/LBA) 与 BPB(数据区参数)
always @(posedge clk or posedge rst) begin
    if (rst) begin
        mbr_type     <= 8'd0;
        mbr_lba      <= 32'd0;
        bps          <= 16'd0;
        spc          <= 8'd0;
        bpb_reserved <= 16'd0;
        bpb_num_fat  <= 8'd0;
        bpb_root_ent <= 16'd0;
        bpb_fat16    <= 16'd0;
        bpb_fat32    <= 32'd0;
    end else if (state == ST_FAT_MBR && sd_sec_read_data_valid) begin
        case (rd_cnt)
            10'd450: mbr_type      <= sd_sec_read_data;
            10'd454: mbr_lba[7:0]  <= sd_sec_read_data;
            10'd455: mbr_lba[15:8] <= sd_sec_read_data;
            10'd456: mbr_lba[23:16] <= sd_sec_read_data;
            10'd457: mbr_lba[31:24] <= sd_sec_read_data;
            default: ;
        endcase
    end else if (state == ST_FAT_BPB && sd_sec_read_data_valid) begin
        case (rd_cnt)
            10'd11: bps[7:0]  <= sd_sec_read_data;
            10'd12: bps[15:8] <= sd_sec_read_data;
            10'd13: spc       <= sd_sec_read_data;
            10'd14: bpb_reserved[7:0]  <= sd_sec_read_data;
            10'd15: bpb_reserved[15:8] <= sd_sec_read_data;
            10'd16: bpb_num_fat        <= sd_sec_read_data;
            10'd17: bpb_root_ent[7:0]  <= sd_sec_read_data;
            10'd18: bpb_root_ent[15:8] <= sd_sec_read_data;
            10'd22: bpb_fat16[7:0]  <= sd_sec_read_data;
            10'd23: bpb_fat16[15:8] <= sd_sec_read_data;
            10'd36: bpb_fat32[7:0]   <= sd_sec_read_data;
            10'd37: bpb_fat32[15:8]  <= sd_sec_read_data;
            10'd38: bpb_fat32[23:16] <= sd_sec_read_data;
            10'd39: bpb_fat32[31:24] <= sd_sec_read_data;
            default: ;
        endcase
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_len_cnt <= 32'd0;
    end else if ((state == ST_LOAD_DATA)) begin
        if (sd_sec_read_data_valid)
            bmp_len_cnt <= bmp_len_cnt + 32'd1;
    end else begin
        bmp_len_cnt <= 32'd0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_byte_idx <= 2'd0;
    end else if ((state == ST_LOAD_DATA)) begin
        if (bmp_data_valid)
            bmp_byte_idx <= (bmp_byte_idx == 2'd2) ? 2'd0 : (bmp_byte_idx + 2'd1);
    end else begin
        bmp_byte_idx <= 2'd0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_data_wr_en <= 1'b0;
        bmp_data       <= 24'd0;
    end else if ((state == ST_LOAD_DATA)) begin
        if (bmp_data_valid) begin
            case (bmp_byte_idx)
                2'd0: begin
                    bmp_data_wr_en <= 1'b0;
                    bmp_data[7:0]  <= sd_sec_read_data;
                end
                2'd1: begin
                    bmp_data_wr_en <= 1'b0;
                    bmp_data[15:8] <= sd_sec_read_data;
                end
                2'd2: begin
                    bmp_data_wr_en  <= 1'b1;
                    bmp_data[23:16] <= sd_sec_read_data;
                end
                default: begin
                    bmp_data_wr_en <= 1'b0;
                end
            endcase
        end else begin
            bmp_data_wr_en <= 1'b0;
        end
    end else begin
        bmp_data_wr_en <= 1'b0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state             <= ST_IDLE;
        state_code        <= 4'd0;
        sd_sec_read       <= 1'b0;
        sd_sec_read_addr  <= 32'd0;
        write_req         <= 1'b0;
        load_data_go      <= 1'b0;
        scan_done         <= 1'b0;
        scan_found_valid  <= 1'b0;
        scan_found_sector <= 32'd0;
        scan_found_total  <= 3'd0;
        scan_sector       <= 32'd0;
        load_sector_latched <= 32'd0;
        partition_lba     <= 32'd0;
        sector_any_other  <= 1'b0;
        scan_blank_run    <= 16'd0;
    end else if (!sd_init_done) begin
        state             <= ST_IDLE;
        state_code        <= 4'd0;
        sd_sec_read       <= 1'b0;
        sd_sec_read_addr  <= 32'd0;
        write_req         <= 1'b0;
        load_data_go      <= 1'b0;
        scan_done         <= 1'b0;
        scan_found_valid  <= 1'b0;
        scan_found_sector <= 32'd0;
        scan_found_total  <= 3'd0;
        scan_sector       <= 32'd0;
        load_sector_latched <= 32'd0;
        partition_lba     <= 32'd0;
        sector_any_other  <= 1'b0;
        scan_blank_run    <= 16'd0;
    end else if (load_abort) begin
        // 仅中止当前装载并回到空闲，保留扫描结果（scan_done / scan_sector 等）。
        // 若在此一并清除 scan_done，上层流水会因“扫描结果丢失且 scan_kicked 已置位”
        // 而整条卡死——坏图/幽灵图超时后底图将永远不再出现。
        state      <= ST_IDLE;
        state_code <= 4'd0;
        sd_sec_read <= 1'b0;
        write_req  <= 1'b0;
        load_data_go <= 1'b0;
    end else begin
        scan_found_valid <= 1'b0;
        load_data_go     <= 1'b0;

        case (state)
            ST_IDLE: begin
                state_code  <= 4'd1;
                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;

                if (scan_start) begin
                    scan_done        <= 1'b0;
                    scan_found_total <= 3'd0;
                    // 先读 MBR 拿分区 LBA，再读 BPB 算数据区起点，最后才从数据区开始
                    // 暴力扫描——跳过高扇区号的 MBR/FAT/根目录死扫描，缩短上电到出图时间。
                    sd_sec_read_addr <= 32'd0;
                    state            <= ST_FAT_MBR;
                end else if (load_start) begin
                    load_sector_latched <= load_sector;
                    sd_sec_read_addr    <= load_sector;
                    state               <= ST_LOAD_HDR;
                end
            end

            ST_SCAN: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;

                // 每个扫描扇区累积“是否含真实数据字节”（非 0x00 / 非 0xFF）
                if (sd_sec_read_data_valid) begin
                    if ((sd_sec_read_data != 8'd0) && (sd_sec_read_data != 8'hFF))
                        sector_any_other <= 1'b1;
                end

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;

                    if (scan_match) begin
                        scan_found_valid  <= 1'b1;
                        scan_found_sector <= scan_sector;
                        scan_found_total  <= scan_found_total + 3'd1;
                        scan_blank_run    <= 16'd0;
                        sector_any_other  <= 1'b0;

                        if ((scan_found_total + 3'd1 >= scan_target_count) || (next_scan_sector_if_match > scan_max_sector)) begin
                            scan_done        <= 1'b1;
                            state            <= ST_IDLE;
                            sd_sec_read_addr <= next_scan_sector_if_match;
                            scan_sector      <= next_scan_sector_if_match;
                        end else begin
                            sd_sec_read_addr <= next_scan_sector_if_match;
                            scan_sector      <= next_scan_sector_if_match;
                        end
                    end else begin
                        // 文件尾部提前终止：连续多个空白扇区后认为没有更多图片，
                        // 避免图片数少于 scan_target_count 时扫完整个 SD 容量而拖慢首图。
                        if (!sector_any_other) begin
                            sector_any_other <= 1'b0;
                            if (scan_blank_run >= (EMPTY_STOP - 16'd1)) begin
                                scan_done <= 1'b1;
                                state     <= ST_IDLE;
                            end else begin
                                scan_blank_run     <= scan_blank_run + 16'd1;
                                sd_sec_read_addr   <= next_scan_sector_if_miss;
                                scan_sector        <= next_scan_sector_if_miss;
                            end
                        end else begin
                            sector_any_other <= 1'b0;
                            scan_blank_run   <= 16'd0;
                            if (scan_sector >= scan_max_sector) begin
                                scan_done <= 1'b1;
                                state     <= ST_IDLE;
                            end else begin
                                sd_sec_read_addr <= next_scan_sector_if_miss;
                                scan_sector      <= next_scan_sector_if_miss;
                            end
                        end
                    end
                end
            end

            ST_FAT_MBR: begin
                state_code  <= 4'd5;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    // 识别 MBR 分区表：分区类型为 FAT 且分区 LBA 非 0 才按 MBR 处理，
                    // 否则按 superfloppy（MBR 缺失，BPB 在扇区 0）处理。
                    if (((mbr_type == 8'h0B) || (mbr_type == 8'h0C) ||
                         (mbr_type == 8'h06) || (mbr_type == 8'h0E) ||
                         (mbr_type == 8'h04) || (mbr_type == 8'h01)) &&
                        (mbr_lba != 32'd0)) begin
                        partition_lba    <= mbr_lba;
                        sd_sec_read_addr <= mbr_lba;
                    end else begin
                        partition_lba    <= 32'd0;
                        sd_sec_read_addr <= 32'd0;
                    end
                    state <= ST_FAT_BPB;
                end
            end

            ST_FAT_BPB: begin
                state_code  <= 4'd5;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    // 数据区起点作为扫描起始；解析失败则回退从 0 扫描，保证不破坏原功能
                    if (bpb_valid) begin
                        scan_sector      <= bpb_first_data;
                        sd_sec_read_addr <= bpb_first_data;
                    end else begin
                        scan_sector      <= 32'd0;
                        sd_sec_read_addr <= 32'd0;
                    end
                    state <= ST_SCAN;
                end
            end

            ST_LOAD_HDR: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (header_match) begin
                        write_req        <= 1'b1;
                        sd_sec_read_addr <= load_sector_latched;
                        state            <= ST_LOAD_WAIT;
                    end else begin
                        state      <= ST_IDLE;
                    end
                end
            end

            ST_LOAD_WAIT: begin
                state_code <= 4'd3;
                if (write_req_ack) begin
                    write_req    <= 1'b0;
                    load_data_go <= 1'b1;
                    state        <= ST_LOAD_DATA;
                end
            end

            ST_LOAD_DATA: begin
                state_code  <= 4'd4;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (bmp_len_cnt >= file_len) begin
                        state <= ST_IDLE;
                    end else begin
                        sd_sec_read_addr <= sd_sec_read_addr + 32'd1;
                    end
                end
            end

            default: begin
                state       <= ST_IDLE;
                state_code  <= 4'd1;
                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;
            end
        endcase
    end
end

endmodule
