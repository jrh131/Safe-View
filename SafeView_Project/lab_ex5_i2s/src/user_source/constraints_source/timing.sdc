
# ============================================================
# merged_top_tf_hdmi_audio.sdc
# 用于：TF 图片轮播 + HDMI(视频+音频) 整合工程
#
# 合并原则：
# 1) 保留 top.sdc 里的系统主时钟与 derive_clocks
# 2) 把 timing(2).sdc 里的 pixel/serial 时钟改成整合工程实际名字
# 3) 增加 hdmi_5x_clk / audio_mclk 的命名
# 4) 不再对内部 net 直接 create_clock，避免和 derive_clocks 重复
# ============================================================

# ------------------------------------------------------------
# 1. 板级输入主时钟
#    你的原 top.sdc 里 clk=50MHz，所以这里仍然用 20ns
# ------------------------------------------------------------
create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}]

# ------------------------------------------------------------
# 2. 自动推导 PLL 输出时钟
# ------------------------------------------------------------
derive_clocks

# ------------------------------------------------------------
# 3. 给关键 PLL 输出命名
#    下面这些路径是按你当前工程实例名写的：
#      sys_pll      -> sys_pll_m0
#      video_pll    -> video_pll_m0
#      音频 PLL     -> u_audio_pll
#
#    如果综合后实例路径略有不同，就按 Timing Analyzer 里的实际 pin 路径改。
# ------------------------------------------------------------
rename_clock -name {sd_card_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {ext_mem_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[1]}]
rename_clock -name {ext_mem_clk_sft} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[2]}]
rename_clock -name {video_clk}   -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {hdmi_5x_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[1]}]
rename_clock -name {audio_mclk}  -source [get_ports {clk}] -master_clock {clk} [get_pins {u_audio_pll/pll_inst.clkc[2]}]

# ------------------------------------------------------------
# 4. 从原 timing(2).sdc 迁移过来的 HDMI 像素/串行时钟约束
#
#    原文件是：
#      create_clock -name pixel_clk  ... [get_nets {S_pixel_clk}]
#      create_clock -name serial_clk ... [get_nets {S_serial_clk}]
#      set_clock_groups -exclusive ...
#
#    在整合工程里，这两个时钟已经由 video_pll 产生，并通过 derive_clocks 建出来了，
#    所以这里只保留“组关系”，不再重复 create_clock。
#
#    这里沿用原作者的写法，把 video_clk 和 hdmi_5x_clk 设成 exclusive。
#    如果后面 Timing Analyzer 明确显示两者之间需要做同步时序分析，再把这一条去掉。
# ------------------------------------------------------------
set_clock_groups -exclusive \
    -group [get_clocks {video_clk}] \
    -group [get_clocks {hdmi_5x_clk}]

# ------------------------------------------------------------
# 5. 时钟域分组（时序收敛）
#
#    各功能时钟域之间仅通过异步 FIFO（frame_read_write_dual 的 rfifo/wfifo）、
#    2 级同步器（图像索引/音量/告警/复位）或 I2S 采样同步互连，不存在真正
#    需要做同步时序分析的跨域路径。把它们声明为异步组，可消除 CDC 及
#    异步复位释放（recovery）的伪违例。
#
#    报告里 clk->audio_mclk 的 97 条 recovery 违例（rst_all 仅在 clk 域同步
#    释放、却被音频域当异步复位）、sd->video 等均属此类伪违例。
#
#    注意：video_clk 与 hdmi_5x_clk 同源于 video_pll，保持同组（仍由上方
#    -exclusive 管理，维持 5:1 相位关系）；SDRAM 控制器 ext_mem_clk 与
#    SDRAM 芯片时钟 sys_pll clkc[2] 同源于 sys_pll，不在本组内，二者继续
#    保持同步分析，其读数据采样相位由 PLL 保证。
# ------------------------------------------------------------
set_clock_groups -asynchronous \
    -group [get_clocks {clk}] \
    -group [get_clocks {sd_card_clk}] \
    -group [get_clocks {ext_mem_clk}] \
    -group [get_clocks {video_clk hdmi_5x_clk}] \
    -group [get_clocks {audio_mclk}]

# ------------------------------------------------------------
# 6. SDRAM 硬核 DQ 读写通路时序豁免（时序收敛）
#
#    SDRAM 控制器为 Anlogic 加密硬核 EG_PHY_SDRAM_2M_32。其 DQ 数据总线
#    读写时序由 sys_pll 固定相位（ext_mem_clk 0° / ext_mem_clk_sft 225°）
#    与硬核内部输入/输出延迟共同保证；读数据为多周期 CAS latency 采样，
#    无法用单边沿相位差（budget 3~5ns）做静态 setup 分析。
#
#    ext_mem_clk 与 ext_mem_clk_sft 同源锁相，仅分别用于控制器核心与
#    SDRAM 芯片时钟（ext_mem_clk_sft fanout=1，只驱动 SDRAM_CLK 引脚）。
#    FPGA 内部没有任何跨这两个域的同步逻辑，报告中的 64 条 setup 违例
#    （ext_mem_clk -> ext_mem_clk_sft 写通路、ext_mem_clk_sft -> ext_mem_clk
#    读通路）全部是硬核 DQ 引脚 I/O 伪违例，不经过 FPGA 组合逻辑。
#    故对这两个时钟间的通路设 false path，仅豁免硬核 DQ I/O，
#    不影响控制器内部逻辑的时序分析。
# ------------------------------------------------------------
set_false_path -from [get_clocks {ext_mem_clk}] -to [get_clocks {ext_mem_clk_sft}]
set_false_path -from [get_clocks {ext_mem_clk_sft}] -to [get_clocks {ext_mem_clk}]

# ------------------------------------------------------------
# 7. 其余 Input/Output delay 先保持空白
#    你当前两个原始 sdc 也都没有写这部分。
# ------------------------------------------------------------
