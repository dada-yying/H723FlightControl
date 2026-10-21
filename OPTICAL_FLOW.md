# MTF01 光流接收与双串口记录

> 当前黑匣子已改为H723通过SPI直接写SD卡，独立1kHz，字段保持自描述V3；见 [BLACKBOX_V3.md](BLACKBOX_V3.md)。本文下方UART7/F407传输及测试描述属于历史版本，光流接收与字段含义保持不变。
> 下文116字节黑匣子帧和V2存储说明仅为历史记录；光流接收及VOFA96字节输出不变。

本轮仅接收、解析、显示和记录；不修改 PID、卡尔曼滤波、轴向或电机安全逻辑。

## 接线与执行位置

- 传感器 TX → PE2 / USART10_RX / AF4；共地，使用兼容 3.3 V 的串口电平。
- PE3 / USART10_TX / AF11 预留，本轮不发送配置命令。
- USART10：115200、8N1、无流控、不反相；共享 USART1 的 D2PCLK2 时钟源。
- DMA1_Stream2 循环接收，256 字节，USART10/DMA 中断优先级均为 6。
- 初始化位于 MX_FREERTOS_Init 创建任务之前；启动接收、解析与恢复位于 OpticalFlowTask。
- 独立任务 Normal 优先级、1024 字节栈，通知驱动，最多每 10 ms 检查恢复。
- 中断按实时 NDTR 搬运新增字节到 1024 字节软件队列，保留每批接收时间；不运行解析或飞控。
- 接收 DMA 独占缓存行，在 AXI SRAM 中；CPU 只读 RX 缓冲，收到数据后失效缓存，不把旧缓存写回 DMA。

解析严格检查 EF / ID51 / LEN20 / 27 字节总长 / 累加校验。设备和系统 ID 不限制；
未知消息不发布。完整原始帧、所有解码字段、状态和保留字段均保存。
错误帧不覆盖最后样本，距离 0 原样记录，状态码不猜测有效性。
UART 错误或队列溢出后丢弃不完整数据并由任务重启；失败每 100 ms 重试。
外设或任务初始化失败仅使光流不可用，不调用全局 Error_Handler。

## 输出格式

原 CH0～19 的映射不变，两路共享基础 80 字节：

| 输出 | 配置 | 新增内容 | 总长度 |
|---|---|---|---|
| USART1 / VOFA | 230400 / 100 Hz / JustFloat | CH20 距离 mm；CH21/22 原始 X/Y，cm/s@1m | 96 字节 |
| UART7 / 黑匣子 | 2 Mbps / 随飞控约 1 kHz / DMA | 27 字节原始帧、4 字节年龄、1 字节 has_sample | 116 字节 |

UART7 字节 80～106 原始帧，107～110 小端 uint32 年龄 ms，111 标志，112～115 为 00 00 80 7F。
这一路不再是纯 JustFloat，不可直接作为 VOFA float 通道解析。
无样本时 raw27 全零、年龄 FFFFFFFF、标志 0；断流保留最后样本且年龄增长。
年龄按接收时间计算，任务维护扩展 tick，支持 uint32 回绕并在 FFFFFFFF 饱和；要求任务正常运行。
原始整数经 UART7 及 CSV 保持精度；VOFA 的距离转换成 float，极大 uint32 值可能有浮点舍入。
原始光流不是实际水平速度，本轮不乘高度。

UART7 DMA 缓冲 128 字节，32 字节对齐，实际发送 116 字节。
UART7 约占线路 58%，每帧约 580 us；USART1 约占 41.7%，每帧约 4.17 ms。
两个串口分别获取快照，不承诺两个串口同一时刻的数据相同；每帧内部原始/解码光流来自同一快照。

F407 必须同步烧录版本 2，日志魔数 SKYBBX2\0，每条记录 128 字节；每块 31 条记录。
F407 tools/blackbox_to_csv.py 导出原 20 通道及全部光流整数、年龄、标志和原始十六进制。
新版不接受旧 84 字节发送端及版本 1 日志。详见另一工程 docs/log-format.md。

## Watch 与拆桨验收

### X/Y 全零问题的分层诊断（2026-10-01）

新增观察量 `g_optical_flow_diag`，`version=0x00010002`。不改变收帧条件、
原始数据、VOFA/黑匣子格式或任何电机保护。诊断不写入黑匣子，也不额外读取 UART RDR。
上电/OpticalFlow_Init 清零；UART 重启保留累计计数及非零证据。

#### DMA 与直接串口中断的对照

上电 `requested_rx_mode=0`、`rx_mode=0`，沿用循环 DMA。拆桨、锁定后在 Watch
写入 `g_optical_flow_diag.requested_rx_mode=1`；光流任务停止原接收、清除不完整数据，
改用 `HAL_UART_Receive_IT(..., 1)` 逐字节接收。HAL 从 USART10 RDR 读取字节后，
通过独立完成回调搬入同一个软件队列；复用同一解析、发布和遥测，不在 ISR 中解析。
该路径不使用 DMA 缓冲或缓存维护，便于对比实际板上接收路径。写回 0 恢复 DMA；
其他请求值忽略。不要直接修改只读观察字段 `rx_mode`。该开关仅用于诊断，不保存 Flash。

切换完成以 `rx_mode` 为准；`rx_mode_switch_count` 记录切换次数。
`it_rx_byte_count` 记录直接中断收到的字节；`it_rearm_failures` 记录单字节重启失败，
失败沿用任务侧恢复。按模式分别累计以下数组，索引 0=DMA、1=直接串口中断：

| 字段 | 含义 |
|---|---|
| rx_mode_valid_frame_count[0/1] | 格式和校验通过的帧 |
| rx_mode_raw_nonzero_frame_count[0/1] | 原始 18～21 字节有非零值的帧 |
| rx_mode_nonzero_frame_count[0/1] | 解码 X/Y 有非零值的帧 |
| raw_decode_mismatch_count | 原始 X/Y 的 16 位值与解码值不一致的次数 |

直接中断模式下，旧名 `dma_trace` 记录 HAL 已读入的字节；其环形索引和更新标记
继续适用，每个搬运片段为 1 字节。`dma_last_remaining/dma_last_pos` 仅对 DMA 模式有意义，
`rx_event_count` 仅统计 DMA/IDLE 事件，不能拿它代替 `it_rx_byte_count`。
对照时保持相同供电、位置、光照和移动方式；若同时旁路记录传感器 TX，只将 USB-TTL RX
接到同一根 TX，并共地。比对原始帧时使用传感器序号及 time_ms 对齐。

若 DMA 始终零而中断模式有非零帧，继续定位 DMA/缓存路径；若两个模式均收到大量校验
通过的帧且原始非零计数均为零，需对齐同一根 TX 的旁路记录，不能继续归因于任务覆盖。
原始非零计数有值但解码计数没有，结合 mismatch_count 检查解码/内存一致性。
中断模式改变 USART10 接收和通知的节拍，应检查 UART 错误、队列峰值和飞控频率；
其主机回归及编译通过不代替实机验证，也不表示全零根因已经确认。

首次测试拆桨并锁定，持续平移传感器，优先观察：

| Watch 字段（均属于 g_optical_flow_diag） | 含义 |
|---|---|
| rx_event_count / rx_ignored_count / rx_empty_count | 光流接收回调总数／接收未开启或恢复中而忽略／DMA 位置未变化；重复 IDLE/HT 等导致 empty 增长是正常现象 |
| valid_frame_count | 格式及校验通过的 0x51 帧数，包含发布时因重启而被丢弃的帧 |
| nonzero_frame_count | 有效帧中 X 或 Y 非零的数量，不判断质量或状态有效性 |
| published_nonzero_count / publish_discard_count | 已发布的非零帧／发布时重启已挂起而丢弃的帧 |
| last_nonzero_x / last_nonzero_y | 最近一帧非零 X/Y，后续零帧不覆盖；不替代正常控制/遥测数据 |
| last_nonzero_raw[27] | 最近非零帧的完整原始字节；X 在 18/19，Y 在 20/21，小端 int16 |
| last_nonzero_seq / last_nonzero_sensor_ms / last_nonzero_rx_tick | 对应序号、传感器时间、接收回调 HAL tick |
| last_valid_raw[27] | 最近一帧有效数据（包括零帧），发布前留存 |
| queue_high_water_bytes | 软件队列峰值，最大 1024；不是 DMA 覆盖检测器 |
| task_max_interval_ms / task_max_batch_frames | 处理函数调用最大间隔／单次调用发布的最大帧数 |
| max_publish_age_ms | 发布时距末字节搬运回调的最大时间，不包含回调执行前的硬件接收延迟 |
| rx_max_interval_ms | 接收回调的最大间隔，断流或恢复也会增大；不等同于中断响应延迟 |
| seq_discontinuity_count / duplicate_seq_count | 同设备/系统连续有效帧序号增量不是 1／增量为 0；包含重复、复位、乱序等，不是确切丢帧数 |

正常 255→0 序号回绕不算跳变；UART 重启或设备/系统 ID 变化重新建立序号基线。
Watch 若看到当前 X/Y=0，但 published_nonzero_count 增长，说明 H723 确实接收并发布过非零值，
不能再将当前快照为零理解为从未收到非零帧。若 valid_frame_count 增长而 nonzero_frame_count
始终为零，需要用同一时段、同一根传感器 TX 的旁路串口记录比对原始字节。

`dma_trace[512]` 循环保留 CPU 实际从 DMA 缓冲搬入队列的最近 512 字节；
`dma_trace_count` 是累计搬运字节数，下一写入下标为 `dma_trace_count % 512`。
不足 512 字节时从下标 0 开始；已有至少 512 字节时从上述下一写入下标开始环形读取。
`dma_last_chunk_begin` 是最近非空搬运片段的累计起点，`dma_last_chunk_size` 是该片段实际搬运长度；
片段第 i 字节在 `dma_trace[(dma_last_chunk_begin+i)%512]`。
`dma_last_remaining/dma_last_pos` 保存最近服务的 NDTR 和位置；
`restart_trace_count` 标记最近一次重启清空队列时的历史边界，不应跨重启把字节拼成一帧。
这些 32 位累计计数按模 2^32 回绕，长时间运行需考虑回绕，短时诊断可重新上电建立基线。

DMA 历史不是 RDR/引脚的独立抓包，不能证明 DMA 未发生覆盖，也不包含溢出后未搬运的字节；
整圈未服务仍可能不可见，empty_count 不能区分整圈覆盖与正常重复通知。
`dma_capture_seq`、`frame_capture_seq` 分别对 DMA 片段/原始帧留存采用奇偶更新标记：
奇数表示正在写入，偶数表示该次完成。它们不是整个诊断结构的一致快照锁。
要比对多字段/完整数组，拆桨锁定后暂停 CPU，并确认相应标记为偶数；若为奇数，运行后重新暂停。
不要在 DMA 接收期间通过 Watch 或外设窗口反复读取 RDR 来做抓包。

主机测试 `tests/pid_tuning/test_optical_diag.c` 覆盖非零锁存、零帧覆盖、批量发布、
DMA 历史回绕、队列满、恢复保留、序号/时间回绕和发布丢弃。实际诊断开销、调度、
DMA 时序仍需拆桨实测，不据此宣称已解决全零根因。

`g_optical_flow_data`：has_sample、rx_count、received_tick、age_ms、distance、flow_vel_x/y、
time_ms、seq、tof_status、flow_status 及 raw[27]。

`g_optical_flow_stats`：initialized、receiving、task_count、byte_count、frame_count、
checksum_errors、length_errors、queue_overflows、uart_errors、restart_count、restart_failures、last_status、last_uart_error。
restart_count 包括首次启动。heap_free_bytes / heap_min_free_bytes 为实时/最低堆余量；
optical_stack_free_words、default_stack_free_words、flight_stack_free_words 为最小剩余栈，单位 4 字节 word。

1. 两板同步升级，先拆桨；VOFA 使用 230400、JustFloat，观察新增 CH20～22。
2. 核对距离、X/Y 方向变化、传感器 time_ms/seq 更新；此步骤不验证光流闭环坐标。
3. 拔掉光流，确认数值保持、age_ms 增长；重连能恢复，飞控保护规则没有改变。
4. 同时记录至少 60 秒，检查 UART 错误、队列溢出、黑匣子忙丢帧不增长，堆/任务栈余量非零。
5. 检查 SD 文件新版工具可恢复，has_sample、年龄、原始帧与 Watch 对应。
6. IMU 断流后 UART7 不继续周期发帧（沿用既有设计）；USART1 和光流任务仍运行。

## 软件验证

`tests/pid_tuning/run.ps1`：原有飞控/电调回归、光流解析/DMA/错误恢复、组帧验证。
F407 的 `tests/run_cross.ps1`：重新生成 H723 真实 116 字节帧，再走 F407 实际解析与 FatFs 写盘，最后 Python 恢复多分段日志逐字段比较。
覆盖无样本、重复样本、损坏扩展、旧日志拒绝、队列满与存储故障。

ARMCC 构建：`MDK-ARM/build_ws2812.ps1 -OutputDirectory optical-flow-verified`。
静态检查：`python tests/verify_optical_build.py`。
当前最大静态调用栈：默认任务 504 字节，飞控任务 408 字节，光流任务 160 字节，各配置 1024 字节。
不包含完整运行时上下文和所有间接调用影响，必须结合实机高水位复核。
F407 普通 SRAM 使用 115720 / 131072 字节，余 15352 字节；记录数组仍为 98304 字节。
编译、主机测试和静态分析均不替代实机缓存/中断时序、传感器、SD 卡及飞行稳定性验证。
