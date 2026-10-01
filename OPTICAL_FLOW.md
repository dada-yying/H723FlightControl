# MTF01 光流接收与双串口记录

> UART7/F407 已升级为自描述 V3，当前黑匣子格式和升级步骤见 [BLACKBOX_V3.md](BLACKBOX_V3.md)。
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
