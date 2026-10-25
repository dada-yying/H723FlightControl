# USART1 VOFA DMA 发送（2026-10-25）

默认任务仍每 10 ms 读取快照并组帧，目标 100 Hz。USART1 保持 PA9/PA10、230400、8N1；
23 个 float32 加 JustFloat 尾 `00 00 80 7F`，共 96 字节，通道含义和单位不变。
`VOFA_SendAttitude` 的 16 字节帧及 `VOFA_SendFlightTelemetry` 的 84 字节兼容接口也走 DMA。
SD 黑匣子使用的 `VOFA_BuildBlackboxFrame` 只负责组帧，传输仍由原 SPI-SD 路径负责。

## 资源与正常发送

- DMA1 Stream4、`DMA_REQUEST_USART1_TX`、普通模式、字节宽度、自增内存、低 DMA 优先级。
  DMA1 Stream0/1/2/3 保留给 UART5、SD TX、光流、SD RX。
- DMA1 Stream4 和 USART1 中断优先级均为 6。DMA TC 后 HAL 还会启用 UART TC 中断，
  最后一个字节真正发送完成后才调用 `HAL_UART_TxCpltCallback`；两条 IRQ 链路都必须保留。
  无用的 DMA 半传输中断在提交后关闭。
- `s_vofa_dma_frame` 为独立 `.vofa_dma` 段、96 字节、32 字节对齐，scatter 显式放入 AXI SRAM。
  发送前复制输入帧，启用 D-cache 时清理完整三条缓存行并执行 DSB。
  不让 DMA 读取调用者栈/DTCM，返回后调用者可立即复用原数组。
- 生产发送者仍为 defaultTask；发送接口为任务上下文接口，ISR 只报告完成/错误。
  不增加任务或发送队列。正常调用只复制/清理缓存/启动 DMA，立即返回，不轮询等待线路完成。

接口返回 `HAL_OK` 表示本帧已提交，**不表示已经发完**。一块缓冲区只允许一帧在途；
忙时返回 HAL_BUSY 并跳过新帧，保持在途内容不变，不排队补发旧快照。
96 字节线路时间仍约 4.17 ms，串口带宽占用仍约 41.7%；DMA 释放的是 CPU 轮询等待，
不会加快这条 230400 波特率的物理线路。CPU 改善和实际发送频率需实机测量。

## 错误、超时与观察

共享 `HAL_UART_ErrorCallback` 新增 USART1 分支，原 UART5/USART10 处理保留。
错误回调只标记恢复并记录错误。每次任务发送先维护上一次状态：
仍在途且达到原 10 ms 超时，或已有错误时，在任务中调用 HAL_UART_AbortTransmit；
成功停止 DMA/TC 后清理两条 pending IRQ，才允许复用缓冲区提交当前新快照。
中止失败继续保留缓冲区所有权，后续发送再次尝试恢复，不强行覆盖在途内存。
已服务的完成回调先于任务超时判定，中止期间或空闲时的重复/迟到完成不计为成功。
DMA 初始化失败时 USART1 不链接 TX DMA，VOFA 返回 HAL_ERROR，不因新增 DMA 失败停住飞控。

`g_vofa_tx_stats`：

| 字段 | 当前含义 |
|---|---|
| submitted_count | HAL 接受的 DMA 提交次数 |
| success_count | UART TC 回调确认完成的次数 |
| failure_count | DMA 启动失败、错误或超时的帧数；忙跳帧另计 |
| busy_drop_count | 因在途/恢复状态未能提交的新帧数 |
| timeout_count / dma_error_count | 等待完成超时 / 在途 UART-DMA 错误次数 |
| recovery_count / abort_failure_count | 成功中止恢复 / 中止失败次数 |
| last_status / last_uart_error | 最近操作状态 / 最近在途错误的 HAL ErrorCode |
| busy | 在途或恢复中，不能重用 DMA 缓冲区 |
| max_duration_ms | 提交至 UART TC 回调最大时间，含 IRQ 服务延迟；不再是阻塞 API 调用耗时 |

## 验证与修改文件

主机 `tests/pid_tuning/test_vofa_esc.c` 检查原帧字节、异步完成计数、调用者内存复用、
在途拒绝覆盖、D-cache 开/关、启动失败、错误/超时、中止失败保持所有权、中止期间迟到回调、
完成早于任务超时处理、重复/无关回调和 tick 回绕。
`test_tuning.c` 检查实际共享错误路由，USART1 错误不触发 UART5 重启。
原黑匣子/光流/IMU/控制及存储回归继续运行；既有加热器 CCR=100 断言问题单独报告。

生产修改：`App/vofa.c/.h`、`App/rc_sbus.c`（共享错误路由）、`Core/Src/usart.c`、
`Core/Inc/usart.h`、`Core/Src/stm32h7xx_it.c`、`Core/Inc/stm32h7xx_it.h`、
`MDK-ARM/sbus_memory.sct`、`CtrlBoard-H7.ioc`。
测试修改：`tests/pid_tuning/stubs/stm32h7xx_hal.h`、`stubs/usart.h`、`test_vofa_esc.c`、
`test_tuning.c`、`tests/verify_optical_build.py`。未新增源文件，因此 Keil 源文件列表无需调整。

未烧录。实机需确认上位机 23 通道帧正确、完成计数接近 100/s、失败/超时/忙跳帧计数，
以及完整任务负载下 BMI088/飞控/SD 频率和运行栈水位。

本轮验证：DMA/控制主机测试通过，`tests/run_sd.ps1` 全部通过（含原格式、转换、
恢复/卡满、基础/扩展各 2000 DATA 和姿态回归）。完整 `tests/pid_tuning/run.ps1`
在通过 DMA、控制、光流、采集/滤波等测试后，仍在既有 heater 第 39 行 CCR=100 断言处失败。
未将该套件记为全部通过，未改温控。

ARMCC 64 个源文件 O0/O3 编译链接成功，输出 `MDK-ARM/vofa-dma-o0/firmware.hex`、
`MDK-ARM/vofa-dma-o3/firmware.hex`；同目录含配套 AXF/map/调用图和构建日志。
沙箱内 armasm/fromelf 发生系统 DLL 重定位启动错误；沙箱外执行相同构建/检查后通过。
静态检查确认缓冲区地址 0x240497C0、大小 96、32 字节对齐；USART1/DMA1 Stream4 IRQ 均已链接。
defaultTask 静态调用栈 O0=544、O3=516 字节，配置 1024 字节；任务堆预算仍为 11648/15360 字节。
这些不代替板上运行时水位与时序测量。
