# WS2812 检查与修复记录

## 检查依据

本地 `references/达妙h7开发板/例程/CtrBoard-H7_WS2812`，以及
`references/达妙h7开发板/原理图/CtrBoard-H7_V1.0-240124.pdf` 第 1、3 页。
用户确认参考例程在同一块板上可以点亮。目前未取得本次固件的板上寄存器、波形或 Watch 数值。

## 发现与处理

- 原寄存器驱动初始化时就置 SPE，随后修改 CR2 的传输长度；发送结束只清标志，不关闭 SPI。
  这与参考 HAL 的“设置长度、使能、发送、结束后关闭”流程不同。
  已使用参考配套的 SPI HAL 源码和现存的 `Core/Src/spi.c`，移除自写寄存器发送流程。
- 原驱动在 FreeRTOS 临界区内无限等 TXP/EOT；异常时会屏蔽系统节拍并卡住。
  现在使用 5 ms HAL 超时，发送期间仅暂停任务切换，保留 SysTick 中断。
  这是短时轮询发送，并非 DMA 异步传输；颜色间隔继续使用 osDelay。
- 保留上一轮恢复的 SysTick：HAL_IncTick；调度器启动后调用 xPortSysTickHandler。
  SVC/PendSV 映射至 FreeRTOS port，最终向量表地址在 main 最开始明确设置。
- 原理图显示 LED3 的 VDD 是 VCC_5VS（系统 5 V），DIN 是 PA7。
  PC15 控制 VCC_5V（对外 5 V），不是 LED3 供电。因此已撤掉之前误加的 PC13/14/15 使能。
- SPI6 配置保持参考值：HSE 24 MHz / 4，8 bit、MSB first、TX only、第二边沿、PA5/PA7 AF8。
  数据编码保持 0x60/0x78、GRB、高位先发；复位保持参考的 100 次单字节零传输。
  去掉原例程无效的 Size=0 调用与无限等待。
- 板级初始化和一帧红色在调度器启动前完成；任务在 MX_FREERTOS_Init 中创建。
  初始颜色不再全黑，任务创建失败会进入 Error_Handler。

## 构建与代码验证

- `MDK-ARM/build_ws2812.ps1` 使用本机 ARMCC 5.06，读取当前 uvprojx 的源码列表与头文件路径。
  完整编译、链接 41 个源文件，输出到独立的 `MDK-ARM/ws2812-hal-verified`。
  不覆盖用户打开的 Keil 工程输出目录。
- 四个新增 SPI HAL 文件和参考例程内容逐一比较一致（忽略行尾格式）。
- 检查最终 AXF 反汇编：main 设置 VTOR=0x08000000；SysTick 调用 HAL 和 RTOS tick；
  WS2812_Init 调用 MX_SPI6_Init；WS2812_Ctrl 链接到 HAL_SPI_Transmit。
- 链接器静态报告的灯任务调用链栈需求 120 字节，配置栈 1024 字节。
  静态栈报告不替代运行时高水位测量。
- HEX 的生成、链接成功不等于开发板已点亮。

## 烧录及定位

可直接使用 `MDK-ARM/ws2812-hal-verified/firmware.hex`。
用 Keil 调试时重新载入已修改的 uvprojx 并 Rebuild、下载，以确保源码和调试符号一致。
不要继续使用旧的 `MDK-ARM/ws2812-verified/firmware.hex`。

| Keil Watch 变量 | 正常值 / 含义 |
| --- | --- |
| ws2812_boot_status | 0：调度器启动前的红色帧 HAL 发送成功；0xFFFFFFFF：尚未完成该步骤 |
| ws2812_task_started | 1：灯任务已进入 |
| ws2812_task_loop_count | 持续增加：任务持续运行 |
| ws2812_frame_count | 持续增加：任务完整发送成功，约每秒 50 次 |
| ws2812_tx_status | 0=HAL_OK，1=HAL_ERROR，2=HAL_BUSY，3=HAL_TIMEOUT |
| ws2812_error_count | 正常为 0；增加说明发送失败 |
| ws2812_spi_error | HAL SPI 错误码，可配合 hspi6 查看 |
| uwTick / xTickCount | 启动调度器后都应持续增加 |

若仅保持红色且任务未进入，定位调度器启动；若错误计数增加，查看 SPI HAL 状态；
若成功帧持续增加仍不亮，则必须查看 PA7 实际波形，不能再仅凭静态代码确定原因。

当前 .ioc 仍是基础模板配置；本次新增 SPI 构建条目和 App 目录属于手动集成。
重新生成 CubeMX 代码前需要先在 CubeMX 配好 SPI6，并保留手动工程分组。
