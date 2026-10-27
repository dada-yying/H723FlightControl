# SBB3 黑匣子：H723 SPI-SD 直接记录

2026-10-26：追加 yaw 目标、误差、保持状态及控制器姿态 epoch，见 [YAW_HOLD.md](YAW_HOLD.md)。
旧字段偏移不变，载荷 218 字节、DATA 包 246 字节，schema 自动更新，通用转换器不变。

2026-10-25：USART1 VOFA 改为 DMA1 Stream4 异步发送，帧/通道/波特率/100 Hz 不变，
见 [VOFA_DMA.md](VOFA_DMA.md)。黑匣子 SPI-SD 存储路径不变。

2026-10-24 滤波/量程更新：`bmi088_gyro_filtered_deg_s` 为三轴 120 Hz／40 Hz 陷波后
再经 50 Hz 二阶低通的最终输出；未滤波字段不变，加速度仍为 m/s²（硬件 ±6g）。
字段布局、schema 和转换工具未改，详见 [BMI088_FILTERS.md](BMI088_FILTERS.md)。

## 2026-10-24 初始化恢复 revision 11

实卡 `B0000009.BIN`（session `23ec509daf6c83dbe4c86ad0f061ed9c`）包含
30912 DATA，跨度 104.657 秒；唯一序号缺口为 1039→74786，共缺 73746 条。
诊断显示 ready_tick_ms=75777、retry_count=16、last_error=SDINIT、SD error=0x80000004，
队列峰值 262080 字节。后续约 29.9 秒无序号缺口、无有效块 CRC 错误。
这是启动初始化反复失败、缓存耗尽的证据；现有 BIN 不含具体失败命令，不能据此确认 CMD0 是唯一原因。

代码中 CMD0 原先只尝试一次，任何短暂无响应都会退出到文件层 1/2/5 秒重试。
revision 11 在初始化内至多尝试 10 次 CMD0，失败后释放 CS、发送 80 个空闲时钟并让出
任务 10 ms；100 ms 墙钟预算控制是否继续下一次尝试（任务抢占可能延长实际完成时间）。
只重试复位命令，不重放写操作；永久失败仍返回原恢复路径。ACMD41 使用独立 2 秒预算。
新增 Watch 累计 CMD0 尝试数、重试恢复数、最近拒绝 R1；无需 Watch 也可用新 BIN
中的 ready_tick_ms、retry_count、dropped_packets 和序号缺口判断效果。

主机已先复现旧实现对连续三次 CMD0 无响应立即失败，再验证新实现同次初始化成功；
永久拒绝仍失败，SD 协议/队列/FatFs/转换回归通过。未改飞控、BMI088、温控、任务优先级、
文件格式或采样率，未烧录。此修复不等于已确认实卡故障消失；需要重复上电、拆桨记录新 BIN。

当前 H723 不再从 UART7/PE8 发送黑匣子数据，F407 不参与记录。
USART1 的 VOFA 仍为 23 通道、96 字节、230400、100 Hz。
存储传输层与温控配置保持原样。2026-10-24 起，飞控改用 BMI088，详见 `BMI088_LOGGING.md`。

当前飞控及 BMI088 姿态采用 RFU（X 右/Y 前/Z 上）：roll 抬头正、pitch 右侧下沉正、yaw 俯视逆时针正。
`roll/pitch/yaw`、目标与控制角速度字段均按此约定解释；原生 BMI088 XYZ 数组不变。
原字段名、单位、偏移和 CRC 算法保持不变；追加 yaw 保持字段后载荷为 218 字节。schema 不编码安装坐标，
不能仅凭相同 schema 将本次固件与历史 FRD/显示符号日志视为同一坐标；分析时应记录固件版本。

## 接线与通信

PE15=CS，PB3=SPI1_SCK，PB4=SPI1_MISO，PD7=SPI1_MOSI，SPI 引脚 AF5。
保留 PA13/PA14 SWD；PB3/PB4 不再可用于 SWO/JTAG。
模块供电由硬件负责，软件不控制其电源，不假定存在插卡检测脚。
SD 启动使用 GPIO SPI mode 0，每个半周期至少 2 us（不超过250 kHz）。
卡就绪后切到 SPI1 7.5 MHz，不改变 SPI2/BMI088 的共享 PLL 时钟。
使用 DMA1 Stream1 TX、Stream3 RX，SPI1/DMA IRQ 优先级6；
UART5 Stream0、USART10 Stream2 不变。
初始化支持 SD v1/v2、SDSC/SDHC；检查命令 CRC7 和数据 CRC16。
sector DMA 采用专用 32字节对齐 AXI SRAM bounce buffer及缓存维护。
只在存储任务内等待 DMA/卡忙，超时后重试挂载；不在 ISR 或采样任务等待 SD。

## 记录节拍与缓存

TIM4 1MHz计数、ARR999，独立1kHz通知 BlackboxTask（1024字节栈/Normal2）。
每个新时隙只采集一次最新一致快照，通知合并时记录错过次数，不补算旧采样。
锁定、ATOM断流仍记录状态及最后可用快照。BMI088读取目标2kHz，日志不是每个BMI样本。
当前 SBB3 DATA 包为246字节；已移除UART的150kB/s限频，新增字段不会自动降低记录频率。
增加字段后需核对写卡吞吐与队列统计；单包上限仍为512字节。

BlackboxStorage任务（4096字节栈/BelowNormal）独占FatFs/SPI。
静态队列256KiB、批量缓存32KiB、工作块4KiB均显式放在AXI SRAM。
当前DATA落盘记录264字节，每4096字节块最多15条，约273kB/s，另加定义和诊断。
队列约容纳992条当前大小的记录（另有定义），无法保证覆盖任意SD停顿。
满时丢新记录，保留旧记录；原SBB3发送序号与块内丢弃计数用于诊断。
启动挂载/预分配期间也会采样，超过队列容量的启动数据会被统计为丢弃。

## 文件与异常

使用FAT32、SKYBBX3版本3：4096字节块、64字节块头、CRC32，
每条记录含本地入队微秒时间、存储序号、包长度及原SBB3包。
外层时间不再是F407接收时间；包头sample_time_us仍为H723采集快照时间，
从本次启动黑匣子任务首次维护时开始，不是UTC。
FAT目录日期沿用无RTC的固定日期，不可用于判断飞行时间。

每次上电创建新的Bnnnnnnn.BIN，正常运行只用该文件。
先写入完整SCHEMA，再写DATA；定义由存储任务在每个新文件写一次，不在DATA间重复插入。
定义前缀记录没有传感器采样时间，其时间/发送序号为0。
全部既有数据字段、单位及载荷偏移保持不变。

保留40MiB分步预分配目标，首个4MiB已提交后即开始记录，其余在队列允许时后台完成。
保持4MiB增长余量、32KiB批量，
不足一批最长250ms提交，每1s同步；预分配尾部可能含旧卡内容，
转换器根据文件/session/块号/CRC过滤，不以文件大小代表有效记录量。
短暂故障恢复同一卡上的原文件并验证最后已写块，避免重复提交完整批次。
普通I/O超时不会直接认定文件损坏；成功读取并反复确认无法安全续写时新建文件，
重新保存完整定义并保留待写数据。
卡满或达到FAT32单文件上限时本次上电停止记录；不覆盖旧文件、不自动格式化。
无法使用卡时记录故障、定时重试，不改变飞控保护。
直接断电可能丢失尚未落盘的数据；同步周期不是掉电损失的硬保证。

## 字段和Watch

新增字段仍只需在App/blackbox_fields.h注册字段并填入字节，不修改通用转换工具。
字段支持固定数组/整数/float/字节，定义上限8192字节、128字段。
保留已有205字节载荷布局：BMI088未滤波deg/s在112，姿态/状态在165，
滤波deg/s/valid/参数在184～204；yaw保持字段在205～217，未来字段从218开始。
两个BMI角速度数组在内部均为rad/s，仅序列化转换为deg/s，均为原生XYZ。
光流原帧和解码字段仍共享同一字节区，不复制载荷。

- g_blackbox_tx_stats：target_hz/effective_hz=1000、record_hz为入队DATA频率，
  missed_trigger_count为错过采样时隙；busy_drop_count为队列拒绝。
  保留旧名称以便Watch迁移，submitted/completed仅表示入队，不表示已写卡；
  原UART错误/中止字段不再使用，bandwidth_skip_count保持0。
- g_blackbox_stats：write_hz为实际写入DATA频率，data_written、written_blocks、
  queue_used/queue_high_water、dropped_frames、max_write_ms/max_dma_ms/max_card_busy_ms、
  last_error/last_sd_error、retry_count/recovery_count、file_number/switch_reason、stopped。
  stack_free_words和heap_free_bytes观察存储任务资源；采样任务栈看g_blackbox_tx_stats。
- 字节/块统计、记录计数可能包含SCHEMA，勿直接当作采样数。
- 队列入队失败、写卡失败和飞控时隙遗漏分别计数，不能仅看“完成”判断日志完整。

## 转换与验证

### 无仪器初始化诊断（2026-10-20）

`g_sd_spi_diag.version=0x20261020`，记录初始化尝试/成功/失败次数、当前与失败阶段、
失败命令/R1/命令是否发出。阶段：1启动时钟、2 CMD0、3 CMD8、4 ACMD41、5 OCR、
6块长度、7CRC启用、8CSD、9CID、10硬件SPI初始化、11就绪。
用户对CMD0跳过前置ready等待的修改保留；本次不再改变初始化命令和时序策略。

`g_sd_spi_diag.cmd0`冻结第一次完整CMD0：

- complete=1后再查看；调试器Watch将其设为0可在下一次初始化重试时重新捕获。
- tx[6]是命令缓冲，mosi_readback[6]是在各发送位低电平建立期读取的PD7输入路径。
  正常二者均为40 00 00 00 00 95。gpio_checked=1才表示执行过真实GPIO读回检查。
- rx[0..rx_count-1]为当前接收方式的逐字节结果；rx_low_phase为同一时钟周期低电平
  建立期的对照采样。对照值不用于协议判断，不自动切换相位。
- cs_not_low_count、sck_not_low_count、sck_not_high_count、mosi_mismatch_count
  正常均0；这些是MCU引脚输入路径的读回检查，不代表模块端波形测量。
- duration_us是CMD0发送六字节及响应轮询的总时间；bit_period_min_cycles/max_cycles
  为相邻位的观测间隔。除以SystemCoreClock/1e6可换算微秒，任务抢占也包含在内。
  诊断代码会增加少量GPIO读取开销，不代表无诊断版本的精确时序。

上电运行期间不要在SPI逐位收发中打断点；运行约10秒后暂停，展开cmd0和失败阶段查看。
冻结记录避免后续重试覆盖首现场，也避免依赖优化后不可见的局部变量。
`tests/test_sd_bitbang.c`执行生产spi_byte位循环，用GPIO寄存器/模式0卡模型检查
所有字节、读回、观测相位、冻结/重新捕获、无响应及计数器回绕；不能代替实机结果。

初始化成功后仍报同步/写入超时的版本使用 `g_sd_spi_diag.revision=2`。
`io_fault`在第一次读、写或CTRL_SYNC错误时冻结，后续复位重试不会覆盖；
Watch设其complete=0可重新捕获。operation=1读、2写、3CTRL_SYNC；
kind=1 HAL单字节失败、2卡忙超时、3 DMA启动失败、4 DMA完成等待超时、
5 DMA错误回调、6读数据起始字节错误/超时、7写数据响应拒绝、8数据CRC错误、9命令响应错误。
同时保留HAL返回值/ErrorCode/State、SPI寄存器、DMA剩余计数、扇区、命令及耗时。
HAL返回值0成功、1错误、2忙、3超时；卡忙与HAL失败不能仅凭0x80000001区分。
read_sectors/written_sectors/sync_calls/sync_successes为累计通信计数，
g_blackbox_stats.data_written才是已提交日志块中的DATA记录数量。

本地HAL的SPI_CloseTransfer在关闭SPI1前加入有限NOP等待（至少1us，当前SCK=7.5MHz），
覆盖ST ES0491 2.21.5对EOT后输出截短的规避要求；DMA和轮询均经过此路径。
SPI2/BMI088、SPI6不执行此等待，不改共享PLL、SPI频率、SD忙等待期限或飞控算法。
该规避是针对已发现的软件缺口，不代表已证实它是所有同步超时的唯一原因。
依据：https://www.st.com/resource/en/errata_sheet/es0491-stm32h72xx73xx-device-errata-stmicroelectronics.pdf
tests/test_sd_eot.c检查SPI1限定及不依赖DWT的等待；首次I/O错误冻结、HAL失败与卡忙区分、
DMA启动/等待/回调及CRC错误诊断纳入run_sd.ps1。仍需实机验证。

revision=3针对实际出现的HAL单字节轮询超时（7ms/timeout=5ms）补充SPI1限定修正：
原HAL即使当前字节TX/RX计数已全部完成，仍在轮询末尾无条件判定累计时间超时。
真实函数主机复现中，在最终RX读取前模拟7ms任务抢占，旧逻辑返回HAL_TIMEOUT，
修正后SPI1进入正常EOT检查；仍未完成的TX/RX和EOT故障继续报错。
不增加原5ms期限，不重发可能已经发送成功的字节，不修改SPI2/6的轮询行为。
`g_spi1_completed_after_deadline_count`统计已完成后避免的超时误判次数。
该计数大于0证明实机触发过此分支，也表明存储任务存在调度延迟；记录频率与丢帧仍需检查。
`tests/run_sd_poll.ps1`每次从生产HAL源文件提取真实函数到tests/build再编译，
覆盖正常收发、7ms抢占、真实TX/RX/EOT超时及SPI2/6不变，避免维护一份伪造HAL实现。
这能证明软件缺陷及修正分支，不单独证明所有实机写卡故障已经消除。

revision=4：实机确认上一修正分支触发后，仍在CMD17后的单字节轮询出现超时，
因此SD硬件SPI阶段的所有字节统一走SPI DMA路径：
命令/响应/令牌/CRC每次1字节，扇区数据每次512字节；不再调用单字节
HAL_SPI_TransmitReceive轮询。慢速GPIO初始化保持原样，SPI1频率不变。
单字节也使用独占AXI bounce buffer；缓存维护范围向上取整至32字节，扇区为512字节。
沿用现有DMA完成回调、100ms等待阈值和故障中止；已完成后任务延迟9ms仍视为完成，
无完成事件/错误回调/启动失败仍记录错误。卡忙500ms期限不变。
byte_dma_count/sector_dma_count观察已完成的1字节/512字节DMA，last_transfer_size观察最近长度；
io_fault.transfer_size在DMA故障时直接区分1或512。非DMA故障时该字段只是最近一次DMA长度。
原g_spi1_completed_after_deadline_count保留为HAL兼容计数，但SD新路径不再触发它，
不再以此计数是否增长判断SD恢复。应看data_written/write_hz及io_fault.complete。
模拟协议测试现在也通过真实spi_byte的1字节DMA分支，覆盖命令→响应→512字节→CRC的交替，
不再在硬件SPI阶段用字节替身跳过此分支。新增IRQ及调度开销、实机1kHz落盘频率仍需测量。

revision=5针对实际日志断续和队列溢出：

- 实测旧B0000003.BIN含8151条DATA，时间跨度121.008835s；551个有效块无CRC坏块，
  最终块统计丢弃131382个记录包。队列高水位262144字节，长期接近满载；
  最大32KiB写入2.55s。启动队列首段约1s后有24.909627s空洞。
  这说明能写文件不等于能持续保存1kHz，不应靠绘图插值掩盖。
- 1字节DMA启动后最多短等20us（DWT工作时），不关中断，高优先级任务仍可抢占；
  通常IRQ很快完成，避免每字节阻塞/唤醒低优先级存储任务。
  如未完成则退回原100ms通知等待。512字节DMA仍通知等待。
  DWT停止时也有有限迭代上限，避免死等；该备用路径不保证精确20us。
  byte_short_complete_count统计短等待内可见完成（含启动已完成），
  byte_notify_wait_count统计需要通知等待的字节；byte_short_wait_max_cycles含任务抢占时间，
  不能当纯CPU占用。实机应看write_hz、record_hz及RECORDING后dropped_frames是否继续增长。
- 首个4MiB已提交即可开始数据写入；继续后台预分配至40MiB。
  prealloc_ms现在表示本文件创建至首段可记录时间，不是全部40MiB完成时间。
  上电采样仍开启，初始化未就绪时超出RAM容量的数据仍会丢弃并计数，不能恢复旧丢失数据。
- 每文件前缀保留完整定义，采样任务不再周期追加SCHEMA。
  避免CSV只含DATA、序号却夹有定义记录时产生额外绘图断线。
  schema_*发送统计为历史兼容字段，不表示当前文件是否有前缀定义。
- 外层序号保留队列丢弃产生的缺口，不再把全部已保存记录强制重新排成连续序号。
  损坏换文件时插入新定义仍保留待写记录内的原有缺口；SBB3载荷字段和转换器不变。
- 新主机测试验证延迟DMA IRQ可在短等待完成且不阻塞通知、超时仍可退回等待，
  验证4MiB时启动记录且40MiB目标尚未完成、真实队列丢弃后序号跳跃及文件恢复。
  不把主机测试视为实机1kHz吞吐保证。

构建一致性：原命令行脚本固定-O3，而当前Keil目标Optim=1对应-O0。
脚本新增-Optimization 0/1/2/3参数（默认仍3），不修改工程优化配置。
本次同时生成sd-spi-throughput-verified（O3）和sd-spi-throughput-keil-verified（O0），
两种设置均编译链接、DMA地址及静态栈/堆检查通过。调试时应使用与固件配套的AXF符号。
示例：MDK-ARM/build_ws2812.ps1 -OutputDirectory sd-spi-throughput-keil-verified -Optimization 0。

## Revision 6：短事务轮询与整扇区 DMA（2026-10-21）

以下为当前实现，替代上文 revision 4/5 的单字节 DMA 路径。
参考 Betaflight 的短事务/块传输分离与 SD 多块写设计，按本工程接口独立实现：
- https://github.com/betaflight/betaflight/blob/master/src/platform/STM32/bus_spi_ll.c
- https://github.com/betaflight/betaflight/blob/master/src/main/drivers/sdcard_spi.c

文件层继续沿用本地 F407 工程的 FatFs、32 KiB 批量写入、预分配和原文件恢复。
接口、引脚、7.5 MHz、1 kHz 采样、队列、文件格式和任务优先级不变。

- 快速短事务直接访问 SPI1 寄存器：命令 6 字节一次传输，响应/令牌逐字节处理；
  无单字节 DMA、缓存维护或通知等待。中断始终开启，检查 TXP/RXP/EOT 和硬件错误。
  每次最多一个未读取字节，避免抢占期间 RX FIFO 溢出。
  完成检查先于 5 ms 超时检查；DWT、HAL tick 和有限迭代共同防止死等。
  FIFO/EOT 故障仍会失败，先冻结诊断再终止外设。关闭 SPI1 前仍执行现有 EOT 保护。
- 每次扇区写 DMA 为 516 字节：FF + FE/FC 令牌 + 512 字节数据 + CRC16。
  最多读取 16 字节等待数据响应，再等卡结束忙状态；停止令牌后保留 N_BR 空闲时钟。
  读数据令牌之后一次 DMA 读取 514 字节：512 字节数据 + CRC16；通过 CRC 后才复制数据。
  CMD25 写流内不逐扇区切换 CS；CMD18/CMD12 读流及 FD 写停止令牌保持协议边界。
- CMD25 前发送 CMD55/ACMD23，提示本批扇区数。
  R1=04（仅 illegal command）允许继续；超时、空闲态、CRC 等其他错误均进入原恢复流程。
- TX/RX 各 544 字节，32 字节对齐、AXI SRAM，缓存维护长度向上对齐到缓存行。
  DMA 100 ms、卡忙 500 ms、读令牌 200 ms 的超时保持；卡忙/等令牌每轮 32 字节后让出一个 tick。
  DMA EOT 回调只表示传输完成，disk_write 成功仍要求卡就绪。
- 短事务与 DMA 有独占状态；错误/复位先撤销回调接收，再 abort 并清理 pending IRQ。
  空闲或短事务期间的迟到回调被忽略并计数。

Watch：`g_sd_spi_diag.revision == 6`。

| 字段 | 说明 |
|---|---|
| `short_transfer_count` / `short_bytes` | 成功短事务数及字节数 |
| `short_timeout_count` / `short_error_count` | 短事务超时、硬件/占用错误 |
| `block_dma_count` / `dma_bytes` | 成功整扇区 DMA 数及线路字节数，读 514 / 写 516 |
| `preerase_count` / `preerase_accepted` | 预擦除尝试及接受次数 |
| `preerase_unsupported` / `preerase_errors` / `preerase_last_r1` | 不支持、错误及最近 R1 |
| `dma_to_irq_max_cycles` | DMA 启动至 SPI EOT 完成回调的最大 CPU 周期数 |
| `dma_resume_max_cycles` | 完成回调至存储任务恢复处理的最大 CPU 周期数 |
| `ignored_callback_count` | 未处于有效 DMA 等待状态的回调数 |

两项周期统计除以 `SystemCoreClock/1000000` 得到微秒。
启动至回调包含中断延迟，恢复时间包含调度延迟，均不是纯 SPI 线路耗时。
旧 `byte_dma_count`、`byte_short_*` 和 `byte_notify_wait_count` 保留，不在新正常路径增长；
旧 `sector_dma_count` 继续表示成功扇区 DMA 次数，与 `block_dma_count` 同步。
这些字段只追加到 Watch 结构，未加入 BIN 诊断块或修改 SBB3 schema。

验证入口：`tests/run_sd.ps1`（包含真实寄存器轮询模型、逐字节协议模型及原文件恢复）；
用 PowerShell 7 执行，避免 Windows PowerShell 5.1 将无 BOM UTF-8 的中文路径误解码。
构建分别使用 `-OutputDirectory sd-spi-rev6-o0 -Optimization 0` 和
`-OutputDirectory sd-spi-rev6-o3 -Optimization 3`，再运行 `tests/verify_optical_build.py`。
主机测试不测卡吞吐：实机需拆桨运行至少 60 秒，分开统计启动丢帧和 RECORDING 后丢帧，
核对采样/写入频率、队列占用、序号/时间戳，以及飞控/BMI088 节拍和实际栈余量。
正常数据块约 256 kB/s；32 KiB 批次对应 128 ms 新数据，256 KiB 队列约缓冲 1 秒。
持续吞吐须覆盖数据生成速度并有余量，新增调度耗时统计用于定位仍存在的瓶颈。
本次不烧录硬件，构建和主机测试不代表完成实机持续写入或实飞验证。

本次软件验证结果：SD 主机回归通过；基础/扩展 schema 各 2000 条 DATA 经原转换器解析，
CRC 错误、未解码记录和记录缺口均为 0。O0/O3 均完成 64 源文件编译链接且无编译警告。
两种构建的 TX/RX 分别位于 0x24049380/0x240495A0，各 544 字节；
存储任务静态栈深分别为 1768/1840 字节（配置 4096），应用任务分配预算 11648/15360 字节。
控制、光流、BMI088 采集/低通和姿态测试通过；完整控制回归在既有温控测试第 39 行停止：
测试仍期望 pwm_ccr=100，与当前生产温控参数不符，本次未更改温控代码或参数。

## Revision 7：命令中途抢占的超时修正（2026-10-21）

已通过新增寄存器模型复现 revision 6 的缺陷：6 字节命令第一个字节完成后，
任务被抢占 9 ms；恢复读取该字节并启动下一字节时，旧事务起点已超过 5 ms，
下一字节尚在正常传输就被误判超时。原测试仅模拟最后一个字节完成后的抢占，未覆盖此情况。

短事务现在按硬件进展刷新等待预算：发送一个字节或消费一个已完成 RX 后，
为下一阶段重新开始 5 ms 停滞检测；总字节数限制为 16，等待仍有 tick、DWT 和有限循环兜底。
已完成 RX/EOT 优先检查；真正卡住的 TXP/RXP/EOT 仍然失败。
这修复了可复现的软件缺陷，但不能在缺少实机故障快照时断言其是本次停写的唯一原因。

Watch revision 为 7，新增 `short_progress_recoveries` 统计延迟超过 5 ms 后仍成功消费 RX 的次数；
`short_fault_sent`、`short_fault_received`、`short_fault_wait_ms` 随首个短事务故障冻结，
用于判断出错时已发送/已接收字节数和本阶段等待时间。原 io_fault 和错误枚举含义不变。

本轮只读检查 G:/B0000001.BIN、B0000002.BIN：文件均为 4,227,072 字节，
分别含 96、864 条有效 DATA（采样跨度约 95 ms、863 ms），已识别有效块无 CRC 错误。
其余块为其他 session 的预分配旧内容，由转换器排除；文件大小不等于有效数据量。
4 MiB 是首段预分配门槛，不能据此诊断为容量限制或 SD 卡完全无法通信。

构建目录为 `MDK-ARM/sd-spi-rev7-o0` 和 `MDK-ARM/sd-spi-rev7-o3`，
HEX 必须搭配同目录 AXF。未烧录硬件；需要实机核对首个故障、写入计数及队列变化。

## Revision 8：超时判定前重新检查实际硬件状态（2026-10-21）

实机 revision 7 首个故障：读 sector=0、CMD17/R1=00、transfer_size=1，
elapsed_ms=10，kind=SHORT_TIMEOUT，但冻结的 SPI_SR=0x301F。
其中 RXP/EOT/TXC 已置位、RXPLVL=1，表示字节已经完成且 RX FIFO 有数据；
没有 OVR/UDR/MODF/TIFRE 错误位。这与单纯 SD 卡未回应、写卡数据 DMA 超时不同。

已在生产轮询循环外模拟中断时序并复现同一故障：检查 RXP 时字节尚未完成，
随后在 HAL_GetTick 前抢占 10 ms，期间硬件完成传输。revision 7 用旧状态直接判超时，
捕获时却已经是 0x301F。修复前测试确实触发此失败；revision 8 在发现等待过期后重新读 SR，
先处理硬件错误，若 RX/TX/EOT 有可推进的进展则继续完成，不由过时状态中止。
保持 5 ms 停滞预算、有限循环、IRQ 开启和 revision 7 的逐进展计时。

同时补齐 DSIZE=8 的 RX FIFO 尾部判断：RXP、RXWNE 或 RXPLVL 表示实际存在数据时才读 RXDR。
仅有 EOT 而 FIFO 为空依旧超时，不能据此伪造一个接收字节。
此次截图中 RXP 本身已置位，直接复现的是前述状态/时间检查竞争；FIFO 尾部处理是相关边界补齐。

Watch 新增 `short_deadline_rechecks`：过期后重新检查 SR 发现仍可推进的次数；
`short_fifo_tail_reads`：RXP 未置位、根据 FIFO 占用读取尾部的次数；
`short_fault_status`：首个短事务故障在决策时的 SR，配合原 sent/received/wait_ms 和 io_fault。
这些为累计诊断计数，并非丢包数；正常记录时可能为 0，也可能递增。
固件自报 revision=8；正常采样/协议/卡频率/任务和控制参数均未调整。

构建目录：`MDK-ARM/sd-spi-rev8-o0`、`MDK-ARM/sd-spi-rev8-o3`。
HEX 和 AXF 使用同一目录。实机需拆桨复位连续运行后确认 data_written 与 write_offset 增长，
state 进入 RECORDING；同时检查 queue_used、dropped_frames 和 io_fault.complete。
若再次失败，保留 io_fault 和 short_fault_* 首个故障，不用历史 last_error 单独判定当前状态。
这次修复已复现的主机故障，不代替实机持续 1 kHz 写卡验证，未烧录硬件。

本轮验证：新增测试先在 revision 7 复现单字节 SR=0x301F、10 ms 的误超时，
修复后通过；同时覆盖命令 6 个字节位置、时间回绕、EOT 判定窗口、RXP 未置位的 FIFO 尾部、
EOT 置位但 FIFO 为空，以及超时重查时出现硬件错误。
SD 全套主机回归通过；基础/扩展 schema 各 2000 条 DATA 经原转换器解析，
无坏块、未解码数据或序号缺口。O0/O3 均完成 64 源文件编译链接，无编译警告。
AXI DMA 缓冲地址和对齐检查通过，存储任务静态栈分别 1776/1840 字节，配置仍为 4096 字节。

## Revision 9：存储调度与 CRC 开销（2026-10-21）

用户提供的 revision 8 Watch 为 RECORDING、io_fault.complete=0，queue_used=0x3FFC0，
data_written=1488。只读解析 G:/B0000004.BIN：当前 session 有 1488 条 DATA、18 条 SCHEMA，
无有效块 CRC 错误；末块累计 dropped_packets=16577。首末采集时间约 0 至 7.804 s，
记录间隔中位数 1000 us，但实际保存的数据有大量缺口；排队中的旧采样时间不能代替当前写入吞吐。
uptime=15.513 s 的诊断块中，队列为 262080 字节，32 KiB 写入最长 1020 ms，
allocated_bytes=4227072。队列持续满载时后台预分配让位数据，约 4 MiB 的文件大小并非容量上限。

用户补充的 dma_to_irq_max_cycles=0x004C3458、dma_resume_max_cycles=0x0121542C，
按 480 MHz 分别约 10.40445 ms、39.503025 ms。
前者包含外设传输及 IRQ 延迟，后者是 EOT 回调之后直到存储任务获调度的延迟。
这些最大值不是每扇区平均耗时，但可确认此前 BelowNormal 存储任务存在长时间无法恢复的情况。

经用户明确允许：存储任务从 BelowNormal 改为 Normal，并同步 .ioc；
与默认 VOFA 和光流任务同级，依靠原有 1 ms RTOS 时间片，继续低于 BMI088 Normal1、
黑匣子采样 Normal2、遥控和飞控。任务栈/堆、SPI 参数及 VOFA 发送实现不变。

H723 的 SBB3 组包、验包和 4 KiB 块 CRC32 改用不可变查表；SD 数据 CRC16 同样查表。
新 `App/storage/blackbox_crc.h` 使用相同 CRC32/ISO-HDLC 和 CRC16/XMODEM 参数，
不减免 CRC 校验、不占用硬件 CRC、不需要临界区或运行时建表。
`App/blackbox_protocol.h` 选择共享 sbb3.h 中新增的可选 CRC 实现入口；
未选择入口的 F407 继续使用原逐位算法。协议字段、schema ID、CRC 结果和转换工具保持一致。

Watch revision=9，增加成功 DMA 的 `dma_latency_samples`、`dma_irq_total_us`、
`dma_resume_total_us`。取两个时刻的总微秒差除以样本数差，可分别得到该窗口平均 IRQ 完成延迟和
任务恢复延迟；只读诊断的 64 位累计量在实时刷新时可能非原子，精确比较应暂停后读取。
原有最大周期数继续保留，用于观察最坏情况。

验证包括原逐位参考与新算法的标准向量、全部单字节、0/FF/递增/随机模式、
32 个非对齐偏移、短长度和 4 KiB 块，及完整组帧→FatFs→原转换器兼容回归。
主机 8 MB CRC32 对比约为查表 0.016 s、逐位 0.141 s，这不是 MCU 耗时或实际 SD 吞吐。
构建目录 `MDK-ARM/sd-spi-rev9-o0`、`MDK-ARM/sd-spi-rev9-o3`，均 64 源文件编译链接通过。
CRC 表位于 Flash，TX/RX 仍各 544 字节、AXI SRAM 对齐；静态任务栈检查和堆预算通过。
H723 SD 回归及基础/扩展 schema 各 2000 条 DATA 的原转换器往返检查通过。
将本次 H723 生成的线包作为跨工程输入，F407 原 tests/run.cmd 全部通过，
包括流解析、缓存/恢复/批量写入、SDIO DMA 和 20 项转换器测试；F407 源码未修改。

实机需拆桨运行至少 60 秒，分别核对启动阶段与进入 RECORDING 后的 dropped_frames，
确认队列不持续满载、data_written 的每秒增量接近 1000，以及 BIN 时间戳/序号的完整性；
并核对 BMI088/飞控实际节拍、VOFA 错误数和各任务栈余量。
该修改针对已确认的调度延迟和计算开销，不宣称已在实机消除全部丢帧；未烧录硬件。

## Revision 10：BMI088 插卡负载下的 SPI2 误超时（2026-10-21）

用户实机在 Bmi088_Fail 入口确认 last_status=HAL_TIMEOUT(3)、last_spi_error=0x100，
并反馈不插卡时不触发、插卡记录时触发。日志已记录 21 段 SPI 错误重启，每段约 1.184 s。
这确认了实际错误类型和负载相关性，尚不能单凭这两个值断言每次超时均已完成全部字节。

从生产 HAL 提取实际函数的诊断测试已复现：SPI2 完整收到 1..8 字节，恰逢 1 ms tick 翻转，
仍返回 HAL_TIMEOUT/0x100。原 SPI1 补丁未覆盖 SPI2；BMI088 把每次 HAL 错误按真实错误恢复，
等待 1000 ms，再进行两次 80 ms 复位等待和寄存器配置，形成约 1.184 s 的中断。

本次只修正 SPI2、8 位主机、既有 timeout=1ms 的 HAL TxRx 轮询路径：
- DWT 测量实际 1ms 等待，避免把跨 tick 的数微秒误当成等待 1ms。
- DWT 不可用/停止时，由超过一个 tick 的兜底和有限迭代防止死等；此降级路径不保证精确 1ms。
- 到期后重读接收 FIFO；只消费已发送字节对应的实际 RX 数据，所有字节齐全后继续 EOT/硬件错误检查。
- EOT 等待也采用同一预算，到期后重查完成标志。TX/RX/EOT 真正停滞及硬件错误仍失败。
- 保留传感器真实错误后的原恢复流程、2kHz 调度、滤波、校准、温控和 ATOM 闭环。
SPI1 的 revision 9 传输实现和 Normal 存储任务未调整；SPI6 及其他 HAL 模式未改。
SD Watch revision=10 仅标识本次固件组合，不表示另换了 SD 协议或参数。

Watch 新增 `g_bmi088_spi2_poll_diag`（revision=1）：
`transfer_count` 为该限定路径调用次数；`tick_boundary_avoided_count` 统计跨 tick 但实际预算未超的传输；
`completed_after_deadline_count` 统计到期后确认数据已齐全的传输，不表示额外重读传感器。
`timeout_count` / `hardware_error_count` 记录真正等待失败/观察到硬件错误的次数。
`first_fault` 在 HAL 清理寄存器之前冻结首错，phase=1 表示 TX/RX 停滞，2 为 EOT，3 为硬件标志；
含未收发字节数、SPI_SR/CR1/CR2/CFG1 和已用微秒，后续成功不会覆盖。
重新上电/复位清零；调试可将 first_fault.complete=0 后捕获下一次。

构建目录为 `MDK-ARM/sd-spi-rev10-o0` / `sd-spi-rev10-o3`，HEX 使用同目录 AXF。
实机拆桨确认 revision=10 后，检查 BMI088 read_hz、read_error_count、数据 valid/age，
以及上述首错与 SD 队列、丢帧。跨 tick 避免计数增长并非采集错误。
本次并未消除此前运行中 SD 丢弃的 311 帧；长写入和后台扩容的队列预算仍需处理和验证。
修复 BMI088 持续采集后，需重新验证同时 2kHz IMU 与 1kHz SD 的负载，不能只用停采期间的写卡速率判断。
未烧录硬件，不把主机回归/编译结果视为实机零丢帧。

本轮软件验证：实际 HAL 函数的 1..10 字节、tick/DWT 回绕、RX 尚未齐全的 tick 边界、
到期后 RX/EOT 完成、FIFO 尾部、真实 TX/RX/EOT 停滞、DWT 禁用/停止、硬件错误与首错冻结测试通过。
SD 全套主机回归及基础/扩展 schema 各 2000 条 DATA 转换检查通过。
O0/O3 均完成 64 源文件编译链接且无编译警告；BMI088 任务静态栈分别 740/360 字节，配置 1024 字节。
控制、光流、BMI088 采集/温补/低通回归通过；完整回归仍在既有温控测试第 39 行的 pwm_ccr=100 断言停止，
本次未修改温控参数。AHRS 单独包含在 SD 回归中且通过。

继续使用STM32F407VET6黑匣子/tools/blackbox_to_csv.py：
```powershell
python tools/blackbox_to_csv.py 'E:/B*.BIN' -o flight.csv --report flight.report.json
```
不同schema输出各自CSV和schema JSON，保留原BIN；转换器无需改动。
H723主机测试：tests/run_sd.ps1。覆盖实际组帧→实际队列/FatFs/恢复→原转换器，
同时验证字段扩展、SPI协议CRC、DMA忙/错误/超时、卡忙和恢复。
tests/pid_tuning/run.ps1回归控制/传感器；温控旧参数断言问题单独报告，不改生产参数。
ARMCC构建：MDK-ARM/build_ws2812.ps1 -OutputDirectory blackbox-spi-sd-verified；
静态资源检查：tests/verify_optical_build.py MDK-ARM/blackbox-spi-sd-verified。

.ioc记录硬件资源，低速GPIO初始化→硬件SPI切换及FatFs接入为手工模块，
CubeMX再生成后须检查这些用户逻辑及链接段，不能用自动SPI初始化替代启动流程。
尚需实机拆桨连续记录至少60秒，检查采样/写入接近1kHz、
队列与错误计数、SD文件转换、断电恢复、飞控频率、BMI2kHz及任务栈余量。
编译和主机模拟不证明实际卡吞吐、线缆信号质量、掉电可靠性或飞行稳定性。
