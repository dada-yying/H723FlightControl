# SBB3 自描述黑匣子

两块板必须同步烧录 V3。USART1 的 23 通道 VOFA（96 字节、230400、100 Hz）不变。
UART7 改为 SBB3，不再能直接用 JustFloat 解码。F407 不解释任何飞控字段。

## 今后新增字段

仅修改 `App/blackbox_fields.h`：

1. 在 `s_fields` 添加 `BF("name","unit",offset,count,type)`。
2. 在 `Blackbox_Sample()` 填入对应小端字节。BMI088原始值/状态占112～164，独立姿态/状态占165～183，新字段从偏移184起，勿覆盖已有字段。
3. 重新编译、烧录 H723。F407 固件和电脑转换工具均不用改。

字段名称为 ASCII 字母、数字、下划线，最多 63 字符，唯一；单位最多 31 个可打印 ASCII 字符。
不要使用转换器元数据列名：session、file_number、block_number、sequence、receive_time_us、send_sequence、sample_time_us。
支持 I8/U8/I16/U16/I32/U32/I64/U64/F32/F64/BYTES，固定数组用 count。
最多 128 字段、484 字节数据载荷、8192 字节定义；超过限制禁用黑匣子，不影响飞控。
光流字段是 raw27 区域的重叠视图，保留整数精度及原始协议字节。
新增字段示例见 `BLACKBOX_TEST_EXTRA_FIELDS` 条件编译部分；生产编译不启用该宏。

## 带宽和状态

DATA 预算 150000 B/s，SCHEMA 预算 10000 B/s（含包头及 CRC）。
独立 BlackboxTask（1024字节栈、Normal2）由 TIM4 通知，与飞控/IMU通知脱钩。
TIM4 计数1MHz、IRQ优先级6，ARR预装载，1428/1429µs余数累加保持绝对700Hz节拍。
当前载荷184字节、包长212字节，700Hz需要148400B/s；字段增加后有效节拍为
min(700, floor(150000/包长)) Hz。任务延迟合并、DMA忙或定义发送时跳过，不补发旧数据。
周期定义占用发送机会，实际完成频率可略低于700Hz，查看 `record_hz`。
BMI088独立读取/姿态更新目标2kHz，黑匣子采集最近的成对一致快照，并非记录每个BMI样本。
飞控端64位微秒时间戳仅保存在包头，转换后为 `sample_time_us`。
删除载荷中的重复 `fc_timestamp_us`；时间本身没有丢失。
从本次上电黑匣子首次维护调用开始计时，在采集遥测快照前读取；不是UTC、IMU测量时间或F407接收时间。
定义启动先发完，再发数据；每2秒启动重发，周期分片至少间隔20ms。
BMI088定义包含数组，CSV会自动展开如 `bmi088_gyro_deg_s[0]`（度每秒，原生XYZ轴）；
新增 `bmi088_roll_deg/pitch_deg/yaw_deg` 及有效/epoch状态见 `BMI088_ATTITUDE.md`。
BMI088原始值语义已统一为校准后、未滤波物理量；未校准运动寄存器及重复校准数组不再发送。
同名旧日志字段为未校准值、新日志已校准，用schema/`bmi088_calibration_version`区分；
保留温度测量、版本及温补域状态，参数依据见 `BMI088_CALIBRATION.md`。
删除可由摄氏温度精确换算的 `bmi088_temperature_raw`，不再重复发送温度寄存器计数。
测量温度与限域补偿温度在域外可以不同，二者并非重复字段。
光流原帧与解码字段是同一字节区的视图，不额外复制载荷；定义周期重发与低频传感器
快照在相邻采样中的重复值仍保留，分别用于晚启动恢复和完整时间序列。
F407 和转换工具仍使用原 SBB3 版本，无需因新增 BMI088 字段修改或重新烧录。
无 IMU 更新或电机锁定时仍记录最新可用快照及已有状态/年龄；快照时间不是新的IMU采样时间。
DWT时间由独立黑匣子任务定期扩展，调试暂停不作为有效运行时间依据。
任务创建或TIM4初始化/启动失败只禁用黑匣子，不影响飞控、BMI读取或加热。

Watch：`g_blackbox_tx_stats` 中 schema_id、payload_size、config_error、record_hz、
bandwidth_skip_count、schema_submitted_count、schema_completed_count、data_completed_count，
以及原有 busy_drop_count、error_count、timeout_count、abort_count。
target_hz/effective_hz为目标/限频后节拍；timer_running/last_timer_status/timer_error_count
观察TIM4；timer_irq_count、missed_trigger_count观察任务合并的错过时隙；stack_free_words为运行栈余量。
bandwidth_skip_count仅统计字段增加后相对700Hz被带宽限制去掉的名义时隙；任务创建失败看task_error_count。
record_hz 为最近统计窗口内已完成 DATA 数量，不是飞控频率。

## 转换

在 F407 工程目录执行：

```powershell
python tools/blackbox_to_csv.py 'E:/B*.BIN' -o flight.csv --report flight.report.json
```

V3 输出 `flight.<schema_id>.csv` 和 `flight.schema.json`，不同定义分开文件。
工具两遍扫描，可用后出现的定义解码前面的数据；缺定义会报告 undecoded_records。
V2 仍可单独只读转换；不可把 V2/V3 混在一次输入中。
原 BIN 应保留。CRC 不等于接收确认，发送完成不证明 SD 卡保存成功。

板载 BMI088 字段、采样配置与 Watch 说明见 `BMI088_LOGGING.md`。

## 验证边界

主机测试覆盖两种 H723 字段注册表 → 同一 F407 实际 FatFs 存储程序 → 同一转换器。
运行 F407 `tests/run_cross.ps1`。ARMCC 构建及静态 SRAM/栈检查不替代实机。
首次拆桨，同时升级两块板，持续记录至少60秒，检查 UART/SD 错误、队列字节占用、
丢包、记录频率和晚启动定义恢复。飞行稳定性未由本次测试证明。
