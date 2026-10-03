# SBB3 自描述黑匣子

两块板必须同步烧录 V3。USART1 的 23 通道 VOFA（96 字节、230400、100 Hz）不变。
UART7 改为 SBB3，不再能直接用 JustFloat 解码。F407 不解释任何飞控字段。

## 今后新增字段

仅修改 `App/blackbox_fields.h`：

1. 在 `s_fields` 添加 `BF("name","unit",offset,count,type)`。
2. 在 `Blackbox_Sample()` 填入对应小端字节。BMI088 校准后原始值/状态数据占用 120～174，新字段从偏移 175 起，勿覆盖已有字段。
3. 重新编译、烧录 H723。F407 固件和电脑转换工具均不用改。

字段名称为 ASCII 字母、数字、下划线，最多 63 字符，唯一；单位最多 31 个可打印 ASCII 字符。
不要使用转换器元数据列名：session、file_number、block_number、sequence、receive_time_us、send_sequence、sample_time_us。
支持 I8/U8/I16/U16/I32/U32/I64/U64/F32/F64/BYTES，固定数组用 count。
最多 128 字段、484 字节数据载荷、8192 字节定义；超过限制禁用黑匣子，不影响飞控。
光流字段是 raw27 区域的重叠视图，保留整数精度及原始协议字节。
新增字段示例见 `BLACKBOX_TEST_EXTRA_FIELDS` 条件编译部分；生产编译不启用该宏。

## 带宽和状态

DATA 预算 150000 B/s，SCHEMA 预算 10000 B/s（含包头及 CRC）。
最小 DATA 间隔为 ceil(包长 × 1000000 / 150000) 微秒，按实际飞控调用取样，不补发。
当前载荷175字节，包长203字节（含 BMI088 校准后原始值及温补状态），DATA 最小间隔1354 µs。
飞控更新约1 kHz时，按现有离散取样逻辑约每两次更新发送一次，约500 Hz；
实际频率看 `record_hz`。BMI088 独立读取约1 kHz，黑匣子采集其最近一致快照。
`fc_timestamp_us` 为飞控端64位微秒时间戳，与包头 `sample_time_us` 相同，
从本次上电黑匣子首次维护调用开始计时，在采集遥测快照前读取；不是UTC、IMU测量时间或F407接收时间。
定义启动先发完，再发数据；每2秒启动重发，周期分片至少间隔20ms。
BMI088 的14个定义包含数组，CSV 会自动展开如 `bmi088_gyro_rad_s[0]`。
BMI088原始值语义已统一为校准后、未滤波物理量；未校准运动寄存器及重复校准数组不再发送。
同名旧日志字段为未校准值、新日志已校准，用schema/`bmi088_calibration_version`区分；
保留温度测量、版本及温补域状态，参数依据见 `BMI088_CALIBRATION.md`。
F407 和转换工具仍使用原 SBB3 版本，无需因新增 BMI088 字段修改或重新烧录。
无 IMU 时仅维护/发送定义，不生成旧采样数据。DWT 时间由飞控任务定期扩展，调试暂停不作为有效运行时间依据。

Watch：`g_blackbox_tx_stats` 中 schema_id、payload_size、config_error、record_hz、
bandwidth_skip_count、schema_submitted_count、schema_completed_count、data_completed_count，
以及原有 busy_drop_count、error_count、timeout_count、abort_count。
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
