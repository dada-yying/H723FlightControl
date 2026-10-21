# 板载 BMI088 读取与黑匣子记录

## 数据语义

从校准版本 **2026100302** 开始，程序、黑匣子及后续文档中的
**BMI088“原始值”均指已校准、尚未软件滤波的物理量**。

- `g_bmi088_data.gyro_rad_s[0..2]`：校准、温补后的传感器XYZ角速度，rad/s。
- `g_bmi088_data.accel_m_s2[0..2]`：六面校准后的XYZ加速度，m/s²，保留重力。
- 删除公开的运动寄存器 `gyro_raw/accel_raw` 和重复的 `*_calibrated_*` 数组。
- 校准前运动值仅在完整读取函数局部存在，校准完成后才发布一致快照，不记录。
  快照读取不会再次扣零偏；不做启动静止清零或自适应校准。
- 温度及温度寄存器不是本次运动校准对象，仍保留在BMI模块供温控/Watch诊断；
  黑匣子仅记录摄氏温度，不重复记录可精确换算的温度寄存器计数。

参数依据和升温模型见 `BMI088_CALIBRATION.md`。参数编译进程序，不在线改参或另写Flash。
温补从首个成功完整读取起执行，不等待40°C。模型数值域22.625～40.25°C，
域外用最近边界系数并标记域外；域内标志不是精度认证。
加速度温补候选未稳定改善日志误差，继续使用六面偏置/比例。

上述原始值不做机体轴向交换或软件滤波。另有独立姿态解算/黑匣子记录，
见 `BMI088_ATTITUDE.md`；其轴向变换不回写原始值，也不成为闭环控制输入。
飞控仍使用ATOM-IMU，现有PID、陷波、卡尔曼、VOFA和电机保护不变。
40°C加热见 `BMI088_HEATER.md`。真实 `temperature_c` 不被模型温度限幅覆盖，
原有无效/超时/45°C过热锁存保护不变。

历史schema `75b52809` 的日志保持当时含义（未校准运动值），不重写源CSV/BIN。
新版同名物理量已校准，以schema及校准版本区分。
历史分析脚本从旧寄存器日志拟合，不可将新版原始值当成未校准数据再次扣偏置。

## 硬件和采样

依据达妙 `CtrBoard-H7_IMU` 例程及
[Bosch BMI088数据手册](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)。
SPI2模式3、8位、MSB first，PLL1Q 240MHz/32=7.5MHz。
PB13/AF5=SCK、PC1/AF5=MOSI、PC2_C/AF5=MISO、PC0=ACC_CS、PC3_C=GYRO_CS。
PC2_C/PC3_C数字开关显式闭合，没有新增SPI DMA/IRQ/EXTI。

加速度±3g、1600Hz、normal带宽（ACC_CONF=0xAC）；
角速度±2000°/s、2000Hz/230Hz内部带宽（GYR_BANDWIDTH=0x81）。
名义换算0.0008974358974m/s²/LSB和0.0010652644360316953rad/s/LSB只用于内部临时值。
温度11位有符号raw×0.125+23°C，内部温度转换约1.28秒更新。

`Bmi088Task` 栈1024字节、Normal1优先级，低于RC/飞控、高于默认遥测。
TIM7计数1MHz、ARR499，每500µs通知一次，IRQ优先级6；
仅在调度器启动后的BMI任务中启动，不在ISR中读SPI或解算。
约每500µs尝试完整读取并更新独立姿态，不补时隙；加速度最高1.6kHz，
读取快照可能重复/跳过转换，陀螺2kHz也不等于已使用DRDY同步。
`sample_count` 是完整SPI读取次数，不是DRDY计数；`sample_time_ms` 是读取完成HAL tick。

四次短SPI burst成功且ID正确才一起发布，每次传输超时1ms，失败总释放CS。
复位等待80ms/配置写读按合作状态机推进，不在等待/传输中长时间关中断。
运行错误保留最后完整校准值、标无效，1秒后重试初始化。
可选外设或任务创建失败不进全局Error_Handler，不影响ATOM/电机。

## Watch

- `gyro_rad_s`、`accel_m_s2`：唯一的校准后、未滤波运动值。
- `temperature_c`：测量温度；`temperature_raw`：温度寄存器计数。
- `compensation_temperature_c`：模型实际使用的限域温度。
- `compensation_in_range`：测量温度在模型数值域内为1，不是精度保证。
- `calibration_version`：首帧前0，成功完整读取后2026100302。
- `has_sample/initialized/valid/sample_count/sample_tick_ms/age_ms`。
  valid是配置完成且最近完整SPI读取年龄不足20ms，不代表温补精度。
- error：0正常、1SPI、2加速度ID、3陀螺仪ID、4加速度配置、5陀螺仪配置、6任务创建失败、7定时器失败。
- `g_bmi088_calibration`：只读偏置、比例、温度斜率、参考温度、边界、版本。
- `g_bmi088_stats`：阶段、初始化/读取/SPI错误计数、ID、寄存器读回、任务间隔/栈余量。
- `read_hz`：完整读取成功次数的1秒窗口统计；`timer_irq_count`：触发计数；
  `skipped_trigger_count`：读取前/执行中合并的触发，不补读；`sampling_timeout_count`：等待通知超时。
  `last_read_interval_us/max_read_interval_us`：读取起始间隔；`max_read_duration_us`：完整采集/解算最长耗时。
  `timer_running/last_timer_status/timer_error_count`：可选TIM7初始化/启动状态。

无样本时运动字段0、年龄0xFFFFFFFF；失效后保留最后值，必须同时看valid/age。
这些状态不参与当前电机安全判定。

## SBB3黑匣子字段

原0～111字节不变（ATOM/RC/电机/光流）。飞控时间戳仅存在包头，CSV仍有
`sample_time_us`；删除重复载荷字段 `fc_timestamp_us`。新BMI布局自动生成schema：

| 字段 | 偏移 | 类型 | 含义/单位 |
|---|---:|---|---|
| bmi088_gyro_deg_s | 112 | F32×3 | 校准、温补后未滤波XYZ，°/s（内部仍rad/s） |
| bmi088_accel_m_s2 | 124 | F32×3 | 六面校准后未滤波XYZ，m/s² |
| bmi088_temperature_c | 136 | F32 | 真实测量温度，°C |
| bmi088_sample_count | 140 | U32 | 完整读取次数 |
| bmi088_sample_time_ms | 144 | U32 | 完成HAL tick，ms |
| bmi088_age_ms | 148 | U32 | 样本年龄，ms |
| bmi088_has_sample | 152 | U8 | 是否曾读取 |
| bmi088_valid | 153 | U8 | 通信样本有效 |
| bmi088_initialized | 154 | U8 | 已配置 |
| bmi088_error | 155 | U8 | 错误原因 |
| bmi088_calibration_version | 156 | U32 | 校准版本 |
| bmi088_compensation_temperature_c | 160 | F32 | 限域模型温度，°C |
| bmi088_compensation_in_range | 164 | U8 | 在模型数值域内 |
| bmi088_gyro_filtered_deg_s | 184 | F32×3 | 三轴Butterworth低通后XYZ，°/s |
| bmi088_gyro_filter_valid | 196 | U8 | 滤波结果有效 |
| bmi088_gyro_filter_cutoff_hz | 197 | F32 | 实际截止频率，Hz |
| bmi088_gyro_filter_sample_hz | 201 | F32 | 实际系数设计采样率，Hz |

追加BMI姿态及状态的偏移165～183见 `BMI088_ATTITUDE.md`。
当前黑匣子通过SPI-SD独立1kHz记录，详见BLACKBOX_V3.md；BMI088采集仍为2kHz。
定义重发/DMA忙/调度延迟可跳过部分时隙，不补发；以 `g_blackbox_tx_stats.record_hz` 为准。
ATOM停止后仍记录最新快照及已有状态/年龄；包头采集时间不表示传感器发生了新转换。
新版不发送运动寄存器、未校准物理量或重复校准数组。
F407固件和转换工具不用改/重烧，CSV按新自描述schema自动增减列；VOFA23通道不变。

## BMI088三轴角速度低通

完整SPI读取、单位转换及校准温补之后，`Bmi088GyroFilter_Compute()`在Bmi088Task中执行。
直接Ⅰ型单精度biquad，三轴独立x1/x2/y1/y2，共用二阶Butterworth系数（Q=1/√2）。
默认截止频率集中定义在 `App/bmi088_gyro_filter.h` 的 `BMI088_GYRO_FILTER_CUTOFF_HZ=50.0f`；
对比40/50/60Hz需改该编译期常量并重新编译，不增加开关或Watch在线配置。
初始系数按TIM7目标500µs/2kHz设计。只有新sample_count更新一次，原始数组不覆盖，
首次有效样本直接作为输出并初始化历史。故障/20ms超时后结果失效，恢复由新样本初始化，
旧缓存不能恢复valid。NaN/Inf、原始陀螺饱和和非法时间/参数不会进入后续历史。
每128个成功有效样本的DWT间隔更新sample_hz，不使用日志频率或毫秒tick推算2kHz。
连续3个窗口相对设计频率偏离超过2%、窗口互相接近（2%容差、同侧）才重算系数；
普通抖动不逐样本重算，换系数保留历史。要求0<cutoff<0.45*sample_hz；低频不支持时
标记无效，持续恢复确认后重新初始化，不绕过滤波提供有效角速度。ISR不滤波/不算系数。
BMI姿态解算使用同次滤波角速度，仍按(nativeY,nativeX,-nativeZ)映射；原始输入的
异常/饱和/启动运动检查保留。加速度、温控和校准均未改；当前飞控仍使用ATOM。

Watch：原始值 `g_bmi088_data.gyro_rad_s[0..2]`，滤波值 `g_bmi088_gyro_filter.gyro_rad_s[0..2]`，
两者均原生XYZ/rad/s。滤波结构中valid/initialized/error、sample_count/sample_tick_ms/age_ms，
cutoff_hz/sample_hz/applied_sample_hz、coefficient_update_count/update_count/reset_count/rejected_count，
以及max_compute_duration_us可观察状态、测得频率、实际参数和成功计算耗时（含偶发重算）。
这些是观察字段，Watch写入不改变私有biquad状态或配置。单次
`Bmi088_GetSnapshotWithFilter()`可读取原始、滤波、姿态的一致快照，旧读取接口保留。
error=0等待，1正常，2源无效，3非有限输入，4陀螺饱和，5时间错误，6参数/频率不支持，7数值错误。
低通引入相位延迟；不将离线曲线平滑度等同于姿态精度、ATOM闭环或飞行稳定性。

## 验证

`tests/pid_tuning/run.ps1` 验证真实读取/温补代码：dummy/burst、完整样本发布、
首样本直接校准、温度边界/非有限值、正负三轴、六面重力、快照不重复扣偏置、
失败/超时/恢复，以及500µs事件/合并/定时器失败、姿态、加热、ATOM控制和串口回归。
F407 `tests/run.cmd` 执行真实接收与FatFs存储测试；
`tests/verify_bmi088_blackbox.py`核对校准后float32值、元数据及校准前运动字段缺失。
数值回归见 `tests/test_bmi088_calibration_analysis.py`、
`tests/test_bmi088_temperature_analysis.py`。
ARMCC构建后用 `tests/verify_optical_build.py <目录>` 检查DMA与静态栈/堆。

主机测试/编译/日志拟合不代替新硬件校准。拆桨、锁定、电机停止，从冷上电连续
记录到40°C稳定后至少60秒，比较分温区角速度均值、加速度模长、温度域状态和版本，
再验证另一轮上电/六面；最冷端覆盖不足与升温温度滞后不能掩盖。

## 插卡负载下的 SPI2 轮询修正（2026-10-21）

revision 10 在 HAL 的 SPI2/8 位主机/1ms TxRx 路径使用 DWT 测量实际等待时间，
修复跨毫秒 tick 即误报 HAL_TIMEOUT 的情况；到期后重查实际 FIFO/EOT，数据齐全后仍检查硬件错误。
不通过增加传感器超时参数或缩短 1000ms 错误恢复等待来掩盖错误。
DWT 异常时仍有 tick/有限循环兜底，真正未完成的数据不作为新样本发布。
新 Watch 结构 `g_bmi088_spi2_poll_diag` 提供 tick 边界避免计数、超时/硬件错误和清理前的首错寄存器快照。
实现与验证说明见 BLACKBOX_V3.md 的 revision 10；实机需同时记录 SD 队列/丢帧与 BMI088 有效性/读取频率。
