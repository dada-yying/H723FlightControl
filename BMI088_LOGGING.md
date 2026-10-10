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
| bmi088_gyro_rad_s | 112 | F32×3 | 校准、温补后未滤波XYZ，rad/s |
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

追加BMI姿态及状态的偏移165～183见 `BMI088_ATTITUDE.md`。
当前载荷184字节、线缆包212字节；独立TIM4/BlackboxTask以700Hz节拍采集，DATA为148.4kB/s。
定义重发/DMA忙/调度延迟可跳过部分时隙，不补发；以 `g_blackbox_tx_stats.record_hz` 为准。
ATOM停止后仍记录最新快照及已有状态/年龄；包头采集时间不表示传感器发生了新转换。
新版不发送运动寄存器、未校准物理量或重复校准数组。
F407固件和转换工具不用改/重烧，CSV按新自描述schema自动增减列；VOFA23通道不变。

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
