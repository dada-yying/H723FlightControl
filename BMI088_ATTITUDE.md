# 板载 BMI088 独立姿态解算与记录

## 参考实现与选择

- [Meta-Team/Meta-Embedded-NG](https://github.com/Meta-Team/Meta-Embedded-NG)：
  同款达妙 DM-MC02 / STM32H723、SPI2 BMI088、FreeRTOS、40°C温控。
  检查了固定提交 `a0ebb9596b9cf5a827140eb4209af1628ea67525` 的
  `module/imu/ins_task.c`，参考其重力初始化、四元数和DWT实测周期的链路；
  本次不照搬其EKF、阻塞初始化、在线校准或控制耦合。
- [xioTechnologies/Fusion](https://github.com/xioTechnologies/Fusion)：嵌入式C四元数
  融合，可仅用陀螺仪与加速度计，MIT许可。本次直接移植其AHRS实现，固定提交
  `a8d7224f36a0ec82345ef49a3db50e65f8d3bab8`，来源/许可见 `ThirdParty/Fusion`。
  它是修订版Madgwick互补融合，不是角速度的一维卡尔曼或四元数EKF。

只有BMI088新读取样本进入本模块。ATOM仍是当前飞控控制源。
不修改ATOM CAN解包、RP陷波/三轴卡尔曼、PID、Mixer、RC、电机保护或VOFA。
不把新估计作为飞控反馈；新增姿态仅用于Watch和黑匣子对比。

## 轴向和单位

依据六面日志的重力主分量：BMI原生 `X=右、Y=前、Z=上`。
姿态模块对陀螺仪与加速度计使用同一个正交旋转：

```text
body_x = native_y    // 前
body_y = native_x    // 右
body_z = -native_z   // 下
```

内部四元数wxyz采用FRD机体→相对NED参考系、ZYX欧拉顺序。
标准NED欧拉Pitch是抬头正，输出时取负以符合当前显示约定。
新记录的Roll右侧下沉正，Pitch机头下俯正，Yaw右偏航正。
原 `bmi088_gyro_rad_s/accel_m_s2` 始终保持校准、温补后未滤波原生XYZ，不在发布层重排。
不能照抄ATOM的角速度符号用于另一块IMU；实机方向仍需拆桨确认。

状态角单位rad；黑匣子字段单位deg。Roll/Yaw一般在±180°，Pitch在±90°；
Pitch接近±90°时欧拉Roll/Yaw耦合并可能跳变，内部单位四元数仍可连续解算。
不计算连续展开的Yaw或绝对磁航向。

## 更新、初始化与异常

- `Bmi088_ReadSample()`完整SPI/ID检查与既有校准完成后，执行
  `Bmi088Attitude_Compute()`。计算在临界区外；匹配的BMI原始值与姿态在同一短临界区发布。
- 沿用TIM7每500µs通知的现有BMI任务，不新增任务或ISR数学。
  DWT实测两次完整读取的完成周期，正常用实际dt积分，不固定假设0.5ms，也不补旧样本。
  重复sample_count不迭代，支持sample_count、HAL tick和DWT计数回绕。
- 启动及故障恢复先累计至少64个、连续不少于50ms的近静止样本；每轴角速度需不超过
  10°/s，加速度模长在0.8～1.2g。平均重力直接初始化Roll/Pitch，Yaw设为0。
  这是合作式初始化，不延时阻塞其他任务；未满足条件保持姿态无效。
- 固定融合增益0.5，倾角残差拒绝阈值10°、恢复门限按名义2kHz/5秒配置；
  `FusionAhrsSkipStartup()`用于已重力对准的状态，避免默认启动过程重复清零Yaw。
- 加速度模长在0.8～1.2g外（含自由落体）不作重力校正，仍可由有效陀螺仪预测。
  Fusion还会判断倾角残差并按其恢复逻辑工作；门控不能识别所有线加速度/振动。
  恢复计数依赖名义采样频率，采样持续降速时实际恢复等待会改变。
- 输入NaN/Inf、异常幅值、角速度任一轴达到±1980°/s、无有效BMI数据，或完整样本间隔
  达到20ms，姿态标无效、保留最后数值，并要求重新重力对准；不跨长断流积分。
  dt=0或内部四元数/欧拉非有限也按无效处理。以上不改变当前电机安全逻辑。
- 有效标志意味着解算输入/数值路径可用，不是姿态准确度证明。
  BMI本身仍±3g/±2000°/s；新读取目标2kHz，陀螺ODR2kHz/230Hz带宽，加速度ODR1.6kHz。
  目前没有传感器DRDY/同步，读取可能重复/跳过转换。此次不改采样架构或校准系数。

**没有磁力计/外部航向：Yaw仅相对于本次初始化方向，残余零偏会累积漂移。**
每次成功重新对准 `epoch` 加1，Yaw重新定义为0；不能把跨epoch航向当连续绝对航向。
启动时持续平移、振动或缓慢转动会影响重力初始化，采集时应静置；温补/安装误差依然存在。

## Watch与黑匣子

`g_bmi088_attitude_state`：roll_rad、pitch_rad、yaw_rad、quaternion[0..3]、dt_s、
accel_norm_m_s2、initialized、valid、accel_used、error、epoch、update_count、reset_count、
rejected_count、accel_rejected_count、sample_count、sample_tick_ms、age_ms。
`Bmi088Attitude_GetState()`会更新年龄/有效性，不生成新样本。
`Bmi088_GetSnapshot()`同时读取BMI运动数据和匹配姿态，用于黑匣子。

error：0等待源、1对准中、2正常、3源无效、4周期中断、5非法输入、6陀螺饱和、7数值错误。

原SBB3载荷0～164不变，追加：

| 字段 | 偏移 | 类型/单位 |
|---|---:|---|
| bmi088_roll_deg | 165 | F32，deg |
| bmi088_pitch_deg | 169 | F32，deg |
| bmi088_yaw_deg | 173 | F32，deg，相对epoch |
| bmi088_attitude_valid | 177 | U8 |
| bmi088_attitude_epoch | 178 | U32 |
| bmi088_attitude_accel_used | 182 | U8 |
| bmi088_attitude_error | 183 | U8 |

载荷184字节，线路包212字节，独立TIM4/BlackboxTask目标700Hz，DATA为148.4kB/s。
定义重发、DMA忙或任务延迟会跳过时隙，查看record_hz；不再依赖飞控更新或ATOM通知。
串口仍UART7 2Mbps；DMA缓冲512字节不变。
**F407生产固件与转换工具无需修改或重烧**，依新schema自动生成上述CSV列。
新数据与ATOM三轴姿态是不同传感器的估计，不是重复字段；VOFA23通道保持现状。
原始/姿态采集2kHz不代表日志保存所有2kHz样本，ATOM断流时原有黑匣子DATA仍停止。

## 验证

`tests/pid_tuning/run.ps1`运行真实Fusion/姿态/采集/校准代码的主机测试：
非水平/倒置/六面初始化，独立正负三轴及耦合转动、Yaw回绕，实测可变dt、重力校正和加速度拒绝，
重复/回绕、未对准、饱和/非有限/超时恢复、成对快照与原始值保持；回归PID/VOFA/保护/加热。
`tests/verify_bmi088_blackbox.py`在F407真实存储测试后检查原数据和新增姿态逐字段还原。
ARMCC构建及 `verify_optical_build.py` 检查DMA、任务静态调用深度与堆；均不能证明实机精度/WCET。

首次拆桨、锁定、静置冷上电。有效后右滚、下俯、右转分别检查对应正方向；
倾斜保持时Roll/Pitch稳定，转动结束后相对Yaw保留（静置会慢漂）。
断开BMI/SPI故障或暂停采集超过20ms，姿态无效而ATOM保护行为不变；恢复静置后epoch增加。
连续记录至少60秒检查新CSV列、BMI read_hz、skipped_trigger_count、
max_read_duration_us、stack_free_words及飞控频率，确认未因解算成本持续掉采样。
对比ATOM需考虑安装零点、不同算法延迟、Yaw参考不同，不以曲线完全重合为正确性条件。

### 本轮已执行的检查（2026-10-03）

- H723主机：9组测试＋额外字段注册表测试全部通过，包含耦合转动和Yaw回绕。
- H723 ARMCC5：59源文件编译、链接和HEX生成通过，无编译警告。
  `MDK-ARM/bmi088-attitude-verified/firmware.hex`，SHA256：
  `635B04E0E086466B89E5AC6D522F3C5F7B60E9C2D91A04EBFBD06A7FEC2E81DE`。
- 原F407固件实际接收/FatFs存储主机回归及20项Python转换回归通过；
  两种H723注册表各2915条记录逐字段还原通过，新增姿态、单位和有效/epoch状态正确。
  F407生产代码和转换工具没有改动。
- 静态栈深度：BMI任务352/1024字节，飞控任务364/1024字节；
  原六任务分配6304/15360字节堆，仍需实机高水位/堆余量确认。
  UART7 DMA缓冲0x24000000/512字节，光流DMA0x24000420/256字节，
  SBUS DMA0x24000520/64字节，均AXI SRAM且32字节对齐。
- 尚未烧录；没有验证实际2kHz耗时、姿态准确度或带桨飞行稳定性。
