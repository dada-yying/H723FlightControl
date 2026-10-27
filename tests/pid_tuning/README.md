# 单轴内环调参

低油门保护：最后有效 CH3 小于 `0x017E` 时，正常模式和单轴调参均输出四路最小油门，
清除 PID 状态并保持解锁；抬油门恢复。规则及专项回归见 [THROTTLE_STOP.md](../../THROTTLE_STOP.md)。

当前正常模式已支持 yaw 松杆捕获实际航向并保持，见 [YAW_HOLD.md](../../YAW_HOLD.md)。
当前反馈为 BMI088 RFU：yaw 俯视逆时针为正。下文 ATOM 映射、卡尔曼参数等旧描述
保留为历史记录，不适用于当前反馈链；单轴调参操作仍按本节执行。

锁定电调后，在 Keil Watch 将 `g_flight_control_tuning.pid_debug` 设为 1。
`active=1` 表示已经进入调参模式；设为 0 请求恢复双环，同样只在锁定时生效。
解锁期间的模式请求会保留，到锁定后生效。上电始终默认 0。

CH5 低位 Roll、中位 Yaw、高位 Pitch，`selected_axis` 分别为 0、1、2。
新档位连续三个有效帧后确认。CH10 滑杆最高为 +90°/s，中位为 0，最低为 -90°/s。
`input_rate_rad_s` 是映射后指令，控制器内部单位仍为 rad/s。

进入调参、切轴、重新解锁后，必须收到新的 CH10 回中有效帧；
`waiting_center=0` 后才接受非零指令。等待期间三个轴修正输出均为零，基础油门仍有效。
仅所选轴运行 PID，另外两轴没有角速度稳定作用。
PID 参数仍使用 `g_flight_control_pid.rate_roll/rate_pitch/rate_yaw` 的 kp、ki、kd。

VOFA 使用 230400、8N1、JustFloat、100 Hz；电脑端需同步设置 230400。CH0/1/2 统一显示控制器 Roll/Pitch/Yaw 姿态角，单位 °，Cube 仍用 CH0~CH2。
姿态映射为 Roll = -imu.pitch、Pitch = +imu.roll、Yaw = -imu.yaw；右滚、机头下俯、右偏航为正。
Roll 符号按最新台架反馈修正：相对最初直接发送原始 IMU 的版本，CH0 为原 CH1 取负，CH1 为原 CH0，CH2 为原 CH2 取负。
姿态角与角速度从同一次 FlightControl_GetState 快照发送；不再单独读取原始 IMU 姿态。
Roll/Pitch 角度外环使用同一姿态状态，公式仍为 Kp × (目标角度 - 实际角度)，限幅 ±4 rad/s。
锁定时不输出电机修正；正常模式 yaw 打杆控制角速度、回中捕获实际航向。单轴调参绕过航向保持。
CH3~CH6 的油门、遥控目标含义不变，CAN 解包与 g_atom_imu_data 原始字段保持不变。
CH7/8 为 Roll 目标/实际，CH9/10 为 Pitch 目标/实际，CH11/12 为 Yaw 目标/实际，单位均为 °/s。
CH13 为 Roll 滤波前角速度（`-gyro_y`），单位 °/s；每帧 23 个 float 加 JustFloat 帧尾，共 96 字节，CH20～22 见下文。
CH14 为 Pitch 滤波前角速度（`+gyro_x`），CH15 为 Yaw 滤波前角速度（`-gyro_z`），单位均为 °/s。
CH16～CH19 依次为 M1 左后、M2 右后、M3 右前、M4 左前的归一化输出（0～1），从 EscPwm_GetState 的 command 读取。
这是安全处理及限幅后的电机指令，不是实测转速；锁定显示 0，对应 1000 µs。
三个原始角速度在滤波入口保存，通过同一次 FlightControl_GetState 获取；电调另取安全状态快照，二者不是原子同步采样。
100 Hz、8N1 下需要 96000 bit/s：230400 占用 41.7%（每帧线路约 4.17 ms）。
理论带宽不代表实际调度/USB 链路稳定；三轴原始角速度仍是 100 Hz 抽样，不是完整高频日志。

Watch 观察 `g_vofa_tx_stats.success_count`、`failure_count`、`last_status`、`max_duration_ms`。
计数覆盖实际 UART 调用（参数校验失败不计入），包括独立姿态发送；上电清零。HAL 返回值为 OK=0、ERROR=1、BUSY=2、TIMEOUT=3。
耗时为 HAL 毫秒 tick 差，含任务抢占，最大值为上电以来峰值；发送失败不立即重试。
拆桨验证至少 60 秒：成功计数约增加 6000，失败计数不增长，最大发送耗时应留有 10 ms 周期余量，飞控频率正常。
通过此检查不能证明实机方向、带桨稳定性或没有高频噪声；串口侧还应核对完整帧和新增通道。
CH7~CH13 的顺序和含义不变，串口发送超时仍为 10 ms；CH7/CH9 随修正后的姿态反馈重新计算。
姿态与角速度均使用控制器机体轴向，不再混用原始姿态坐标与控制坐标。
角速度映射为 Roll = -gyro_y、Pitch = +gyro_x、Yaw = -gyro_z，先映射再滤波。
Pitch 角速度按最新台架反馈修正为 +gyro_x，机头下俯转动为正；Pitch 姿态角仍为 +imu.roll，不改变。
Roll/Pitch 在卡尔曼前增加 100 Hz / BW 60 Hz 陷波，Yaw 仅经过卡尔曼；详见 ../../GYRO_NOTCH.md。
PID 与 CH8/10/12 共用最终滤波反馈，CH10 同步体现修正后的 Pitch 角速度。
`g_flight_control_state.roll_rate_raw_rad_s` 保存参与本次滤波的原始 Roll 输入，内部单位 rad/s。
仅在新有效 GYRO 样本时更新，上电为零；无新样本或 IMU 无效/超时时保持上次值，恢复后随新样本更新。
CH8 与 CH13 从同一次飞控快照取值。CH13 仅供观察，不替代 PID 反馈。
两者均以 100 Hz 抽样发送，不是完整的高频原始数据记录；比较滤波效果时可同时观察 CH7、CH8、CH13。

## SPI-SD 黑匣子

当前记录方式为H723 SPI1直接写FAT32 SD卡，TIM4独立1kHz采样。
USART1 VOFA不变；UART7不再发送。详见 ../../BLACKBOX_V3.md。
test_blackbox.c验证现有字段、单位与SD入队；test_blackbox_schedule.c验证
100秒定时、通知合并、不补发、锁定/断流记录、队列拒绝和故障隔离。
../run_sd.ps1覆盖SPI协议/CRC/DMA超时及真实FatFs恢复，基础与扩展字段
分别经未修改的通用转换工具恢复。物理SD、信号时序及飞行效果需实机验证。

## 角速度一维卡尔曼滤波参数

三轴反馈各自维护角速度估计 x 和协方差 P，使用随机游走模型。
Roll/Pitch 共用 q/r，Yaw 独立使用 yaw_q/yaw_r，各自按 dt 计算 Q_step。
Roll/Pitch 的 measurement 为前置陷波的输出，Yaw 的 measurement 为机体系原始角速度。
滤波常开，PID 和 VOFA CH8/10/12 使用同一滤波结果，不融合姿态角、不估计零偏。
`g_atom_imu_data.gyro_x/y/z` 仍保留原始 rad/s 数据。

Keil Watch 接口（旧 cutoff_hz / applied_cutoff_hz 已移除）：

| 字段 | 默认值 | 含义 |
|---|---|---|
| `g_flight_control_filter.q` | 0.015 | Roll/Pitch 每 1 ms 的过程噪声协方差参数 |
| `g_flight_control_filter.r` | 1.0 | Roll/Pitch 测量噪声协方差参数 |
| `g_flight_control_filter.applied_q` | 0.015 | Roll/Pitch 实际生效 q，只读观察 |
| `g_flight_control_filter.applied_r` | 1.0 | Roll/Pitch 实际生效 r，只读观察 |
| `g_flight_control_filter.yaw_q` | 0.015 | Yaw 每 1 ms 的过程噪声协方差参数 |
| `g_flight_control_filter.yaw_r` | 1.0 | Yaw 测量噪声协方差参数 |
| `g_flight_control_filter.applied_yaw_q` | 0.015 | Yaw 实际生效 q，只读观察 |
| `g_flight_control_filter.applied_yaw_r` | 1.0 | Yaw 实际生效 r，只读观察 |
| `g_flight_control_filter.gain[0..2]` | 初始化前 0 | 最近更新增益，Roll/Pitch/Yaw；首样本直接采纳时为 1 |

q/r 的名义协方差单位均为 (rad/s)^2；q 在每次更新按 dt/0.001 缩放。
保留用户当前生产默认值 q=0.015、r=1，不是实测噪声标定。
主机参考响应测试显式设 q=0.07510204、r=1，在 1 kHz 下稳态 K 约 0.239057，接近原 50 Hz 一阶低通系数；不修改生产默认值。
固定采样周期、固定 Q/R 时，稳态响应仍相当于固定增益的一阶平滑。

两组 q、r 均只接受 1e-6～1e3 的有限值；一组内任一非法则该组拒绝，继续使用该组上一有效值，另一组独立生效。
解锁期间也允许修改，下一次新 GYRO 样本更新生效，不重置 x/P 或 PID 历史。
Watch 逐个写入时，每个新样本会校验当时读到的一组 q/r；没有额外的提交开关。
增大 q/r 比值会更快跟随、减弱平滑；减小则增强平滑、增加延迟。

更新：P_pred=P+q*dt/0.001，K=P_pred/(P_pred+r)，x+=K*(measurement-x)。
协方差使用 Joseph 形式更新：P=(1-K)^2*P_pred+K^2*r。
首次有效样本及 IMU 超时恢复直接初始化 x，并用当前 Q_step/r 的稳态后验值初始化 P。
重复计数不执行预测或更新，也不应用新参数；跳过的样本不补算。
锁定、RC 失联、切换模式或轴不重置正常滤波；上电恢复默认 q/r，不保存参数。
在线改参会改变闭环响应，先拆桨验证；曲线更平滑不代表闭环或飞行稳定性通过。

## 验证

陷波专项参数、更新率校正、Watch 和测试说明见 ../../GYRO_NOTCH.md。

运行 `tests/pid_tuning/run.ps1`：用 MSVC 编译实际 rc_sbus、flight_control、vofa 源码，
替代 HAL/RTOS/电调接口来检查映射、有效帧确认、回中门控、单轴 Mixer、保护和串口字节布局。
这不验证实际中断调度、串口时序或电机响应。
测试还覆盖滤波初始化、默认稳态增益、正负阶跃、独立轴、变周期、重复样本、在线改 q/r、
非法参数整组拒绝、超时恢复、计数回绕、极端 q/r 下协方差与增益，以及 PID/VOFA 使用滤波反馈。
姿态映射回归检查正负方向、三轴独立性、外环回正方向和限幅、锁定输出，以及 CH0~CH2 与状态快照一致。
原始姿态 (0.1, -0.2, 0.3) rad 对应控制姿态 (0.2, 0.1, -0.3) rad；零目标、两个外环 Kp=0.5 时，
Roll/Pitch 目标角速度分别为 -0.1/-0.05 rad/s。继续检查两轴内环电机修正、CH8/10 反馈及原始 IMU 字段不被改写。
新增静态倾斜回正测试：零目标、零角速度、正 P 增益且 I/D=0，左侧下沉（原始 imu.pitch>0）时左侧 M1/M4 增加、右侧 M2/M3 减少；右侧下沉时反向。
此测试按用户报告的方向建立，不代替实机轴向、电机位置和正负号复核。
Pitch 角速度阻尼测试隔离角度外环（角度 Kp=0、角速度 Ki/Kd=0、正角速度 Kp），目标角速度为零：
机头向下转动（gyro_x>0）时前方 M3/M4 增加、后方 M1/M2 减少；向上转动时相反。
同时检查正负首样本、后续滤波更新、CH10 与 PID 反馈一致，以及 Pitch 姿态角符号和 CH1 不变。
CH13～CH15 测试检查上电零值、原始值符号、正负阶跃、重复样本、失效/超时恢复、单位转换和旧 20 通道兼容接口的 84 字节帧尾位置；
同时回归通道布局、PID 仍使用滤波反馈，以及独立三通道姿态发送接口。
test_vofa_esc.c 编译真实 esc_pwm 和 vofa，检查四路限幅、PWM 映射、安全锁定后输出归零、串口错误统计和 tick 回绕。
这些测试不执行默认任务的真实调度；默认任务快照来源和 100 Hz 周期另作源码检查与实机验证。

固件完整构建：`MDK-ARM/build_ws2812.ps1 -OutputDirectory optical-flow-verified`。
在 Keil Rebuild 并烧录后，先拆桨、保持锁定，确认右滚使 CH0 增大、下俯使 CH1 增大、右偏航使 CH2 增大。
零姿态目标、小的正外环增益下检查回正指令方向，再回归 CH5/CH10、模式切换、回中和失联保护。
拆桨验证 Pitch 纯内环时先令角度增益为零、角速度目标为零，用小的正角速度 P 检查动态阻尼方向；
机头向下转动时 CH10 应为正，前方 M3/M4 输出应高于后方 M1/M2。不要把停止转动后的静态姿态回正与角速度阻尼混淆。
主机测试与编译不验证实机方向或闭环稳定性；不要直接沿用高增益解锁测试。

## 光流扩展回归

见 ../../OPTICAL_FLOW.md。test_optical_flow.c 检查任意分片、粘包、噪声、错误帧、未知消息、整数边界、缓存/DMA 回绕、重复事件、队列溢出、UART 恢复/重试、tick 回绕及年龄饱和。运行 run.ps1 会输出 H723 真实组帧文件到 F407 tests/build/h723_wire.bin；随后运行 F407 tests/run.cmd 或直接 tests/run_cross.ps1，验证真实接收代码和 FatFs 写出的多分段文件，再由 Python 逐字段恢复。该测试不代替实机 DMA/缓存时序和 SD 卡可靠性验证。
## BMI088 logging tests

`test_bmi088.c` exercises the production acquisition module with a register/SPI
model, including the accel dummy byte, coherent sample publication, signed
raw/physical values, temperature, delayed config verification, failures/retries
and tick wrap. `test_blackbox.c` checks the added schema/payload and unchanged
VOFA. The optional task and sensor have no flight-control consumer.
After the F407 storage tests, run `python -B tests/verify_bmi088_blackbox.py`
from the H723 directory to check every restored BMI088 field.

`test_bmi088_heater.c` uses the real sensor model to verify fixed 40 C thermal
PI, saturation/anti-windup, independent stale-sample shutdown, manual/latched
stops, failures, tick wrap, and ESC isolation. The thermal plant model is not
a real-board temperature guarantee.

BMI088 raw now means calibrated/unfiltered physical values in the canonical
gyro_rad_s/accel_m_s2 arrays. Register/nominal motion values and duplicate
calibrated arrays are no longer published/logged. Acquisition tests check the
compiled static/temperature coefficients, cold/intermediate/hot cases, model
bounds/invalid temperatures, signed gyro correction, six +/-XYZ gravity vectors,
no reapplication on reads, coherent fault/timeout hold and recovery. Blackbox
tests check the revised schema/version and absence of pre-calibration motion;
the unchanged F407/converter cross-check compares exact float32 results.
`tests/test_bmi088_calibration_analysis.py` separately checks tilted-face fits,
invalid orientation coverage, temperature warm-up and collision guards using
the bundled Python numpy/pandas runtime. No tests imply a real-board calibration
or flight accuracy guarantee.

`tests/test_bmi088_temperature_analysis.py` checks anchored gyro bias regression,
gravity-norm thermal fitting and equal recording weights. The real-log analysis
uses leave-one-recording-out validation and rejects poorly repeatable accel
thermal models; colder incomplete coverage is reported, not silently validated.

Blackbox regression now checks a 218-byte payload/246-byte wire DATA packet (including yaw hold fields),
the added independent BMI attitude/status, capture time only in the header/CSV sample_time_us, and removal
of the duplicate Celsius-convertible temperature counts. The calibrated BMI
values/metadata and optical bytes are unchanged. Schema periodic recovery,
low-rate sensor snapshot repetition, VOFA layout and all safety logic remain.

`test_bmi088_attitude.c` executes the pinned MIT x-io Fusion code, production
adapter and real SPI publisher. It checks gravity/6-face/non-level/inverted
alignment; independent positive/negative roll/pitch/yaw; variable measured dt;
gyro-only prediction, magnitude/angular gating; sample/DWT/tick wrap; coherent
calibrated raw/attitude pairs; stale/gap/error recovery and relative-Yaw epochs.
It also rejects non-finite/saturated inputs, zero dt and numerical corruption.
The 500us timer fixture runs 4000 acquisition events without changing RTOS tick,
coalesces old triggers and covers timer init/start failures. None proves physical
2kHz timing, attitude accuracy or flying safety. Check live read_hz/stack headroom.

`test_bmi088_gyro_filter.c` tests the actual three-axis single-precision DF-I
Butterworth against an independent double-precision sin/cos coefficient/reference
recurrence and analytic frequency response. Covers exact first-value seeding,
positive/negative steps, DC/5/50/200 Hz gains, independent axes, duplicate counts,
128-interval/three-window rate changes, ordinary jitter, unsupported Nyquist
rates, NaN/Inf/invalid coefficients/numerical corruption, timeout/init recovery,
tick/DWT/count wrap, raw preservation, PID-reset isolation and filtered AHRS input.
The logger retains raw deg/s fields and adds filtered deg/s/valid/cutoff/design
rate. Full H723/F407/CSV fixtures check these through unchanged receiver/tools.
Current ARMCC static BMI task depth is 360/1024 (was 352); this is not runtime
WCET/stack proof. The heater's pre-existing old-parameter assertion is reported
separately; production heater/calibration/ATOM paths are not altered by the LPF.
