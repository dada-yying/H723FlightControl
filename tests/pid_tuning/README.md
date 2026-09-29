# 单轴内环调参

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

VOFA 使用 115200、8N1、JustFloat、100 Hz。原 CH0~CH6 保持不变，Cube 仍用 CH0~CH2。
CH7/8 为 Roll 目标/实际，CH9/10 为 Pitch 目标/实际，CH11/12 为 Yaw 目标/实际，单位均为 °/s。
CH13 为 Roll 滤波前角速度（`-gyro_y`），单位 °/s；每帧 14 个 float 加 JustFloat 帧尾，共 60 字节。
原 CH0~CH12 的顺序和含义不变，串口发送超时仍为 10 ms。
新增实际值采用控制器机体坐标；CH0~CH2 仍为原 IMU 姿态坐标。
角速度映射为 Roll = -gyro_y、Pitch = -gyro_x、Yaw = -gyro_z，先映射再滤波。
PID 与 CH8/10/12 共用这些滤波反馈；目标角速度、CAN 解包及姿态角映射未交换。
`g_flight_control_state.roll_rate_raw_rad_s` 保存参与本次滤波的原始 Roll 输入，内部单位 rad/s。
仅在新有效 GYRO 样本时更新，上电为零；无新样本或 IMU 无效/超时时保持上次值，恢复后随新样本更新。
CH8 与 CH13 从同一次飞控快照取值。CH13 仅供观察，不替代 PID 反馈。
两者均以 100 Hz 抽样发送，不是完整的高频原始数据记录；比较滤波效果时可同时观察 CH7、CH8、CH13。

## 角速度一维卡尔曼滤波

三轴反馈各自维护角速度估计 x 和协方差 P，使用随机游走模型，共用 Q/R 参数。
滤波常开，PID 和 VOFA CH8/10/12 使用同一滤波结果，不融合姿态角、不估计零偏。
`g_atom_imu_data.gyro_x/y/z` 仍保留原始 rad/s 数据。

Keil Watch 接口（旧 cutoff_hz / applied_cutoff_hz 已移除）：

| 字段 | 默认值 | 含义 |
|---|---|---|
| `g_flight_control_filter.q` | 0.07510204 | 每 1 ms 的过程噪声协方差参数 |
| `g_flight_control_filter.r` | 1.0 | 测量噪声协方差参数 |
| `g_flight_control_filter.applied_q` | 0.07510204 | 实际生效 q，只读观察 |
| `g_flight_control_filter.applied_r` | 1.0 | 实际生效 r，只读观察 |
| `g_flight_control_filter.gain[0..2]` | 初始化前 0 | 最近更新增益，Roll/Pitch/Yaw；首样本直接采纳时为 1 |

q/r 的名义协方差单位均为 (rad/s)^2；q 在每次更新按 dt/0.001 缩放。
默认值是响应匹配参数，不是实测噪声标定。在 1 kHz 下稳态 K 约 0.239057，
接近原 50 Hz 一阶低通；不保证默认设置比原低通更能抑制振荡。
固定采样周期、固定 Q/R 时，稳态响应仍相当于固定增益的一阶平滑。

q、r 均只接受 1e-6～1e3 的有限值；任一非法则整组拒绝，继续使用上一组有效值。
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

运行 `tests/pid_tuning/run.ps1`：用 MSVC 编译实际 rc_sbus、flight_control、vofa 源码，
替代 HAL/RTOS/电调接口来检查映射、有效帧确认、回中门控、单轴 Mixer、保护和串口字节布局。
这不验证实际中断调度、串口时序或电机响应。
测试还覆盖滤波初始化、默认稳态增益、正负阶跃、独立轴、变周期、重复样本、在线改 q/r、
非法参数整组拒绝、超时恢复、计数回绕、极端 q/r 下协方差与增益，以及 PID/VOFA 使用滤波反馈。
轴交换回归分别检查 Roll/Pitch 的 PID 电机修正、CH8/10 对应关系，以及姿态角和目标值保持不变。
CH13 测试检查上电零值、原始值符号、正负阶跃、重复样本、失效/超时恢复、单位转换和 60 字节帧尾位置；
同时回归原 CH0~CH12 布局、PID 仍使用滤波反馈，以及独立三通道姿态发送接口。

固件完整构建：`MDK-ARM/build_ws2812.ps1 -OutputDirectory roll-raw-telemetry-verified`。
在 Keil Rebuild 并烧录后，先拆桨验证 CH5/CH10、模式切换、回中和失联保护。
