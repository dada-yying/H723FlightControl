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

VOFA 使用 230400、8N1、JustFloat、100 Hz；电脑端需同步设置 230400。CH0/1/2 统一显示控制器 Roll/Pitch/Yaw 姿态角，单位 °，Cube 仍用 CH0~CH2。
姿态映射为 Roll = -imu.pitch、Pitch = +imu.roll、Yaw = -imu.yaw；右滚、机头下俯、右偏航为正。
Roll 符号按最新台架反馈修正：相对最初直接发送原始 IMU 的版本，CH0 为原 CH1 取负，CH1 为原 CH0，CH2 为原 CH2 取负。
姿态角与角速度从同一次 FlightControl_GetState 快照发送；不再单独读取原始 IMU 姿态。
Roll/Pitch 角度外环使用同一姿态状态，公式仍为 Kp × (目标角度 - 实际角度)，限幅 ±4 rad/s。
锁定时仍计算外环目标，但不输出电机修正；Yaw 仍仅使用角速度环，不新增航向角控制。
CH3~CH6 的油门、遥控目标含义不变，CAN 解包与 g_atom_imu_data 原始字段保持不变。
CH7/8 为 Roll 目标/实际，CH9/10 为 Pitch 目标/实际，CH11/12 为 Yaw 目标/实际，单位均为 °/s。
CH13 为 Roll 滤波前角速度（`-gyro_y`），单位 °/s；每帧 20 个 float 加 JustFloat 帧尾，共 84 字节。
CH14 为 Pitch 滤波前角速度（`+gyro_x`），CH15 为 Yaw 滤波前角速度（`-gyro_z`），单位均为 °/s。
CH16～CH19 依次为 M1 左后、M2 右后、M3 右前、M4 左前的归一化输出（0～1），从 EscPwm_GetState 的 command 读取。
这是安全处理及限幅后的电机指令，不是实测转速；锁定显示 0，对应 1000 µs。
三个原始角速度在滤波入口保存，通过同一次 FlightControl_GetState 获取；电调另取安全状态快照，二者不是原子同步采样。
100 Hz、8N1 下需要 84000 bit/s：115200 占用 72.9%（单帧至少 7.29 ms），230400 占用 36.5%（至少 3.65 ms）。
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
卡尔曼滤波及内环算法不变；PID 与 CH8/10/12 共用这些滤波反馈，CH10 同步体现修正后的 Pitch 角速度。
`g_flight_control_state.roll_rate_raw_rad_s` 保存参与本次滤波的原始 Roll 输入，内部单位 rad/s。
仅在新有效 GYRO 样本时更新，上电为零；无新样本或 IMU 无效/超时时保持上次值，恢复后随新样本更新。
CH8 与 CH13 从同一次飞控快照取值。CH13 仅供观察，不替代 PID 反馈。
两者均以 100 Hz 抽样发送，不是完整的高频原始数据记录；比较滤波效果时可同时观察 CH7、CH8、CH13。

## UART7 黑匣子

- PE8 / AF7 / UART7_TX：2000000、8N1、仅发送、无流控、不反相；接黑匣子 RX 并共地，接收端必须兼容 3.3 V UART 电平。
- USART1 保持 230400 / 100 Hz。两路实际任务通过同一个 `VOFA_BuildFlightFrame` 组帧，均为 CH0～CH19、84 字节 JustFloat，不新增时间戳或校验字段。
- 每次 FlightControl_Run 返回后采集快照并尝试 DMA 发送，约 1 kHz；不是把 100 Hz 数据重复十次。锁定也记录；IMU 停止触发后不周期发送、不补算或补帧。
- 黑匣子需支持连续 84 kB/s（约 5.04 MB/min）。8N1 线路占用 42%，一帧约 420 µs；发送完成并不代表接收端已经落盘。
- 使用 DMA1_Stream1 / UART7_TX，IRQ 优先级 6；保留 UART5 的 Stream0。仅 UART TC 完成回调释放在途缓冲，忙时丢本次帧、不覆盖、不排队。
- 96 字节缓冲独占三个缓存行，32 字节对齐，`.blackbox_dma` 显式放入 AXI SRAM；仅发前 84 字节，D-Cache 开启时清理完整 96 字节。不能搬入 DTCM。
- 发送忙超过 5 ms 或 UART7 错误，在飞控任务中非阻塞中止；同步/异步中止完成后恢复。中止再超过 5 ms 或返回失败则本次上电禁用记录。没有 IMU 通知时，每 5 ms 的任务超时分支继续维护发送状态。
- 初始化失败只禁用黑匣子；运行错误不重置飞控、不修改 PID/滤波/电机安全规则。无确认、无重传；日志无序号，不能仅靠文件恢复准确丢帧时刻。

Watch：`g_blackbox_tx_stats.initialized/busy/disabled/state`，
`submitted_count/completed_count/busy_drop_count/error_count/timeout_count/abort_count`，
`last_status/last_uart_error`。状态依次为 DISABLED=0、IDLE=1、SENDING=2、RECOVER=3、ABORTING=4。
`last_status` 是最近发送/中止 API 状态；UART 错误码保留在 `last_uart_error`。成功中止可将 last_status 更新为 HAL_OK，须结合计数看历史故障。

主机测试 `test_blackbox.c` 编译真实组帧、黑匣子、RC、飞控和电调源码，模拟 DMA 启动/中止与 UART 完成/错误回调：
核对两路相同输入的完整帧、旧接口兼容、锁定归零、缓冲区持有、cache 调用、忙丢帧、5 ms 边界、同步/异步中止、tick 回绕及 UART5 回调分流。
HAL 外设行为、链接地址和真实时序不由这些桩测试证明。ARMCC 构建：`MDK-ARM/build_ws2812.ps1 -OutputDirectory uart7-blackbox-verified`。
本次构建通过 54 个源文件；链接结果黑匣子缓冲为 0x24000000 / 96 字节，满足 AXI SRAM 及 32 字节对齐。
静态最大调用栈：FlightControlTask 328 字节、默认任务 408 字节，均配置 1024 字节；不含完整运行时上下文/所有间接调用影响，仍需实机高水位检查。
CubeMX 再生成后需复核自定义 UART7 非致命错误返回、黑匣子调用点和 scatter 段，不能只保留自动生成初始化。

实机拆桨验收至少 60 秒：USART1 仍约 100 Hz；UART7 提交/完成帧数接近实际飞控更新数，忙丢帧、错误与超时计数不增长，飞控频率保持正常。
检查文件可按 84 字节解析、数据单位及四路电机顺序正确；检查锁定记录、IMU 停止后不再提交新帧、原电机保护照常生效。
静态调用栈深度须低于任务配置，运行时仍应复核任务栈高水位；编译通过不等于 DMA/USB/存储链路或飞行稳定性已验证。

## 角速度一维卡尔曼滤波参数

三轴反馈各自维护角速度估计 x 和协方差 P，使用随机游走模型，共用 Q/R 参数。
滤波常开，PID 和 VOFA CH8/10/12 使用同一滤波结果，不融合姿态角、不估计零偏。
`g_atom_imu_data.gyro_x/y/z` 仍保留原始 rad/s 数据。

Keil Watch 接口（旧 cutoff_hz / applied_cutoff_hz 已移除）：

| 字段 | 默认值 | 含义 |
|---|---|---|
| `g_flight_control_filter.q` | 0.015 | 每 1 ms 的过程噪声协方差参数 |
| `g_flight_control_filter.r` | 1.0 | 测量噪声协方差参数 |
| `g_flight_control_filter.applied_q` | 0.015 | 实际生效 q，只读观察 |
| `g_flight_control_filter.applied_r` | 1.0 | 实际生效 r，只读观察 |
| `g_flight_control_filter.gain[0..2]` | 初始化前 0 | 最近更新增益，Roll/Pitch/Yaw；首样本直接采纳时为 1 |

q/r 的名义协方差单位均为 (rad/s)^2；q 在每次更新按 dt/0.001 缩放。
保留用户当前生产默认值 q=0.015、r=1，不是实测噪声标定。
主机参考响应测试显式设 q=0.07510204、r=1，在 1 kHz 下稳态 K 约 0.239057，接近原 50 Hz 一阶低通系数；不修改生产默认值。
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
姿态映射回归检查正负方向、三轴独立性、外环回正方向和限幅、锁定输出，以及 CH0~CH2 与状态快照一致。
原始姿态 (0.1, -0.2, 0.3) rad 对应控制姿态 (0.2, 0.1, -0.3) rad；零目标、两个外环 Kp=0.5 时，
Roll/Pitch 目标角速度分别为 -0.1/-0.05 rad/s。继续检查两轴内环电机修正、CH8/10 反馈及原始 IMU 字段不被改写。
新增静态倾斜回正测试：零目标、零角速度、正 P 增益且 I/D=0，左侧下沉（原始 imu.pitch>0）时左侧 M1/M4 增加、右侧 M2/M3 减少；右侧下沉时反向。
此测试按用户报告的方向建立，不代替实机轴向、电机位置和正负号复核。
Pitch 角速度阻尼测试隔离角度外环（角度 Kp=0、角速度 Ki/Kd=0、正角速度 Kp），目标角速度为零：
机头向下转动（gyro_x>0）时前方 M3/M4 增加、后方 M1/M2 减少；向上转动时相反。
同时检查正负首样本、后续滤波更新、CH10 与 PID 反馈一致，以及 Pitch 姿态角符号和 CH1 不变。
CH13～CH15 测试检查上电零值、原始值符号、正负阶跃、重复样本、失效/超时恢复、单位转换和 84 字节帧尾位置；
同时回归通道布局、PID 仍使用滤波反馈，以及独立三通道姿态发送接口。
test_vofa_esc.c 编译真实 esc_pwm 和 vofa，检查四路限幅、PWM 映射、安全锁定后输出归零、串口错误统计和 tick 回绕。
这些测试不执行默认任务的真实调度；默认任务快照来源和 100 Hz 周期另作源码检查与实机验证。

固件完整构建：`MDK-ARM/build_ws2812.ps1 -OutputDirectory vofa-20ch-verified`。
在 Keil Rebuild 并烧录后，先拆桨、保持锁定，确认右滚使 CH0 增大、下俯使 CH1 增大、右偏航使 CH2 增大。
零姿态目标、小的正外环增益下检查回正指令方向，再回归 CH5/CH10、模式切换、回中和失联保护。
拆桨验证 Pitch 纯内环时先令角度增益为零、角速度目标为零，用小的正角速度 P 检查动态阻尼方向；
机头向下转动时 CH10 应为正，前方 M3/M4 输出应高于后方 M1/M2。不要把停止转动后的静态姿态回正与角速度阻尼混淆。
主机测试与编译不验证实机方向或闭环稳定性；不要直接沿用高增益解锁测试。
