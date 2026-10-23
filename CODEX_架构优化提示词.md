# STM32H723 飞控项目：Codex 代码缺陷调研、实查问题与重构提示词

## 给 Codex 的重构提示词

```text
你是资深飞控固件架构师，为一个基于 STM32H723 + FreeRTOS(CMSIS-RTOS2) 的四旋翼飞控项目做架构重构。
工具链：AC5(armcc)，工程文件在 MDK-ARM/，代码在 App/、Core/、Devices/、tests/。
现有功能：CAN 输入的 Atom IMU（主控源）、板载 BMI088（经 SPI2，2 kHz 采样，当前仅记录）、
SBUS 遥控(UART5+DMA)、PWM ESC(TIM1/TIM2)、光流(UART10)、黑匣子(TIM4 触发→SD/FatFs)、
VOFA 遥测、WS2812、BMI088 加热(TIM3)。

【最高优先级：安全红线，任何阶段不得违反】
1. 禁止改变任何已确认的物理语义：控制律系数、PID 输出符号、轴映射（roll=-imu.pitch、
   pitch=imu.roll、yaw=-imu.yaw；gyro 映射 raw=[-gyro_y, gyro_x, -gyro_z]）、
   M1~M4 电机混控矩阵、SBUS 通道与端点、arming 档位序列与阈值。需要动这些时，只允许把它们
   原样搬到参数/常量定义处，数值一个 bit 都不许变，并在变更清单中逐条列出。
2. 禁止修改 CubeMX 生成区（USER CODE BEGIN/END 之外）、Drivers/、Middlewares/、ThirdParty/。
3. 禁止引入动态内存(malloc/new)、C++、浮点 printf 到实时任务；禁止在 ISR 与 1 kHz 以上
   周期任务中加入任何阻塞调用（FatFs、轮询外设、长循环）。
4. 禁止删除或弱化 tests/ 下的现有测试；所有纯逻辑重构必须同步更新/补充主机侧测试并保证通过。
5. 每完成一个阶段：必须用 Keil 命令行（或我提供的构建脚本）实际编译，报告 warning/error 数；
   必须运行 tests/pid_tuning 下可在主机编译的测试（用 gcc 编译，stub 已存在）。
   编译没过或测试没过，禁止进入下一阶段。
6. 所有文件使用 UTF-8 编码、CRLF 换行、英文注释；不得生成 GBK/乱码注释。
7. 每个阶段只做该阶段列出的事。你发现的其他问题，写进"发现但未改动"清单，禁止顺手修改。
8. 每个阶段结束后输出：①变更文件清单与每个文件的改动摘要 ②风险点 ③编译与测试结果
   ④"发现但未改动"清单。等我确认后再进入下一阶段。

【总体目标架构】
- 分层（依赖只能从上到下）：
  App/(模式与任务编排) → Service/(消息总线、参数系统、黑匣子) →
  Drivers/(器件驱动：bmi088、rc_sbus、esc、optical_flow、heater) →
  BSP/(引脚/句柄/段/DWT 时基) → Core/HAL(CubeMX)。
- 消灭跨文件可写全局变量：模块状态收进模块内 static context 结构体；对外只暴露
  Init/Get 快照/命令接口。跨任务数据统一用两种原语之一：
  (a) 单写者 + taskENTER_CRITICAL 内整体快照拷贝（读接口）；
  (b) ISR→任务用任务通知。禁止再出现裸 g_xxx 多写者变量。
- 所有可调参数集中到 App/parameters/flight_params.h：命名常量 + 类型 + 默认值 + 范围，
  后续阶段接入 Flash 持久化。代码中零魔数（0/1 计数、寄存器值除外）。

【阶段 0 — 只审计，不改任何代码】
通读 App/ 与 Core/Src/freertos.c、main.c，输出：
- 每个任务的优先级、周期/触发源、栈大小(字节)、共享数据访问矩阵（哪个任务读/写哪个全局）；
- 标注所有竞态点、阻塞点、CubeMX 保护区风险点；
- 给出你建议的目标目录结构与文件迁移映射表。
此阶段不得编辑任何文件。

【阶段 1 — 文件卫生与重复设施收敛】
- 修复 flight_control.c、main.c 中所有乱码注释（按上下文还原为英文），全项目注释统一英文；
- 统一代码风格（参照 bmi088.c：展开的大括号、运算符两侧空格、语句不压缩）；
- 把 DWT 时基收敛为 BSP 层唯一模块（如 bsp_dwt.c/h，提供 BspDwt_Init/Us/周期测量），
  各处改为调用它；注意 DWT->CYCCNT 只允许在调度器启动前复位一次，其余地方禁止复位；
- 合并 Devices/WS2812/ws2812 与 App/ws2812_ctrl 为一个驱动模块，删除空函数 App_Main_Start；
- 清理恒真式、死代码、误导性命名的计数器（如把 submitted/completed 计数器按真实语义重命名）。
验收：编译 0 warning；乱码为 0；grep 确认 DWT 使能只出现在 BSP 一处。

【阶段 2 — 并发安全（P0）】
- 修复 ESC 竞态：EscPwm 的 armed/inhibit 状态转移只允许在一个任务（建议 RcTask）执行；
  FlightControlTask 只通过"取一份锁内状态快照 + 锁内写电机命令"的接口输出，
  所有对 g_esc_pwm_state 的读写都在同一把临界区/同一抽象内；
- 消灭 volatile 结构体强转：PID/参数配置改为"控制环持一份只读工作副本，调参更新时
  在临界区整体替换"的模式，禁止把 volatile 成员地址强转为非 volatile 指针；
- 制定并落实全项目共享数据规范：ISR 共享计数用 volatile 单字，结构快照用临界区，
  单写者 seqlock 仅保留 atom_imu 风格；g_optical_flow_stats 等裸全局统计量按规范封装；
- 为每个临界区注释说明保护对象与最长关中断时间估计。
验收：编译 0 warning；输出共享数据访问矩阵证明无多写者；主机测试中用并发 stub 验证
arming 与电机输出不出现撕裂（写测试，不允许只口头保证）。

【阶段 3 — 拆分 flight_control.c（P1）】
拆为：filters/(rate_notch.c、scalar_kalman.c)、control/(pid_rate.c、pid_angle.c)、
control/mixer.c、control/arming_guard.c（安全仲裁）、control/tuning.c（调参状态机）、
flight_control.c（仅编排）。与 bmi088_gyro_filter 统一滤波器接口与坐标系约定，
明确两个 IMU 数据源角色（Atom=控制源、BMI088=记录/备用），在头文件与架构文档中声明。
PID/滤波/混控全部为无 static 全局的纯函数（状态走 context 指针），可在主机侧单测。
验收：原 tests/pid_tuning 全部通过并迁移到新接口；黑匣子/VOFA 字段一字段不少；
控制环实际执行时间（DWT 统计）不得劣化超过 5%。

【阶段 4 — 启动序列与任务模型】
- 建立显式启动表（集中的 App_Init 序列，函数指针+名称+失败处理策略），
  消灭 main.c / App_Main_Init / MX_FREERTOS_Init 三处分散初始化；
  外设初始化顺序在表中显式声明，删除 OpticalFlow_Init(HAL_ERROR) 这类重载 hack；
- 任务栈大小统一为"字节，且用 STACK_WORDS*sizeof(StackType_t) 宏表达"，
  集中在一处定义并附栈高水位实测注释；
- 重命名 defaultTask 为 TelemetryTask；引入轻量消息总线（可先做 FreeRTOS queue 封装的
  topic 发布/订阅，静态分配队列），先把 RC、IMU 两路数据改为发布订阅，
  黑匣子与遥测改为订阅者，禁止任务间互相调用 Getter 拉数据。
验收：启动失败路径逐一可测（注入 HAL_ERROR 验证每个 Optional 设备不阻塞飞行任务）；
队列内存全静态，xPortGetMinimumEverFreeHeapSize 有启动后记录。

【阶段 5 — 参数系统与实时性收尾】
- 建立参数表（名称/默认值/范围/单位/版本），PID、notch、超时、SBUS 端点、最大倾角全部入表，
  提供参数读写接口与参数校验（越界拒绝并计数）；Flash 持久化作为独立服务接入；
- 为 BMI088 读取路径加最坏执行时间硬保护（超时/错误时按既有 fail/retry 状态机处理，
  不得在实时任务里引入 ms 级阻塞）；评估 SPI DMA 改造方案并先出设计，不直接改；
- 全项目补 _Static_assert（配置表长度、DMA 缓冲与包长关系、结构体大小）；
- 消除所有隐式类型转换（按 -Wconversion 或 cppcheck 结果逐项修，修不掉的写明理由）。

【工作方式硬性要求】
- 每个阶段先给出改动计划，等我确认再动文件；
- 一次提交只做一件事，提交信息说明 why，不允许把"顺手重构"混进来；
- 不允许编造 HAL/FreeRTOS API；对芯片约束（DTCM、DMA、cache、SPI CS 时序）不确定时，
  先查阅并引用依据，不允许凭猜测写；
- 任何你拿不准是否改变飞行行为的改动，列入风险清单并默认不做，等我裁决。
```

---

## 参考来源

- OpenAI Developer Community：Repeated scope failures in real Codex projects（范围漂移、不必要发明、过早抽象）；Codex is rapidly degrading（高回归率、清理自身 bug 的额度消耗）；Am I using Codex wrong（无限纠正循环、跨文件丢失约束）；Codex ignoring instructions and making unsafe patches。
- Promwad, *Why GenAI Code Generation Breaks Down in Real Firmware*（2026）：ISR 共享状态无保护、DMA cache 一致性、缓冲区生命周期、优先级反转、无需求追溯。
- Ac6 Training, *Letting an LLM write embedded code*（2026）：内存屏障/volatile 缺失、临界区宽度、禁动态分配处的分配、栈尺寸拍脑袋。
- Embedded Artistry, *The Problems with Global Variables*：全局变量的隐式耦合与隐藏依赖。
- 电子工程专辑《AI 编写代码，谁来保证质量？》（2026）：MISRA C:2012 隐式转换合规问题。
- RunTime Recruitment（2026）：无架构治理的 AI 复制全局状态/指针算术/不校验边界等遗留模式。
- 《通用嵌入式软件架构分层设计思想》（电子工程专辑）：Arch-Platform-Target 与分层依赖。
- PCBCool, *How to Build Reliable STM32 Firmware with FreeRTOS*（2026）：任务优先级/阻塞源/职责划分。
- PX4 官方文档：uORB Messaging、Parameters & Configurations——发布订阅总线与参数系统的参考实现。
