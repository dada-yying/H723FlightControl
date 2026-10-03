# 板载 BMI088 40°C 加热

依据原理图 `references/达妙h7开发板/原理图/CtrBoard-H7_V1.0-240124.pdf` 第2页IMU加热电路
和 `CtrBoard-H7_IMU_TempCtrl` 例程：PB1/TIM3_CH4高电平驱动Q9/CJ2310，R77～R84加热电阻
由 **VCC_IN** 供电。只有USB时可以读取BMI088，但不能给该加热电路供电。

目标固定 **40.0°C**。温控在既有 `Bmi088Task` 中运行，不新增任务；仅温度用于温控。
BMI角速度/加速度仍不参与姿态滤波或控制，电机保护和飞控算法不变。
加热不以40°C就绪作为解锁条件。黑匣子已记录实际温度，本次不改变数据包或F407/转换器。

## PWM 与PI

TIM3时钟240 MHz，PSC=23，ARR=9999，PWM频率1 kHz。CCR4限幅0～500，即 **0～5%占空比**，
沿用板厂例程上限。PB1启动先保持低；PWM初始为0，只有收到有效BMI完整样本后才允许加热。

每20 ms最多更新一次（50 Hz），使用实际经过时间，不对未更新的SPI快照积分、不补算遗漏周期：
`P=100*(40-temperature_c)`，`I += 50*error*dt_s`，积分限幅0～500、饱和时停止向饱和方向积分，
最终CCR限幅0～500。没有D项。例程的 `KI*(err+err_l+err_ll)` 是最近三次误差之和，不是长期积分；
本次使用标准带时间尺度的PI，KI的量纲不同，不能认为动态响应与例程完全相同。
软件只加热，环境高于40°C时不能主动降温到40°C。

温度寄存器在传感器内部约每 **1.28 s** 更新一次（
[Bosch数据手册5.3.7](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)），
期间重复读到同一温度是正常现象。50 Hz是控制计算频率，不是温度转换频率；PI使用保持的测量值。
20 ms检查的是完整SPI读取是否继续成功，不用于判断内部温度转换是否更新。

默认任务每10 ms独立检查温度样本，即使BMI任务停止执行，也会在发现年龄≥20 ms时停热。
温控和检查只使用短临界区和CCR写入，不执行SPI/UART或等待。预装载更新可能额外延迟一个
PWM周期（1 ms），同时存在任务调度延迟。

## 停热条件

- BMI初始化未完成、无首个完整样本、SPI/ID/配置错误、任务创建失败或样本年龄≥20 ms。
- 温度NaN/无穷或超出-40～85°C工作检查范围。
- 有效温度 **≥45°C**：锁定停热、清零积分；本次运行内降温或切换force_stop不会解除。
  重新上电/复位会复位锁存；温度仍≥45°C时会再次停热。
- `g_bmi088_heater_force_stop=1`：手动停热、清零积分；恢复0后只在其他条件正常时继续。
- TIM3初始化/PWM启动失败：只禁用加热，不进入全局Error_Handler，不阻止BMI读取或飞控启动。
- Error_Handler、HardFault/BusFault/UsageFault/NMI/MemManage：直接关闭TIM3_CH4并清CCR，
  不调用RTOS或带超时的HAL。故障中的温控Watch状态是故障前快照。

## Watch

实际温度：`g_bmi088_data.temperature_c`。
`g_bmi088_heater_state` 是观察值，目标和增益为源码常量：

| 字段 | 含义 |
|---|---|
| target_c / temperature_c / error_c | 固定目标40°C、反馈温度、温差 |
| pwm_ccr / duty | CCR指令0～500、占空比0～0.05（百分比乘100） |
| integral_ccr | 积分贡献，CCR计数 |
| initialized / heating | PWM是否正常启动、是否请求非零加热 |
| sample_age_ms / update_count | 样本年龄、控制更新次数 |
| stop_reason | 0正常、1等待样本、2传感器无效、3超时、4温度异常、5超温、6手动、7PWM错误 |
| overtemperature_latched / overtemperature_count | 超温锁存和次数 |
| last_status | 初始化/启动HAL返回值 |

唯一运行时请求是 `g_bmi088_heater_force_stop`，上电为0；不保存到Flash。

## 验证

主机测试使用真实BMI SPI模型和温控模块，覆盖：启动关闭、20 ms边界、独立任务超时停热、
新样本/周期门控、实际dt、限幅/积分饱和、45°C锁存、手动停止、SPI/PWM失败和恢复、
非法温度、tick回绕及TIM1/TIM2电调CCR隔离。一阶热模型用于验证逻辑，不代表实际板卡响应。
运行 `tests/pid_tuning/run.ps1` 和ARMCC构建，并检查默认/BMI任务栈及原DMA位置。

实机先拆桨并正常VCC_IN供电：核对冷启动PWM为0、随后升温并在40°C附近保持；
记录温度、CCR、占空比及停止原因，检查60秒稳定段。通过force_stop和传感器故障验证停热。
可在台架调试注入已解码温度测试超温锁存，无需实际升温到45°C。
编译和主机模型不代替实机温控验证。
调试暂停前先设置force_stop=1并确认CCR为0；CPU暂停时软件温控和超时检查不能执行，
不要冻结一个正在加热的TIM3输出。
