# DAPLink（Horco CMSIS-DAP）上板测量操作手册

> 本手册只适用于当前这一个测量构建。重编译后必须重新 `prepare`，地址不可沿用。
> 全程命令在工程根目录执行：`D:\stm32Project\无人机\STM32H723`
> 本次探针实测由用户手动完成；助手不连接设备。

## 0. 工具与探针自检（只做一次）

1. **关闭 Keil**（Keil 会独占探针，Keil 与 pyocd 不能同时连接）。
2. **重新插拔一次 Horco 探针的 USB 线**。审查时发现探针 CMSIS-DAP 接口（bulk/HID）均能打开但无响应、且其 U 盘未挂载盘符，符合探针固件卡死的特征；重新插拔是标准恢复手段。
3. 确认「此电脑」里出现 Horco/DAPLINK 的 U 盘盘符（探针的 MSC 接口）。
4. 验证 python 与 pyocd：
   ```powershell
   python -m pyocd --version
   ```
   应输出 `0.46.0`。若提示找不到 pyocd，执行 `python -m pip install pyocd`。
5. 验证探针与目标芯片：
   ```powershell
   python -m pyocd list --probes
   python -m pyocd list --targets | findstr h723
   ```
   探针应显示 `Horco CMSIS-DAP`（唯一），目标应含 `stm32h723xx`。
6. 若 `list --probes` 正常但后续连接超时：确认飞控已供电（探针自身的 DAP_Info 与目标无关，廉价探针固件可能受目标侧影响），再重插一次；仍不行则在手册末尾看故障排查。

## 1. 接线与安全

- **拆除全部桨叶**。测量全程禁止带桨。
- 飞控独立供电（电池/电源），SWD 四线：SWDIO、SWCLK、GND、3V3（参考电平）。
- 遥测串口接好（探针 COM17 或独立 USB-TTL），SD 卡插好，SBUS 接收机接好、遥控器开机。
- 机架平放在安全位置；解锁后电机怠速空转，手不要碰。
- 加热场景注意：15% 占空比、目标 40°C、45°C 过温锁存，留意散热，不要长时间加热。

## 2. 烧录测量固件（二选一）

**方式 A：U 盘拖拽（最简单）**
把 `audit\dwt_profiler\measurement_ac5\firmware.hex` 直接拖到 Horco/DAPLINK 盘符，等待盘符自动重连（烧录完成后探针会自动复位飞控并开始自由运行）。

**方式 B：pyocd 烧录**
```powershell
python -X utf8 -B measurement\snapshot_export\dap_capture.py flash --hex audit\dwt_profiler\measurement_ac5\firmware.hex
```
烧完会自动复位运行。

烧录后验证：VOFA 串口遥测帧（100Hz）与平时一致；profiler 不向串口添加任何通道，VOFA 协议不应出现新字段。

## 3. 运行中监控（不暂停 CPU）

Keil 必须关闭。新开一个 PowerShell，在工程根目录：
```powershell
python -X utf8 -B measurement\snapshot_export\dap_capture.py watch --manifest audit\dwt_export\bundle\manifest.json
```
每秒打印一行，关注：
- `sequence` 持续增长（约每秒 100，随遥测节奏）；
- `complete=1`、`valid=1`、`faults=0`、`depth=0`（启动最初 valid=0 属正常）；
- 若 faults 非 0 或 depth 卡在非 0：本轮作废，重插、重新烧录自由运行。

watch 只读内存、不 halt、不复位，可长期挂着。Ctrl+C 退出仅断开连接，飞控继续运行。

**禁止**：Keil 断点、单步、复位、CYCCNT 清零、调试器里调用函数。一旦中途 halt，本轮作废。

## 4. 跑场景（建议 3~5 分钟，按顺序）

每个场景至少持续 20~30 秒，让计数器采到最大值：

1. **启动**：上电后先自由跑 30 秒，观察启动配对（startup_pairs）。
2. **稳态采集+黑匣子**：正常录制黑匣子，覆盖 1 kHz 飞控与 2 kHz BMI 采集的常规负载。
3. **RC（重点）**：正常帧 → 快速打杆突发 → 关闭遥控器触发 100ms 超时/failsafe → 重新开机恢复；重复 2~3 次。
4. **拆桨解锁/锁定（重点）**：CH7 中档（油门最低）预解锁 → 高档解锁 → 电机怠速 → 锁定；重复 2~3 次。
5. **IMU 失效安全门**：解锁状态下拔掉 Atom IMU（FDCAN 线）触发 10ms 超时，观察电机立即降到最小；重新插回。
6. **SD 卡**：正常写入/同步；可在录制中制造一次轻微繁忙（不必人为损坏卡）。满卡/错误路径不建议在本轮冒险制造，标 NOT MEASURED 即可。
7. **光流**：接上光流模块，覆盖正常 DMA 接收；若方便切换 IT 诊断接收则再覆盖一条。
8. **加热（重点）**：让加热器经历冷启动 PI 升温 → 接近 40°C → 稳态；若做独立超时守卫路径，注意别触发 45°C 锁存后无法恢复。加热两个站点的最大嵌套深度必须 ≥2（包含 BMI getter）。
9. **遥测/WS2812**：保持现有帧与 LED 活动即可，不要注入第二种串口格式。

未跑到的路径最终在报告里标 NOT MEASURED，**不得**报成 0 μs。

## 5. 最终 halt 与导出（只做一次）

1. 全部场景结束后：遥控器锁定（油门最低、CH7 低档），电机停转。
2. 保持飞控供电、探针连接，让它自由运行至少 10 秒（再跑几个遥测周期）。
3. 不要再动遥控器，另开 PowerShell 执行：
   ```powershell
   python -X utf8 -B measurement\snapshot_export\dap_capture.py capture --manifest audit\dwt_export\bundle\manifest.json --capture audit\dwt_capture_run1
   ```
   脚本会：先读一次 live 头部 → 最终 halt → 导出快照前后头、1716 字节快照、fault/depth、heap、10 个任务句柄、解引用 TCB 并 dump 全部栈。
   结束后飞控**保持 halt**，不会 resume/reset。
4. 看到 `Header consistent across before/full/after dumps.` 和 `Capture complete` 即为成功。
5. 然后断电（或直接进入第 7 步回刷）。

## 6. 离线解码

```powershell
python -X utf8 -B measurement\snapshot_export\snapshot_tool.py decode --manifest audit\dwt_export\bundle\manifest.json --capture audit\dwt_capture_run1
```
产出：
- `report.json`：头部字段、全局外层最大 body/envelope、faults、heap、任务栈；
- `sites.csv`：41 个站点逐条 hits、body/envelope 微秒上限、预算余量、最大嵌套深度；
- `tasks.csv`：10 个任务栈高水位（RcTask 测量构建是 1024B，不能反推生产 512B）。

把这三个文件（以及 `capture_info.json`）发给助手判读。验收条件：faults=0、valid=1、complete=1、current_depth=0、sequence 有推进、前后头一致。

## 7. 回刷生产固件（务必执行）

**方式 A**：把 `audit\phase2_c_close\ac5_O3\firmware.hex` 拖到 Horco U 盘。
**方式 B**：
```powershell
python -X utf8 -B measurement\snapshot_export\dap_capture.py flash --hex audit\phase2_c_close\ac5_O3\firmware.hex
```
回刷后确认 VOFA 恢复正常。生产 HEX SHA256：
`dd8733815e5a3cbe95905669876949d94f94cdf380723626d993b5355dd7d012`

## 8. 故障排查

- **flash 时报 `AssertionError`（`safe_reset_and_halt` / `is_reset_asserted`）**：本探针只接了 SWD 四线、没有 nRST，而 `halt` 连接模式会先拉低物理 nRST 并回读确认，必然失败。脚本已改为 `attach` 连接（flash loader 自行 halt 内核，烧后用 SYSRESETREQ 软复位），直接重跑 flash 命令即可；若以后补接了 nRST 线，可把 `cmd_flash` 改回 `halt`。
- **flash 时报 `TypeError: unsupported operand type(s) for &: 'str' and 'int'`（`set_swj_clock`）**：pyocd 的 `frequency` 选项必须是整数，脚本里固定为 `2000000`（int），不要改成字符串。
- **Timeout reading from probe / assert self.closed**：Keil 未关干净（任务管理器确认无 UV4.exe）→ 重插探针 USB → 重开 PowerShell 再试。
- **U 盘盘符不出现**：探针固件异常，重插 USB；仍不行换一根 USB 线/一个 USB 口（避免 USB Hub）。
- **watch 里 sequence 不动**：飞控未运行或固件未烧成功，检查供电与烧录步骤。
- **faults 非 0 / depth 不为 0**：本轮作废，记录故障位名称后重新烧录、重跑，不要修改任何字节。
- **capture 后前后头不一致**：不要接受数据，按手册重新自由运行一轮。
- 若 pyocd 与该探针始终不兼容，备选 OpenOCD（xpack-openocd，`interface/cmsis-dap.cfg` + `target/stm32h7x.cfg`），脚本 `audit/dwt_export/bundle/openocd_dump.tcl` 已备好，到时让助手给你具体命令。

## 边界声明

- 实测最大值是所跑场景下的**观测最大值**，不是形式化 WCET 证明；FreeRTOS 内核/HAL 内部的关中断窗口未覆盖。
- body 是真实关中断屏蔽体，envelope 是含潜在抢占的保守包络，二者分别对照预算，不得把 envelope 当精确关中断时间。
- RcTask 生产栈 512B / 静态链 456B 是已登记的阶段 5 隐患；测量构建栈为 1024B，水位不能用来证明生产栈安全。
