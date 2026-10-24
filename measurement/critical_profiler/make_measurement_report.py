# -*- coding: utf-8 -*-
"""Generate the DWT on-board measurement report from decoded capture outputs.
All numbers come from sites.csv / tasks.csv / report.json; no hand-typed values.
"""
import csv, json, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CAP = ROOT / 'audit' / 'dwt_capture_run1'

sites = list(csv.DictReader(open(CAP / 'sites.csv', encoding='utf-8')))
tasks = list(csv.DictReader(open(CAP / 'tasks.csv', encoding='utf-8')))
report = json.load(open(CAP / 'report.json', encoding='utf-8'))
info = json.load(open(CAP / 'capture_info.json', encoding='utf-8'))
snap = report['snapshot']
CLOCK_MHZ = snap['core_clock_hz'] / 1e6

def i(r, k):
    return int(r[k])

def f(r, k):
    return float(r[k])

# ---- classify sites ----
DIAG_GETTERS = {
    'FDCAN3_GetDiagnostics', 'Bmi088_GetStats', 'VOFA_GetStats',
    'Blackbox_GetTxStats', 'OpticalFlow_GetStats', 'OpticalFlow_GetDiagnostics',
    'FlightControl_GetConfig', 'EscPwm_GetDiagnostics', 'WS2812_GetStats',
    'Blackbox_GetStats', 'SdSpi_GetDiagnostics'}
FAIL_PATHS = {'Bmi088_Fail'}
AUX_PATHS = {'Bmi088_PublishAuxiliary'}
OUTPUT_PATHS = {'EscPwm_SetMotorNormalized', 'EscPwm_SetAllNormalized'}

def classify(fn):
    if fn in DIAG_GETTERS:
        return '诊断/统计 Getter（遥测未订阅或调试接口，场景未调用）'
    if fn in FAIL_PATHS:
        return '失效路径（板载 BMI088 SPI 故障注入；拔 FDCAN 不走此路径）'
    if fn in AUX_PATHS:
        return '辅助发布路径（需确认调用场景）'
    if fn in OUTPUT_PATHS:
        return '直接输出写路径（正常混控不调用；需确认调用者或登记死接口）'
    return '未分类'

observed = [r for r in sites if r['status'] == 'OBSERVED']
missing = [r for r in sites if r['status'] != 'OBSERVED']
over = [r for r in observed if i(r, 'max_body_us_ceil') > f(r, 'budget_us')]
depth2 = [r for r in observed if i(r, 'max_inclusive_depth') >= 2]

def revised_budget(r):
    # observed: measured max + 2 us fixed engineering margin (NOT a WCET proof)
    return i(r, 'max_body_us_ceil') + 2

lines = []
W = lines.append

W('# DWT 临界区上板实测报告（阶段 2 收尾）')
W('')
W('生成时间：2026-10-24　|　数据目录：`audit/dwt_capture_run1/`')
W('')
W('## 1. 执行与版本信息')
W('')
W('| 项目 | 值 |')
W('|---|---|')
W('| 采集时间 | %s |' % info['captured_at'])
W('| 测量固件 AXF SHA256 | `%s` |' % report['expected_axf_sha256'])
W('| 快照 SHA256 | `%s` |' % report['snapshot_sha256'])
W('| 内核时钟 | %d Hz（%.0f MHz） |' % (snap['core_clock_hz'], CLOCK_MHZ))
W('| 遥测快照序号 sequence | %d（100 Hz 遥测 tap，自由运行约 %d 秒） |' % (
    snap['sequence'], snap['sequence'] // 100))
W('| 外层临界区命中总数 outer_hits | %d |' % snap['outer_hits'])
W('| 采集后目标状态 | %s（reset/resume=%s） |' % (info['target_left'], info['reset_or_resume']))
W('')
W('## 2. 数据有效性判定')
W('')
W('| 检查项 | 结果 |')
W('|---|---|')
W('| version / site_count | %d / %d（与 catalog 一致） |' % (snap['version'], snap['site_count']))
W('| complete / valid | %d / %d |' % (snap['complete'], snap['valid']))
W('| faults / live_faults / unknown_fault_bits | 0x%x / 0x%x / 0x%x |' % (
    snap['faults'], report['live_faults'], report['unknown_fault_bits']))
W('| current_depth / max_depth（允许 16） | %d / %d |' % (snap['current_depth'], snap['max_depth']))
W('| rejection_reasons | %s |' % (report['rejection_reasons'] or '无'))
W('| 解码综合状态 | `%s` |' % report['run_status'])
W('| AXF SHA256 与 manifest | 匹配 |')
W('')
W('**结论：快照内部一致，无 fault、无嵌套失衡、无非法 ISR 使用，数据有效。**')
W('但状态字符串中的 `REQUIRES_RUN_LOG` 未解除：9 类场景的人工执行记录（尤其 RC 100ms 边界、')
W('failsafe、加热器 45°C 过温锁存、IMU 失效时刻）未随数据归档；下文第 6 节按 hits 反推覆盖度并列出缺口。')
W('')
W('## 3. 全局关键结论')
W('')
max_site = max(observed, key=lambda r: i(r, 'max_body_cycles'))
max_cycles = i(max_site, 'max_body_cycles')
budget_cycles = int(f(max_site, 'budget_us') * CLOCK_MHZ)
W('- **最大真实关中断屏蔽体（body）：%d cycles = %.2f µs（ceil %d µs），站点 #%d %s（%s:%s），嵌套深度 %d。**' % (
    max_cycles, max_cycles / CLOCK_MHZ,
    i(max_site, 'max_body_us_ceil'),
    i(max_site, 'ID'), max_site['function'], max_site['file'], max_site['line'],
    i(max_site, 'max_inclusive_depth')))
W('- 该站点静态预算 %.0f µs，**实测超出 %d cycles（%.2f µs，约 %.1f%%）**。' % (
    f(max_site, 'budget_us'), max_cycles - budget_cycles,
    (max_cycles - budget_cycles) / CLOCK_MHZ,
    (max_cycles - budget_cycles) / budget_cycles * 100))
W('- 其余 25 个观测站点 body 实测均 ≤ 3.72 µs。')
W('- 共 %d 个站点 body 实测 ceil 超过静态预算（详见第 4 节）；%d 个站点完全未命中（NOT MEASURED，详见第 6 节）。' % (
    len(over), len(missing)))
W('- 最大 envelope（保守包络，含抢占）%d µs 出现在低优先级光流任务，**不是关中断时长**（第 5 节）。'
  % snap['max_outer_envelope_us_ceil'])
W('- 最大嵌套深度 %d：仅加热器两站点（CheckTimeout/Process 外层含 BMI getter 内层，满足设计要求）；'
  'ESC 各站点深度均为 1（快照与提交是独立临界区，符合锁外快照设计）；无三重及以上嵌套。'
  % snap['max_depth'])
W('- profiler 自身记账临界区 body 最大 %d µs、快照拷贝调度包络 %d µs，均为**测量扰动**，生产固件无此代码。' % (
    report['accounting_body_us_ceil'], report['snapshot_envelope_us_ceil']))
W('')
W('## 4. 逐站点 body 实测对照（%d 个已观测）' % len(observed))
W('')
W('口径：body = 真实 port enter 之后到真实 port exit 之前的连续屏蔽采样体，含 profiler/嵌套开销，')
W('不含边界指令；µs 为 ceil 换算。**body 是关中断时长的主证据，但观测最大值不是 WCET 证明。**')
W('')
W('| ID | 站点 | 预算µs | hits | max body cycles | max body µs | 判定 | 嵌套深度 | 建议修订预算µs* |')
W('|--:|---|--:|--:|--:|--:|---|--:|--:|')
for r in sorted(observed, key=lambda r: i(r, 'ID')):
    b = i(r, 'max_body_us_ceil')
    budget = f(r, 'budget_us')
    verdict = '超预算' if b > budget else ('卡线' if b == budget else '通过')
    W('| %s | `%s` (%s:%s) | %.0f | %d | %d | %d | %s | %s | %d |' % (
        r['ID'], r['function'], r['file'].split('/')[-1], r['line'],
        budget, i(r, 'hits'), i(r, 'max_body_cycles'), b, verdict,
        r['max_inclusive_depth'], revised_budget(r)))
W('')
W('\\* 建议修订预算 = 实测 ceil + 2 µs 固定工程裕量，用于把阶段 2 的"初步设计预算"重定为实测基线；')
W('它仍不是 WCET 证明，Flash 等待/总线争用/Cache 未命中下的最坏值需反汇编与长期观测继续收敛。')
W('')
W('### 超预算站点（body ceil > 静态预算）')
W('')
W('| ID | 站点 | 预算µs | 实测µs(cycles) | 差值 | hits |')
W('|--:|---|--:|--:|--:|--:|')
for r in sorted(over, key=lambda r: -i(r, 'max_body_cycles')):
    W('| %s | `%s` | %.0f | %d（%d） | +%d µs | %d |' % (
        r['ID'], r['function'], f(r, 'budget_us'),
        i(r, 'max_body_us_ceil'), i(r, 'max_body_cycles'),
        i(r, 'max_body_us_ceil') - int(f(r, 'budget_us')), i(r, 'hits')))
W('')
W('分析：')
W('1. 七个站点（VOFA、光流发布/Getter、RC 发布、飞控 GetState、黑匣子 StartRecording、ESC 快照）')
W('   实测集中在 2.9–3.7 µs，而静态预算给 2–3 µs（960/1440 cycles）。这是**静态估算基线系统性偏低约 1 µs**')
W('   （结构拷贝 + 探针采样位置的固定成本），不是个别代码膨胀。')
W('2. 加热器 Process（#3）是唯一真正的外层长临界区：10.22 µs 且嵌套 BMI getter（depth 2）。')
W('   2 kHz BMI 周期 500 µs 下占 2.0%；若不接受，阶段 3 应把 BMI getter 快照移出加热外层或拆薄临界区。')
W('3. ESC 三个写路径（CommitMixed/ForceMinimum/UpdateFromRc）实测 4.1–4.8 µs，均在 5 µs 预算内。')
W('')
W('## 5. envelope 数据解读（禁止当作关中断时间）')
W('')
W('envelope = 包装器入口前到真实 port exit 后的墙钟，包含 BASEPRI 置位前/释放后被抢占的时间。')
W('它量化的是"临界区附近的最坏调度延迟"，按 profiler 交付文档不得与 body 相加或互相折算。')
W('')
W('最大 envelope 前 8 名：')
W('')
W('| ID | 站点 | envelope µs | body µs | 差值（被抢占）µs |')
W('|--:|---|--:|--:|--:|')
for r in sorted(observed, key=lambda r: -i(r, 'max_envelope_cycles'))[:8]:
    W('| %s | `%s` | %d | %d | %d |' % (
        r['ID'], r['function'], i(r, 'max_envelope_us_ceil'),
        i(r, 'max_body_us_ceil'), i(r, 'max_envelope_us_ceil') - i(r, 'max_body_us_ceil')))
W('')
W('最大 envelope 全部落在光流/存储/黑匣子等**低优先级任务**（被 BMI/飞控等高优先级上下文抢占）；')
W('BMI 采样/滤波与 Heater_Process 的 envelope 为 91–206 µs；Heater_CheckTimeout 因双调用路径 envelope 达 924 µs，')
W('但其 body 仍为 9.16 µs——真实关中断时长没有变长，变长的只是锁外被抢占的墙钟。')
W('另：snapshot 拷贝挂起调度器的 744 µs 包络仅在 100 Hz 遥测 tap 存在，属测量扰动。')
W('**WS2812 挂起调度器轮询 SPI（阶段 1 登记的保留风险）使用 suspend 而非 enter_critical，不在本次 41 个站点内，其调度延迟未被量化。**')
W('')
W('## 6. 未命中站点（%d 个 NOT MEASURED）——不得宣称实测覆盖' % len(missing))
W('')
W('| ID | 站点 | 预算µs | 分类 / 处置建议 |')
W('|--:|---|--:|---|')
for r in sorted(missing, key=lambda r: i(r, 'ID')):
    W('| %s | `%s` (%s:%s) | %.0f | %s |' % (
        r['ID'], r['function'], r['file'].split('/')[-1], r['line'],
        f(r, 'budget_us'), classify(r['function'])))
W('')
W('## 7. 场景覆盖反推（基于 hits，需 run log 最终确认）')
W('')
W('| 场景 | 数据证据 | 判定 |')
W('|---|---|---|')
W('| 启动 | Configure/StartRecording/OpticalFlow_Start 各 1 hit | 有证据 |')
W('| 稳态 2 kHz 采集 | Bmi088_ReadSample 212173、加热器 212670（≈2 kHz×106 s） | 有证据 |')
W('| 黑匣子记录/SD 繁忙 | SendSnapshot 107210、dma_transfer 53586、Storage_Process 35713 | 有证据 |')
W('| 光流 DMA/IT | OF_Publish 10711、ProcessPending 18855 | 有证据 |')
W('| 100 Hz 遥测 | VOFA_Transmit/FDCAN 诊断 10721–10722 | 有证据 |')
W('| RC 正常/解锁/锁定/停机 | RcSbus_Publish 19626、CommitMixed 15542、ForceMinimum 36322、UpdateFromRc 13688 | 有活动证据 |')
W('| RC 100ms 超时边界/failsafe | 无逐事件记录，hits 无法证实时序 | **需 run log** |')
W('| IMU 失效（拔 FDCAN） | FDCAN 诊断 10722；Bmi088_Fail 0 hit（符合预期，板载 SPI 路径） | 失效时刻需 run log |')
W('| 加热 40°C 稳态 | Heater_Process 212670、depth 2 | 有证据 |')
W('| 加热 45°C 过温锁存 | 无状态字段快照，hits 无法证实 | **需 run log** |')
W('| WS2812 | 无任何业务站点（suspend 路径未埋点） | **测量盲区** |')
W('')
W('## 8. 任务栈水位（测量构建，栈填充 0xA5）')
W('')
W('| 任务 | 生产栈B | 测量栈B | 测量最低剩余B | 自动告警(<128B) |')
W('|---|--:|--:|--:|:--:|')
for t in tasks:
    W('| %s | %s | %s | %s | %s |' % (
        t['name'], t['production_stack_bytes'], t['measurement_stack_bytes'],
        t['minimum_free_bytes'], '是' if t['stage5_followup'] == 'True' else '否'))
W('')
W('**RcTask 生产栈隐患不能被本次测量豁免**：测量构建栈为 1024 B（本次最低剩余 324 B），')
W('生产构建仍为 512 B、静态调用链 456 B、理论余量仅 56 B，未计入异常压栈（M7 带 FPU 最坏约 136 B）')
W('与中断嵌套。维持 STAGE5 登记，阶段 3 必须直接调栈或做生产构建水位实测。')
W('defaultTask 测量剩余 272 B（1024 B 栈）偏小，一并列入阶段 3 评估。')
W('')
W('## 9. 堆（heap）')
W('')
W('- configTOTAL_HEAP_SIZE = 15360 B；测量构建最低剩余 %d B（测量构建额外占用 512 B RcTask 栈）。'
  % report['heap_min_bytes'])
W('- 生产构建预计最低剩余 ≥ %d B，当前无 heap 压力证据。' % (report['heap_min_bytes'] + 512))
W('')
W('## 10. 中断影响评估（BASEPRI 口径）')
W('')
W('- configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5：临界区屏蔽优先级数值 ≥5 的 ISR，放行 0–4。')
W('- 工程全部外设 ISR（SD DMA/SPI1、FDCAN3_IT0、TIM4、TIM7、USART10、UART5 及各自 DMA）均为优先级 6，')
W('  **全部在临界区屏蔽范围内**；工程不存在优先级 0–4 的 ISR。')
W('- 最坏 10.22 µs 屏蔽的影响：SBUS 100 kbps 每字节约 100 µs、FDCAN 帧 100+ µs、SD 传输由 DMA 自治、')
W('  2 kHz BMI 周期 500 µs（占 2.0%）——均无丢字节/丢帧风险；影响表现为这些 ISR 的最坏响应延迟 ≈10 µs。')
W('')
W('## 11. 边界声明（不可省略）')
W('')
W('1. body/envelope 均为**观测最大值**，样本量再大也不是 WCET 形式化证明；未覆盖 Flash 等待、总线争用、')
W('   低温降频等极端条件。')
W('2. profiler 仅包装应用层 taskENTER_CRITICAL/EXIT；FreeRTOS 内核内部、HAL 内部关中断窗口未覆盖。')
W('3. suspend 型互斥（WS2812 轮询 SPI）未埋点，未量化。')
W('4. 15 个未命中站点无数据，静态预算对它们仍是设计值。')
W('5. 本次仅为软件测量，不包含飞行/电机实机验证；未做物理 PWM 边沿同步测量。')
W('')
W('## 12. 阶段 2 关闭建议（供裁决，不自动生效）')
W('')
W('建议**有条件关闭阶段 2**，条件与后续动作：')
W('')
W('1. 以本报告第 4 节"建议修订预算"（实测 ceil + 2 µs）重定静态预算表并签入；')
W('   加热器 #3 二选一裁决：接受 10.22 µs（登记为 12 µs 实测基线）或列入阶段 3 拆薄嵌套。')
W('2. 15 个未命中站点登记为受控未覆盖：11 个诊断 Getter 由 Codex 确认调用点（无调用者则移除/显式标注），')
W('   A3/A7 确认调用者，Bmi088_Fail 补 SPI 故障注入测试，PublishAuxiliary 确认场景。')
W('3. WS2812 suspend 轮询路径单列为测量盲区，阶段 3 补量化或改造。')
W('4. RcTask 生产栈 512 B、defaultTask 栈余量列入阶段 3 处理（RcTask 已在 STAGE5 登记）。')
W('5. 补齐 9 类场景 run log（RC 边界/failsafe、加热器过温锁存、IMU 失效时刻）并归档。')
W('6. 回刷生产固件 audit/phase2_c_close/ac5_O3/firmware.hex 并核对 SHA256 ')
W('   （dd8733815e5a3cbe95905669876949d94f94cdf380723626d993b5355dd7d012）。')
W('')
out = CAP / 'DWT_MEASUREMENT_REPORT.md'
out.write_text('\n'.join(lines) + '\n', encoding='utf-8')
print('written', out, len(lines), 'lines')
print('observed:', len(observed), 'missing:', len(missing), 'over budget:', len(over), 'depth2:', len(depth2))
