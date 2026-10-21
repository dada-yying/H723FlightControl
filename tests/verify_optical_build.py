"""Static ARMCC map/callgraph checks; not hardware timing or stack validation."""
from pathlib import Path
import re
import sys

root = Path(__file__).resolve().parents[1]
directory = Path(sys.argv[1]) if len(sys.argv) > 1 else root / "MDK-ARM/optical-flow-verified"
mapping = (directory / "firmware.map").read_text(errors="replace")
graph = (directory / "firmware.htm").read_text(errors="replace")
for symbol, expected in (("of_dma", 256), ("tx_dma", 544), ("rx_dma", 544), ("s_uart5_dma_buffer", 64), ("block_words",32768)):
    found = re.search(r"^\s*" + symbol + r"\s+(0x[0-9a-fA-F]+)\s+Data\s+(\d+)", mapping, re.M)
    assert found, symbol
    address, size = int(found[1], 16), int(found[2])
    assert size == expected and address % 32 == 0
    assert 0x24000000 <= address < address + size <= 0x24050000
    print(f"{symbol}: AXI SRAM 0x{address:08x}, {size} bytes, aligned")
for task in ("StartDefaultTask", "StartFlightControlTask", "StartOpticalFlowTask", "StartBmi088Task", "StartBlackboxTask", "StartBlackboxStorageTask"):
    found = re.search(task + r"</STRONG>.*?Max Depth = (\d+)(.*?)</UL>", graph, re.S)
    assert found and "Cycle" not in found[2], task
    depth = int(found[1])
    limit = 4096 if task == "StartBlackboxStorageTask" else 1024
    assert depth < limit, task
    print(f"{task}: static depth {depth}/{limit}; runtime high-water check still required")
assert re.search(r"ucHeap\s+0x[0-9a-fA-F]+\s+Data\s+15360", mapping)
assert re.search(r"Idle_TCB\s+0x[0-9a-fA-F]+\s+Data\s+92", mapping)
# Eight dynamically allocated application stacks, 92-byte TCBs;
# each heap_4 allocation adds an 8-byte header and 8-byte alignment.
task_heap = sum(((stack + 8 + 7) & ~7) + ((92 + 8 + 7) & ~7)
                for stack in (1024, 1024, 512, 1024, 1024, 1024, 1024, 4096))
assert task_heap < 15360
print(f"Application task allocations: {task_heap}/15360 bytes; other allocations/runtime remain to check")
for irq in ("USART10_IRQHandler", "DMA1_Stream2_IRQHandler", "TIM7_IRQHandler", "TIM4_IRQHandler", "SPI1_IRQHandler", "DMA1_Stream3_IRQHandler"):
    assert re.search(irq + r"\s+0x[0-9a-fA-F]+.*stm32h7xx_it\.o", mapping), irq
print("Optical flow static checks passed")

for name, obj in (("Bmi088Attitude_Compute", "bmi088_attitude.o"),
                  ("Bmi088GyroFilter_Compute", "bmi088_gyro_filter.o"),
                  ("FusionAhrsUpdateNoMagnetometer", "FusionAhrs.o")):
    assert re.search(re.escape(name) + r"\s+0x[0-9a-fA-F]+.*" + re.escape(obj), mapping), name
timer = (root / "Core/Src/tim.c").read_text(encoding="utf-8")
assert "htim7.Init.Prescaler = 239U" in timer and "htim7.Init.Period = BMI088_READ_PERIOD_US - 1U" in timer
assert "HAL_NVIC_SetPriority(TIM7_IRQn, 6U, 0U)" in timer
print("BMI088 TIM7/Butterworth/independent AHRS symbols present; runtime timing still requires measurement")

# Local ES0491 workaround must survive HAL/CubeMX regeneration, before SPE=0.
spi = (root / "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi.c").read_text(encoding="utf-8")
close = spi[spi.index("static void SPI_CloseTransfer(SPI_HandleTypeDef *hspi)\n{"):]
assert close.index("SPI1_SD_EotGuard(hspi)") < close.index("__HAL_SPI_DISABLE(hspi)")
print("SPI1 EOT output guard remains before SPI disable; SPI2/SPI6 exclusion tested on host")
assert "g_spi1_completed_after_deadline_count" in spi
assert re.search(r"g_spi1_completed_after_deadline_count\s+0x[0-9a-fA-F]+\s+Data", mapping)
print("SPI1 polling completed-after-deadline guard/counter present; actual HAL function tested on host")
sd = (root / "App/storage/sd_spi.c").read_text(encoding="utf-8")
assert "HAL_SPI_TransmitReceive(" not in sd
assert "dma_transfer(1)" not in sd
assert "dma_transfer(514)" in sd and "dma_transfer(516)" in sd
assert "poll_transfer(p,NULL,6)" in sd
assert "SPI1_SD_EotGuard(&hspi1_sd)" in sd
assert "short_deadline_rechecks" in sd and "SPI_SR_RXPLVL" in sd
assert "g_sd_spi_diag.revision=10" in sd
assert "Bmi088Spi2Poll_Begin(&bmi088_guard,hspi,Timeout,tickstart)" in spi
assert "Bmi088Spi2Poll_WaitEot(&bmi088_guard,hspi)" in spi
assert re.search(r"g_bmi088_spi2_poll_diag\s+0x[0-9a-fA-F]+\s+Data", mapping)
protocol = (root / "App/blackbox_protocol.h").read_text(encoding="utf-8")
assert "#define SBB_CRC32_IMPL BlackboxCrc32" in protocol
tasks = (root / "Core/Src/freertos.c").read_text(encoding="utf-8")
assert re.search(r'blackboxStorageTask_attributes\s*=\s*\{[^}]*\.priority\s*=\s*osPriorityNormal\s*\}', tasks)
assert '.priority = (osPriority_t) osPriorityNormal1' in tasks
assert '.priority = (osPriority_t) osPriorityNormal2' in tasks
assert '.priority = (osPriority_t) osPriorityHigh' in tasks
ioc = (root / "CtrlBoard-H7.ioc").read_text(encoding="utf-8")
assert 'BlackboxStorage,0,1024,StartBlackboxStorageTask' in ioc
for obj, size in (("blackbox", 1024), ("blackbox_storage", 1024),
                  ("blackbox_stream", 1024), ("sd_spi", 512)):
    found = re.search(r"^\s*table\s+(0x[0-9a-fA-F]+)\s+Data\s+" + str(size) +
                      r"\s+" + obj + r"\.o\(\.constdata\)", mapping, re.M)
    assert found and 0x08000000 <= int(found[1], 16) < 0x08100000, obj
print("CRC32/CRC16 tables are read-only Flash data, no RAM/RTOS heap allocation")
print("Revision 10: BMI088 SPI2 precise timeout/fault capture; SD CRC/DMA and task priorities retained")
