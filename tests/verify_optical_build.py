"""Static ARMCC map/callgraph checks; not hardware timing or stack validation."""
from pathlib import Path
import re
import sys

root = Path(__file__).resolve().parents[1]
directory = Path(sys.argv[1]) if len(sys.argv) > 1 else root / "MDK-ARM/optical-flow-verified"
mapping = (directory / "firmware.map").read_text(errors="replace")
graph = (directory / "firmware.htm").read_text(errors="replace")
for symbol, expected in (("of_dma", 256), ("s_dma_frame", 512), ("s_uart5_dma_buffer", 64)):
    found = re.search(r"^\s*" + symbol + r"\s+(0x[0-9a-fA-F]+)\s+Data\s+(\d+)", mapping, re.M)
    assert found, symbol
    address, size = int(found[1], 16), int(found[2])
    assert size == expected and address % 32 == 0
    assert 0x24000000 <= address < address + size <= 0x24050000
    print(f"{symbol}: AXI SRAM 0x{address:08x}, {size} bytes, aligned")
for task in ("StartDefaultTask", "StartFlightControlTask", "StartOpticalFlowTask"):
    found = re.search(task + r"</STRONG>.*?Max Depth = (\d+)(.*?)</UL>", graph, re.S)
    assert found and "Cycle" not in found[2], task
    depth = int(found[1])
    assert depth < 1024, task
    print(f"{task}: static depth {depth}/1024; runtime high-water check still required")
assert re.search(r"ucHeap\s+0x[0-9a-fA-F]+\s+Data\s+15360", mapping)
assert re.search(r"Idle_TCB\s+0x[0-9a-fA-F]+\s+Data\s+92", mapping)
# Five dynamically allocated application stacks (4*1024 + 512), 92-byte TCBs;
# each heap_4 allocation adds an 8-byte header and 8-byte alignment.
task_heap = sum(((stack + 8 + 7) & ~7) + ((92 + 8 + 7) & ~7)
                for stack in (1024, 1024, 512, 1024, 1024))
assert task_heap < 15360
print(f"Application task allocations: {task_heap}/15360 bytes; other allocations/runtime remain to check")
for irq in ("USART10_IRQHandler", "DMA1_Stream2_IRQHandler"):
    assert re.search(irq + r"\s+0x[0-9a-fA-F]+.*stm32h7xx_it\.o", mapping), irq
print("Optical flow static checks passed")
