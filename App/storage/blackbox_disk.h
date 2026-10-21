#ifndef BLACKBOX_DISK_H
#define BLACKBOX_DISK_H
#include <stdint.h>
extern volatile uint32_t g_bb_disk_error;
extern volatile uint32_t g_bb_disk_max_ms;
extern volatile uint32_t g_bb_disk_init_failed;
extern volatile uint32_t g_bb_disk_dma_max_ms, g_bb_disk_busy_max_ms, g_bb_disk_init_max_ms;
void BB_DiskReset(void);
#define BB_DISK_TIMEOUT 0x80000001U
#define BB_DISK_REMOVED 0x80000002U
#endif
