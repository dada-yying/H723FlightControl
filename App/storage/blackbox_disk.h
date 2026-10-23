#ifndef BLACKBOX_DISK_H
#define BLACKBOX_DISK_H
#include <stdint.h>




void BB_DiskReset(void);
uint32_t BB_DiskError(void);
uint32_t BB_DiskMaxMs(void);
uint32_t BB_DiskInitFailed(void);
uint32_t BB_DiskDmaMaxMs(void);
uint32_t BB_DiskBusyMaxMs(void);
uint32_t BB_DiskInitMaxMs(void);

#define BB_DISK_TIMEOUT 0x80000001U
#define BB_DISK_REMOVED 0x80000002U
#endif
