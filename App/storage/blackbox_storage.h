#ifndef BLACKBOX_STORAGE_H
#define BLACKBOX_STORAGE_H
#include <stdint.h>
typedef enum {
    BLACKBOX_WAIT_CARD, BLACKBOX_INITIALIZING, BLACKBOX_READY,
    BLACKBOX_RECORDING, BLACKBOX_FAULT, BLACKBOX_RECOVERING, BLACKBOX_SPACE_WAIT
} BlackboxState;
typedef enum {
    BB_ERROR_NONE, BB_ERROR_PORT, BB_ERROR_RNG, BB_ERROR_MOUNT,
    BB_ERROR_FILESYSTEM, BB_ERROR_DIRECTORY, BB_ERROR_CREATE, BB_ERROR_PREALLOC,
    BB_ERROR_WRITE, BB_ERROR_SYNC, BB_ERROR_CLOSE, BB_ERROR_REMOVED,
    BB_ERROR_UART, BB_ERROR_SDINIT, BB_ERROR_READ, BB_ERROR_IDENTITY,
    BB_ERROR_GROW, BB_ERROR_SPACE
} BlackboxError;
typedef enum {
    BB_SWITCH_NONE, BB_SWITCH_CARD, BB_SWITCH_LOG, BB_SWITCH_LIMIT
} BlackboxSwitch;
typedef struct {
    BlackboxState state;
    BlackboxError error;
    uint32_t fatfs_result, sd_error, last_uart_error;
    uint32_t received_frames, dropped_frames, resyncs, uart_errors, uart_restarts;
    uint32_t queue_used, queue_high_water, written_records, written_blocks;
    uint32_t max_write_ms, max_disk_ms, file_number;
    uint32_t last_rx_tick;
    uint32_t bad_packets; /* Header/version/length/CRC rejects. Queue sizes are bytes. */
    BlackboxError last_error;
    uint32_t last_fatfs_result, last_sd_error, fault_count;
    uint32_t retry_count, recovery_count, retry_due;
    uint32_t write_offset, allocated_bytes, synced_offset;
    uint32_t growth_steps, max_growth_ms, switch_count;
    BlackboxSwitch switch_reason;
    uint32_t led_code;
    uint32_t ready_tick_ms, prealloc_ms, max_mount_ms;
    uint32_t metadata_syncs, data_syncs, max_sync_ms, max_data_sync_ms;
    uint32_t max_dma_ms, max_card_busy_ms, max_sd_init_ms;
    uint32_t last_drop_tick_ms;
    uint32_t write_calls, max_write_bytes;
    uint32_t diagnostic_blocks, rate_limit_hz, sampled_out_packets;
    uint32_t stack_free_words, heap_free_bytes, write_hz, data_written;
    uint32_t stopped;
} BlackboxStats;
extern volatile BlackboxStats g_blackbox_stats; /* Watch only; application must not write. */
void BlackboxStorage_Init(void);
void BlackboxStorage_Process(void);
int BlackboxStorage_Enqueue(const uint8_t *packet, uint16_t size, uint64_t us);
void BlackboxStorage_TaskFailed(void);
const volatile BlackboxStats *Blackbox_GetStats(void);
#endif
