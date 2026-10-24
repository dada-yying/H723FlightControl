#ifndef SD_SPI_H
#define SD_SPI_H
#include "stm32h7xx_hal.h"
#include <stdint.h>
#ifndef BB_HOST_TEST
extern SPI_HandleTypeDef hspi1_sd;
extern DMA_HandleTypeDef hdma_spi1_tx, hdma_spi1_rx;
#endif
/* Observations only. First CMD0 freezes until Watch sets complete=0. */
typedef enum {
    SD_INIT_NONE=0, SD_INIT_CLOCKS, SD_INIT_CMD0, SD_INIT_CMD8,
    SD_INIT_ACMD41, SD_INIT_OCR, SD_INIT_BLOCK_LENGTH, SD_INIT_CRC,
    SD_INIT_CSD, SD_INIT_CID, SD_INIT_FAST_SPI, SD_INIT_READY
} SdSpiInitStage_t;
typedef struct {
    uint32_t complete, attempt, tick_ms, duration_us;
    uint32_t gpio_checked, cs_not_low_count;
    uint32_t sck_not_low_count, sck_not_high_count, mosi_mismatch_count;
    uint32_t bit_period_min_cycles, bit_period_max_cycles;
    uint8_t tx[6], mosi_readback[6], rx[16], rx_low_phase[16];
    uint8_t rx_count, r1;
    uint8_t cs_mode, sck_mode, mosi_mode, miso_mode, miso_pull;
} SdSpiCmd0Trace_t;
typedef enum {SD_IO_NONE=0,SD_IO_READ,SD_IO_WRITE,SD_IO_SYNC} SdSpiIoOperation_t;
typedef enum {
    SD_IO_FAULT_NONE=0,SD_IO_FAULT_BYTE_HAL,SD_IO_FAULT_CARD_BUSY,
    SD_IO_FAULT_DMA_START,SD_IO_FAULT_DMA_WAIT,SD_IO_FAULT_DMA_IRQ,
    SD_IO_FAULT_READ_TOKEN,SD_IO_FAULT_DATA_RESPONSE,SD_IO_FAULT_DATA_CRC,
    SD_IO_FAULT_COMMAND,
    SD_IO_FAULT_SHORT_TIMEOUT,SD_IO_FAULT_SHORT_HW,SD_IO_FAULT_TRANSPORT_BUSY
} SdSpiIoFaultKind_t;
typedef struct {
    uint32_t complete, init_attempt, tick_ms, elapsed_ms;
    SdSpiIoOperation_t operation;
    SdSpiIoFaultKind_t kind;
    uint32_t sector, sector_count, error_code;
    uint32_t hal_status, hal_error, hal_state;
    uint32_t spi_sr, spi_cr1, spi_cr2, spi_cfg1, spi_ier;
    uint32_t tx_dma_remaining, rx_dma_remaining, transfer_size;
    uint8_t command, r1, byte_tx, byte_rx, fast_mode;
    uint8_t dma_complete, dma_failed;
} SdSpiIoFault_t;
typedef struct {
    uint32_t version, revision, init_attempts, init_successes, init_failures;
    SdSpiInitStage_t stage, failed_stage;
    uint32_t command_count, ready_timeouts, response_timeouts;
    uint32_t last_argument, failure_argument;
    uint8_t last_command, last_r1, command_sent, last_response_count;
    uint8_t failure_command, failure_r1, failure_command_sent;
    SdSpiCmd0Trace_t cmd0;
    uint32_t read_sectors, written_sectors, sync_calls, sync_successes;
    uint32_t byte_dma_count, sector_dma_count, last_transfer_size;
    uint32_t byte_short_complete_count, byte_notify_wait_count;
    uint32_t byte_short_wait_max_cycles;
    SdSpiIoFault_t io_fault;
    /* Revision 6 additions: cumulative counters, maxima in CPU cycles.
     * Old byte-DMA counters above retain their meaning and stay unchanged. */
    uint32_t short_transfer_count, short_timeout_count, short_error_count;
    uint32_t short_bytes, block_dma_count, dma_bytes;
    uint32_t preerase_count, preerase_accepted, preerase_unsupported, preerase_errors;
    uint32_t dma_to_irq_max_cycles, dma_resume_max_cycles, ignored_callback_count;
    uint8_t preerase_last_r1;
    /* Revision 7: delayed servicing of completed RX; first short-fault stage. */
    uint32_t short_progress_recoveries;
    uint32_t short_fault_sent, short_fault_received, short_fault_wait_ms;
    /* Revision 8: fresh SR after deadline and 8-bit FIFO tail occupancy. */
    uint32_t short_deadline_rechecks, short_fifo_tail_reads, short_fault_status;
    /* Revision 9: accumulated successful DMA latency (task owner, us).
     * Window deltas / sample-count delta distinguish sustained starvation
     * from a single startup outlier in the existing maxima. */
    uint32_t dma_latency_samples;
    uint64_t dma_irq_total_us, dma_resume_total_us;
    uint32_t cmd0_attempt_count, cmd0_retry_recovered;
    uint8_t cmd0_last_rejected_r1;
} SdSpiDiag_t;
extern volatile SdSpiDiag_t g_sd_spi_diag;
int BB_CardPresent(void); /* No detect pin: presence is determined by commands. */
void BB_CardIdentity(uint32_t cid[4]);
HAL_StatusTypeDef BB_RandomSession(uint8_t session[16]);
void BB_DiskReset(void);
#endif
