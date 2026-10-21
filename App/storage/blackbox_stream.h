#ifndef BLACKBOX_STREAM_H
#define BLACKBOX_STREAM_H
#include <stdint.h>
#include <stddef.h>

#include "../blackbox_protocol.h"
#define BB_FRAME_SIZE SBB_MAX_PACKET
#define BB_QUEUE_SRAM_BYTES 262144U
#define BB_QUEUE_CCM_BYTES 0U
#define BB_QUEUE_BYTES (BB_QUEUE_SRAM_BYTES + BB_QUEUE_CCM_BYTES)
#define BB_RECORD_HEADER 16U
#define BB_BLOCK_SIZE 4096U
#define BB_HEADER_SIZE 64U
#define BB_DMA_SIZE 4096U
#define BB_FILE_HEADER_FLAG 1U

typedef struct {
    uint64_t time_us;
    uint32_t sequence;
    uint16_t length;
    uint8_t frame[BB_FRAME_SIZE];
} BB_Record;

typedef struct {
    volatile uint32_t frames;
    volatile uint32_t dropped;
    volatile uint32_t resyncs;
    volatile uint32_t high_water;
    volatile uint32_t bad_packets;
} BB_StreamStats;

/* Single ISR producer, single foreground consumer; neither may be reentered. */
typedef struct {
    uint8_t records[BB_QUEUE_SRAM_BYTES];
    uint8_t *extension; /* CPU-only CCM tail, never passed to a DMA peripheral. */
    uint32_t capacity;
    volatile uint32_t head, tail;
    uint32_t write_index, read_index; /* Independent of uint32 occupancy wrap. */
    BB_StreamStats stats;
    uint8_t history[BB_FRAME_SIZE];
    uint32_t history_count;
} BB_Stream;

typedef struct { uint32_t last; uint64_t value; } BB_Clock;
void BB_StreamInit(BB_Stream *s);
void BB_StreamInitExtended(BB_Stream *s, uint8_t *extension);
void BB_StreamResetParser(BB_Stream *s);
int BB_StreamEnqueue(BB_Stream *s, const uint8_t *packet, uint16_t size, uint64_t time_us);
void BB_StreamFeed(BB_Stream *s, const uint8_t *data, size_t size, uint64_t time_us);
uint32_t BB_StreamAvailable(const BB_Stream *s);
int BB_StreamPop(BB_Stream *s, BB_Record *record);
uint16_t BB_StreamPeekSize(const BB_Stream *s);
void BB_StreamDmaDrain(BB_Stream *s, const uint8_t *dma, uint32_t size,
                       uint32_t *cursor, uint32_t position, uint64_t time_us);
uint64_t BB_ClockExtend(BB_Clock *clock, uint32_t counter);
uint32_t BB_Crc32(const uint8_t *data, size_t length);
void BB_BlockBegin(uint8_t *block, const uint8_t session[16], uint32_t number,
                   uint32_t file_number, uint32_t flags);
int BB_BlockRecord(uint8_t *block, uint32_t index, const BB_Record *record);
void BB_BlockFinish(uint8_t *block, uint16_t count, uint32_t drops,
                    uint32_t uart_errors, uint32_t resyncs);
#endif
