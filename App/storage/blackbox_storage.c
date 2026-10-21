#include "blackbox_storage.h"
#include "blackbox.h"
#include "blackbox_stream.h"
#include "sd_spi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "blackbox_disk.h"
#include "blackbox_config.h"
#include "blackbox_diagnostic.h"
#include "ff.h"
#include <string.h>
#include <stdio.h>

typedef char capacity_aligned[(BB_FILE_BYTES >= 8192U && BB_FILE_BYTES % 4096U == 0 &&
    BB_MAX_FILE_BYTES >= BB_FILE_BYTES && BB_MAX_FILE_BYTES % 4096U == 0) ? 1 : -1];
typedef char batch_size_supported[(BB_WRITE_BATCH_BLOCKS >= 1U && BB_WRITE_BATCH_BLOCKS <= 8U) ? 1 : -1];
volatile BlackboxStats g_blackbox_stats;
/* Producer and consumer share CPU cache; only sd_spi bounce buffers use DMA. */
#if defined(BB_HOST_TEST)
#define BB_AXI
#else
#define BB_AXI __attribute__((section(".blackbox_storage"), aligned(32)))
#endif
static BB_Stream stream BB_AXI;
static uint32_t block_words[BB_WRITE_BATCH_BLOCKS * BB_BLOCK_SIZE / 4U] BB_AXI;
static uint32_t work_words[BB_BLOCK_SIZE / 4U] BB_AXI;
#define BLOCK ((uint8_t *)block_words)
#define WORK ((uint8_t *)work_words)
static FATFS filesystem;
static FIL file;
static FF_RESERVE reserve;
static uint8_t session[16];
static uint32_t original_cid[4], probe_cid[4];
static char current_path[16];
static uint32_t block_number, last_sync;
static uint16_t schema_offset;
static uint8_t schema_packet[SBB_MAX_PACKET];
static uint32_t rate_tick, rate_records;
static uint32_t prefix_records, next_record_sequence;
static uint32_t last_capture_sequence;
static uint8_t have_capture_sequence;
static uint8_t session_ready;
static uint32_t batch_started;
static uint32_t last_diagnostic;
static uint8_t batch_blocks;
static uint32_t write_offset, allocated, synced_offset, synced_crc, written_crc;
static uint32_t grow_target, space_retry, retry_level, invalid_probes;
static uint32_t prealloc_started;


static uint8_t have_path, header_ready, pending, mounted, allocating, retry_now;
static BlackboxSwitch next_switch;
static uint8_t space_blocked;
static uint16_t block_count;
static uint8_t receiving;
static volatile uint8_t accepting;
static void stop_full(void);

const volatile BlackboxStats *Blackbox_GetStats(void) { return &g_blackbox_stats; }
static void diagnose(BlackboxError e, FRESULT fr)
{
    if (g_blackbox_stats.error == BB_ERROR_NONE) {
        g_blackbox_stats.error = e;
        g_blackbox_stats.fatfs_result = (uint32_t)fr;
        g_blackbox_stats.sd_error = g_bb_disk_error;
    }
    g_blackbox_stats.last_error = e;
    g_blackbox_stats.last_fatfs_result = (uint32_t)fr;
    g_blackbox_stats.last_sd_error = g_bb_disk_error;
    g_blackbox_stats.fault_count++;
}
static void fault(BlackboxError e, FRESULT fr)
{
    uint32_t delay;
    if (g_blackbox_stats.state == BLACKBOX_FAULT) return;
    diagnose(e, fr);
    if (e == BB_ERROR_PORT || e == BB_ERROR_RNG || e == BB_ERROR_UART) {
        g_blackbox_stats.state = BLACKBOX_FAULT;
        return;
    }
    delay = retry_level == 0 ? 1000U : (retry_level == 1 ? 2000U : 5000U);
    if (retry_level < 2) retry_level++;
    g_blackbox_stats.retry_due = HAL_GetTick() + delay;
    g_blackbox_stats.state = BLACKBOX_RECOVERING;
}
static void update_stats(void)
{
    g_blackbox_stats.received_frames = stream.stats.frames;
    g_blackbox_stats.dropped_frames = stream.stats.dropped;
    g_blackbox_stats.resyncs = stream.stats.resyncs;
    g_blackbox_stats.bad_packets = stream.stats.bad_packets;
    g_blackbox_stats.queue_used = BB_StreamAvailable(&stream);
    g_blackbox_stats.queue_high_water = stream.stats.high_water;
    g_blackbox_stats.max_disk_ms = g_bb_disk_max_ms;
    g_blackbox_stats.write_offset = write_offset;
    g_blackbox_stats.allocated_bytes = allocated;
    g_blackbox_stats.synced_offset = synced_offset;
    g_blackbox_stats.max_dma_ms = g_bb_disk_dma_max_ms;
    g_blackbox_stats.max_card_busy_ms = g_bb_disk_busy_max_ms;
    g_blackbox_stats.max_sd_init_ms = g_bb_disk_init_max_ms;
    {uint32_t now=HAL_GetTick(); if((uint32_t)(now-rate_tick)>=1000){g_blackbox_stats.write_hz=(uint32_t)((uint64_t)(g_blackbox_stats.data_written-rate_records)*1000/(uint32_t)(now-rate_tick));rate_records=g_blackbox_stats.data_written;rate_tick=now;}}
}
static void update_activity(void)
{
    /* Read the ISR timestamp first. Sampling now first permits the ISR to
     * publish a newer timestamp in between, underflowing unsigned subtraction. */
    uint32_t last_rx = g_blackbox_stats.last_rx_tick;
    uint32_t now = HAL_GetTick();
    g_blackbox_stats.state = stream.stats.frames && (uint32_t)(now - last_rx) < 1000U
        ? BLACKBOX_RECORDING : BLACKBOX_READY;
}
static int check(FRESULT fr, BlackboxError e)
{
    if (fr == FR_OK) return g_blackbox_stats.state != BLACKBOX_FAULT;
    fault(e, fr); return 0;
}
static int write_bytes(const uint8_t *bytes, UINT length)
{
    UINT written = 0;
    uint32_t start = HAL_GetTick(), duration;
    FRESULT fr = f_write(&file, bytes, length, &written);
    g_blackbox_stats.write_calls++;
    if (length > g_blackbox_stats.max_write_bytes) g_blackbox_stats.max_write_bytes = length;
    duration = HAL_GetTick() - start;
    if (duration > g_blackbox_stats.max_write_ms) g_blackbox_stats.max_write_ms = duration;
    if (!check(fr, BB_ERROR_WRITE)) return 0;
    if (written != length) { diagnose(BB_ERROR_SPACE, FR_DENIED); stop_full(); return 0; }
    return 1;
}
static int sync_file(void)
{
    uint32_t start = HAL_GetTick(), duration;
    FRESULT fr = f_sync(&file);
    duration = HAL_GetTick() - start;
    g_blackbox_stats.metadata_syncs++;
    if (duration > g_blackbox_stats.max_sync_ms) g_blackbox_stats.max_sync_ms = duration;
    if (!check(fr, BB_ERROR_SYNC)) return 0;
    allocated = (uint32_t)f_size(&file) & ~4095UL;
    reserve.dirty = 0;
    synced_offset = write_offset; synced_crc = written_crc; last_sync = HAL_GetTick();
    return 1;
}
static int sync_data(void)
{
    uint32_t start = HAL_GetTick(), duration;
    FRESULT fr = f_sync_reserved_data(&file, allocated);
    duration = HAL_GetTick() - start;
    g_blackbox_stats.data_syncs++;
    if (duration > g_blackbox_stats.max_data_sync_ms) g_blackbox_stats.max_data_sync_ms = duration;
    if (!check(fr, BB_ERROR_SYNC)) return 0;
    synced_offset = write_offset; synced_crc = written_crc; last_sync = HAL_GetTick();
    return 1;
}
static int find_file_number(void)
{
    DIR directory;
    FILINFO info;
    FRESULT fr;
    uint32_t highest = g_blackbox_stats.file_number, n, i;
    if (!check(f_opendir(&directory, "0:/"), BB_ERROR_DIRECTORY)) return 0;
    for (;;) {
        fr = f_readdir(&directory, &info);
        if (fr != FR_OK || info.fname[0] == 0) break;
        if (strlen(info.fname) != 12 || info.fname[0] != 'B' || strcmp(info.fname + 8, ".BIN")) continue;
        n = 0;
        for (i = 1; i < 8 && info.fname[i] >= '0' && info.fname[i] <= '9'; i++)
            n = n * 10U + (uint32_t)(info.fname[i] - '0');
        if (i == 8 && n > highest) highest = n;
    }
    (void)f_closedir(&directory);
    if (!check(fr, BB_ERROR_DIRECTORY)) return 0;
    g_blackbox_stats.file_number = highest; return 1;
}
static void reframe_pending(void)
{
    uint32_t i;
    uint32_t previous_old=0;
    uint8_t have_old=0;
    next_record_sequence=prefix_records;
    for (i = 0; i < (uint32_t)batch_blocks + (uint32_t)(block_count != 0); i++) {
        uint8_t *b = BLOCK + i * BB_BLOCK_SIZE;
        unsigned j,pos=64,count=i<batch_blocks?sbb_u16(b+32):block_count;
        for(j=0;j<count;j++) {
            uint32_t old=sbb_u32(b+pos+8);
            if(have_old)next_record_sequence+=(uint32_t)(old-previous_old)-1U;
            sbb_p32(b+pos+8,next_record_sequence++);
            previous_old=old;have_old=1;
            pos+=(16U+sbb_u16(b+pos+12)+3U)&~3U;
        }
        sbb_p32(b + 28, block_number + i); sbb_p32(b + 48, g_blackbox_stats.file_number);
        if (i < batch_blocks) { sbb_p32(b + 60, 0); sbb_p32(b + 60, sbb_crc(b, 4096)); }
    }
}
static int new_file(BlackboxSwitch reason)
{
    FRESULT fr;
    uint32_t number;
    char name[16];
    if (reason != BB_SWITCH_NONE) next_switch = reason;
    if (!find_file_number()) return 0;
    number = g_blackbox_stats.file_number;
    do {
        if (number >= 9999999U) { fault(BB_ERROR_CREATE, FR_DENIED); return 0; }
        number++;
        (void)snprintf(name, sizeof(name), "0:/B%07lu.BIN", (unsigned long)number);
        fr = f_open(&file, name, FA_READ | FA_WRITE | FA_CREATE_NEW);
    } while (fr == FR_EXIST);
    /* Remember the chosen name even when create reports an I/O failure:
     * the directory entry may already have reached the card. Recovery retries
     * this name rather than leaking an empty file on every transient error. */
    g_blackbox_stats.file_number = number;
    memcpy(current_path, name, sizeof(name)); BB_CardIdentity(original_cid);
    have_path = 1; header_ready = 0; allocating = 1;
    prealloc_started = HAL_GetTick();
    write_offset = synced_offset = allocated = 0; written_crc = synced_crc = 0;
    block_number = 1; grow_target = BB_FILE_BYTES; memset(&reserve, 0, sizeof(reserve));
    invalid_probes = 0;
    if (next_switch != BB_SWITCH_NONE) {
        g_blackbox_stats.switch_count++; g_blackbox_stats.switch_reason = next_switch;
    }
    next_switch = BB_SWITCH_NONE; space_blocked = 0;
    schema_offset=0;prefix_records=next_record_sequence;
    /* Insert new definitions ahead of pending records, preserving the global
     * local sequence across files in this boot session. */
    {unsigned i;for(i=0;i<(unsigned)batch_blocks+(block_count!=0);i++){
        uint8_t *b=BLOCK+i*BB_BLOCK_SIZE;
        unsigned count=i<batch_blocks?sbb_u16(b+32):block_count;
        if(count){prefix_records=sbb_u32(b+64+8);break;}
    }}
    reframe_pending();
    if(fr==FR_DENIED){diagnose(BB_ERROR_SPACE,fr);stop_full();return 0;}
    return check(fr, BB_ERROR_CREATE);
}
static int descriptor(void)
{
    if (!check(f_lseek(&file, 0), BB_ERROR_PREALLOC)) return 0;
    BB_BlockBegin(WORK, session, 0, g_blackbox_stats.file_number, BB_FILE_HEADER_FLAG);
    BB_BlockFinish(WORK, 0, stream.stats.dropped, g_blackbox_stats.uart_errors, stream.stats.resyncs);
    if (!write_bytes(WORK, BB_BLOCK_SIZE)) return 0;
    write_offset = 4096;
    if (!sync_file()) return 0;
    header_ready = 1; allocated = (uint32_t)f_size(&file);
    write_offset = synced_offset = 4096; last_sync = HAL_GetTick();
    return 1;
}
/* WORK is independent of BLOCK so recovery never destroys queued data. */
static int read_block(uint32_t offset)
{
    UINT n = 0;
    FRESULT fr = f_lseek(&file, offset);
    if (fr == FR_INT_ERR) return -2; /* Definitive broken FAT chain, not a bus timeout. */
    if (!check(fr, BB_ERROR_READ)) return -1;
    fr = f_read(&file, WORK, 4096, &n);
    if (fr == FR_INT_ERR) return -2;
    if (!check(fr, BB_ERROR_READ)) return -1;
    return n == 4096;
}
static int valid_block(uint32_t number, int description)
{
    uint32_t crc = sbb_u32(WORK + 60), calculated;
    uint32_t flags = sbb_u32(WORK + 52);
    if (memcmp(WORK, "SKYBBX3", 8) || sbb_u16(WORK + 8) != 3 || sbb_u16(WORK + 10) != 64 ||
        memcmp(WORK + 12, session, 16) || sbb_u32(WORK + 28) != number ||
        sbb_u32(WORK + 48) != g_blackbox_stats.file_number ||
        (description ? flags != BB_FILE_HEADER_FLAG : (flags != 0U && flags != BB_DIAGNOSTIC_FLAG))) return 0;
    sbb_p32(WORK + 60, 0); calculated = sbb_crc(WORK, 4096); sbb_p32(WORK + 60, crc);
    if (crc != calculated) return 0;
    if (description) return sbb_u16(WORK + 32) == 0 && sbb_u16(WORK + 34) == 0;
    if (flags == BB_DIAGNOSTIC_FLAG) return sbb_u16(WORK + 32) == 0 &&
        sbb_u16(WORK + 34) == BB_DIAGNOSTIC_BYTES && !memcmp(WORK + 64, BB_DIAGNOSTIC_MAGIC, 8);
    return sbb_u16(WORK + 32) != 0 && sbb_u16(WORK + 34) <= 4032;
}
static void acknowledge(void)
{
    uint16_t count = sbb_u16(BLOCK + 32);
    written_crc = sbb_u32(BLOCK + 60);
    write_offset += 4096; block_number++;
    g_blackbox_stats.written_records += count;
    {unsigned p=64,i; for(i=0;i<count;i++){unsigned n=sbb_u16(BLOCK+p+12);if(BLOCK[p+16+5]==SBB_DATA)g_blackbox_stats.data_written++;p+=(16+n+3)&~3U;}}
    g_blackbox_stats.written_blocks++;
    if (sbb_u32(BLOCK + 52) == BB_DIAGNOSTIC_FLAG) g_blackbox_stats.diagnostic_blocks++;
    batch_blocks--;
    if (batch_blocks || block_count) memmove(BLOCK, BLOCK + BB_BLOCK_SIZE,
        (batch_blocks + (block_count != 0)) * BB_BLOCK_SIZE);
    pending = batch_blocks != 0;
}
static void cleanup(void)
{
    /* Never f_close here: it could flush stale metadata onto a replacement card. */
    BB_DiskReset();
    (void)f_mount(NULL, "0:", 0);
    memset(&file, 0, sizeof(file)); memset(&filesystem, 0, sizeof(filesystem));
    memset(&reserve, 0, sizeof(reserve)); mounted = 0;
}
static int unsafe_log(void)
{
    invalid_probes++;
    if (invalid_probes < 3) { fault(BB_ERROR_IDENTITY, FR_INVALID_OBJECT); return 0; }
    diagnose(BB_ERROR_IDENTITY, FR_INVALID_OBJECT);
    /* Only successful reads/FR_NO_FILE reach this branch, never I/O failures. */
    have_path = 0;
    return new_file(BB_SWITCH_LOG);
}
static int resume_file(void)
{
    FRESULT fr;
    uint32_t offset;
    int result;
    BB_CardIdentity(probe_cid);
    if (memcmp(probe_cid, original_cid, 16)) {
        have_path = 0; return new_file(BB_SWITCH_CARD);
    }
    fr = f_open(&file, current_path, FA_READ | FA_WRITE | FA_OPEN_EXISTING);
    if(fr==FR_NO_FILE && !header_ready) {
        fr=f_open(&file,current_path,FA_READ|FA_WRITE|FA_CREATE_NEW);
        if(fr==FR_DENIED){diagnose(BB_ERROR_SPACE,fr);stop_full();return 0;}
    }
    if (fr == FR_NO_FILE) return unsafe_log();
    if (!check(fr, BB_ERROR_READ)) { invalid_probes = 0; return 0; }
    if (!header_ready) {
        /* This exact file was created by us but its initial descriptor failed. */
        return descriptor();
    }
    result = read_block(0);
    if (result == -2) return unsafe_log();
    if (result < 0) { invalid_probes = 0; return 0; }
    if (!result || !valid_block(0, 1) || f_size(&file) < write_offset) return unsafe_log();
    offset = synced_offset > 4096 ? synced_offset - 4096 : 4096;
    for (; offset < write_offset; offset += 4096) {
        result = read_block(offset);
        if (result == -2) return unsafe_log();
        if (result < 0) { invalid_probes = 0; return 0; }
        if (!result || !valid_block(offset / 4096, 0) ||
            (offset + 4096 == synced_offset && synced_offset > 4096 && sbb_u32(WORK + 60) != synced_crc) ||
            (offset + 4096 == write_offset && sbb_u32(WORK + 60) != written_crc)) return unsafe_log();
    }
    allocated = (uint32_t)f_size(&file) & ~4095UL;
    while (pending && write_offset < BB_MAX_FILE_BYTES && allocated >= write_offset + 4096U) {
        result = read_block(write_offset);
        if (result == -2) return unsafe_log();
        if (result < 0) { invalid_probes = 0; return 0; }
        if (result && !memcmp(WORK, BLOCK, 4096)) acknowledge();
        else break;
    }
    invalid_probes = 0;
    if (!check(f_lseek(&file, write_offset), BB_ERROR_READ)) return 0;
    if (!sync_data()) return 0;
    synced_offset = write_offset; synced_crc = written_crc; last_sync = HAL_GetTick();
    if (grow_target < allocated) grow_target = allocated;
    if (grow_target < BB_FILE_BYTES) grow_target = BB_FILE_BYTES;
    space_blocked = 0;
    allocating = allocated < BB_FILE_BYTES;
    return 1;
}
static int mount_storage(void)
{
    FRESULT fr;
    uint32_t start = HAL_GetTick(), duration;
    cleanup();
    fr = f_mount(&filesystem, "0:", 1);
    duration = HAL_GetTick() - start;
    if (duration > g_blackbox_stats.max_mount_ms) g_blackbox_stats.max_mount_ms = duration;
    if (!check(fr, g_bb_disk_init_failed ? BB_ERROR_SDINIT : BB_ERROR_MOUNT)) {
        invalid_probes = 0; return 0;
    }
    mounted = 1;
    if (filesystem.fs_type != FS_FAT32) { fault(BB_ERROR_FILESYSTEM, FR_NO_FILESYSTEM); return 0; }
    return have_path ? resume_file() : new_file(BB_SWITCH_NONE);
}
static void stop_full(void)
{
    accepting=0;g_blackbox_stats.stopped=1;
    g_blackbox_stats.state=BLACKBOX_SPACE_WAIT;
}
static int write_schema(void)
{
    uint16_t offset=schema_offset,total=Blackbox_SchemaSize(),count=0;
    BB_Record r;
    uint32_t crc;
    if(allocated-write_offset<4096U){stop_full();return 0;}
    BB_BlockBegin(WORK,session,block_number,g_blackbox_stats.file_number,0);
    while(offset<total) {
        uint16_t n=total-offset; if(n>96)n=96;
        sbb_p16(schema_packet+24,total);sbb_p16(schema_packet+26,offset);
        memcpy(schema_packet+28,Blackbox_SchemaData()+offset,n);
        r.length=sbb_packet(schema_packet,SBB_SCHEMA,n+4,g_blackbox_tx_stats.schema_id,0,0);
        memcpy(r.frame,schema_packet,r.length);
        /* Prefix definitions have no capture time. Local sequence begins after prefix. */
        r.sequence=prefix_records+count; r.time_us=0;
        if(!BB_BlockRecord(WORK,count,&r))break;
        count++;offset+=n;
    }
    BB_BlockFinish(WORK,count,stream.stats.dropped,0,0);
    crc=sbb_u32(WORK+60);
    if(!check(f_lseek(&file,write_offset),BB_ERROR_WRITE) || !write_bytes(WORK,4096))return 0;
    write_offset+=4096;block_number++;written_crc=crc;
    schema_offset=offset;prefix_records+=count;
    reframe_pending();
    if(!sync_data())return 0;
    return 1;
}
int BlackboxStorage_Enqueue(const uint8_t *p,uint16_t n,uint64_t us)
{
    if(!accepting)return 0;
    g_blackbox_stats.last_rx_tick=HAL_GetTick();
    return BB_StreamEnqueue(&stream,p,n,us);
}
void BlackboxStorage_TaskFailed(void)
{
    accepting=0;g_blackbox_stats.stopped=1;g_blackbox_stats.state=BLACKBOX_FAULT;
    g_blackbox_stats.last_error=BB_ERROR_PORT;
}
static int reserve_step(void)
{
    uint32_t start = HAL_GetTick();
    uint32_t commit_bytes = allocating ? BB_INITIAL_COMMIT_BYTES : BB_GROW_COMMIT_BYTES;
    FRESULT fr = f_reserve_step(&file, &reserve, grow_target);
    uint32_t duration;
    g_blackbox_stats.growth_steps++;
    if (fr != FR_OK && fr != FR_DENIED) {
        fault(allocating ? BB_ERROR_PREALLOC : BB_ERROR_GROW, fr); return 0;
    }
    if (reserve.dirty && (f_size(&file) - allocated >= commit_bytes ||
        f_size(&file) >= grow_target || fr == FR_DENIED ||
        (!allocating && allocated - write_offset < BB_BLOCK_SIZE))) {
        if (!sync_file()) return 0;
    }
    duration = HAL_GetTick() - start;
    if (duration > g_blackbox_stats.max_growth_ms) g_blackbox_stats.max_growth_ms = duration;
    if (fr == FR_DENIED) {
        diagnose(BB_ERROR_SPACE, fr); space_retry = HAL_GetTick() + 5000U; space_blocked = 1;
        reserve.searched = 0;
        if (allocated - write_offset < 4096U) stop_full();
        else if (allocating) {allocating=0;g_blackbox_stats.state=BLACKBOX_READY;}

        return 0;
    }
    return 1;
}
void BlackboxStorage_Init(void)
{
    memset((void *)&g_blackbox_stats, 0, sizeof(g_blackbox_stats));
    BB_StreamInit(&stream);
    accepting=1; schema_offset=0; rate_tick=rate_records=0;
    prefix_records=next_record_sequence=session_ready=0;
    last_capture_sequence=0;have_capture_sequence=0;

    have_path = header_ready = pending = mounted = allocating = receiving = retry_now = 0;
    write_offset = allocated = synced_offset = block_count = 0;
    batch_blocks = 0; batch_started = 0;
    last_diagnostic = 0;
    retry_level = invalid_probes = grow_target = space_retry = 0;
    next_switch = BB_SWITCH_NONE; space_blocked = 0;
    /* Session is filled by the storage task after the scheduler starts. */
    memset(session,0,sizeof(session));
    g_blackbox_stats.state = BLACKBOX_WAIT_CARD;
}
void BlackboxStorage_Process(void)
{
    uint32_t now = HAL_GetTick();
    BB_Record record;
    int was_recovery;
    update_stats();
    if(g_blackbox_stats.stopped)return;
    if(!session_ready) {
        if(BB_RandomSession(session)!=HAL_OK){fault(BB_ERROR_RNG,FR_OK); accepting=0;return;}
        session_ready=1;
    }
    if (g_blackbox_stats.state == BLACKBOX_FAULT) return;
    if (g_blackbox_stats.state == BLACKBOX_RECOVERING || g_blackbox_stats.state == BLACKBOX_WAIT_CARD) {
        if (!BB_CardPresent()) { retry_now = 1; return; }
        if (g_blackbox_stats.state == BLACKBOX_RECOVERING && !retry_now &&
            (int32_t)(now - g_blackbox_stats.retry_due) < 0) return;
        retry_now = 0; was_recovery = g_blackbox_stats.state == BLACKBOX_RECOVERING;
        if (was_recovery) g_blackbox_stats.retry_count++;
        g_blackbox_stats.state = BLACKBOX_INITIALIZING;
        if (!mount_storage()) return;
        if (was_recovery) g_blackbox_stats.recovery_count++;
        retry_level = 0;
    }
    if (!BB_CardPresent()) { fault(BB_ERROR_REMOVED, FR_NOT_READY); return; }
    if (!mounted) return;
    if (!header_ready && !descriptor()) return;
    if (g_blackbox_stats.state == BLACKBOX_SPACE_WAIT) {stop_full();return;}
    if (allocating) {
        uint32_t start_bytes=BB_INITIAL_COMMIT_BYTES<BB_FILE_BYTES?BB_INITIAL_COMMIT_BYTES:BB_FILE_BYTES;
        if (!reserve_step()) return;
        /* Begin logging after the first committed reserve, then finish the
         * existing 40MiB target in the background with queue-drain priority. */
        if (allocated < start_bytes) return;
        allocating = 0;
        g_blackbox_stats.prealloc_ms = HAL_GetTick() - prealloc_started;
    }
    /* Recovery may reopen a fully allocated file after the final startup sync
     * reached the card but returned an error. RX must still be started then. */
    if (!receiving) {
        receiving=1;
        if (!g_blackbox_stats.ready_tick_ms) {
            g_blackbox_stats.ready_tick_ms = HAL_GetTick();
            last_diagnostic = HAL_GetTick();
            g_blackbox_stats.prealloc_ms = HAL_GetTick() - prealloc_started;
        }
    }
    if (write_offset == BB_MAX_FILE_BYTES) {
        (void)sync_data(); stop_full(); return;
    }
    if (schema_offset < Blackbox_SchemaSize()) {
        if(!write_schema())return;
        if(schema_offset < Blackbox_SchemaSize())return;
    }
    if (allocated - write_offset < BB_GROW_MARGIN && allocated < BB_MAX_FILE_BYTES && grow_target <= allocated)
        grow_target = BB_MAX_FILE_BYTES - allocated < BB_FILE_BYTES ? BB_MAX_FILE_BYTES : allocated + BB_FILE_BYTES;
    if (grow_target > allocated && (!space_blocked || (int32_t)(now - space_retry) >= 0) &&
        (BB_StreamAvailable(&stream) < BB_QUEUE_BYTES / 4U || allocated - write_offset < 4096U)) {
        if (!reserve_step() && g_blackbox_stats.state != BLACKBOX_READY &&
            g_blackbox_stats.state != BLACKBOX_RECORDING && g_blackbox_stats.state != BLACKBOX_INITIALIZING) return;
    }
    if (allocated - write_offset < 4096U) {
        if (g_blackbox_stats.state != BLACKBOX_RECOVERING) {
            diagnose(BB_ERROR_SPACE, FR_DENIED); stop_full();
        }
        return;
    }
    if (!pending) {
        uint32_t limit = (allocated - write_offset) / BB_BLOCK_SIZE;
        uint8_t *b;
        now = HAL_GetTick();
        uint32_t cluster_bytes = (uint32_t)filesystem.csize * 512U;
        if (limit > BB_WRITE_BATCH_BLOCKS) limit = BB_WRITE_BATCH_BLOCKS;
        /* Keep a batch within the cluster so FatFs can issue one contiguous
         * transfer. The first data batch follows the 4 KiB descriptor. */
        if (cluster_bytes >= BB_BLOCK_SIZE) {
            uint32_t until_boundary = (cluster_bytes - write_offset % cluster_bytes) / BB_BLOCK_SIZE;
            if (limit > until_boundary) limit = until_boundary;
        }
        if (!batch_blocks && !block_count) batch_started = now;
#if BB_DIAGNOSTIC_MS != 0
        if (!batch_blocks && !block_count && (uint32_t)(now - last_diagnostic) >= BB_DIAGNOSTIC_MS) {
            const uint32_t values[BB_DIAGNOSTIC_FIELDS] = {
                now, g_blackbox_stats.ready_tick_ms, g_blackbox_stats.prealloc_ms,
                stream.stats.frames, stream.stats.dropped, g_blackbox_stats.sampled_out_packets,
                g_blackbox_stats.rate_limit_hz, (uint32_t)g_blackbox_stats.state,
                (uint32_t)g_blackbox_stats.last_error, g_blackbox_stats.last_fatfs_result,
                g_blackbox_stats.last_sd_error, g_blackbox_stats.retry_count, g_blackbox_stats.recovery_count,
                g_blackbox_stats.max_write_ms, g_bb_disk_max_ms, g_bb_disk_dma_max_ms,
                g_bb_disk_busy_max_ms, g_bb_disk_init_max_ms, g_blackbox_stats.max_mount_ms,
                g_blackbox_stats.max_sync_ms, g_blackbox_stats.max_data_sync_ms,
                g_blackbox_stats.max_growth_ms, BB_StreamAvailable(&stream), stream.stats.high_water,
                g_blackbox_stats.write_calls, g_blackbox_stats.max_write_bytes, allocated, write_offset,
                cluster_bytes, BB_WRITE_BATCH_BLOCKS * BB_BLOCK_SIZE
            };
            uint32_t i;
            BB_BlockBegin(BLOCK, session, block_number, g_blackbox_stats.file_number, BB_DIAGNOSTIC_FLAG);
            memcpy(BLOCK + 64, BB_DIAGNOSTIC_MAGIC, 8);
            for (i = 0; i < BB_DIAGNOSTIC_FIELDS; i++) sbb_p32(BLOCK + 72 + i * 4U, values[i]);
            sbb_p16(BLOCK + 34, BB_DIAGNOSTIC_BYTES);
            BB_BlockFinish(BLOCK, 0, stream.stats.dropped, g_blackbox_stats.uart_errors, stream.stats.resyncs);
            batch_blocks = 1; last_diagnostic = now;
        }
#endif
        while (batch_blocks < limit && BB_StreamAvailable(&stream)) {
            b = BLOCK + batch_blocks * BB_BLOCK_SIZE;
            if (!block_count) {
                BB_BlockBegin(b, session, block_number + batch_blocks, g_blackbox_stats.file_number, 0);
            }
            if (sbb_u16(b + 34) + BB_StreamPeekSize(&stream) > BB_BLOCK_SIZE - BB_HEADER_SIZE) {
                BB_BlockFinish(b, block_count, stream.stats.dropped, g_blackbox_stats.uart_errors, stream.stats.resyncs);
                batch_blocks++; block_count = 0; continue;
            }
            if (!BB_StreamPop(&stream, &record)) break;
            /* Keep queue-loss gaps visible in the outer record sequence too.
             * Prefix SCHEMA records add an offset, not a renumbering that can
             * hide seconds of missing captured DATA in a sequence-based plot. */
            if(have_capture_sequence)
                next_record_sequence+=(uint32_t)(record.sequence-last_capture_sequence)-1U;
            last_capture_sequence=record.sequence;have_capture_sequence=1;
            record.sequence=next_record_sequence++;
            (void)BB_BlockRecord(b, block_count++, &record);
        }
        /* Flush a partial batch by the configured latency deadline. Never
         * assemble beyond committed capacity or a cluster/file boundary. */
        if (batch_blocks == limit || (uint32_t)(now - batch_started) >= BB_BATCH_FLUSH_MS) {
            if (block_count) {
                b = BLOCK + batch_blocks * BB_BLOCK_SIZE;
                BB_BlockFinish(b, block_count, stream.stats.dropped, g_blackbox_stats.uart_errors, stream.stats.resyncs);
                batch_blocks++; block_count = 0;
            }
            pending = batch_blocks != 0;
        }
    }
    if (pending) {
        /* Direct sequential writes already leave fptr at the next block.
         * Seek only after recovery/reads displaced it. */
        if (f_tell(&file) != write_offset && !check(f_lseek(&file, write_offset), BB_ERROR_WRITE)) return;
        if (!write_bytes(BLOCK, batch_blocks * BB_BLOCK_SIZE)) return;
        while (batch_blocks) acknowledge();
    }
    if (write_offset != synced_offset && (uint32_t)(HAL_GetTick() - last_sync) >= BB_DATA_SYNC_MS && !sync_data()) return;
    if (g_blackbox_stats.state == BLACKBOX_FAULT || g_blackbox_stats.state == BLACKBOX_RECOVERING) return;
    update_activity();
    update_stats();
}
