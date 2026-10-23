/* Real H723 queue, file recovery and FatFs against RAM media; no physical card. */
#include "stm32h7xx_hal.h"
#include "../App/storage/blackbox_storage.h"
#include "../App/blackbox.h"
#include "../App/storage/blackbox_disk.h"
#include "ff.h"
#include "diskio.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
FRESULT InjectWrite(FIL *,const void *,UINT,UINT *);
FRESULT InjectOpen(FIL *,const TCHAR *,BYTE);
#define f_write InjectWrite
#define f_open InjectOpen
#include "../App/storage/blackbox_storage.c"
#undef f_write
#undef f_open
#include "../App/storage/blackbox_stream.c"
#define SECTORS 131072U
#define FATS 1024U
#define DATA_START (32+2*FATS)
static uint8_t *media;
static uint8_t definition_bytes[8192],packet[512];
static uint16_t definition_size,packet_size;
static unsigned disk_fail,partial_fault,sync_fail,init_fail,packet_sequence,inject_count;
static unsigned create_fail;
volatile BlackboxTxStats_t g_blackbox_tx_stats;
volatile uint32_t g_bb_disk_error,g_bb_disk_max_ms,g_bb_disk_init_failed;
volatile uint32_t g_bb_disk_dma_max_ms,g_bb_disk_busy_max_ms,g_bb_disk_init_max_ms;
uint16_t Blackbox_SchemaSize(void){return definition_size;}
const uint8_t *Blackbox_SchemaData(void){return definition_bytes;}
void BB_DiskReset(void){g_bb_disk_error=g_bb_disk_init_failed=0;}
int BB_CardPresent(void){return 1;}
void BB_CardIdentity(uint32_t c[4]){memset(c,0,16);c[0]=1234;}
HAL_StatusTypeDef BB_RandomSession(uint8_t *s){memset(s,0x42,16);return HAL_OK;}
static void capture(void)
{
    sbb_packet(packet,SBB_DATA,packet_size-28,g_blackbox_tx_stats.schema_id,packet_sequence++,(uint64_t)test_tick*1000);
    (void)BlackboxStorage_Enqueue(packet,packet_size,(uint64_t)test_tick*1000);
}
DSTATUS disk_initialize(BYTE d){if(init_fail){init_fail--;g_bb_disk_init_failed=1;return STA_NOINIT;}return d?STA_NOINIT:0;}
DSTATUS disk_status(BYTE d){return d?STA_NOINIT:0;}
DRESULT disk_read(BYTE d,BYTE *p,DWORD s,UINT n){assert(!d && s+n<=SECTORS);if(disk_fail)return RES_ERROR;memcpy(p,media+s*512,n*512);return RES_OK;}
DRESULT disk_write(BYTE d,const BYTE *p,DWORD s,UINT n){
    unsigned count=inject_count;inject_count=0;while(count--){test_tick++;capture();}
    assert(!d && s+n<=SECTORS);if(disk_fail){g_bb_disk_error=BB_DISK_TIMEOUT;return RES_ERROR;}
    memcpy(media+s*512,p,n*512);return RES_OK;
}
DRESULT disk_ioctl(BYTE d,BYTE cmd,void *p){(void)d;(void)p;assert(cmd==CTRL_SYNC);if(sync_fail){sync_fail--;return RES_ERROR;}return disk_fail?RES_ERROR:RES_OK;}
FRESULT InjectWrite(FIL *f,const void *p,UINT n,UINT *out){
    if(partial_fault){unsigned mode=partial_fault;FRESULT r;partial_fault=0;r=f_write(f,p,mode==1?512:n,out);assert(r==FR_OK);g_bb_disk_error=BB_DISK_TIMEOUT;return FR_DISK_ERR;}
    return f_write(f,p,n,out);
}
FRESULT InjectOpen(FIL *f,const TCHAR *p,BYTE mode){
    if((mode&FA_CREATE_NEW) && create_fail){unsigned k=create_fail;create_fail=0;if(k==2)assert(f_open(f,p,mode)==FR_OK);return FR_DISK_ERR;}
    return f_open(f,p,mode);
}
static void format_ram(int full)
{
    unsigned i,k;free(media);media=calloc(SECTORS,512);assert(media);
    memcpy(media,"\xeb\x58\x90MSDOS5.0",11);sbb_p16(media+11,512);media[13]=1;sbb_p16(media+14,32);media[16]=2;
    media[21]=0xf8;sbb_p32(media+32,SECTORS);sbb_p32(media+36,FATS);sbb_p32(media+44,2);
    sbb_p16(media+48,1);sbb_p16(media+50,6);media[66]=0x29;memcpy(media+82,"FAT32   ",8);sbb_p16(media+510,0xaa55);
    for(k=0;k<2;k++){uint8_t *fat=media+(32+k*FATS)*512;sbb_p32(fat,0x0ffffff8);sbb_p32(fat+4,0xffffffff);sbb_p32(fat+8,0x0fffffff);
        if(full)for(i=3;i<SECTORS-DATA_START+2;i++)sbb_p32(fat+4*i,0x0fffffff);}
}
static void pump(unsigned n){while(n--){test_tick++;BlackboxStorage_Process();}}
static void boot(void){f_mount(NULL,"0:",0);test_tick=100;disk_fail=partial_fault=sync_fail=init_fail=inject_count=0;BlackboxStorage_Init();pump(300);assert(schema_offset==definition_size);}
static void load_wire(const char *path)
{
    FILE *f=fopen(path,"rb");uint8_t p[512];unsigned n;assert(f);
    while(fread(p,1,8,f)==8){n=28+sbb_u16(p+6);assert(n<=512 && fread(p+8,1,n-8,f)==n-8 && sbb_valid(p,n));
        g_blackbox_tx_stats.schema_id=sbb_u32(p+8);
        if(p[5]==SBB_SCHEMA){definition_size=sbb_u16(p+24);assert(definition_size<=8192);memcpy(definition_bytes+sbb_u16(p+26),p+28,n-32);}
        else {packet_size=(uint16_t)n;memcpy(packet,p,n);}}
    fclose(f);assert(definition_size && packet_size && sbb_crc(definition_bytes,definition_size)==g_blackbox_tx_stats.schema_id);
}
static void validate_file(void)
{
    FIL f;UINT n;uint8_t b[4096];uint32_t seq=0,blocks=0,data=0;
    assert(f_open(&f,current_path,FA_READ)==FR_OK);
    while(f_read(&f,b,4096,&n)==FR_OK && n==4096){unsigned i,pos=64,count=sbb_u16(b+32);uint32_t crc;
        if(memcmp(b,"SKYBBX3",8))break;
        crc=sbb_u32(b+60);sbb_p32(b+60,0);assert(sbb_crc(b,4096)==crc);
        assert(sbb_u32(b+28)==blocks++);
        for(i=0;i<count;i++){unsigned len=sbb_u16(b+pos+12),actual=sbb_u32(b+pos+8);
            if(blocks==2 && i==0)seq=actual;
            assert((uint32_t)(actual-seq)<0x80000000U);
            if(!stream.stats.dropped)assert(actual==seq);
            seq=actual+1;assert(sbb_valid(b+pos+16,len));
            if(b[pos+21]==SBB_DATA){assert(len==packet_size);assert(!memcmp(b+pos+40,packet+24,len-28));data++;}
            pos+=(16+len+3)&~3U;}
    }
    f_close(&f);printf("validate file %u data %u total %u blocks %u\n",s_storage_stats.file_number,data,s_storage_stats.data_written,blocks);
    assert(s_storage_stats.file_number>1 || data==s_storage_stats.data_written);
}
static void export_log(const char *path)
{
    FIL f;UINT n;uint8_t b[4096];FILE *out=fopen(path,"wb");assert(out);
    assert(f_open(&f,current_path,FA_READ)==FR_OK);
    do{assert(f_read(&f,b,sizeof(b),&n)==FR_OK);assert(fwrite(b,1,n,out)==n);}while(n);
    f_close(&f);fclose(out);
}
int main(int argc,char **argv)
{
    unsigned i,before,file_no;assert(argc==3);load_wire(argv[1]);
    /* Actual production reserve sizes: DATA can start after 4MiB, without
     * waiting for the entire 40MiB target. Other host variants use tiny files. */
    if(BB_FILE_BYTES>BB_INITIAL_COMMIT_BYTES){
        unsigned limit=5000;
        format_ram(0);f_mount(NULL,"0:",0);test_tick=100;BlackboxStorage_Init();
        while(!s_storage_stats.ready_tick_ms && limit--)pump(1);
        assert(limit && allocated>=BB_INITIAL_COMMIT_BYTES && allocated<BB_FILE_BYTES);
        for(i=0;i<150;i++)capture();pump(1);
        assert(s_storage_stats.data_written>0 && allocated<BB_FILE_BYTES);
        puts("PASS logging starts with committed 4MiB while 40MiB reservation remains in background");
        free(media);return 0;
    }
    format_ram(0);boot();
    for(i=0;i<2000;i++){capture();pump(1);}pump(500);
    assert(s_storage_stats.data_written==2000 && s_storage_stats.file_number==1);validate_file();export_log(argv[2]);
    /* Complete or partial failed batch: retry same file without duplicate samples. */
    for(i=1;i<=2;i++){
        format_ram(0);boot();before=s_storage_stats.data_written;file_no=s_storage_stats.file_number;
        for(unsigned j=0;j<150;j++)capture();partial_fault=i;pump(5000);
        assert(s_storage_stats.file_number==file_no && s_storage_stats.recovery_count && s_storage_stats.data_written==before+150);validate_file();
    }
    format_ram(0);boot();for(i=0;i<200;i++){capture();pump(1);}sync_fail=1;pump(5000);
    assert(s_storage_stats.file_number==1 && s_storage_stats.recovery_count);validate_file();
    /* Every queued old packet retained when producer overruns stalled SD. */
    format_ram(0);boot();disk_fail=1;for(i=0;i<2000;i++){capture();pump(1);}
    assert(stream.stats.dropped>0 && stream.stats.high_water<=BB_QUEUE_BYTES);
    disk_fail=0;pump(6000);assert(s_storage_stats.file_number==1);validate_file();
    {unsigned missing=stream.stats.dropped;
     uint32_t old_sequence=next_record_sequence;
     capture();pump(500);
     assert(next_record_sequence==old_sequence+missing+1);
     validate_file();}
    /* Good reads prove descriptor corruption before a new file is permitted. */
    for(i=DATA_START;i<SECTORS;i++)if(!memcmp(media+i*512,"SKYBBX3",8) && sbb_u32(media+i*512+28)==0){media[i*512+60]^=1;break;}
    assert(i<SECTORS);fault(BB_ERROR_WRITE,FR_DISK_ERR);pump(15000);assert(s_storage_stats.file_number==2);
    capture();pump(500);assert(schema_offset==definition_size);validate_file();
    /* Card full is terminal for this power session. */
    format_ram(1);f_mount(NULL,"0:",0);BlackboxStorage_Init();pump(1000);
    assert(s_storage_stats.stopped && s_storage_stats.state==BLACKBOX_SPACE_WAIT);
    /* Transient initialisation failure does not become fatal. */
    format_ram(0);f_mount(NULL,"0:",0);BlackboxStorage_Init();init_fail=2;pump(10000);
    assert(s_storage_stats.file_number==1 && !s_storage_stats.stopped && schema_offset==definition_size);
    for(i=1;i<=2;i++){
        format_ram(0);f_mount(NULL,"0:",0);BlackboxStorage_Init();create_fail=i;pump(10000);
        assert(s_storage_stats.file_number==1 && schema_offset==definition_size && s_storage_stats.recovery_count);
    }
    write_offset=BB_MAX_FILE_BYTES;BlackboxStorage_Process();
    assert(s_storage_stats.stopped && s_storage_stats.file_number==1);
    accepting=1; /* Isolate the ring test from the deliberately stopped logger. */
    /* Single-producer ring, physical and uint32 counter wrap. */
    BB_StreamInit(&stream);stream.head=stream.tail=UINT32_MAX-100;
    for(i=0;i<4000;i++){BB_Record r;capture();assert(BB_StreamPop(&stream,&r));assert(r.length==packet_size && sbb_valid(r.frame,r.length));}
    assert(!BB_StreamAvailable(&stream));
    /* A replacement-file schema may shift pending records more than once.
     * Their original gap must survive, including uint32 capture wrap. */
    {BB_Record r;uint32_t second=64U+((16U+packet_size+3U)&~3U),crc;
     memset(&r,0,sizeof(r));r.length=packet_size;memcpy(r.frame,packet,packet_size);
     BB_BlockBegin(BLOCK,session,1,1,0);r.sequence=UINT32_MAX-15U;
     assert(BB_BlockRecord(BLOCK,0,&r));r.sequence=16;
     assert(BB_BlockRecord(BLOCK,1,&r));BB_BlockFinish(BLOCK,2,0,0,0);
     batch_blocks=1;block_count=0;prefix_records=100;reframe_pending();
     assert(sbb_u32(BLOCK+72)==100 && sbb_u32(BLOCK+second+8)==132);
     prefix_records=200;reframe_pending();
     assert(sbb_u32(BLOCK+72)==200 && sbb_u32(BLOCK+second+8)==232);
     crc=sbb_u32(BLOCK+60);sbb_p32(BLOCK+60,0);assert(sbb_crc(BLOCK,4096)==crc);}
    free(media);
    puts("PASS H723 actual FatFs: packet preservation, CRC, queue wrap/full, partial/complete writes, sync failure, same-file recovery, corrupt-file switch, card full, init retry");return 0;
}

uint32_t Blackbox_SchemaId(void) { return g_blackbox_tx_stats.schema_id; }
uint32_t BB_DiskError(void) { return g_bb_disk_error; }
uint32_t BB_DiskMaxMs(void) { return g_bb_disk_max_ms; }
uint32_t BB_DiskInitFailed(void) { return g_bb_disk_init_failed; }
uint32_t BB_DiskDmaMaxMs(void) { return g_bb_disk_dma_max_ms; }
uint32_t BB_DiskBusyMaxMs(void) { return g_bb_disk_busy_max_ms; }
uint32_t BB_DiskInitMaxMs(void) { return g_bb_disk_init_max_ms; }
