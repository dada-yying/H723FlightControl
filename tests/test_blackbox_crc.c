/* Compare optimized H723 CRCs against independent bitwise implementations,
 * including the exact unselected shared implementation used by F407. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "../../references/blackbox_protocol/sbb3.h"
#include "../App/storage/blackbox_crc.h"
static uint16_t reference16(const uint8_t *p,size_t n)
{
    uint16_t c=0;unsigned b;
    while(n--){c^=(uint16_t)*p++<<8;for(b=0;b<8;b++)c=(uint16_t)((c<<1)^((c&0x8000)?0x1021:0));}
    return c;
}
static volatile uint32_t benchmark_sink;
int main(void)
{
    uint8_t data[4096+32];size_t n,offset;unsigned i,pattern;
    static const size_t lengths[]={0,1,2,3,15,16,17,31,32,33,229,233,512,514,516,1024,4096};
    const uint8_t vector[]="123456789";
    uint32_t random=0x42f38195U;
    assert(BlackboxCrc32(vector,9)==0xcbf43926U && BlackboxCrc16(vector,9)==0x31c3);
    assert(BlackboxCrc32(NULL,0)==0 && BlackboxCrc16(NULL,0)==0);
    for(i=0;i<256;i++){
        uint8_t v=(uint8_t)i;
        assert(BlackboxCrc32(&v,1)==sbb_crc(&v,1));
        assert(BlackboxCrc16(&v,1)==reference16(&v,1));
    }
    for(pattern=0;pattern<4;pattern++){
        for(i=0;i<sizeof(data);i++){
            random^=random<<13;random^=random>>17;random^=random<<5;
            data[i]=pattern==0?0:pattern==1?0xff:pattern==2?(uint8_t)i:(uint8_t)random;
        }
        for(offset=0;offset<32;offset++)for(i=0;i<sizeof(lengths)/sizeof(lengths[0]);i++){
            n=lengths[i];
            assert(BlackboxCrc32(data+offset,n)==sbb_crc(data+offset,n));
            assert(BlackboxCrc16(data+offset,n)==reference16(data+offset,n));
        }
        for(n=0;n<=516;n++){
            assert(BlackboxCrc32(data,n)==sbb_crc(data,n));
            assert(BlackboxCrc16(data,n)==reference16(data,n));
        }
    }
    { /* Host-only timing, never used as an MCU throughput acceptance test. */
        uint32_t (*volatile fast32)(const uint8_t *,size_t)=BlackboxCrc32;
        uint32_t (*volatile slow32)(const uint8_t *,size_t)=sbb_crc;
        clock_t start=clock(),fast_time,slow_time;
        for(i=0;i<2000;i++)benchmark_sink^=fast32(data,4096);
        fast_time=clock()-start;start=clock();
        for(i=0;i<2000;i++)benchmark_sink^=slow32(data,4096);
        slow_time=clock()-start;
        printf("HOST ONLY CRC32 8MB: table %.3fs, reference %.3fs (not MCU performance)\n",
               (double)fast_time/CLOCKS_PER_SEC,(double)slow_time/CLOCKS_PER_SEC);
    }
    puts("PASS CRC32/SD-CRC16: known vectors, every byte, all short lengths, 0/FF/ramp/random patterns, 32 unaligned offsets, 4KiB blocks, unchanged F407 reference");
    return 0;
}
