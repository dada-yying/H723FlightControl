#include "blackbox_stream.h"
#include <string.h>
#if defined(BB_HOST_TEST)
#define BB_BARRIER() ((void)0)
#else
#include "stm32h7xx.h"
#define BB_BARRIER() __DMB()
#endif
static unsigned record_size(unsigned n) { return (BB_RECORD_HEADER+n+3U)&~3U; }
void BB_StreamInit(BB_Stream *s) { memset(s,0,sizeof(*s)); s->capacity=BB_QUEUE_SRAM_BYTES; }
void BB_StreamInitExtended(BB_Stream *s, uint8_t *extension)
{
    BB_StreamInit(s); s->extension=extension;
    if(extension)s->capacity=BB_QUEUE_BYTES;
}
void BB_StreamResetParser(BB_Stream *s) { s->history_count=0; }
static void ring_write(BB_Stream *s,const uint8_t *p,unsigned n)
{
    while(n) {
        unsigned pos=s->write_index, end=pos<BB_QUEUE_SRAM_BYTES?BB_QUEUE_SRAM_BYTES:s->capacity;
        unsigned chunk=end-pos;uint8_t *dst;
        if(chunk>n)chunk=n;
        dst=pos<BB_QUEUE_SRAM_BYTES?s->records+pos:s->extension+pos-BB_QUEUE_SRAM_BYTES;
        memcpy(dst,p,chunk);p+=chunk;n-=chunk;s->write_index+=chunk;
        if(s->write_index==s->capacity)s->write_index=0;
    }
}
static void ring_read(const BB_Stream *s,unsigned pos,uint8_t *p,unsigned n)
{
    while(n) {
        unsigned end=pos<BB_QUEUE_SRAM_BYTES?BB_QUEUE_SRAM_BYTES:s->capacity;
        unsigned chunk=end-pos;const uint8_t *src;
        if(chunk>n)chunk=n;
        src=pos<BB_QUEUE_SRAM_BYTES?s->records+pos:s->extension+pos-BB_QUEUE_SRAM_BYTES;
        memcpy(p,src,chunk);p+=chunk;n-=chunk;pos+=chunk;
        if(pos==s->capacity)pos=0;
    }
}
static int emit(BB_Stream *s,unsigned n,uint64_t us)
{
    uint8_t h[16]={0},pad[3]={0};unsigned bytes=record_size(n),used=s->head-s->tail;
    uint32_t seq=s->stats.frames++;
    if(bytes>s->capacity-used){s->stats.dropped++;return 0;}
    sbb_p64(h,us);sbb_p32(h+8,seq);sbb_p16(h+12,(uint16_t)n);
    ring_write(s,h,16);ring_write(s,s->history,n);ring_write(s,pad,bytes-16-n);
    BB_BARRIER();s->head+=bytes;
    if(used+bytes>s->stats.high_water)s->stats.high_water=used+bytes;
    return 1;
}
void BB_StreamFeed(BB_Stream *s,const uint8_t *data,size_t size,uint64_t us)
{
    while(size--) {
        s->history[s->history_count++]=*data++;
        for(;;) {
            unsigned n;
            if(s->history_count<4)break;
            if(memcmp(s->history,"SBB3",4))goto discard;
            if(s->history_count<8)break;
            n=28U+sbb_u16(s->history+6);
            if(s->history[4]!=3 || (s->history[5]!=SBB_SCHEMA && s->history[5]!=SBB_DATA) || n>512) {
                s->stats.bad_packets++;goto discard;
            }
            if(s->history_count<n)break;
            if(!sbb_valid(s->history,n)){s->stats.bad_packets++;goto discard;}
            emit(s,n,us);
            s->history_count-=n;memmove(s->history,s->history+n,s->history_count);continue;
discard:
            s->stats.resyncs++;s->history_count--;memmove(s->history,s->history+1,s->history_count);
        }
    }
}
uint32_t BB_StreamAvailable(const BB_Stream *s){return s->head-s->tail;}
uint16_t BB_StreamPeekSize(const BB_Stream *s)
{
    uint8_t h[16];if(s->head==s->tail)return 0;BB_BARRIER();ring_read(s,s->read_index,h,16);
    return (uint16_t)record_size(sbb_u16(h+12));
}
int BB_StreamPop(BB_Stream *s,BB_Record *r)
{
    uint8_t h[16];unsigned bytes,pos;
    if(s->head==s->tail)return 0;BB_BARRIER();ring_read(s,s->read_index,h,16);
    r->time_us=(uint64_t)sbb_u32(h)|((uint64_t)sbb_u32(h+4)<<32);r->sequence=sbb_u32(h+8);r->length=sbb_u16(h+12);
    bytes=record_size(r->length);pos=(s->read_index+16)%s->capacity;ring_read(s,pos,r->frame,r->length);
    BB_BARRIER();s->read_index=(s->read_index+bytes)%s->capacity;s->tail+=bytes;return 1;
}
void BB_StreamDmaDrain(BB_Stream *s,const uint8_t *dma,uint32_t size,uint32_t *cursor,uint32_t pos,uint64_t us)
{
    uint32_t old=*cursor;if(!size||pos>size||old>=size)return;pos%=size;if(old==pos)return;
    if(pos<old){BB_StreamFeed(s,dma+old,size-old,us);old=0;}BB_StreamFeed(s,dma+old,pos-old,us);*cursor=pos;
}
uint64_t BB_ClockExtend(BB_Clock *c,uint32_t n){c->value+=(uint32_t)(n-c->last);c->last=n;return c->value;}
uint32_t BB_Crc32(const uint8_t *p,size_t n){return sbb_crc(p,n);}
void BB_BlockBegin(uint8_t *b,const uint8_t session[16],uint32_t number,uint32_t file,uint32_t flags)
{
    memset(b,0,BB_BLOCK_SIZE);memcpy(b,"SKYBBX3",8);sbb_p16(b+8,3);sbb_p16(b+10,64);memcpy(b+12,session,16);
    sbb_p32(b+28,number);sbb_p32(b+48,file);sbb_p32(b+52,flags);
}
int BB_BlockRecord(uint8_t *b,uint32_t index,const BB_Record *r)
{
    unsigned used=sbb_u16(b+34),bytes=record_size(r->length);uint8_t *p=b+64+used;
    (void)index;
    if(r->length<28 || r->length>512 || used+bytes>4032)return 0;
    sbb_p64(p,r->time_us);sbb_p32(p+8,r->sequence);sbb_p16(p+12,r->length);sbb_p16(p+14,0);
    memcpy(p+16,r->frame,r->length);memset(p+16+r->length,0,bytes-16-r->length);
    sbb_p16(b+34,(uint16_t)(used+bytes));return 1;
}
void BB_BlockFinish(uint8_t *b,uint16_t count,uint32_t drops,uint32_t errors,uint32_t resyncs)
{
    sbb_p16(b+32,count);sbb_p32(b+36,drops);sbb_p32(b+40,errors);sbb_p32(b+44,resyncs);sbb_p32(b+60,0);sbb_p32(b+60,sbb_crc(b,4096));
}

/* Complete local SBB3 packet: no byte-stream scanning needed. Single producer. */
int BB_StreamEnqueue(BB_Stream *s, const uint8_t *p, uint16_t n, uint64_t us)
{
    if(n < 28 || n > BB_FRAME_SIZE || !sbb_valid(p,n)) {s->stats.bad_packets++;return 0;}
    memcpy(s->history,p,n);
    return emit(s,n,us);
}
