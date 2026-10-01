#include <assert.h>
#include <stdio.h>
#include "../../App/optical_flow.c"
#include "../../App/vofa.c"

static uint32_t dma_write;
void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *u) { (void)u; }
static void boot(void)
{
    test_tick=0; test_rx_status=test_rx_abort_status=HAL_OK;
    OpticalFlow_Init(HAL_OK); OpticalFlow_ProcessPending(); dma_write=0;
    assert(g_optical_flow_stats.receiving);
}
static void deliver(const uint8_t *p, unsigned n, int process)
{
    unsigned i;
    for(i=0;i<n;i++) {
        of_dma[dma_write++]=p[i]; dma_write%=256;
        test_rx_remaining=256-dma_write;
        if(dma_write==0 || dma_write==128) OpticalFlow_OnRxEvent(&huart10);
    }
    OpticalFlow_OnRxEvent(&huart10);
    OpticalFlow_OnRxEvent(&huart10); /* duplicate IDLE/HT uses live position */
    if(process) OpticalFlow_ProcessPending();
}
static void checksum(uint8_t *p, unsigned n)
{
    unsigned i; p[n-1]=0;
    for(i=0;i<n-1;i++) p[n-1]=(uint8_t)(p[n-1]+p[i]);
}
static void sample(uint8_t *p)
{
    uint8_t f[27]={0xef,0x81,0x92,0x51,0xfe,20,
        0xff,0xff,0xff,0xff, 0xfe,0xff,0xff,0xff,
        7,8,9,10, 0x00,0x80, 0xff,0x7f, 13,14,0x34,0x12,0};
    memcpy(p,f,27); checksum(p,27);
}
int main(int argc, char **argv)
{
    uint8_t good[27], bad[71], wire[116], display[96];
    OpticalFlowData_t d;
    FlightControlState_t flight={0}; RcSbusData_t rc={0}; EscPwmState_t esc={0};
    unsigned i, split;
    uint32_t count;
    FILE *out;
    sample(good);
    for(split=1;split<27;split++) {
        boot(); deliver(good,split,1); assert(!g_optical_flow_data.has_sample);
        test_tick=42; deliver(good+split,27-split,1);
        OpticalFlow_GetData(&d);
        assert(d.rx_count==1 && d.received_tick==42 && d.age_ms==0);
        assert(d.time_ms==UINT32_MAX && d.distance==UINT32_MAX-1);
        assert(d.flow_vel_x==-32768 && d.flow_vel_y==32767);
        assert(d.dev_id==0x81 && d.sys_id==0x92 && d.seq==0xfe);
        assert(d.strength==7 && d.precision==8 && d.tof_status==9 && d.reserved1==10);
        assert(d.flow_quality==13 && d.flow_status==14 && d.reserved2==0x1234);
        assert(!memcmp(d.raw,good,27));
    }
    boot();
    for(i=0;i<200;i++) { deliver(good,27,1); }
    assert(g_optical_flow_data.rx_count==200 && g_optical_flow_stats.byte_count==5400);
    memcpy(bad,good,27); bad[26]^=1; deliver(bad,27,1);
    assert(g_optical_flow_stats.checksum_errors && g_optical_flow_data.rx_count==200);
    memset(bad,0x55,sizeof(bad)); deliver(bad,71,1);
    memcpy(bad,good,27); bad[5]=255; deliver(bad,27,1);
    assert(g_optical_flow_stats.length_errors);
    memcpy(bad,good,27); bad[3]=0x52; checksum(bad,27); deliver(bad,27,1);
    assert(g_optical_flow_data.rx_count==200);
    deliver(good,27,1); assert(g_optical_flow_data.rx_count==201);
    /* Zero distance is preserved; not reclassified as a protocol failure. */
    memcpy(bad,good,27); memset(bad+10,0,4); checksum(bad,27); deliver(bad,27,1);
    assert(g_optical_flow_data.distance==0 && g_optical_flow_data.rx_count==202);
    test_tick+=500; OpticalFlow_GetData(&d); assert(d.age_ms==500);
    /* Overflow invalidates partial data and retries in task context. */
    count=g_optical_flow_data.rx_count;
    for(i=0;i<40;i++) deliver(good,27,0);
    assert(g_optical_flow_stats.queue_overflows==1);
    OpticalFlow_ProcessPending(); dma_write=0;
    assert(g_optical_flow_data.rx_count==count);
    deliver(good,27,1); assert(g_optical_flow_data.rx_count==count+1);
    huart10.ErrorCode=8; OpticalFlow_OnError(&huart10);
    test_rx_status=HAL_BUSY; OpticalFlow_ProcessPending();
    count=g_optical_flow_stats.restart_count;
    test_tick+=99; OpticalFlow_ProcessPending(); assert(g_optical_flow_stats.restart_count==count);
    test_tick++; test_rx_status=HAL_OK; OpticalFlow_ProcessPending(); dma_write=0;
    assert(g_optical_flow_stats.receiving && g_optical_flow_stats.restart_count==count+1);
    deliver(good,27,1); assert(g_optical_flow_data.rx_count==203+1);
    count=g_optical_flow_stats.uart_errors; OpticalFlow_OnError(&huart5);
    OpticalFlow_OnRxEvent(&huart7); assert(g_optical_flow_stats.uart_errors==count);
    test_rx_abort_status=HAL_TIMEOUT; OpticalFlow_OnError(&huart10); OpticalFlow_ProcessPending();
    assert(!g_optical_flow_stats.receiving);
    /* Tick wrap and >49 day stale saturation: no stale sample resurrection. */
    boot(); test_tick=0xfffffff0U; deliver(good,27,1);
    test_tick=0x10; OpticalFlow_GetData(&d); assert(d.age_ms==32);
    test_tick=0xfffffff0U; OpticalFlow_ProcessPending();
    test_tick=0x20; OpticalFlow_GetData(&d); assert(d.age_ms==UINT32_MAX);
    deliver(good,27,1); OpticalFlow_GetData(&d); assert(d.age_ms==0);
    OpticalFlow_Init(HAL_ERROR); OpticalFlow_ProcessPending(); assert(!g_optical_flow_stats.receiving);
    /* An incomplete oversized unknown candidate must resync to the embedded
     * complete known frame after checksum failure, retaining its own RX tick. */
    boot(); memset(bad,0,71); bad[0]=0xef; bad[3]=0x52; bad[5]=64;
    memcpy(bad+6,good,27); bad[70]=0x11;
    test_tick=100; deliver(bad,33,1); assert(!g_optical_flow_data.has_sample);
    test_tick=110; deliver(bad+33,38,1); OpticalFlow_GetData(&d);
    assert(d.has_sample && d.received_tick==100 && d.age_ms==10);
    /* Several complete packets in one DMA event, not just one packet/event. */
    boot(); memcpy(bad,good,27); memcpy(bad+27,good,27); deliver(bad,54,1);
    assert(g_optical_flow_data.rx_count==2);
    /* Export production-built absent/present/repeated wire frames for F407 FatFs tests. */
    assert(argc==2); out=fopen(argv[1],"wb"); assert(out);
    boot(); OpticalFlow_GetData(&d);
    for(i=0;i<4;i++) esc.command[i]=(float)(i+1)*0.1f;
    VOFA_BuildBlackboxFrame(wire,&flight,&rc,&esc,&d);
    assert(wire[111]==0 && wire[107]==255);
    assert(fwrite(wire,1,116,out)==116);
    deliver(good,27,1); test_tick=7; OpticalFlow_GetData(&d);
    VOFA_BuildBlackboxFrame(wire,&flight,&rc,&esc,&d);
    VOFA_BuildFlightFrame(display,&flight,&rc,&esc,&d);
    assert(!memcmp(wire,display,80) && !memcmp(wire+80,good,27));
    assert(wire[107]==7 && wire[111]==1 && !memcmp(wire+112,display+92,4));
    { float f[3]; memcpy(f,display+80,12);
      assert(f[0]==(float)(UINT32_MAX-1) && f[1]==-32768 && f[2]==32767); }
    assert(fwrite(wire,1,116,out)==116);
    test_tick=19; OpticalFlow_GetData(&d);
    VOFA_BuildBlackboxFrame(wire,&flight,&rc,&esc,&d);
    assert(wire[107]==19 && !memcmp(wire+80,good,27));
    assert(fwrite(wire,1,116,out)==116); fclose(out);
    assert(test_critical_depth==0);
    puts("PASS: optical parser/DMA/queue/recovery/age/telemetry, H723 cross-project wire fixture exported");
    return 0;
}
