#ifndef BLACKBOX_FIELDS_H
#define BLACKBOX_FIELDS_H
#include "../../references/blackbox_protocol/sbb3.h"
#include "vofa.h"
/* Add fields here and populate their bytes in Blackbox_Sample below.
 * Offsets may overlap for decoded views of an existing raw byte region.
 * Payload size and schema ID are derived, never hard-coded by the receiver. */
typedef struct { const char *name; const char *unit; uint16_t offset,count; uint8_t type; } BlackboxField;
#define BF(n,u,o,c,t) {n,u,o,c,t}
static const BlackboxField s_fields[] = {
    BF("roll_deg","deg",0,1,SBB_F32), BF("pitch_deg","deg",4,1,SBB_F32), BF("yaw_deg","deg",8,1,SBB_F32),
    BF("throttle","1",12,1,SBB_F32), BF("roll_target_deg","deg",16,1,SBB_F32), BF("pitch_target_deg","deg",20,1,SBB_F32),
    BF("yaw_rate_target_deg_s","deg/s",24,1,SBB_F32),
    BF("roll_rate_target_deg_s","deg/s",28,1,SBB_F32), BF("roll_rate_deg_s","deg/s",32,1,SBB_F32),
    BF("pitch_rate_target_deg_s","deg/s",36,1,SBB_F32), BF("pitch_rate_deg_s","deg/s",40,1,SBB_F32),
    BF("yaw_control_rate_target_deg_s","deg/s",44,1,SBB_F32), BF("yaw_rate_deg_s","deg/s",48,1,SBB_F32),
    BF("roll_rate_raw_deg_s","deg/s",52,1,SBB_F32), BF("pitch_rate_raw_deg_s","deg/s",56,1,SBB_F32), BF("yaw_rate_raw_deg_s","deg/s",60,1,SBB_F32),
    BF("motor_command_0","1",64,1,SBB_F32), BF("motor_command_1","1",68,1,SBB_F32), BF("motor_command_2","1",72,1,SBB_F32), BF("motor_command_3","1",76,1,SBB_F32),
    BF("of_raw","",80,27,SBB_BYTES), BF("of_head","",80,1,SBB_U8), BF("of_dev_id","",81,1,SBB_U8),
    BF("of_sys_id","",82,1,SBB_U8), BF("of_msg_id","",83,1,SBB_U8), BF("of_seq","",84,1,SBB_U8), BF("of_len","byte",85,1,SBB_U8),
    BF("of_time_ms","ms",86,1,SBB_U32), BF("of_distance_mm","mm",90,1,SBB_U32), BF("of_strength","",94,1,SBB_U8),
    BF("of_precision","",95,1,SBB_U8), BF("of_tof_status","",96,1,SBB_U8), BF("of_reserved1","",97,1,SBB_U8),
    BF("of_flow_x","cm/s@1m",98,1,SBB_I16), BF("of_flow_y","cm/s@1m",100,1,SBB_I16),
    BF("of_flow_quality","",102,1,SBB_U8), BF("of_flow_status","",103,1,SBB_U8), BF("of_reserved2","",104,1,SBB_U16),
    BF("of_checksum","",106,1,SBB_U8), BF("of_age_ms","ms",107,1,SBB_U32), BF("of_has_sample","",111,1,SBB_U8),
    BF("fc_timestamp_us","us",112,1,SBB_U64),
#ifdef BLACKBOX_TEST_EXTRA_FIELDS
    BF("new_float","test",120,1,SBB_F32), BF("new_uint32","test",124,1,SBB_U32), BF("new_array","test",128,3,SBB_I16),
#endif
};
#undef BF
static uint8_t s_schema[SBB_SCHEMA_MAX];
static uint16_t s_schema_size,s_payload_size;
static uint32_t s_schema_id;
static int Blackbox_CreateSchema(void)
{
    unsigned i,j,n,u,size,end,pos=4,count=sizeof(s_fields)/sizeof(s_fields[0]);
    s_payload_size=0;
    if(count>SBB_FIELDS_MAX)return 0;
    for(i=0;i<count;i++) {
        const BlackboxField *f=&s_fields[i];
        n=(unsigned)strlen(f->name);u=(unsigned)strlen(f->unit);size=sbb_type_size(f->type);
        end=f->offset+f->count*size;
        if(!n || n>63 || u>31 || !size || !f->count || end>SBB_MAX_PAYLOAD || pos+8+n+u>SBB_SCHEMA_MAX)return 0;
        for(j=0;j<i;j++)if(!strcmp(f->name,s_fields[j].name))return 0;
        for(j=0;j<n;j++)if(!((f->name[j]>='a'&&f->name[j]<='z')||(f->name[j]>='A'&&f->name[j]<='Z')||(f->name[j]>='0'&&f->name[j]<='9')||f->name[j]=='_'))return 0;
        for(j=0;j<u;j++)if((unsigned char)f->unit[j]<32 || (unsigned char)f->unit[j]>126)return 0;
        if(end>s_payload_size)s_payload_size=(uint16_t)end;
        s_schema[pos]=f->type;s_schema[pos+1]=(uint8_t)n;s_schema[pos+2]=(uint8_t)u;s_schema[pos+3]=0;
        sbb_p16(s_schema+pos+4,f->offset);sbb_p16(s_schema+pos+6,f->count);
        memcpy(s_schema+pos+8,f->name,n);memcpy(s_schema+pos+8+n,f->unit,u);pos+=8+n+u;
    }
    sbb_p16(s_schema,(uint16_t)count);sbb_p16(s_schema+2,s_payload_size);
    s_schema_size=(uint16_t)pos;s_schema_id=sbb_crc(s_schema,pos);return 1;
}
static void Blackbox_Sample(uint8_t *payload, uint64_t timestamp_us)
{
    /* Static snapshots: sole caller is FlightControlTask; keep its stack small. */
    static FlightControlState_t flight;static RcSbusData_t rc;static EscPwmState_t esc;static OpticalFlowData_t optical;
    FlightControl_GetState(&flight);RcSbus_GetData(&rc);EscPwm_GetState(&esc);OpticalFlow_GetData(&optical);
    memset(payload,0,s_payload_size);
    /* Existing builder supplies the unchanged base and exact optical bytes.
     * Its legacy tail is outside the baseline payload and overwritten below/CRC. */
    VOFA_BuildBlackboxFrame(payload,&flight,&rc,&esc,&optical);
    /* Same capture instant and clock as the SBB3 packet header. */
    sbb_p64(payload+112,timestamp_us);
#ifdef BLACKBOX_TEST_EXTRA_FIELDS
    { const float v=1.25f;memcpy(payload+120,&v,4);sbb_p32(payload+124,0xfffffffeU);
      sbb_p16(payload+128,(uint16_t)-32768);sbb_p16(payload+130,0);sbb_p16(payload+132,32767); }
#endif
}
#endif
