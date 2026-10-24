#ifndef BLACKBOX_FIELDS_H
#define BLACKBOX_FIELDS_H
#include "blackbox_protocol.h"
#include "vofa.h"
#include "bmi088.h"
#include "bmi088_attitude.h"
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
    /* Flight capture time exists once, in the SBB3 header (sample_time_us). */
    BF("bmi088_gyro_deg_s","deg/s",112,3,SBB_F32),
    BF("bmi088_accel_m_s2","m/s^2",124,3,SBB_F32),
    BF("bmi088_temperature_c","degC",136,1,SBB_F32),
    /* "Raw" BMI088 motion is now calibrated/unfiltered. No pre-calibration
     * registers/physical arrays or duplicate calibrated arrays are logged. */
    /* Celsius exactly represents the 11-bit temperature counts; no second copy. */
    BF("bmi088_sample_count","",140,1,SBB_U32),
    BF("bmi088_sample_time_ms","ms",144,1,SBB_U32),
    BF("bmi088_age_ms","ms",148,1,SBB_U32),
    BF("bmi088_has_sample","",152,1,SBB_U8),
    BF("bmi088_valid","",153,1,SBB_U8),
    BF("bmi088_initialized","",154,1,SBB_U8),
    BF("bmi088_error","",155,1,SBB_U8),
    BF("bmi088_calibration_version","",156,1,SBB_U32),
    BF("bmi088_compensation_temperature_c","degC",160,1,SBB_F32),
    BF("bmi088_compensation_in_range","",164,1,SBB_U8),
    BF("bmi088_roll_deg","deg",165,1,SBB_F32),
    BF("bmi088_pitch_deg","deg",169,1,SBB_F32),
    BF("bmi088_yaw_deg","deg",173,1,SBB_F32), /* Relative within attitude epoch. */
    BF("bmi088_attitude_valid","",177,1,SBB_U8),
    BF("bmi088_attitude_epoch","",178,1,SBB_U32),
    BF("bmi088_attitude_accel_used","",182,1,SBB_U8),
    BF("bmi088_attitude_error","",183,1,SBB_U8),
    BF("bmi088_gyro_filtered_deg_s","deg/s",184,3,SBB_F32),
    BF("bmi088_gyro_filter_valid","",196,1,SBB_U8),
    BF("bmi088_gyro_filter_cutoff_hz","Hz",197,1,SBB_F32),
    BF("bmi088_gyro_filter_sample_hz","Hz",201,1,SBB_F32),
#ifdef BLACKBOX_TEST_EXTRA_FIELDS
    BF("new_float","test",205,1,SBB_F32), BF("new_uint32","test",209,1,SBB_U32), BF("new_array","test",213,3,SBB_I16),
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
static void Blackbox_Sample(uint8_t *payload)
{
    /* Static snapshots: sole caller is BlackboxTask; keep its stack small. */
    static FlightControlState_t flight;static RcSbusData_t rc;static EscPwmState_t esc;static OpticalFlowData_t optical;
    static Bmi088Data_t bmi;
    static Bmi088AttitudeState_t attitude;
    static Bmi088GyroFilterState_t filter;
    FlightControl_GetState(&flight);RcSbus_GetData(&rc);EscPwm_GetState(&esc);OpticalFlow_GetData(&optical);
    Bmi088_GetSnapshotWithFilter(&bmi, &filter, &attitude);
    memset(payload,0,s_payload_size);
    /* Existing builder supplies the unchanged base and exact optical bytes.
     * Its legacy tail is outside the baseline payload and overwritten below/CRC. */
    VOFA_BuildBlackboxFrame(payload,&flight,&rc,&esc,&optical);
    /* Legacy builder's JustFloat tail is replaced by the first BMI float. */
    { unsigned axis;
      for(axis=0;axis<3;axis++) {
          const float rate_deg_s=bmi.gyro_rad_s[axis]*57.295779513f;
          memcpy(payload+112+4*axis,&rate_deg_s,4);
      } }
    memcpy(payload+124,bmi.accel_m_s2,12);
    memcpy(payload+136,&bmi.temperature_c,4);
    sbb_p32(payload+140,bmi.sample_count);sbb_p32(payload+144,bmi.sample_tick_ms);
    sbb_p32(payload+148,bmi.age_ms);
    payload[152]=bmi.has_sample;payload[153]=bmi.valid;
    payload[154]=bmi.initialized;payload[155]=bmi.error;
    sbb_p32(payload+156,bmi.calibration_version);
    memcpy(payload+160,&bmi.compensation_temperature_c,4);payload[164]=bmi.compensation_in_range;
    { const float roll=attitude.roll_rad*57.295779513f, pitch=attitude.pitch_rad*57.295779513f,
                  yaw=attitude.yaw_rad*57.295779513f;
      memcpy(payload+165,&roll,4);memcpy(payload+169,&pitch,4);memcpy(payload+173,&yaw,4); }
    payload[177]=attitude.valid;sbb_p32(payload+178,attitude.epoch);
    payload[182]=attitude.accel_used;payload[183]=attitude.error;
    { unsigned axis;
      for(axis=0;axis<3;axis++) {
          const float rate_deg_s=filter.gyro_rad_s[axis]*57.295779513f;
          memcpy(payload+184+4*axis,&rate_deg_s,4);
      } }
    /* Same field/units: now the final 120 Hz notch -> 50 Hz LPF result.
     * cutoff_hz remains the LPF cutoff, not the notch center. */
    payload[196]=filter.valid;
    memcpy(payload+197,&filter.cutoff_hz,4);memcpy(payload+201,&filter.applied_sample_hz,4);
#ifdef BLACKBOX_TEST_EXTRA_FIELDS
    { const float v=1.25f;memcpy(payload+205,&v,4);sbb_p32(payload+209,0xfffffffeU);
      sbb_p16(payload+213,(uint16_t)-32768);sbb_p16(payload+215,0);sbb_p16(payload+217,32767); }
#endif
}
#endif
