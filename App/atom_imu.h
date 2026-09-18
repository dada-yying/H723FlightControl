#ifndef ATOM_IMU_H
#define ATOM_IMU_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ATOM_IMU_DEVICE_ID             48U
#define ATOM_IMU_GYRO_PACKET_TYPE      1U
#define ATOM_IMU_EULR_PACKET_TYPE      3U
#define ATOM_IMU_GYRO_CAN_ID           (ATOM_IMU_DEVICE_ID + ATOM_IMU_GYRO_PACKET_TYPE)
#define ATOM_IMU_EULR_CAN_ID           (ATOM_IMU_DEVICE_ID + ATOM_IMU_EULR_PACKET_TYPE)
#define ATOM_IMU_CAN_DLC_BYTES         8U

typedef struct
{
    float gyro_x;          /* rad/s */
    float gyro_y;          /* rad/s */
    float gyro_z;          /* rad/s */
    float roll;            /* rad, ATOM raw coordinate system */
    float pitch;           /* rad, ATOM raw coordinate system */
    float yaw;             /* rad, ATOM raw coordinate system */
    uint32_t gyro_rx_count;
    uint32_t euler_rx_count;
    uint8_t gyro_valid;
    uint8_t euler_valid;
} AtomImuData_t;

/*
 * Keil Watch-visible latest IMU data. The FDCAN ISR is the sole writer;
 * tasks should use AtomImu_GetData() for a coherent application snapshot.
 */
extern volatile AtomImuData_t g_atom_imu_data;

void AtomImu_Init(void);
void AtomImu_ProcessCanFrame(uint32_t id, const uint8_t data[ATOM_IMU_CAN_DLC_BYTES]);
void AtomImu_GetData(AtomImuData_t *data);

#ifdef __cplusplus
}
#endif

#endif /* ATOM_IMU_H */
