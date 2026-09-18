#include "atom_imu.h"

#include <string.h>

#include "main.h"

#define ATOM_IMU_ENCODER_21_MAX_INT    ((1UL << 21U) - 1UL)
#define ATOM_IMU_PI                    3.14159265358979323846f
#define ATOM_IMU_GYRO_MIN_RAD_S        (-2000.0f * ATOM_IMU_PI / 180.0f)
#define ATOM_IMU_GYRO_MAX_RAD_S        ( 2000.0f * ATOM_IMU_PI / 180.0f)

/*
 * A single writer (FDCAN ISR) publishes a whole sample between two sequence
 * increments.  Readers retry only when an ISR overlaps their structure copy.
 * This avoids both a long interrupt mask and a partially updated snapshot.
 */
volatile AtomImuData_t g_atom_imu_data;
static volatile uint32_t atom_imu_sequence;

static float AtomImu_DecodeFloat21(uint32_t encoded, float min, float max)
{
    const float normalized = (float)(encoded & ATOM_IMU_ENCODER_21_MAX_INT) /
                             (float)ATOM_IMU_ENCODER_21_MAX_INT;
    return min + normalized * (max - min);
}

/*
 * Equivalent to atom-sdk CanData3 packed little-endian bitfield layout.
 * Explicit byte operations are used so this does not depend on ARM bitfield
 * layout or unaligned accesses.
 */
static void AtomImu_UnpackCanData3(const uint8_t data[ATOM_IMU_CAN_DLC_BYTES],
                                   uint32_t *data1,
                                   uint32_t *data2,
                                   uint32_t *data3)
{
    *data1 = ((uint32_t)data[0]) |
             ((uint32_t)data[1] << 8U) |
             (((uint32_t)data[2] & 0x1FU) << 16U);
    *data2 = (((uint32_t)data[2] >> 5U) & 0x07U) |
             ((uint32_t)data[3] << 3U) |
             ((uint32_t)data[4] << 11U) |
             (((uint32_t)data[5] & 0x03U) << 19U);
    *data3 = (((uint32_t)data[5] >> 2U) & 0x3FU) |
             ((uint32_t)data[6] << 6U) |
             (((uint32_t)data[7] & 0x7FU) << 14U);
}

void AtomImu_Init(void)
{
    atom_imu_sequence = 0U;
    (void)memset((void *)&g_atom_imu_data, 0, sizeof(g_atom_imu_data));
    __DMB();
}

void AtomImu_ProcessCanFrame(uint32_t id, const uint8_t data[ATOM_IMU_CAN_DLC_BYTES])
{
    uint32_t data1;
    uint32_t data2;
    uint32_t data3;

    if ((data == NULL) ||
        ((id != ATOM_IMU_GYRO_CAN_ID) && (id != ATOM_IMU_EULR_CAN_ID)))
    {
        return;
    }

    AtomImu_UnpackCanData3(data, &data1, &data2, &data3);

    atom_imu_sequence++;
    __DMB();

    if (id == ATOM_IMU_GYRO_CAN_ID)
    {
        g_atom_imu_data.gyro_x = AtomImu_DecodeFloat21(data1,
                                                       ATOM_IMU_GYRO_MIN_RAD_S,
                                                       ATOM_IMU_GYRO_MAX_RAD_S);
        g_atom_imu_data.gyro_y = AtomImu_DecodeFloat21(data2,
                                                       ATOM_IMU_GYRO_MIN_RAD_S,
                                                       ATOM_IMU_GYRO_MAX_RAD_S);
        g_atom_imu_data.gyro_z = AtomImu_DecodeFloat21(data3,
                                                       ATOM_IMU_GYRO_MIN_RAD_S,
                                                       ATOM_IMU_GYRO_MAX_RAD_S);
        g_atom_imu_data.gyro_rx_count++;
        g_atom_imu_data.gyro_valid = 1U;
    }
    else
    {
        /* Official EULR ordering: data1=pitch, data2=roll, data3=yaw. */
        g_atom_imu_data.pitch = AtomImu_DecodeFloat21(data1, -ATOM_IMU_PI, ATOM_IMU_PI);
        g_atom_imu_data.roll = AtomImu_DecodeFloat21(data2, -ATOM_IMU_PI, ATOM_IMU_PI);
        g_atom_imu_data.yaw = AtomImu_DecodeFloat21(data3, -ATOM_IMU_PI, ATOM_IMU_PI);
        g_atom_imu_data.euler_rx_count++;
        g_atom_imu_data.euler_valid = 1U;
    }

    __DMB();
    atom_imu_sequence++;
}

void AtomImu_GetData(AtomImuData_t *data)
{
    uint32_t start_sequence;
    uint32_t end_sequence;

    if (data == NULL)
    {
        return;
    }

    do
    {
        start_sequence = atom_imu_sequence;
        __DMB();
        (void)memcpy(data, (const void *)&g_atom_imu_data, sizeof(*data));
        __DMB();
        end_sequence = atom_imu_sequence;
    } while ((start_sequence != end_sequence) || ((start_sequence & 1U) != 0U));
}
