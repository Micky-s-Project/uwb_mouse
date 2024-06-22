#ifndef IMU_DATA_PARSE_H
#define IMU_DATA_PARSE_H

#include <stdint.h>

typedef struct IMU_DATA_t
{
    int16_t gyro_data_raw[3];
    int16_t acc_data_raw[3];
    float gyro_data[3];
    float acc_data[3];
} IMU_DATA_t;

void imu_data_parse(int16_t gyro_data_raw[3], int16_t acc_data_raw[3]);
void gyro_data_zero_cali(float gyro_data[3], int16_t acc_data_raw[3]);

#endif // IMU_DATA_PARSE_H