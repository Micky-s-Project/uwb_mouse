#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include <stdint.h>


void attitude_calculate(float t, uint8_t update);
void attitude_calculator_get_euler(float *pDst);
void attitude_calculator_get_a(float *pDst);
void gyro_data_zero_cali(float gyro_data[3], int16_t acc_data_raw[3]);
#endif // IMU_HANDLER_H