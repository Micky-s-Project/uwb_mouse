#ifndef ALGO_INTERFACES_H
#define ALGO_INTERFACES_H

#include <stdint.h>

void algorithm_init();
void algo_get_gyro_data(float *pdata);
void algo_get_acc_data(float *pdata);

void algo_imu_data_update_event_handler(int16_t gyro_data_raw[3], int16_t acc_data_raw[3]);
#endif // ALGO_INTERFACES_H