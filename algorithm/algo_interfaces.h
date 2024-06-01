#ifndef ALGO_INTERFACES_H
#define ALGO_INTERFACES_H

#include <stdint.h>

void algo_get_gyro_data(float *pdata);
void algo_get_acc_data(float *pdata);
void algo_get_uwb_data_xy(float *pdata);
void algo_get_uwb_data_aoa(float *pdata);

void algo_imu_data_update_event_handler(int16_t gyro_data_raw[3], int16_t acc_data_raw[3]);
void algo_uwb_data_update_event_handler(char *data_str);
#endif // ALGO_INTERFACES_H