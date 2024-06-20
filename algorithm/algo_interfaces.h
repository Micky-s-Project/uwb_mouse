#ifndef ALGO_INTERFACES_H
#define ALGO_INTERFACES_H

#include <stdint.h>

typedef uint64_t Tick_t;
Tick_t get_tick();

void algorithm_init();
void algo_get_gyro_data(float *pdata);
void algo_get_acc_data(float *pdata);

void algo_imu_data_update_event_handler(int16_t gyro_data_raw[3], int16_t acc_data_raw[3]);
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif
#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
#endif // ALGO_INTERFACES_H