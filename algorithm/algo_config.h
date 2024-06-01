#include <stdint.h>
#include <stdio.h>

#define ALGO_DEBUG               printf

#define SENSOR_GYRO_RANGE        2000
#define SENSOR_ACC_RANGE         16

#define TAG_ROLE                 1
#define G_CONST                  9.80665
#define US_2_S                   1000000
#define IMU_DATA_RATE            8000
#define UWB_DATA_RATE            100
#define CALCULATE_T              0.000125
#define CALCULATE_RATE           8000
#define CALCULATE_TT2            0.0000000078125
#define ATTITUDE_KALMAN_CAL_RATE 1000
#define POSITION_KALMAN_CAL_RATE 100

#if SENSOR_GYRO_RANGE == 2000
#define GYRO_DATA_SCALE 0.001065264436032 // 2000/180*pi/32768
#elif SENSOR_GYRO_RANGE == 1000
#define GYRO_DATA_SCALE 0.000532632218016 // 1000/180*pi/32768
#elif SENSOR_GYRO_RANGE == 500
#define GYRO_DATA_SCALE 0.000266316109008 // 500/180*pi/32768
#elif SENSOR_GYRO_RANGE == 250
#define GYRO_DATA_SCALE 0.000133158054504 // 250/180*pi/32768
#endif

#if SENSOR_ACC_RANGE == 16
#define ACC_DATA_SCALE 0.004790039062500 // acc_scale/data_scale 16g
#elif SENSOR_ACC_RANGE == 8
#define ACC_DATA_SCALE 0.002395019531250 // acc_scale/data_scale 16g
#elif SENSOR_ACC_RANGE == 4
#define ACC_DATA_SCALE 0.001197509765625 // acc_scale/data_scale 16g
#elif SENSOR_ACC_RANGE == 2
#define ACC_DATA_SCALE 0.0005987548828125 // acc_scale/data_scale 16g
#endif

#define UWB_DATA_SCALE 0.001