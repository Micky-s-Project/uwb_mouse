#include <stdint.h>
#include <stdio.h>
#include "platform_api.h"

// #define ALGO_DEBUG               platform_printf
#define ALGO_DEBUG \
    do             \
    {              \
    } while (0);

#define SENSOR_GYRO_RANGE 2000
#define SENSOR_ACC_RANGE 16

#define TAG_ROLE 1
// #define G_CONST                  9.80665
#define G_CONST 10.00            // 换板子需要重新计算此值 计算方式为sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2])
#define US_2_S 1000000

#define ACC_UPDATE_KALMAN_RATE 1 // 垂直角不准或垂直角漂移加大这个
#define UWB_UPDATE_KALMAN_RATE 2 // 水平角不准或水平角漂移加大这个
#define MOUSE_MOVE_RATE 50       // 鼠标输出速率
#define ACC_DYNAMIC_R 1          // 上下抖尝试关闭(注释)这个,垂直角不准尝试打开这个
// #define UWB_DYNAMIC_R 1          // 左右抖尝试关闭(注释)这个,水平角不准尝试打开这个
#define UWB_R 10000.0            // 抖可以调大这个
#define ACC_R 1000000.0          // 上下抖可以调大这个
#define KALMAN_Q 0.0001

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
