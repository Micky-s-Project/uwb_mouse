#include "algo_config.h"
#include "stdio.h"
#include <string.h>
#include <math.h>
#include "platform_api.h"

/*
            时间戳
*/
typedef uint64_t Tick_t;
Tick_t tick, last_imu_data_tick, last_uwb_data_tick;
Tick_t get_tick()
{
    return platform_get_us_time();
}
float tick_2_second(Tick_t tick)
{
    return (float)tick / 1000000.0f;
}

/*
            imu数据更新事件处理
*/
typedef struct IMU_DATA_t
{
    int16_t gyro_data_raw[3];
    int16_t acc_data_raw[3];
    float gyro_data[3];
    float acc_data[3];
} IMU_DATA_t;
IMU_DATA_t imu_data;
void imu_data_parse(int16_t gyro_data_raw[3], int16_t acc_data_raw[3]);
void algo_imu_data_update_event_handler(int16_t gyro_data_raw[3], int16_t acc_data_raw[3])
{
    tick = get_tick();
    float t = tick_2_second(tick - last_imu_data_tick);

    imu_data_parse(gyro_data_raw, acc_data_raw);
    gyro_data_zero_cali(imu_data.gyro_data, acc_data_raw);
    attitude_calculate(t, 0);
    position_calculate(t, 0);

    last_imu_data_tick = tick;
}

void imu_data_parse(int16_t gyro_data_raw[3], int16_t acc_data_raw[3])
{
    for (uint8_t i = 0; i < 3; i++)
    {
        imu_data.gyro_data_raw[i] = gyro_data_raw[i];
        imu_data.gyro_data[i] = (float)gyro_data_raw[i] * GYRO_DATA_SCALE;
        imu_data.acc_data_raw[i] = acc_data_raw[i];
        imu_data.acc_data[i] = (float)acc_data_raw[i] * ACC_DATA_SCALE;
    }
}

typedef struct UWB_DATA_t
{
    float x;
    float y;
    float dis;
    float aoa;
} UWB_DATA_t;
UWB_DATA_t uwb_data = {0};
uint8_t uwb_data_pool[128] = {0};
uint8_t uwb_data_pool_index = 0;
void algo_uwb_data_update_event_handler(char data_char)
{
    uwb_data_pool[uwb_data_pool_index++] = data_char;
    char *d1_index = strstr((char *)uwb_data_pool, "NO");
    if (d1_index != NULL)
    {
        char *end_index = strstr((char *)d1_index, "*");
        if (end_index != NULL)
        {
            int tmp = 0;
            sscanf_s(d1_index, "NO(%d). D: %f, A: %f, *", &tmp, &(uwb_data.dis), &(uwb_data.aoa));
            strcpy_s((char *)uwb_data_pool, 128, (char *)(end_index));
            uwb_data_pool_index = 0;

            tick = get_tick();
            float t = tick_2_second(tick - last_uwb_data_tick);

            uwb_data.aoa *= 0.01745329;
            uwb_data.x = uwb_data.dis * sinf(uwb_data.aoa);
            uwb_data.y = uwb_data.dis * cosf(uwb_data.aoa);
            ALGO_DEBUG("uwb_data:%f,%f,%f,%f\n", uwb_data.dis, uwb_data.aoa * 57.3, uwb_data.x, uwb_data.y);
            last_uwb_data_tick = tick;
            attitude_calculate(t, 1);
            position_calculate(t, 1);
        }
    }
}

// void algo_uwb_data_update_event_handler(char *data_str)
//{
//     tick = get_tick();
//     float t = tick_2_second(tick - last_uwb_data_tick);
//     int tmp = 0;
//     sscanf_s(data_str, "NO(%d). D: %f, A: %f, *", &tmp, &(uwb_data.dis), &(uwb_data.aoa));
//     uwb_data.aoa *= 0.01745329;
//     uwb_data.x = uwb_data.dis * sinf(uwb_data.aoa);
//     uwb_data.y = uwb_data.dis * cosf(uwb_data.aoa);
//     ALGO_DEBUG("uwb_data:%f,%f,%f,%f\n", uwb_data.dis, uwb_data.aoa * 57.3, uwb_data.x, uwb_data.y);
//     last_uwb_data_tick = tick;
//     attitude_calculate(t, 1);
//     position_calculate(t, 1);
// }

void algo_get_gyro_data(float *pdata)
{
    pdata[0] = imu_data.gyro_data[0];
    pdata[1] = imu_data.gyro_data[1];
    pdata[2] = imu_data.gyro_data[2];
}

void algo_get_acc_data(float *pdata)
{
    pdata[0] = imu_data.acc_data[0];
    pdata[1] = imu_data.acc_data[1];
    pdata[2] = imu_data.acc_data[2];
}

void algo_get_uwb_data_xy(float *pdata)
{
    pdata[0] = uwb_data.x;
    pdata[1] = uwb_data.y;
}

void algo_get_uwb_data_aoa(float *pdata)
{
    pdata[0] = uwb_data.aoa;
}