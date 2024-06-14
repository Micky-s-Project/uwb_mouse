#include "algo_config.h"
#include "stdio.h"
#include <string.h>
#include <math.h>
#include "platform_api.h"
#include "attitude_calculator.h"
#include "position_calculator.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "uwb.h"
#include "mouse.h"
#include "sc7i22.h"
/*
            时间戳
*/
#define MOUSE_MOVE_RATE 100

SemaphoreHandle_t imu_data_mutex = NULL;

void algo_uwb_data_update_event_handler(void *p);
void algorithm_init()
{
    while (imu_data_mutex == NULL)
    {
        imu_data_mutex = get_imu_data_mutex();
        vTaskDelay(1);
    }
    xTaskCreate(algo_uwb_data_update_event_handler, "uwb task", 512 * 2, NULL, configMAX_PRIORITIES - 1 /* tskIDLE_PRIORITY */, NULL);
    mouse_init();
    platform_printf("algo inited!\n");
}
void uwb_data_parse();
typedef uint64_t Tick_t;
Tick_t tick, last_imu_data_tick, last_uwb_data_tick, last_mouse_tick;
Tick_t get_tick()
{
    return platform_get_us_time();
}
float tick_2_second(Tick_t tick)
{
    return (float)tick / 1000000.0f;
}
typedef struct UWB_DATA_t
{
    float x;
    float y;
    float dis;
    float aoa;
    uint8_t ready;
} UWB_DATA_t;
UWB_DATA_t uwb_data = {0};
uint8_t uwb_data_pool[256] = {0};
uint16_t uwb_data_pool_index = 0;
int16_t no_index[2] = {-1, -1};
int16_t uwb_data_run_count = 0;
typedef struct IMU_DATA_t
{
    int16_t gyro_data_raw[3];
    int16_t acc_data_raw[3];
    float gyro_data[3];
    float acc_data[3];
} IMU_DATA_t;
IMU_DATA_t imu_data;
uint32_t imu_data_run_count = 0;
void imu_data_parse(int16_t gyro_data_raw[3], int16_t acc_data_raw[3]);
void algo_imu_data_update_event_handler(int16_t gyro_data_raw[3], int16_t acc_data_raw[3])
{
    tick = get_tick();
    uwb_data_parse();
    imu_data_run_count++;
    if (imu_data_run_count == 1600)
    {
        imu_data_run_count = 0;
        // platform_printf("imu:%lld\n", tick);
    }
    // if (xSemaphoreTake(imu_data_mutex, 0) == pdTRUE)
    // {
        imu_data_parse(gyro_data_raw, acc_data_raw);
        gyro_data_zero_cali(imu_data.gyro_data, acc_data_raw);
    //     xSemaphoreGive(imu_data_mutex);
    // }
		float t = tick_2_second(tick - last_imu_data_tick);
    if (uwb_data.ready == 0)
    {
        
        attitude_calculate(t, 0);
        // position_calculate(t, 0);
    }
    else
    {
        // float t = tick_2_second(tick - last_uwb_data_tick);
        last_uwb_data_tick = tick;
        attitude_calculate(t, 1);
        // position_calculate(t, 1);
        uwb_data.ready = 0;
    }

    if (tick - last_mouse_tick > 1000000 / MOUSE_MOVE_RATE)
    {
        last_mouse_tick = tick;
        float euler[3] = {0};
        attitude_calculator_get_euler(euler);
        mouse_cal_pix(euler[0], euler[2]);
        //platform_printf("t:%f,w:%f,%f,%f,a:%f,%f,%f\n", tick_2_second(tick - last_imu_data_tick), imu_data.gyro_data[0], imu_data.gyro_data[1], imu_data.gyro_data[2], imu_data.acc_data[0], imu_data.acc_data[1], imu_data.acc_data[2]);
        //platform_printf("e:%f,%f,%f\n", euler[0] *57.3, euler[1]*57.3, euler[2]*57.3);
    }
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
uint32_t uwb_tmp_count = 0;
void algo_uwb_data_update_event_handler(void *p)
{
    QueueHandle_t uwb_queue = 0;
    char data_char;

    // return;
    while (uwb_queue == NULL)
    {
        uwb_queue = get_uwb_queue();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    while (1)
    {
        // vTaskDelay(pdMS_TO_TICKS(10));
        if (xQueueReceive(uwb_queue, &data_char, 0xFFFFFFFF))
        {
            // platform_printf("%c", data_char);
            if (data_char == 'O' && uwb_data_pool[uwb_data_pool_index - 1] == 'N')
            {
                if (no_index[0] == -1)
                {
                    no_index[0] = 0;
                }
                else
                {
                    no_index[1] = uwb_data_pool_index;
                }
            }
            uwb_data_pool[uwb_data_pool_index++] = data_char;
            if (uwb_data_pool_index == 256)
            {
                memset(uwb_data_pool, 0, 256);
                uwb_data_pool_index = 0;
            }
            uwb_data_run_count++;
            if (uwb_data_run_count == 1000)
            {
                uwb_data_run_count = 0;
                // platform_printf("uwb:%lld\n", tick);
            }
        }
    }

    //
}
void uwb_data_parse()
{
    if (no_index[0] != -1 && no_index[1] != -1)
    {
        float dis_tmp = 0, aoa_tmp = 0;
        // platform_printf("uwb_data:%d,%d,%s\n", no_index[0], no_index[1], uwb_data_pool);
        if (sscanf((char *)uwb_data_pool, "%*[^:]: %f%*[^:]: %f", &(dis_tmp), &(aoa_tmp)) == 2)
        {
            // platform_printf("uwb_data:%f,%f\n", dis_tmp, aoa_tmp);
            if ((fabsf(dis_tmp - uwb_data.dis) < 0.5 && fabsf(0.01745329 * aoa_tmp - uwb_data.aoa) < 0.5 || uwb_data.dis == 0) && dis_tmp > 0)
            {
                uwb_data.dis = dis_tmp;
                uwb_data.aoa = 0.01745329 * aoa_tmp;
                uwb_data.x = uwb_data.dis * sinf(uwb_data.aoa);
                uwb_data.y = uwb_data.dis * cosf(uwb_data.aoa);
                uwb_data.ready = 1;
                // platform_printf("uwb_data:%f,%f,%f,%f\n", uwb_data.dis, uwb_data.aoa * 57.3, uwb_data.x, uwb_data.y);
            }

            //
        }

        memset(uwb_data_pool, 0, 256);
        uwb_data_pool[0] = 'N';
        uwb_data_pool[1] = 'O';
        uwb_data_pool_index = 2;
        no_index[0] = 0;
        no_index[1] = -1;
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