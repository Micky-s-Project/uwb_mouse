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
#include "uwb_data_parser.h"
#include "RTE_Components.h" // Component selection
/*
            时间戳
*/
#define MOUSE_MOVE_RATE 1

SemaphoreHandle_t imu_data_mutex = NULL;

void algorithm_init()
{
    while (imu_data_mutex == NULL)
    {
        imu_data_mutex = get_imu_data_mutex();
        vTaskDelay(1);
    }

    xTaskCreate(uwb_data_parse_task, "uwb_data_parse_task", 512 * 2, NULL, configMAX_PRIORITIES - 1 /* tskIDLE_PRIORITY */, NULL);
    mouse_init();
    platform_printf("algo inited!\n");
}

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
UWB_DATA_t uwb_data = {0};
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

    get_uwb_data(&uwb_data);

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
        //

        attitude_calculate(t, 1);
        // position_calculate(t, 1);
        // uwb_data.ready = 0;
    }
    static uint16_t uwb_count = 0;
    if (tick - last_mouse_tick > 1000000 / MOUSE_MOVE_RATE)
    {
        last_mouse_tick = tick;
        float euler[3] = {0};
        attitude_calculator_get_euler(euler);
        mouse_cal_pix(euler[0], euler[2], uwb_data.dis);
        platform_printf("u:%d\n", uwb_count);
        uwb_count = 0;
        // platform_printf("t:%f,w:%f,%f,%f,a:%f,%f,%f\n", tick_2_second(tick - last_imu_data_tick), imu_data.gyro_data[0], imu_data.gyro_data[1], imu_data.gyro_data[2], imu_data.acc_data[0], imu_data.acc_data[1], imu_data.acc_data[2]);
        // platform_printf("e:%f,%f,%f\n", euler[0] * 57.3, euler[1] * 57.3, euler[2] * 57.3);
    }
    last_imu_data_tick = tick;

    if (uwb_data.ready == 1)
    {
        last_uwb_data_tick = tick;
        uwb_count++;
        // platform_printf("uwb_data:%d,%d,%d,%d\n", (int)(uwb_data.dis * 1000), (int)(uwb_data.aoa * 57.3 * 1000), (int)(uwb_data.x * 1000), (int)(uwb_data.y * 1000));
    }
    float uwbt = tick_2_second(tick - last_uwb_data_tick);
    if (uwbt > 10)
    {
        // uwb_uart_init();
        platform_printf("reinit\n");
        uwb_parser_test();
        uwb_uart_test();
        last_uwb_data_tick = tick;
        uwb_count = 0;
    }

    uwb_data.ready = 0;
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
