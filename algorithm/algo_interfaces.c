/*
 * @Author: Jhaoz0133 JHao_Z@163.com
 * @Date: 2024-06-01 00:09:11
 * @LastEditors: Jhaoz0133 JHao_Z@163.com
 * @LastEditTime: 2024-06-23 01:31:17
 * @FilePath: \uwb_mouse\algorithm\algo_interfaces.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "algo_config.h"
#include "stdio.h"
#include <string.h>
#include <math.h>
#include "platform_api.h"
#include "attitude_calculator.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "uwb.h"
#include "mouse.h"
#include "sc7i22.h"
#include "uwb_data_parser.h"
#include "imu_data_parser.h"
#include "algo_interfaces.h"
#include "algo_main.h"
#include "RTE_Components.h"
#include "portable.h"


void algorithm_init()
{
    //    mouse_init();
    platform_printf("algo inited!\n");
}

Tick_t get_tick()
{
    return platform_get_us_time();
}
float tick_2_second(Tick_t tick)
{
    return (float)tick / 1000000.0f;
}

extern IMU_DATA_t imu_data;

void algo_imu_data_update_event_handler(int16_t gyro_data_raw_x, int16_t gyro_data_raw_y, int16_t gyro_data_raw_z, int16_t acc_data_raw_x, int16_t acc_data_raw_y, int16_t acc_data_raw_z)
{
    static Tick_t tick, last_imu_data_tick, last_uwb_data_tick, last_mouse_tick;
    tick = get_tick();

    // imu 数据解析与校正
    int16_t gyro_data_raw[3] = {gyro_data_raw_x, gyro_data_raw_y, gyro_data_raw_z};
    int16_t acc_data_raw[3] = {acc_data_raw_x, acc_data_raw_y, acc_data_raw_z};
    imu_data_parse(gyro_data_raw, acc_data_raw);

    // 算法主函数
    algo_main();

    // 鼠标相关参数上报
    if (tick - last_mouse_tick > 1000000 / MOUSE_MOVE_RATE)
    {
        last_mouse_tick = tick;
        float euler[3] = {0};
        attitude_calculator_get_euler(euler);
        mouse_data_send(0, euler[0], euler[2]);

        platform_printf("t:%f,w:%f,%f,%f,a:%f,%f,%f\n", tick_2_second(tick - last_imu_data_tick), imu_data.gyro_data[0], imu_data.gyro_data[1], imu_data.gyro_data[2], imu_data.acc_data[0], imu_data.acc_data[1], imu_data.acc_data[2]);
        platform_printf("e:%f,%f,%f\n", euler[0] * 57.3, euler[1] * 57.3, euler[2] * 57.3);
    }
    last_imu_data_tick = tick;
}
