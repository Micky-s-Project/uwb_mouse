/*
 * @Author: Jhaoz0133 JHao_Z@163.com
 * @Date: 2024-06-20 19:03:19
 * @LastEditors: Jhaoz0133 JHao_Z@163.com
 * @LastEditTime: 2024-06-22 20:30:11
 * @FilePath: \uwb_mouse\algorithm\algo_main.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include "algo_main.h"
#include "algo_config.h"
#include "algo_interfaces.h"
#include "uwb_data_parser.h"
#include "attitude_calculator.h"
#include "imu_data_parser.h"
/**
 * @description: 此函数为算法运行主循环函数，由于FreeRtos系统调度间隔最少为1ms，计算频率达不到1KHZ+，所以此函数应由IMU中断调用
 * @return {*}
 */
extern IMU_DATA_t imu_data;
void algo_main()
{
    // 记录数据时间
    static Tick_t prev_imu_data_tick = 0, prev_uwb_data_tick = 0;
    Tick_t us_tick = get_tick();

    /* 由于CPU计算资源有限， IMU获取到的数据不能每个都进行卡尔曼更新*/
    static Tick_t prev_acc_predict_tick = 0;
    static Tick_t prev_uwb_predict_tick = 0;
    if (us_tick - prev_uwb_predict_tick > 1000000 / UWB_UPDATE_KALMAN_RATE) 
    {
        // 获取有效UWB数据
        uint8_t uwb_data_ready = 0;
        UWB_DATA_t uwb_data = {0};
        uwb_data_ready = get_uwb_data(&uwb_data);
        if (uwb_data_ready)
        {
            // UWB量测更新
            prev_uwb_predict_tick = us_tick;
            attitude_uwb_update();
            // platform_printf("uwbupt:w:%f,%f,%f,a:%f,%f,%f\n", imu_data.gyro_data[0], imu_data.gyro_data[1], imu_data.gyro_data[2], imu_data.acc_data[0], imu_data.acc_data[1], imu_data.acc_data[2]);

            return;
        }
    }

    if (us_tick - prev_acc_predict_tick > 1000000 / ACC_UPDATE_KALMAN_RATE) // 0.5s
    {
        // 加速度计量测更新
        prev_acc_predict_tick = us_tick;
        attitude_acc_update();
        // platform_printf("accupt:w:%f,%f,%f,a:%f,%f,%f\n", imu_data.gyro_data[0], imu_data.gyro_data[1], imu_data.gyro_data[2], imu_data.acc_data[0], imu_data.acc_data[1], imu_data.acc_data[2]);

        return;
    }

    // 积分预测
    float t = tick_2_second(us_tick - prev_imu_data_tick);
    attitude_predict(t);
    prev_imu_data_tick = us_tick;
}