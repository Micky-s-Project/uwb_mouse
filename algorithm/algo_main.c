#include "algo_main.h"
#include "algo_config.h"
#include "algo_interfaces.h"

/**
 * @description: 此函数为算法运行主循环函数，由于FreeRtos系统调度间隔最少为1ms，计算频率达不到1KHZ+，所以此函数应由IMU中断调用
 * @return {*}
 */
void algo_main()
{
    // 记录数据时间
    static Tick_t prev_imu_data_tick = 0, prev_uwb_data_tick = 0;
    Tick_t us_tick = get_tick();

    /* 由于CPU计算资源有限， IMU获取到的数据不能每个都进行卡尔曼更新*/
    static Tick_t prev_att_predict_tick = 0;
    if (us_tick - prev_att_predict_tick > 1000000) // 1s
    {
        /* UWB data filt ?*/
        // kalman update code
        prev_att_predict_tick = us_tick;
    }
    else
    {
        // kalman predict code
    }

    prev_imu_data_tick = us_tick;
}