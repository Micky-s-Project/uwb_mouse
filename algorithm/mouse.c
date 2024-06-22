
#include "my_queue.h"
#include "platform_api.h"
#include <math.h>
#include "bsp_usb_hid.h"
#include "algo_interfaces.h"

//#define RES_1K
//#define RES_2K
//#define RES_4K
//#define SCREEN_RES RES_2K
//#if SCREEN_RES == RES_4K
//#define SCREEN_RES_X 3840.0
//#define SCREEN_RES_Y 2160.0
//#elif SCREEN_RES == RES_2K
//#define SCREEN_RES_X 2560.0
//#define SCREEN_RES_Y 1440.0
//#elif SCREEN_RES == RES_1K
//#define SCREEN_RES_X 1920.0
//#define SCREEN_RES_Y 1080.0
//#endif

///*均值滤波用*/
//#define queue_len 100
//float posx_data[queue_len] = {0}, posy_data[queue_len] = {0};
//Queue q_posx, q_posy;

///*均值滤波用*/
//#define euler_queue_len 5
//float pitch_data[euler_queue_len] = {0}, yaw_data[euler_queue_len] = {0};
//Queue q_pitch, q_yaw;

//float y_weight = 1;
//float y_last = 0.5;
//float y = 0.5;
//int16_t pix_x = 0, pix_y = 0, last_pix_x = 0, last_pix_y = 0;
//uint16_t data_count;
//float x_sensi = 57.3f * 19.2f / 2.0f;
//float y_sensi = 57.3f * 10.8f / 2.0f;

//void mouse_init()
//{
//    queue_init(&q_posx, posx_data, queue_len, QUENE_ANALYZE_OPEN);
//    queue_init(&q_posy, posy_data, queue_len, QUENE_ANALYZE_OPEN);
//    queue_init(&q_pitch, pitch_data, euler_queue_len, QUENE_ANALYZE_OPEN);
//    queue_init(&q_yaw, yaw_data, euler_queue_len, QUENE_ANALYZE_OPEN);
//}

//float init_pitch[2];
//float init_yaw[2];
//uint8_t init_count = 0;
//uint64_t init_tick = 0;
///**
// * @description:
// * @param {float} tag_pitch tag的俯仰角 程序中对应euler[0]
// * @param {float} tag_yaw tag的航向角 程序中对应euler[2]
// * @param {float} anchor_aoa anchor的水平角
// * @param {float} dis 两个板子间距离，两个板子输出的应该一样
// * @return {*}
// */
//void mouse_control_init(float tag_pitch, float tag_yaw, float anchor_aoa, float dis)
//{
//    if (init_count == 1 && platform_get_us_time() - init_tick > 10000000) // 10s
//    {
//        init_count = 0;
//    }
//    if (init_count != 2)
//    {
//        init_pitch[init_count] = tag_pitch + 1.5708;
//        init_yaw[init_count] = -tag_yaw;
//        init_count++;
//    }
//    else
//    {
//        bsp_usb_handle_hid_mouse_report(0, 0, 1);
//    }

//    if (init_count == 1)
//    {
//        init_tick = platform_get_us_time();
//    }

//    if (init_count == 2)
//    {
//        x_sensi = SCREEN_RES_X / (fabsf(init_yaw[1] - init_yaw[0])) / 2.0f;
//        y_sensi = SCREEN_RES_Y / (fabsf(init_pitch[1] - init_pitch[0])) / 2.0f;
//        y = dis * cosf(anchor_aoa);
//        y_last = y;
//        // platform_printf("mouse_sens:%f,%f,%d,%d,%d,%d\n", q_yaw.mean * 57.3, q_pitch.mean * 57.3, dx, dy, pix_x, pix_y);
//    }
//}
//void mouse_data_input(float xy[2])
//{
//    data_count++;
//    queue_input(&q_posx, xy[0]);
//    queue_input(&q_posy, xy[1]);
//}

//int16_t angel_2_pix(float angel, float weight, uint8_t h)
//{
//    // return 750 * tanf(angel);
//    if (h == 1)
//    {
//        return (int16_t)(x_sensi * angel * weight);
//    }
//    else
//    {
//        return (int16_t)(y_sensi * angel * weight);
//    }
//}

///**
// * @description:
// * @param {float} tag_pitch tag的俯仰角 程序中对应euler[0]
// * @param {float} tag_yaw tag的航向角 程序中对应euler[2]
// * @param {float} anchor_aoa anchor的水平角
// * @param {float} dis 两个板子间距离，两个板子输出的应该一样
// * @return {*}
// */
//void mouse_cal_pix(float tag_pitch, float tag_yaw, float anchor_aoa, float dis)
//{
//    // 均值滤波
//    queue_input(&q_pitch, tag_pitch);
//    queue_input(&q_yaw, tag_yaw);
//    queue_input(&q_posy, dis * cosf(anchor_aoa));

//    // 防止UWB数据不准来回抖动
//    if (fabsf(MAX(0.1, q_posy.mean) - y_last) > MAX(q_posy.mean * 0.25f, 0.15))
//    {
//        y_weight = MAX(0.1, q_posy.mean) / y;
//        // platform_printf("y_weight:%f,%f,%f,%f\n", y_weight, q_posy.mean, y_last, y);
//        y_last = MAX(0.1, q_posy.mean);
//    }

//    // 用角度计算坐标点
//    pix_x = angel_2_pix(-q_yaw.mean, y_weight, 1);
//    pix_y = angel_2_pix((q_pitch.mean + 1.5708), y_weight, 1);

//    // 计算坐标点差值
//    int16_t dx = pix_x - last_pix_x, dy = pix_y - last_pix_y;
//    last_pix_x = pix_x;
//    last_pix_y = pix_y;

//    // float aoa = 0;
//    // algo_get_uwb_data_aoa(&aoa);
//    // platform_printf("mouse:%f,%f,%f,%f,%f,%f,%d,%d,%d,%d\n", q_yaw.mean * 57.3, q_pitch.mean * 57.3, aoa * 57.3, q_posy.mean, y, dis, dx, dy, pix_x, pix_y);

//    // 初始向右下角
//    extern uint8_t ready_output_xy;
//    if (ready_output_xy)
//    {
//        bsp_usb_handle_hid_mouse_report(dx, dy, 0);
//    }
//    else
//    {
//        bsp_usb_handle_hid_mouse_report(100, 100, 0);
//    }
//}

void mouse_data_send(uint8_t btn, float pitch, float yaw)
{
}


void btn_down_event_handler(uint8_t btn_index)
{
    mouse_data_send(btn_index, 0, 0);
}

