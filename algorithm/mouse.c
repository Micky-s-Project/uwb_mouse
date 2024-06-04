
#include "my_queue.h"
#include "platform_api.h"
#include <math.h>
#include "bsp_usb_hid.h"

#define queue_len 50
float posx_data[queue_len] = {0}, posy_data[queue_len] = {0};
Queue q_posx, q_posy;

#define euler_queue_len 3
float pitch_data[euler_queue_len] = {0}, yaw_data[euler_queue_len] = {0};
Queue q_pitch, q_yaw;

int16_t y = 750;
int16_t pix_x = 0, pix_y = 0, last_pix_x = 0, last_pix_y = 0;
uint16_t data_count;
void mouse_init()
{
    queue_init(&q_posx, posx_data, queue_len, QUENE_ANALYZE_OPEN);
    queue_init(&q_posy, posy_data, queue_len, QUENE_ANALYZE_OPEN);
    queue_init(&q_pitch, pitch_data, euler_queue_len, QUENE_ANALYZE_OPEN);
    queue_init(&q_yaw, yaw_data, euler_queue_len, QUENE_ANALYZE_OPEN);
}

void mouse_data_input(float xy[2])
{
    data_count++;
    queue_input(&q_posx, xy[0]);
    queue_input(&q_posy, xy[1]);
}

int16_t angel_2_pix(float angel, uint8_t h)
{
    // return 750 * tanf(angel);
    if (h == 1)
    {
        return (int16_t)(57.3f * 19.2f * angel);
    }
    else
    {
        return (int16_t)(57.3f * 10.8f * angel);
    }
}

void mouse_cal_pix(float pitch, float yaw)
{
    queue_input(&q_pitch, pitch);
    queue_input(&q_yaw, yaw);
    pix_x = angel_2_pix(-q_yaw.mean, 1);
    pix_y = angel_2_pix((q_pitch.mean + 1.5708), 1);
    int16_t dx = pix_x - last_pix_x, dy = pix_y - last_pix_y;
    last_pix_x = pix_x;
    last_pix_y = pix_y;
    platform_printf("mouse:%f,%f,%d,%d,%d,%d\n", q_yaw.mean * 57.3, q_pitch.mean * 57.3, dx, dy, pix_x, pix_y);
    bsp_usb_handle_hid_mouse_report(dx, dy, 0);
}
