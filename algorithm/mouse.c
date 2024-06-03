
#include "my_queue.h"
#include "platform_api.h"
#include <math.h>

#define queue_len 50
float posx_data[queue_len] = {0}, posy_data[queue_len] = {0};
Queue q_posx, q_posy;

#define euler_queue_len 10
float pitch_data[euler_queue_len] = {0}, yaw_data[euler_queue_len] = {0};
Queue q_pitch, q_yaw;

int16_t y = 500;
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

void mouse_cal_pix(float pitch, float yaw)
{
    queue_input(&q_pitch, pitch);
    queue_input(&q_yaw, yaw);
    pix_x = y * tanf(q_yaw.mean);
    pix_y = y * tanf(q_pitch.mean);
    int16_t dx = pix_x - last_pix_x, dy = pix_y - last_pix_y;
    last_pix_x = pix_x;
    last_pix_y = pix_y;
    platform_printf("mouse:%d,%d\n", dx, dy);
}