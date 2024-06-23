#include "wireless_data_parser.h"
#include "platform_api.h"
#include "mouse.h"

int16_t pitch_raw = 0, yaw_raw = 0;
void wireless_data_parse(uint8_t data[6])
{
#if WIRELESS_SLAVE
    if (data[0] == 0xAA)
    {
        uint8_t btn = data[1];
        if (btn == 0)
        {
            pitch_raw = ((int16_t)data[2] | (int16_t)(data[3] << 8));
            yaw_raw = ((int16_t)data[4] | (int16_t)(data[5] << 8));
        }

        float aoa = 0, dis = 0;

        algo_get_uwb_data_aoa(&aoa);
        algo_get_uwb_data_dis(&dis);
        // platform_printf("btn:%d,%d,%d,%d,%d\n", btn, yaw_raw, pitch_raw, (int)(dis * 1000), (int)(aoa * 57.3));
        btn_down_event_handler(btn);
    }
#endif
}

float get_pitch()
{
    return (float)pitch_raw / 5730.0f;
}

float get_yaw()
{
    return (float)yaw_raw / 5730.0f;
}