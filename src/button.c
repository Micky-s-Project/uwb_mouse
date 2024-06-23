
#include "button.h"
#include <stdio.h>
#include <string.h>
#include "board.h"
#include "platform_api.h"
#include "uwb.h"

#define BTN_IO_INDEX GIO_GPIO_3

// uint8_t ready_output_xy = 0;
__attribute__((weak)) void btn_down_event_handler(uint8_t btn_index)
{
}

void btn_task(void *p)
{
    PINCTRL_SetPadMux(BTN_IO_INDEX, IO_SOURCE_GPIO);
    PINCTRL_Pull(BTN_IO_INDEX, PINCTRL_PULL_UP);
    GIO_SetDirection((GIO_Index_t)BTN_IO_INDEX, (GIO_Direction_t)GIO_DIR_INPUT);
    uint8_t btn_state;
    uint8_t last_btn_state;
    uint8_t down_count = 0;
    platform_printf("btn_task inited\n");
    vTaskDelay(10);
    while (1)
    {
        btn_state = GIO_ReadValue(BTN_IO_INDEX);
        vTaskDelay(10);
        if (last_btn_state != btn_state && btn_state == GIO_ReadValue(BTN_IO_INDEX))
        {
            last_btn_state = btn_state;
            if (!btn_state)
            {
                // mouse_control_init();
                platform_printf("btn down\n");
                // down_count++;
                // if (down_count >= 2)
                // {
                //     ready_output_xy = 1;
                // }

                btn_down_event_handler(1);
            }
        }
    }
}
