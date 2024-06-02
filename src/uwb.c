#include "uwb.h"
#include <stdio.h>
#include <string.h>
#include "ingsoc.h"
#include "peripheral_uart.h"
#include "platform_api.h"

#define UWB_RX_PIN 4
#define UWB_PWR_PIN 5

#define RING_BUF_SIZE 256
char ring_buf[RING_BUF_SIZE];
char *ring_head_p;
uint16_t ring_seq = 0;
uint32_t uart1_isr(void *user_data)
{
    uint32_t status;
    // printf("@%x #%x #%x\n", APB_UART1->IntMask, APB_UART1->IntRaw, APB_UART1->Interrupt);
    while (1)
    {
        status = apUART_Get_all_raw_int_stat(APB_UART1);

        if (status == 0)
            break;

        APB_UART1->IntClear = status;

        if (status & (1 << bsUART_RECEIVE_INTENAB))
        {
            uint8_t uart_data = UART_ReceData(APB_UART1);
            ring_buf[ring_seq++] = uart_data;
            ring_seq = ring_seq > RING_BUF_SIZE ? 0 : ring_seq;
            platform_printf("%c", uart_data);
        }

        if (status & (1 << bsUART_TIMEOUT_INTENAB))
        {
            platform_printf("time out\n");
        }
        if (status & (1 << bsUART_FRAME_INTENAB))
        {
            platform_printf("bsUART_FRAME_INTENAB\n");
        }
        if (status & (1 << bsUART_PARITY_INTENAB))
        {
            platform_printf("bsUART_PARITY_INTENAB\n");
        }
        if (status & (1 << bsUART_BREAK_INTENAB))
        {
            platform_printf("bsUART_BREAK_INTENAB\n");
        }
    }
    return 0;
}

void uwb_uart_init()
{
    apUART_Enable_RECEIVE_INT(APB_UART1);
    PINCTRL_SelUartRxdIn(UART_PORT_1, UWB_RX_PIN);
    PINCTRL_SelUartIn(1, 4, 127);
    PINCTRL_SetPadMux(4, 104);
    platform_set_irq_callback(PLATFORM_CB_IRQ_UART1, uart1_isr, NULL);
}