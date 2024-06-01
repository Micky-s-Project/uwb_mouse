#include "uwb.h"
#include <stdio.h>
#include <string.h>
#include "ingsoc.h"
#include "peripheral_uart.h"
#include "platform_api.h"

#define UWB_RX_PIN 4
#define UWB_PWR_PIN 5

#define bsDMA_INT_C_MASK 1
#define bsDMA_DST_REQ_SEL 4
#define bsDMA_SRC_REQ_SEL 8
#define bsDMA_DST_ADDR_CTRL 12
#define bsDMA_SRC_ADDR_CTRL 14
#define bsDMA_DST_MODE 16
#define bsDMA_SRC_MODE 17
#define bsDMA_DST_WIDTH 18
#define bsDMA_SRC_WIDTH 21
#define bsDMA_SRC_BURST_SIZE 24

#define DMA_RX_CHANNEL_ID 1
#define DMA_TX_CHANNEL_ID 0

DMA_Descriptor test __attribute__((aligned(8)));
char dst[256];
char src[] = "Finished to receive a frame!\n";

void setup_peripheral_dma()
{
    printf("setup_peripheral_dma\n");
    SYSCTRL_ClearClkGate(SYSCTRL_ITEM_APB_DMA);

    // 配置DMA接收
    APB_DMA->Channels[1].Descriptor.Ctrl = ((uint32_t)0x0 << bsDMA_INT_C_MASK) | ((uint32_t)0x0 << bsDMA_DST_REQ_SEL) | ((uint32_t)0x0 << bsDMA_SRC_REQ_SEL) | ((uint32_t)0x0 << bsDMA_DST_ADDR_CTRL) | ((uint32_t)0x2 << bsDMA_SRC_ADDR_CTRL) // DMA_ADDRESS_FIXED
                                           | ((uint32_t)0x0 << bsDMA_DST_MODE) | ((uint32_t)0x1 << bsDMA_SRC_MODE)                                                                                                                             //
                                           | ((uint32_t)0x0 << bsDMA_DST_WIDTH) | ((uint32_t)0x0 << bsDMA_SRC_WIDTH) | ((uint32_t)0x2 << bsDMA_SRC_BURST_SIZE);                                                                                // 4 transefers

    APB_DMA->Channels[1].Descriptor.SrcAddr = (uint32_t)&APB_UART1->DataRead;
    APB_DMA->Channels[1].Descriptor.DstAddr = (uint32_t)dst;
    APB_DMA->Channels[1].Descriptor.TranSize = 48;

    DMA_EnableChannel(1, &APB_DMA->Channels[1].Descriptor);
}

// 添加UART通过DMA发送的配置
void UART_trigger_DmaSend(void)
{
    DMA_PrepareMem2Peripheral(&test,
                              SYSCTRL_DMA_UART1_TX,
                              src, strlen(src),
                              DMA_ADDRESS_INC, 0);
    DMA_EnableChannel(DMA_TX_CHANNEL_ID, &test);
}

void setup_peripheral_uart()
{
    APB_UART1->FifoSelect = (0 << bsUART_TRANS_INT_LEVEL) |
                            (0X7 << bsUART_RECV_INT_LEVEL);
    APB_UART1->IntMask = (0 << bsUART_RECEIVE_INTENAB) | (0 << bsUART_TRANSMIT_INTENAB) |
                         (1 << bsUART_TIMEOUT_INTENAB);
    APB_UART1->Control = 1 << bsUART_RECEIVE_ENABLE |
                         1 << bsUART_TRANSMIT_ENABLE |
                         1 << bsUART_ENABLE |
                         0 << bsUART_CTS_ENA |
                         0 << bsUART_RTS_ENA;
}

uint32_t uart1_isr(void *user_data)
{
    uint32_t status;
    printf("@%x #%x #%x\n", APB_UART1->IntMask, APB_UART1->IntRaw, APB_UART1->Interrupt);

    while (1)
    {
        status = apUART_Get_all_raw_int_stat(APB_UART1);

        if (status == 0)
            break;

        APB_UART1->IntClear = status;

        // rx timeout_int
        if (status & (1 << bsUART_TIMEOUT_INTENAB))
        {
            while (apUART_Check_RXFIFO_EMPTY(APB_UART1) != 1)
            {
                char c = APB_UART1->DataRead;
                int index = APB_DMA->Channels[1].Descriptor.DstAddr - (uint32_t)dst;
                dst[index] = c;
                if (index == 255)
                {
                    APB_DMA->Channels[1].Descriptor.DstAddr = (uint32_t)dst;
                    UART_trigger_DmaSend();
                }
                else
                    APB_DMA->Channels[1].Descriptor.DstAddr++;

                APB_DMA->Channels[1].Descriptor.TranSize = 48;
            }
            printf("\nlen=%d, dst = %s\n", APB_DMA->Channels[1].Descriptor.TranSize, dst);
            APB_DMA->Channels[1].Descriptor.DstAddr = (uint32_t)dst;
            UART_trigger_DmaSend();
            
        }
    }
    return 0;
}

void uwb_uart_init()
{
    platform_set_irq_callback(PLATFORM_CB_IRQ_UART1, uart1_isr, NULL);
    PINCTRL_SelUartRxdIn(UART_PORT_1, UWB_RX_PIN);
    setup_peripheral_uart();
    setup_peripheral_dma();
}