#include <stdio.h>
#include <string.h>
#include "ingsoc.h"
#include "platform_api.h"
#include "FreeRTOS.h"
#include "task.h"
#include "eflash.h"
#include "peripheral_i2c.h"
#include "board.h"
#include "sc7i22.h"
#include "uwb.h"
#include "../data/setup_soc.cgen"
#include "bsp_usb_hid.h"
#include "profile.h"
#include "wireless.h"

static uint32_t cb_hard_fault(hard_fault_info_t *info, void *_)
{
    platform_printf("HARDFAULT:\nPC : 0x%08X\nLR : 0x%08X\nPSR: 0x%08X\n"
                    "R0 : 0x%08X\nR1 : 0x%08X\nR2 : 0x%08X\nP3 : 0x%08X\n"
                    "R12: 0x%08X\n",
                    info->pc, info->lr, info->psr,
                    info->r0, info->r1, info->r2, info->r3, info->r12);
    for (;;)
        ;
}

static uint32_t cb_assertion(assertion_info_t *info, void *_)
{
    platform_printf("[ASSERTION] @ %s:%d\n",
                    info->file_name,
                    info->line_no);
    for (;;)
        ;
}

static uint32_t cb_heap_out_of_mem(uint32_t tag, void *_)
{
    platform_printf("[OOM] @ %d\n", tag);
    for (;;)
        ;
}

#define PRINT_PORT APB_UART0

uint32_t cb_putc(char *c, void *dummy)
{
    while (apUART_Check_TXFIFO_FULL(PRINT_PORT) == 1)
        ;
    UART_SendData(PRINT_PORT, (uint8_t)*c);
    return 0;
}

int fputc(int ch, FILE *f)
{
    cb_putc((char *)&ch, NULL);
    return ch;
}

void config_uart(uint32_t freq, uint32_t baud, uint32_t int_mask)
{
    UART_sStateStruct config;

    config.word_length = UART_WLEN_8_BITS;
    config.parity = UART_PARITY_NOT_CHECK;
    config.fifo_enable = 1;
    config.two_stop_bits = 0;
    config.receive_en = 1;
    config.transmit_en = 1;
    config.UART_en = 1;
    config.cts_en = 0;
    config.rts_en = 0;
    config.rxfifo_waterlevel = 1;
    config.txfifo_waterlevel = 1;
    config.ClockFrequency = freq;
    config.BaudRate = baud;

    apUART_Initialize(PRINT_PORT, &config, int_mask);
}

#define ARRAY_LEN(x) (sizeof(x) / sizeof(x[0]))

uint32_t uart_isr(void *user_data)
{
    __disable_irq();
    PRINT_PORT->IntClear = apUART_Get_all_raw_int_stat(PRINT_PORT);
    while (!apUART_Check_RXFIFO_EMPTY(PRINT_PORT))
    {
        volatile uint32_t trash = PRINT_PORT->DataRead;
        (void)trash;
    }
    config_uart(OSC_CLK_FREQ, 921600, 0);

    typedef void (*pFunction)(void);
    __disable_irq();
#if (INGCHIPS_FAMILY == INGCHIPS_FAMILY_918)
#define BOOT_ADDR 0x44000
#elif (INGCHIPS_FAMILY == INGCHIPS_FAMILY_916)
#define BOOT_ADDR 0x2042000
#endif
    {
        int i;
        SysTick->CTRL = 0;

#if (INGCHIPS_FAMILY == INGCHIPS_FAMILY_918)
        SYSCTRL_WriteBlockRst(0);
        SYSCTRL_WriteBlockRst(0x3ffffful);
#elif (INGCHIPS_FAMILY == INGCHIPS_FAMILY_916)
        for (i = 0; i < SYSCTRL_ITEM_NUMBER; i++)
        {
            SYSCTRL_ResetBlock((SYSCTRL_ResetItem)i);
            SYSCTRL_ReleaseBlock((SYSCTRL_ResetItem)i);
        }
#endif
        __set_MSP(*(uint32_t *)(BOOT_ADDR));
        ((pFunction)(*(uint32_t *)(BOOT_ADDR + 4)))();
    }

    return 0;
}

void setup_peripherals(void)
{
    int i;
    SYSCTRL_ClearClkGateMulti(0 | (1 << SYSCTRL_ClkGate_APB_TMR0)
#if (INGCHIPS_FAMILY == INGCHIPS_FAMILY_918)
                              | (1 << SYSCTRL_ClkGate_APB_GPIO)
#elif (INGCHIPS_FAMILY == INGCHIPS_FAMILY_916)
                              | (1 << SYSCTRL_ClkGate_APB_GPIO0) | (1 << SYSCTRL_ClkGate_APB_GPIO1) | (1 << SYSCTRL_ClkGate_APB_UART1)
#endif
                              | (1 << SYSCTRL_ClkGate_APB_PinCtrl) | (1 << SYSCTRL_ClkGate_APB_PWM) | (1 << SYSCTRL_ClkGate_APB_I2C0));
    config_uart(OSC_CLK_FREQ, 115200, 1 << bsUART_RECEIVE_INTENAB);

#if (INGCHIPS_FAMILY == INGCHIPS_FAMILY_918)
    PINCTRL_SetPadMux(IIC_SCL_PIN, IO_SOURCE_I2C0_SCL_OUT);
    PINCTRL_SetPadMux(IIC_SDA_PIN, IO_SOURCE_I2C0_SDA_OUT);
    PINCTRL_SelI2cSclIn(I2C_PORT, IIC_SCL_PIN);
#elif (INGCHIPS_FAMILY == INGCHIPS_FAMILY_916)
//    PINCTRL_SelI2cIn(I2C_PORT, IIC_SCL_PIN, IIC_SDA_PIN);
#else
#error unknown or unsupported chip family
#endif

    platform_set_irq_callback(PLATFORM_CB_IRQ_UART0, uart_isr, NULL);
}

float read_adc(int channel)
{
#if (INGCHIPS_FAMILY == INGCHIPS_FAMILY_918)
    ADC_Reset();
    ADC_PowerCtrl(1);

    ADC_SetClkSel(ADC_CLK_EN | ADC_CLK_128);
    ADC_SetMode(ADC_MODE_LOOP);
    ADC_EnableChannel(channel == 0 ? 1 : 0, 1);
    ADC_EnableChannel(channel, 1);
    ADC_Enable(1);

    while (ADC_IsChannelDataValid(channel) == 0)
        ;
    uint16_t voltage = ADC_ReadChannelData(channel);

    ADC_ClearChannelDataValid(channel);
    while (ADC_IsChannelDataValid(channel) == 0)
        ;
    voltage = ADC_ReadChannelData(channel);

    ADC_Enable(0);
    ADC_PowerCtrl(0);

    return voltage * (3.3f / 1024.f);
#elif (INGCHIPS_FAMILY == INGCHIPS_FAMILY_916)
#warning WIP: 916 read_adc
    return (platform_rand() & 0xff) * 3.3f / 256.f;
#endif
}

uint32_t on_deep_sleep_wakeup(void *dummy, void *user_data)
{
    (void)(dummy);
    (void)(user_data);
    setup_peripherals();
    return 0;
}

uint32_t query_deep_sleep_allowed(void *dummy, void *user_data)
{
    (void)(dummy);
    (void)(user_data);
    // TODO: return 0 if deep sleep is not allowed now; else deep sleep is allowed
    return 0;
}

#define BTN_IO_INDEX GIO_GPIO_3
uint8_t ready_output_xy = 0;
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

                wireless_send("wireless test", 14);
                // mouse_control_init();
                platform_printf("btn down\n");
                down_count++;
                if (down_count >= 2)
                {
                    ready_output_xy = 1;
                }
            }
        }
    }
}


int app_main()
{

    cube_soc_init();
    // setup handlers
    platform_set_evt_callback(PLATFORM_CB_EVT_PROFILE_INIT, setup_profile, NULL);
    platform_set_evt_callback(PLATFORM_CB_EVT_HARD_FAULT, (f_platform_evt_cb)cb_hard_fault, NULL);
    platform_set_evt_callback(PLATFORM_CB_EVT_ASSERTION, (f_platform_evt_cb)cb_assertion, NULL);
    platform_set_evt_callback(PLATFORM_CB_EVT_HEAP_OOM, (f_platform_evt_cb)cb_heap_out_of_mem, NULL);
    platform_set_evt_callback(PLATFORM_CB_EVT_ON_DEEP_SLEEP_WAKEUP, on_deep_sleep_wakeup, NULL);
    platform_set_evt_callback(PLATFORM_CB_EVT_QUERY_DEEP_SLEEP_ALLOWED, query_deep_sleep_allowed, NULL);
    platform_set_evt_callback(PLATFORM_CB_EVT_PUTC, (f_platform_evt_cb)cb_putc, NULL);
    SYSCTRL_Init();

    setup_peripherals();
    cube_setup_peripherals();

    // sc7122_init();
    // uwb_uart_init();
    // algorithm_init();
    bsp_usb_init();

    ing2p4g_init_dual_mode();
    wireless_init();
    xTaskCreate(btn_task, "btn_task", 128, NULL, 2, NULL);
    
    return 0;
}
