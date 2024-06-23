#include "wireless.h"
#include "task.h"
#include "platform_api.h"
#include "btstack_event.h"
#include "profile.h"
#include "wireless_data_parser.h"
static comm_mode_t comm_mode = MODE_BLE;

static uint8_t master_tx_len = 20;
static uint8_t slave_tx_len = 4;
static uint8_t tx_data[] = {0, 5, 4, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31};
static uint8_t rx_data[256];
static ING2P4G_RxPacket RxPkt111;
static ING2P4G_Config_t ing_2p4g_config;
static uint8_t continus_2g4 = 0;
QueueHandle_t wireless_rx_queue;

QueueHandle_t wireless_get_rx_queue()
{
    return wireless_rx_queue;
}

void wireless_send(uint8_t *data, uint16_t len)
{

    ing2p4g_status_t state = ing2p4g_start_2p4g_tx(len, data);
    if (state == ING2P4G_MODE_ERROR)
    {
        platform_printf("Not in 2.4g mode.\r\n");
    }
    else
    {
        platform_printf("tx result:%d.\r\n", state);
    }
}

void wireless_rx_continus()
{
    ing2p4g_status_t state;
    // ing2p4g_set_2g4_work_mode(MODE_SLAVE);
    continus_2g4_txrx_on();
    state = ing2p4g_start_2p4g_rx(master_tx_len, tx_data);
    if (state == ING2P4G_MODE_ERROR)
    {
        platform_printf("Not in 2.4g mode.\r\n");
    }
    else
    {
        platform_printf("rx continus result:%d.\r\n", state);
        if (state != ING2P4G_SUCCESS)
        {
            continus_2g4_txrx_off();
        }
    }
}

void wireless_tx_continus()
{
    ing2p4g_status_t state;
    continus_2g4_txrx_on();
    // ing2p4g_set_2g4_work_mode(MODE_MASTER);
    state = ing2p4g_start_2p4g_tx(master_tx_len, tx_data);
    if (state == ING2P4G_MODE_ERROR)
    {
        platform_printf("Not in 2.4g mode.\r\n");
    }
    else
    {
        platform_printf("tx continus result:%d.\r\n", state);
        if (state != ING2P4G_SUCCESS)
        {
            continus_2g4_txrx_off();
        }
    }
}

void continus_2g4_txrx_on(void)
{
    continus_2g4 = 1;
}

void continus_2g4_txrx_off(void)
{
    continus_2g4 = 0;
}
void ing24g_test_do_switch_to_2p4g(void)
{
    if (ing_2p4g_config.Mode == MODE_MASTER)
    {
        platform_printf("DO SWITCH 2.4G: MASTER.\n");
    }
    else
    {
        platform_printf("DO SWITCH 2.4G: SLAVE.\n");
    }
    ing2p4g_switch_to_2G4(&ing_2p4g_config);
}

void ing24g_test_do_switch_to_BLE(void)
{
    ing2p4g_switch_to_ble_mode_start();
    ing2p4g_switch_to_BLE();
}

// ================================================================================
static void ble_switch_to_ble_trigger(void)
{
    ing2p4g_work_state_t state;
    if (ing2p4g_get_state(&state) == ING2P4G_SUCCESS)
    {
        if (state == ING2P4G_STATE_IDLE)
        {
            // do switch.
            platform_printf("DO SWITCH BLE\n");
            ing24g_test_do_switch_to_BLE();
        }
        else
        {
            // 2.4G not idle, re-trigger switch to ble mode.
            ing24g_test_switch_mode_trigger(MODE_BLE);
        }
    }
    else
    {
        // is already BLE mode.
        platform_printf("Already BLE mode.\n");
    }
}

// ================================================================================

// ================================================================================

void ing24g_test_switch_mode_callback(void *data, uint16_t usr_val)
{
    comm_mode_t mode = (comm_mode_t)usr_val;
    switch (mode)
    {
    case MODE_2G4:
    {
        comm_mode = MODE_2G4;
        ble_switch_to_2p4g_trigger();
        break;
    }
    case MODE_BLE:
    {
        comm_mode = MODE_BLE;
        ble_switch_to_ble_trigger();
        break;
    }
    }
}

void ing24g_test_switch_mode_trigger(comm_mode_t mode)
{
    btstack_push_user_runnable(ing24g_test_switch_mode_callback, NULL, (uint16_t)mode);
}

// ================================================================================
static void percent_cnt(uint16_t T_CNT, ing2p4g_status_t status, int8_t rssi)
{
    static uint16_t test_cnt = 0;
    static uint16_t ack_cnt = 0;
    static uint16_t miss_cnt = 0;
    static uint32_t tick_start, tick_end;

    test_cnt++;
    if (status == ING2P4G_SUCCESS)
    {
        ack_cnt++;
    }
    else
    {
        miss_cnt++;
    }

    if (test_cnt >= T_CNT)
    {
        tick_end = platform_get_us_time();
        double rate = 1000 * ack_cnt / (float)(tick_end - tick_start);
        // platform_printf("tick_interval:%d us\n", (tick_end - tick_start));
        platform_printf("Test %d packet! miss: %d,rev: %d, rate: %.3fK pack/s, rssi:%d\r\n", T_CNT, miss_cnt, ack_cnt, rate, rssi);
        ack_cnt = 0;
        miss_cnt = 0;
        test_cnt = 0;
        tick_start = platform_get_us_time();
    }
}

ADDITIONAL_ATTRIBUTE static void EventIrqCallBack(void)
{
    static ing2p4g_work_mode_t mode;
    BaseType_t xHigherPriorityTaskWoken;
    ing2p4g_clear_event_int();
    ing2p4g_get_2g4_work_mode(&mode);
    ing2p4g_status_t status = ing2p4g_get_rx_data(&RxPkt111);
#if WIRELESS_SLAVE
    if (status == ING2P4G_SUCCESS)
    {
        xQueueSendFromISR(wireless_rx_queue, &RxPkt111, &xHigherPriorityTaskWoken);
    }
#endif
    tx_data[0]++;

    // gpio_pluse_num(1);
    if (continus_2g4 == 1)
    {
        if (mode == MODE_MASTER)
        {
            percent_cnt(1000, status, RxPkt111.RSSI);
            ing2p4g_start_2p4g_tx(master_tx_len, tx_data);
        }
        else
        {
            ing2p4g_start_2p4g_rx(slave_tx_len, tx_data);
        }
    }
}

static void RxPktIrqCallBack(void)
{
    // ing2p4g_err_bit_pos_t status = ing2p4g_get_rx_data(&RxPkt111);
    ing2p4g_clear_rx_int();
    // gpio_pluse_num(1);
}

ADDITIONAL_ATTRIBUTE static void TxPktIrqCallBack(void)
{
    // ing2p4g_err_bit_pos_t status = ing2p4g_get_rx_data(&RxPkt111);
    ing2p4g_clear_tx_int();
    // gpio_pluse_num(2);
}

void ing_2p4g_config_init(void)
{
    ing_2p4g_config.Mode = MODE_MASTER;
    ing_2p4g_config.AccAddr = 0x1234567A;
    ing_2p4g_config.PHY = LLE_PHY_2M;
    ing_2p4g_config.Channel = 2400;
    ing_2p4g_config.TXPOW = 63;
    ing_2p4g_config.WhiteEn = 0x1;
    ing_2p4g_config.WhiteIdx = 0x0;
    ing_2p4g_config.CRCInit = 0x123456;
    ing_2p4g_config.TimeOut = 1600; // 10000;//6.25s
    ing_2p4g_config.RxPktIntEn = 1;
    ing_2p4g_config.TxPktIntEn = 1;
}

void wireless_task(void *p)
{
    extern uint8_t bt_is_inited;
    ING2P4G_RxPacket rx_packet;
    while (!bt_is_inited)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    platform_printf("2.4g wireless init done\n");
    ing24g_test_switch_mode_trigger(MODE_2G4);

#if WIRELESS_SLAVE
    ing2p4g_set_2g4_work_mode(MODE_SLAVE);
    platform_printf("2.4g wireless slave mode\n");
    wireless_rx_continus();
#else
    ing2p4g_set_2g4_work_mode(MODE_MASTER);
    platform_printf("2.4g wireless master mode\n");
    // wireless_tx_continus("c");
#endif
#if WIRELESS_SLAVE == 0
    // 重置anchor
    uint8_t buff[6] = {0};
    buff[0] = 0xAA;
    buff[1] = 9;
    wireless_send(buff, 6);
#endif
    while (1)
    {
#if WIRELESS_SLAVE

        if (xQueueReceive(wireless_rx_queue, &rx_packet, 0xFFFFFFFF))
        {
            // platform_printf("Rx data: len:%d ", rx_packet.DataLen);
            wireless_data_parse(rx_packet.Data);

            //             for (uint16_t i = 0; i < rx_packet.DataLen; i++)
            // #if 1 // print String
            //                 platform_printf("%c", rx_packet.Data[i]);
            // #else
            //                 platform_printf("0x%02X", rx_packet.Data[i]);
            // #endif
            // platform_printf(".\r\n");
            // platform_printf("%.s .\r\n", rx_packet.DataLen, rx_packet.Data);
        }
#else
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        // vTaskDelete(NULL);
#endif
    }
}

void wireless_init()
{
#if WIRELESS_SLAVE
    wireless_rx_queue = xQueueCreate(10, sizeof(ING2P4G_RxPacket));
    if (wireless_rx_queue == 0)
    {
        platform_printf("wireless_que create fail\n");
    }
#endif
    ing2p4g_set_irq_callback(ING2P4G_CB_EVENT, EventIrqCallBack);
    ing2p4g_set_irq_callback(ING2P4G_CB_RX, RxPktIrqCallBack);
    ing2p4g_set_irq_callback(ING2P4G_CB_TX, TxPktIrqCallBack);
    ing_2p4g_config_init();
    xTaskCreate(wireless_task, "wireless_task", 256, NULL, 0, NULL);
}
