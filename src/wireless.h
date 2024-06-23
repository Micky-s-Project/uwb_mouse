#ifndef __WIRELESS_H__
#define __WIRELESS_H__

#include "ing_2p4g.h"
#include "FreeRTOS.h"
#include "queue.h"

#define WIRELESS_SLAVE 1

void ing24g_test_do_switch_to_2p4g(void);
void ing24g_test_switch_mode_handler(void);
void ing24g_test_init(void);

void ing24g_test_switch_mode_trigger(comm_mode_t mode);
void continus_2g4_txrx_on(void);
void continus_2g4_txrx_off(void);

void wireless_init();
void wireless_send(uint8_t *data, uint16_t len);
void wireless_tx_continus();
void wireless_rx_continus();
QueueHandle_t wireless_get_rx_queue();

#endif /* __WIRELESS_H__ */