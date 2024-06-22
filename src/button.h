#ifndef BUTTON_H
#define BUTTON_H

#include <stdint.h>

void btn_task(void *p);
void btn_down_event_handler(uint8_t btn_index);
#endif // BUTTON_H