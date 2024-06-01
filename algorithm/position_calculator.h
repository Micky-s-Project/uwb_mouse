#ifndef POSITION_CALCULATOR_H
#define POSITION_CALCULATOR_H

#include <stdint.h>

void position_calculate(float t, uint8_t uwb_ready);
void position_calculator_get_pos(float pdata[2]);

#endif // POSITION_CALCULATOR_H