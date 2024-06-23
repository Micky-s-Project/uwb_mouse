#ifndef WIRELESS_DATA_PARSER_H
#define WIRELESS_DATA_PARSER_H

#include <stdint.h>
void wireless_data_parse(uint8_t data[6]);
float get_pitch();
float get_yaw();
#endif // WIRELESS_DATA_PARSER_H