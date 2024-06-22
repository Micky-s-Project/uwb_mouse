#ifndef UWB_DATA_PARSER_H
#define UWB_DATA_PARSER_H

#include <stdint.h>

typedef struct UWB_DATA_t
{
    float x;
    float y;
    float dis;
    float aoa;
    uint8_t ready;
} UWB_DATA_t;

void uwb_data_parse_task(void *p);
void algo_get_uwb_data_xy(float *pdata);
void algo_get_uwb_data_aoa(float *pdata);
void algo_get_uwb_data_dis(float *pdata);

uint8_t get_uwb_data(UWB_DATA_t *p);
void uwb_parser_test();
#endif // UWB_DATA_PARSER_H