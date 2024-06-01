#ifndef __QUEUE_H_
#define __QUEUE_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

typedef struct
{
    bool _is_inited;     // 是否初始化
    float *_data_addr; // 数据
    uint8_t len;         // 容量
    uint8_t _data_i;     // 当前入队数据指针

    bool _open_analyze; // 是否需要以下数据统计
    float _max_i;     // 当前最大数据指针
    float _min_i;     // 当前最小数据指针
    float max;        // 当前最大数据
    float min;        // 当前最小数据
    float abs_max;    // 当前绝对值最大数据
    float mean;         // 当前数据平均值
    float sum;        // 当前数据总和
} Queue;
enum QUENE_ANALYZE_SWITCH {
    QUENE_ANALYZE_CLOSE = 0,
    QUENE_ANALYZE_OPEN
};
bool queue_init(Queue *queue_addr, float *data_addr, uint8_t len, bool open_analyze);
float queue_input(Queue *queue_addr, float input_data);
float queue_get(Queue *queue_addr, uint8_t index);

#endif
