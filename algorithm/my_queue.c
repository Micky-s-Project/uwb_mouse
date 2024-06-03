#include "my_queue.h"
#include <math.h>
/**
 * @description: 初始化队列
 * @param {Queue} *queue_addr 队列地址
 * @param {float} *data_addr 队列数据地址
 * @param {uint8_t} len 队列数据容量
 * @param {bool} open_analyze 是否开启数据队列统计
 * @return {bool} 是否初始化成功
 */
bool queue_init(Queue *queue_addr, float *data_addr, uint8_t len, bool open_analyze)
{
    // 如果队列没有经过初始化，则进行初始化
    if (queue_addr->_is_inited == false) {
        queue_addr->_data_addr = data_addr;
        queue_addr->len = len;
        queue_addr->_open_analyze = open_analyze;
        queue_addr->_is_inited = true;
        return true;
    } else {
        return false;
    }
}

/**
 * @description: 向队列里加入数据
 * @param {Queue} *queue_addr 队列地址
 * @param {float} input_data 插入的数据
 * @return {float} 弹出的数据
 */
float queue_input(Queue *queue_addr, float input_data)
{
    // 如果队列经过初始化，则加入数据
    if (queue_addr->_is_inited) {
        uint8_t index = queue_addr->_data_i;
        uint8_t len = queue_addr->len;

        // update index
        index += 1;
        if (index == len) {
            index = 0;
        }
        queue_addr->_data_i = index;

        // input new data
        float pop_data = queue_addr->_data_addr[index];
        queue_addr->_data_addr[index] = input_data;

        // 如果打开数据统计
        if (queue_addr->_open_analyze) {
            uint8_t max_index = queue_addr->_max_i;
            uint8_t min_index = queue_addr->_min_i;
            float max = queue_addr->max;
            float min = queue_addr->min;
            // cal mean
            queue_addr->sum -= pop_data;
            queue_addr->sum += input_data;
            queue_addr->mean = queue_addr->sum * 1.0 / len;

            // cal max
            if (max_index == index) {
                max = 0;
                for (uint8_t i = 0; i < len; i++) {
                    if (queue_addr->_data_addr[i] > max) {
                        max = queue_addr->_data_addr[i];
                        max_index = i;
                    }
                }
            } else if (input_data > max) {
                max = input_data;
                max_index = index;
            }
            queue_addr->max = max;
            queue_addr->_max_i = max_index;

            // cal min
            if (min_index == index) {
                min = 0;
                for (uint8_t i = 0; i < len; i++) {
                    if (queue_addr->_data_addr[i] < min) {
                        min = queue_addr->_data_addr[i];
                        min_index = i;
                    }
                }
            } else if (input_data < min) {
                min = input_data;
                min_index = index;
            }
            queue_addr->min = min;
            queue_addr->_min_i = min_index;
            if (fabsf(max) > fabsf(min)) {
                queue_addr->abs_max = fabsf(max);
            } else {
                queue_addr->abs_max = fabsf(min);
            }
        }
        return pop_data;
    } else {
        return 0;
    }
}

/**
 * @description: 获取前n次插入的数据 n < len
 * @param {Queue} *queue_addr
 * @param {uint8_t} last_num
 * @return {float} 前n次插入的数据
 */
float queue_get(Queue *queue_addr, uint8_t last_num)
{
    if (last_num < queue_addr->len) {
        return queue_addr->_data_addr[(queue_addr->_data_i - last_num + queue_addr->len) % queue_addr->len];
    } else {
        return 0;
    }
}