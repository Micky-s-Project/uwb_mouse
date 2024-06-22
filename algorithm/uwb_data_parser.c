#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "uwb.h"
#include "uwb_data_parser.h"
uint8_t uwb_data_pool[512] = {0};
uint16_t uwb_data_pool_index = 0;
static UWB_DATA_t uwb_data = {0};
static SemaphoreHandle_t uwb_data_mutex = NULL;
void uwb_parser_test()
{
    platform_printf("uwb_data_raw:%s,%d\n", uwb_data_pool, uwb_data_pool_index);
}
void uwb_data_parse_task(void *p)
{

    QueueHandle_t uwb_queue = 0;

    char data_char;

    int16_t no_index[2] = {-1, -1};
    int16_t uwb_data_run_count = 0;
    int aoa = 0;
    float dis = 0;
    // return;
    while (uwb_queue == NULL)
    {
        uwb_queue = get_uwb_queue();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    while (uwb_data_mutex == NULL)
    {
        uwb_data_mutex = xSemaphoreCreateMutex();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    platform_printf("uwb_data_parse_task inited!\n");

    while (1)
    {
        // vTaskDelay(pdMS_TO_TICKS(10));
        if (xQueueReceive(uwb_queue, &data_char, pdMS_TO_TICKS(1000)))
        {
            // platform_printf("%c", data_char);

            if (data_char == 'O' && uwb_data_pool[uwb_data_pool_index - 1] == 'N')
            {
                if (no_index[0] == -1)
                {
                    no_index[0] = 0;
                }
                else
                {
                    no_index[1] = uwb_data_pool_index;
                }
            }

            uwb_data_pool[uwb_data_pool_index++] = data_char;
            if (no_index[0] != -1 && no_index[1] != -1)
            {
                float dis_tmp = 0, aoa_tmp = 0;
                // platform_printf("uwb_data_raw:%s\n", uwb_data_pool);
                if (sscanf((char *)uwb_data_pool, "%*[^:]: %f, %*[^:]: %f", &(dis_tmp), &(aoa_tmp)) == 2)
                // if (sscanf((char *)uwb_data_pool, "NO(%*d). D(m): %f, A: %d,%*d", &dis, &aoa) == 2)
                {
                    // platform_printf("uwb_data:%d,%d\n", (int)(dis_tmp * 1000), (int)aoa_tmp);
                    if (((fabsf(dis_tmp - uwb_data.dis) < 0.5 && fabsf(0.01745329 * aoa_tmp - uwb_data.aoa) < 0.5) || uwb_data.dis == 0) && dis_tmp > 0)
                    {
                        if (xSemaphoreTake(uwb_data_mutex, 0) == pdTRUE)
                        {
                            uwb_data.dis = dis_tmp;
                            uwb_data.aoa = 0.01745329 * aoa_tmp;
                            uwb_data.x = uwb_data.dis * sinf(uwb_data.aoa);
                            uwb_data.y = uwb_data.dis * cosf(uwb_data.aoa);
                            uwb_data.ready = 1;
                            xSemaphoreGive(uwb_data_mutex);
                            // platform_printf("uwb_data:%d,%d,%d,%d\n", (int)(uwb_data.dis * 1000), (int)(uwb_data.aoa * 57.3 * 1000), (int)(uwb_data.x * 1000), (int)(uwb_data.y * 1000));
                        }
                    }
                }

                memset(uwb_data_pool, 0, 256);
                uwb_data_pool[0] = 'N';
                uwb_data_pool[1] = 'O';
                uwb_data_pool_index = 2;
                no_index[0] = 0;
                no_index[1] = -1;
            }
            if (uwb_data_pool_index == 256)
            {
                memset(uwb_data_pool, 0, 256);
                uwb_data_pool_index = 0;
            }
        }
        else
        {
            // uwb串口死机重启
            platform_printf("reinit\n");
            uwb_parser_test();
            uwb_uart_reset();
        }
    }
}

void algo_get_uwb_data_xy(float *pdata)
{
    if (xSemaphoreTake(uwb_data_mutex, 0) == pdTRUE)
    {
        pdata[0] = uwb_data.x;
        pdata[1] = uwb_data.y;
        xSemaphoreGive(uwb_data_mutex);
    }
}

void algo_get_uwb_data_aoa(float *pdata)
{
    if (xSemaphoreTake(uwb_data_mutex, 0) == pdTRUE)
    {
        pdata[0] = uwb_data.aoa;
        xSemaphoreGive(uwb_data_mutex);
    }
}

void algo_get_uwb_data_dis(float *pdata)
{
    if (xSemaphoreTake(uwb_data_mutex, 0) == pdTRUE)
    {
        pdata[0] = uwb_data.dis;
        xSemaphoreGive(uwb_data_mutex);
    }
}

uint8_t get_uwb_data(UWB_DATA_t *p)
{
    if (uwb_data.ready)
    {
        if (xSemaphoreTake(uwb_data_mutex, 0) == pdTRUE)
        {
            memcpy(p, &uwb_data, sizeof(UWB_DATA_t));
            uwb_data.ready = 0;
            xSemaphoreGive(uwb_data_mutex);
        }
        return 1;
    }
    else
    {
        return 0;
    }
}
