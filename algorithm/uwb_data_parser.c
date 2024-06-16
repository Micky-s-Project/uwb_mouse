#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "uwb.h"
#include "uwb_data_parser.h"

UWB_DATA_t uwb_data = {0};
SemaphoreHandle_t uwb_data_mutex = NULL;
void uwb_data_parse_task(void *p)
{
    QueueHandle_t uwb_queue = 0;
    char data_char;
    uint8_t uwb_data_pool[512] = {0};
    uint16_t uwb_data_pool_index = 0;
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
        if (xQueueReceive(uwb_queue, &data_char, 0xFFFFFFFF))
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
                // platform_printf("uwb_data_raw:%d,%d,%s\n", no_index[0], no_index[1], uwb_data_pool);
                // if (sscanf((char *)uwb_data_pool, "%*[^:]: %f, %*[^:]: %f", &(dis_tmp), &(aoa_tmp)) == 2)
                if (sscanf((char *)uwb_data_pool, "NO(%*d). D(m): %f, A: %d,%*d", &dis, &aoa) == 2)
                {
                    // platform_printf("uwb_data:%f,%d\n", dis, aoa);
                    if (abs(aoa) < 180)
                    {
                        platform_printf("uwb_error_data:%f,%d\n", dis, aoa);
                    }
                    // if (((fabsf(dis_tmp - uwb_data.dis) < 0.5 && fabsf(0.01745329 * aoa_tmp - uwb_data.aoa) < 0.5 ) || uwb_data.dis == 0) && dis_tmp > 0)
                    // {
                    //     if (xSemaphoreTake(uwb_data_mutex, 0) == pdTRUE)
                    //     {
                    //         uwb_data.dis = dis_tmp;
                    //         uwb_data.aoa = 0.01745329 * aoa_tmp;
                    //         uwb_data.x = uwb_data.dis * sinf(uwb_data.aoa);
                    //         uwb_data.y = uwb_data.dis * cosf(uwb_data.aoa);
                    //         uwb_data.ready = 1;
                    //         xSemaphoreGive(uwb_data_mutex);
                    //         // platform_printf("uwb_data:%f,%f,%f,%f\n", uwb_data.dis, uwb_data.aoa * 57.3, uwb_data.x, uwb_data.y);
                    //     }
                    // }
                    //
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
            uwb_data_run_count++;
            if (uwb_data_run_count == 1000)
            {
                uwb_data_run_count = 0;
                // platform_printf("uwb:%lld\n", tick);
            }
        }
    }
}

void algo_get_uwb_data_xy(float *pdata)
{
    pdata[0] = uwb_data.x;
    pdata[1] = uwb_data.y;
}

void algo_get_uwb_data_aoa(float *pdata)
{
    pdata[0] = uwb_data.aoa;
}

void get_uwb_data(UWB_DATA_t *p)
{
    if (uwb_data.ready)
    {
        if (xSemaphoreTake(uwb_data_mutex, 0) == pdTRUE)
        {
            memcpy(p, &uwb_data, sizeof(UWB_DATA_t));
            uwb_data.ready = 0;
            xSemaphoreGive(uwb_data_mutex);
        }
    }
}