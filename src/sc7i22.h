
#ifndef __SI7I22_H__
#define __SI7I22_H__

#include "ingsoc.h"
#include "FreeRTOS.h"
#include "semphr.h"


void sc7122_init();
SemaphoreHandle_t get_imu_data_mutex();
#endif