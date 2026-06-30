#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "bms_types.h"

void App_Tasks_Start(void);
uint8_t App_Tasks_Get_Latest_Power_Sample(bms_power_sample_t *sample);

#endif
