#ifndef BMS_STATE_H
#define BMS_STATE_H

#include "bms_types.h"

void Bms_State_Init(void);
void Bms_State_Set_Status(const bms_status_t *status);
void Bms_State_Get_Status(bms_status_t *status);

#endif
