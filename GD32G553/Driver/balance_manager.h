#ifndef BALANCE_MANAGER_H
#define BALANCE_MANAGER_H

#include "bms_types.h"

void Balance_Manager_Init(void);
uint16_t Balance_Manager_Update(const bms_afe_data_t *afe,
                                const bms_charge_parameters_t *parameters,
                                uint8_t charge_state);

#endif
