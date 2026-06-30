#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include "bms_types.h"
#include "param_storage.h"

typedef struct {
    uint16_t availablePowerW;
    uint16_t requestedCurrentMa;
    uint16_t limitedCurrentMa;
    uint16_t inputLimitedCurrentMa;
    uint16_t thermalLimitedCurrentMa;
    uint8_t deratingActive;
} power_manager_state_t;

void Power_Manager_Init(void);
void Power_Manager_Configure(const bms_system_config_t *config);
void Power_Manager_Update(uint32_t period_ms,
                          const bms_afe_data_t *afe,
                          const bms_power_sample_t *sample,
                          const bms_charge_parameters_t *parameters,
                          uint8_t charge_state,
                          uint32_t faults);
uint16_t Power_Manager_Limit_Current(uint16_t requested_current_ma);
void Power_Manager_Set_Input_Limit(uint16_t current_limit_ma, uint16_t power_limit_w);
void Power_Manager_Get_State(power_manager_state_t *state);

#endif
