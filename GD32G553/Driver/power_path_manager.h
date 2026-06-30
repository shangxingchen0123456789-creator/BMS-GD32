#ifndef POWER_PATH_MANAGER_H
#define POWER_PATH_MANAGER_H

#include "bms_types.h"

#define POWER_PATH_PRECONNECT_FLAG_READY      (1U << 0)
#define POWER_PATH_PRECONNECT_FLAG_ACTIVE     (1U << 1)
#define POWER_PATH_PRECONNECT_FLAG_VOUT_HIGH  (1U << 2)
#define POWER_PATH_PRECONNECT_FLAG_VOUT_LOW   (1U << 3)

#define POWER_PATH_PRECONNECT_REASON_OK       0U
#define POWER_PATH_PRECONNECT_REASON_NO_INPUT 1U
#define POWER_PATH_PRECONNECT_REASON_NO_SAMPLE 2U
#define POWER_PATH_PRECONNECT_REASON_FULL     3U
#define POWER_PATH_PRECONNECT_REASON_VOUT_LOW 4U
#define POWER_PATH_PRECONNECT_REASON_VOUT_HIGH 5U
#define POWER_PATH_PRECONNECT_REASON_FAULT    6U
#define POWER_PATH_PRECONNECT_REASON_IDLE     7U
#define POWER_PATH_PRECONNECT_REASON_FET_ON_FAILED 8U
#define POWER_PATH_PRECONNECT_REASON_PRECHARGE_FET_FAILED 9U

typedef struct {
    uint8_t externalPowerPresent;
    uint8_t batteryPathEnabled;
    uint8_t pathOffCommanded;
    uint8_t preconnectFlags;
    uint8_t preconnectReason;
    uint8_t preconnectConfirmCount;
    uint16_t preconnectDeltaMv;
    uint16_t preconnectThresholdMv;
} power_path_manager_state_t;

void Power_Path_Manager_Init(void);
void Power_Path_Manager_Force_Off(void);
void Power_Path_Manager_Update(const bms_afe_data_t *afe,
                               const bms_power_sample_t *power_sample,
                               const bms_charge_parameters_t *parameters,
                               bms_charge_state_t charge_state,
                               uint32_t fault_bitmap);
void Power_Path_Manager_Get_State(power_path_manager_state_t *state);

#endif
