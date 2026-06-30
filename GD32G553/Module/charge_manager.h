#ifndef CHARGE_MANAGER_H
#define CHARGE_MANAGER_H

#include "bms_types.h"

typedef enum {
    BMS_CMD_RESULT_OK = 0,
    BMS_CMD_RESULT_ERROR = 1
} bms_command_result_t;

typedef enum {
    BMS_CMD_ERROR_NONE = 0,
    BMS_CMD_ERROR_BAD_LENGTH = 1,
    BMS_CMD_ERROR_UNKNOWN_COMMAND = 2,
    BMS_CMD_ERROR_INVALID_PARAM = 3,
    BMS_CMD_ERROR_FAULT_ACTIVE = 4,
    BMS_CMD_ERROR_INVALID_MODE = 5
} bms_command_error_t;

typedef struct {
    uint8_t result;
    uint8_t errorCode;
} bms_command_reply_t;

#define BMS_CMD_DIGITAL_POWER_SET             0x06U
#define BMS_CMD_SET_WORK_MODE                 0x07U
#define BMS_CMD_SET_FET_MASK                  0x08U

void Charge_Manager_Init(void);
void Charge_Manager_Get_Parameters(bms_charge_parameters_t *parameters);
bms_command_reply_t Charge_Manager_Set_Parameters(const bms_charge_parameters_t *parameters);
bms_command_reply_t Charge_Manager_Handle_Command(uint8_t command_id, uint8_t argument, uint8_t has_argument);
bms_command_reply_t Charge_Manager_Handle_Digital_Power_Command(uint8_t enable,
                                                               uint16_t target_voltage_mv,
                                                               uint16_t current_limit_ma);
uint8_t Charge_Manager_Is_Digital_Power_Active(void);
uint8_t Charge_Manager_Get_Work_Mode(void);
bms_command_reply_t Charge_Manager_Set_Work_Mode(uint8_t work_mode);
void Charge_Manager_Update(uint32_t period_ms,
                           const bms_afe_data_t *afe,
                           const bms_power_sample_t *power_sample,
                           bms_status_t *status);

#endif
