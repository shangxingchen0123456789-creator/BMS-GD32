#ifndef CHARGE_MANAGER_INTERNAL_H
#define CHARGE_MANAGER_INTERNAL_H

#include "charge_manager.h"

#include "bms_board_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "afe_gd30bm2016.h"
#include "param_storage.h"
#include "power_control.h"
#include "power_manager.h"
#include "power_path_manager.h"
#include "safety_manager.h"

#include <string.h>

#define CMD_START_CHARGE                       0x01U
#define CMD_STOP_CHARGE                        0x02U
#define CMD_EMERGENCY_STOP                     0x03U
#define CMD_CLEAR_FAULT                        0x04U
#define CMD_SWITCH_MODE                        0x05U
#define CMD_DIGITAL_POWER_SET                  BMS_CMD_DIGITAL_POWER_SET
#define CMD_SET_WORK_MODE                      BMS_CMD_SET_WORK_MODE
#define CMD_SET_FET_MASK                       BMS_CMD_SET_FET_MASK

#define PACK_OVP_MARGIN_MV                     BMS_DEFAULT_OUTPUT_OVP_MARGIN_MV
#define TRICKLE_ENTER_CELL_MV                  3100U
#define TRICKLE_EXIT_CELL_MV                   3200U
#define CV_ENTRY_MARGIN_MV                     200U
#define CV_DONE_CONFIRM_COUNT                  100U
#define CV_DONE_DPM_MARGIN_MA                  20U
#define TRICKLE_CURRENT_MA                     BMS_DEFAULT_TRICKLE_CURRENT_MA
#define MANUAL_FET_TAKEOVER_DELAY_MS           25U
#define CHARGE_PATH_SETTLE_MS                  300U
#define DIGITAL_POWER_AFELESS_FAULT_MASK       (BMS_FAULT_CELL_OVP | \
                                                BMS_FAULT_CELL_UVP | \
                                                BMS_FAULT_AFE_COMM | \
                                                BMS_FAULT_INPUT_UV | \
                                                BMS_FAULT_BATTERY_DISCONNECT | \
                                                BMS_FAULT_SHORT_CIRCUIT | \
                                                BMS_FAULT_FUSE | \
                                                BMS_FAULT_AFE_PROTECTION)
#define DIGITAL_POWER_STARTUP_IGNORE_MASK      (DIGITAL_POWER_AFELESS_FAULT_MASK | \
                                                BMS_FAULT_PACK_OVP)
#define AFE_RECOVERABLE_LATCH_FAULT_MASK       (BMS_FAULT_CHARGE_OCP | \
                                                BMS_FAULT_SHORT_CIRCUIT | \
                                                BMS_FAULT_AFE_PROTECTION)

typedef struct {
    bms_charge_parameters_t params;
} charge_manager_config_context_t;

typedef struct {
    bms_charge_state_t state;
    uint8_t runRequest;
    uint8_t mode;
    uint8_t workMode;
    uint16_t cvDoneCounter;
} charge_manager_control_context_t;

typedef struct {
    uint8_t enabled;
    uint16_t targetVoltageMv;
    uint16_t currentLimitMa;
} charge_manager_digital_context_t;

typedef struct {
    uint32_t latchedFaults;
    uint32_t presentFaults;
    uint8_t outputOvpConfirmCount;
    uint8_t digitalOutputOvpConfirmCount;
    uint8_t chargeOcpConfirmCount;
} charge_manager_fault_context_t;

typedef struct {
    uint8_t active;
    uint8_t mask;
} charge_manager_manual_fet_context_t;

typedef struct {
    uint16_t settleRemainingMs;
    uint8_t lastBatteryPathEnabled;
    uint16_t settleTargetMv;
    uint16_t preconnectTargetMv;
} charge_manager_path_context_t;

typedef struct {
    charge_manager_config_context_t config;
    charge_manager_control_context_t control;
    charge_manager_digital_context_t digital;
    charge_manager_fault_context_t fault;
    charge_manager_manual_fet_context_t manualFet;
    charge_manager_path_context_t path;
} charge_manager_context_t;

extern charge_manager_context_t g_charge_manager;

bms_command_reply_t Reply_Ok(void);
bms_command_reply_t Reply_Error(uint8_t error_code);
void Charge_Manager_Default_Parameters(bms_charge_parameters_t *parameters);
void Charge_Manager_Clamp_Parameters(bms_charge_parameters_t *parameters);
uint8_t Charge_Manager_Validate_Parameters(const bms_charge_parameters_t *parameters);
uint8_t Charge_Manager_Validate_Digital_Power(uint16_t target_voltage_mv,
                                              uint16_t current_limit_ma);

uint16_t Charge_Manager_Calc_Preconnect_Target_Mv(const bms_afe_data_t *afe,
                                                  const bms_charge_parameters_t *params);
uint16_t Charge_Manager_Preconnect_Target_Mv(const bms_afe_data_t *afe,
                                             const bms_power_sample_t *power_sample,
                                             const bms_charge_parameters_t *params);
uint16_t Charge_Manager_Preconnect_Current_Limit_Ma(const bms_afe_data_t *afe,
                                                    const bms_power_sample_t *power_sample,
                                                    uint32_t period_ms);
uint8_t Charge_Manager_Preconnect_Can_Drive(const bms_afe_data_t *afe,
                                            const bms_power_sample_t *power_sample);
uint8_t Charge_Manager_Cv_Entry_Ready(const bms_afe_data_t *afe,
                                      const bms_power_sample_t *power_sample,
                                      const bms_charge_parameters_t *params);
void Charge_Manager_Clear_Manual_Fet(void);
void Charge_Manager_Clear_Path_Settle_State_Unlocked(void);
void Charge_Manager_Clear_Path_Settle_Unlocked(void);
void Charge_Manager_Clear_Path_Settle(void);
void Charge_Manager_Clear_Path_Settle_Only(void);
uint8_t Charge_Manager_Path_Settle_Active(void);
uint8_t Charge_Manager_Battery_Path_Rising_Edge(void);
void Charge_Manager_Update_Path_Settle(uint8_t battery_path_enabled,
                                       const bms_afe_data_t *afe,
                                       const bms_charge_parameters_t *params);
void Charge_Manager_Decay_Path_Settle(uint32_t period_ms);

uint32_t Charge_Manager_Filter_Digital_Power_Afeless_Faults(uint32_t faults);
uint8_t Charge_Manager_Afeless_Filter_Active(uint8_t digital_power_enabled,
                                             uint8_t power_requested);
uint8_t Charge_Manager_Afe_Sample_Valid(const bms_afe_data_t *afe);
uint8_t Charge_Manager_Temperature_Valid(int16_t temperature_x10);
uint8_t Charge_Manager_Power_Ocp_Check_Active(uint8_t power_requested,
                                              const power_control_state_t *power_state);
void Charge_Manager_Reset_Output_Ovp_Counters(void);
void Charge_Manager_Reset_Charge_Ocp_Counter(void);
uint8_t Charge_Manager_Output_Ovp_Confirmed(uint16_t output_voltage_mv,
                                            uint16_t target_voltage_mv,
                                            uint8_t run_request,
                                            uint8_t digital_power);
void Charge_Manager_Record_Faults(uint32_t realtime_faults, uint32_t latchable_faults);
uint32_t Charge_Manager_Collect_Faults(const bms_afe_data_t *afe,
                                       const bms_power_sample_t *power_sample,
                                       uint8_t power_requested);
uint32_t Charge_Manager_Collect_Digital_Power_Faults(const bms_power_sample_t *power_sample,
                                                     uint16_t target_voltage_mv,
                                                     uint16_t current_limit_ma);

uint16_t Charge_Manager_Target_Current_For_State(bms_charge_state_t state);
uint8_t Charge_Manager_Control_Mode_For_State(uint8_t mode, bms_charge_state_t state);
uint8_t Charge_Manager_Cv_Done_Allowed(const bms_power_sample_t *power_sample,
                                       const bms_charge_parameters_t *params);
bms_command_reply_t Charge_Manager_Handle_Fet_Mask_Command(uint8_t fet_mask);

#endif
