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

/*
 * 充电监督状态机。
 *
 * 本模块统一管理上位机命令、充电参数、锁存故障和高层充电状态。
 * 它不直接操作 PWM 寄存器，而是向 power_control.c 下发电压/电流目标。
 */
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

static bms_charge_parameters_t s_params;
static bms_charge_state_t s_state;
static uint8_t s_run_request;
static uint8_t s_mode;
static uint8_t s_work_mode;
static uint32_t s_latched_faults;
static uint32_t s_present_faults;
static uint16_t s_cv_done_counter;
static uint8_t s_digital_power_enabled;
static uint16_t s_digital_power_target_voltage_mv;
static uint16_t s_digital_power_current_limit_ma;
static uint8_t s_output_ovp_confirm_count;
static uint8_t s_digital_output_ovp_confirm_count;
static uint8_t s_charge_ocp_confirm_count;
static uint8_t s_manual_fet_active;
static uint8_t s_manual_fet_mask;
static uint16_t s_path_settle_remaining_ms;
static uint8_t s_last_battery_path_enabled;
static uint16_t s_path_settle_target_mv;
static uint16_t s_preconnect_target_mv;

static bms_command_reply_t Reply_Ok(void)
{
    bms_command_reply_t reply;

    reply.result = (uint8_t)BMS_CMD_RESULT_OK;
    reply.errorCode = (uint8_t)BMS_CMD_ERROR_NONE;
    return reply;
}

static bms_command_reply_t Reply_Error(uint8_t error_code)
{
    bms_command_reply_t reply;

    reply.result = (uint8_t)BMS_CMD_RESULT_ERROR;
    reply.errorCode = error_code;
    return reply;
}

static void Charge_Manager_Default_Parameters(bms_charge_parameters_t *parameters)
{
    /* 默认参数对应 bms_types.h 中的 9 串锂电池方案。 */
    parameters->targetVoltageMv = BMS_DEFAULT_TARGET_VOLTAGE_MV;
    parameters->targetCurrentMa = BMS_DEFAULT_TARGET_CURRENT_MA;
    parameters->cutoffCurrentMa = BMS_DEFAULT_CUTOFF_CURRENT_MA;
    parameters->cellOvpMv = BMS_DEFAULT_CELL_OVP_MV;
    parameters->cellUvpMv = BMS_DEFAULT_CELL_UVP_MV;
    parameters->tempOtpX10 = BMS_DEFAULT_TEMP_OTP_X10;
    parameters->balanceDeltaMv = BMS_DEFAULT_BALANCE_DELTA_MV;
}

static void Charge_Manager_Clamp_Parameters(bms_charge_parameters_t *parameters)
{
    if(parameters == 0) {
        return;
    }

    if(parameters->targetCurrentMa < 600U) {
        parameters->targetCurrentMa = 600U;
    } else if(parameters->targetCurrentMa > 1000U) {
        parameters->targetCurrentMa = 1000U;
    }
    if(parameters->cutoffCurrentMa < 100U) {
        parameters->cutoffCurrentMa = 100U;
    } else if(parameters->cutoffCurrentMa > 200U) {
        parameters->cutoffCurrentMa = 200U;
    }
    if(parameters->cutoffCurrentMa >= parameters->targetCurrentMa) {
        parameters->cutoffCurrentMa = 150U;
    }
    if(parameters->targetVoltageMv > BMS_BATTERY_FULL_CHARGE_VOLTAGE_MV) {
        parameters->targetVoltageMv = BMS_BATTERY_FULL_CHARGE_VOLTAGE_MV;
    }
}

static uint8_t Charge_Manager_Validate_Parameters(const bms_charge_parameters_t *parameters)
{
    uint32_t pack_limit_mv;

    if(parameters == 0) {
        return 0U;
    }

    pack_limit_mv = (uint32_t)parameters->cellOvpMv * BMS_CELL_COUNT;

    /*
     * 即使上位机已经做了校验，固件仍然要再次限制参数范围。
     * 固件是防止危险命令或损坏帧进入功率级的最后一道保护。
     */
    if(parameters->targetVoltageMv < 25000U ||
       parameters->targetVoltageMv > BMS_BATTERY_FULL_CHARGE_VOLTAGE_MV ||
       parameters->targetVoltageMv > pack_limit_mv) {
        return 0U;
    }
    if(parameters->targetCurrentMa < 600U || parameters->targetCurrentMa > 1000U) {
        return 0U;
    }
    if(parameters->cutoffCurrentMa < 100U ||
       parameters->cutoffCurrentMa > 200U ||
       parameters->cutoffCurrentMa > parameters->targetCurrentMa) {
        return 0U;
    }
    if(parameters->cellOvpMv < 4100U || parameters->cellOvpMv > 4300U) {
        return 0U;
    }
    if(parameters->cellUvpMv < 2500U || parameters->cellUvpMv > 3300U) {
        return 0U;
    }
    if(parameters->tempOtpX10 < 450 || parameters->tempOtpX10 > 800) {
        return 0U;
    }
    if(parameters->balanceDeltaMv < 5U || parameters->balanceDeltaMv > 200U) {
        return 0U;
    }

    return 1U;
}

static uint8_t Charge_Manager_Validate_Digital_Power(uint16_t target_voltage_mv,
                                                     uint16_t current_limit_ma)
{
    if(target_voltage_mv < BMS_DIGITAL_POWER_MIN_OUTPUT_MV ||
       target_voltage_mv > BMS_DIGITAL_POWER_MAX_OUTPUT_MV) {
        return 0U;
    }
    if(current_limit_ma < BMS_DIGITAL_POWER_MIN_CURRENT_MA ||
       current_limit_ma > BMS_DIGITAL_POWER_MAX_CURRENT_MA) {
        return 0U;
    }

    return 1U;
}

static uint32_t Charge_Manager_Filter_Digital_Power_Afeless_Faults(uint32_t faults)
{
#if BMS_DIGITAL_POWER_AFELESS_DEBUG
    return faults & ~DIGITAL_POWER_AFELESS_FAULT_MASK;
#else
    return faults;
#endif
}

static uint8_t Charge_Manager_Afeless_Filter_Active(uint8_t digital_power_enabled,
                                                    uint8_t power_requested)
{
#if BMS_DIGITAL_POWER_AFELESS_DEBUG
    if((digital_power_enabled != 0U) || (power_requested == 0U)) {
        return 1U;
    }
#else
    (void)digital_power_enabled;
    (void)power_requested;
#endif

    return 0U;
}

static uint8_t Charge_Manager_Afe_Sample_Valid(const bms_afe_data_t *afe)
{
    uint32_t i;

    if(afe == 0) {
        return 0U;
    }
    if((afe->faultBitmap & BMS_FAULT_AFE_COMM) != 0U) {
        return 0U;
    }
    if(afe->cellMinMv == 0U || afe->cellMaxMv == 0U) {
        return 0U;
    }

    for(i = 0U; i < BMS_CELL_COUNT; i++) {
        if(afe->cellMv[i] == 0U) {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t Charge_Manager_Temperature_Valid(int16_t temperature_x10)
{
    if(temperature_x10 == (int16_t)BMS_TEMP_UNAVAILABLE_X10) {
        return 0U;
    }
    if(temperature_x10 < (int16_t)BMS_TEMP_MIN_VALID_X10 ||
       temperature_x10 > (int16_t)BMS_TEMP_MAX_VALID_X10) {
        return 0U;
    }

    return 1U;
}

static uint8_t Charge_Manager_Power_Ocp_Check_Active(uint8_t power_requested,
                                                     const power_control_state_t *power_state)
{
    if(power_requested == 0U || power_state == 0) {
        return 0U;
    }

    /*
     * Power-board current and FAULT_OC inputs can float when no charger is
     * connected. Only treat them as OCP evidence after PWM control is enabled.
     * During preconnect, the fast Power_Control loop and PA12 hardware path
     * remain active; the 20 ms supervisor can otherwise latch stale/transient
     * ADC or PACK/LD recovery noise while BM2016 main FETs are still open.
     */
    if(power_state->enabled == 0U || power_state->preconnectActive != 0U) {
        return 0U;
    }

    return 1U;
}

static void Charge_Manager_Reset_Output_Ovp_Counters(void)
{
    s_output_ovp_confirm_count = 0U;
    s_digital_output_ovp_confirm_count = 0U;
}

static void Charge_Manager_Reset_Charge_Ocp_Counter(void)
{
    s_charge_ocp_confirm_count = 0U;
}

static uint16_t Charge_Manager_Calc_Preconnect_Target_Mv(const bms_afe_data_t *afe,
                                                         const bms_charge_parameters_t *params)
{
    uint32_t target_mv;

    if(params == 0) {
        return 0U;
    }

    (void)params;
    if((afe == 0) || (afe->packVoltageMv == 0U)) {
        return 0U;
    }

    /*
     * 24V 输入方案下，预连接目标只略高于 Pack，并保持在主 FET 闭合确认
     * 窗口内。这样 Boost 能把 Vout 推到电池电压附近，同时避免目标本身
     * 高出 Pack 超过 |Vout-Pack| < 50 mV 的 handover 判据。
     */
    target_mv = (uint32_t)afe->packVoltageMv +
                (uint32_t)BMS_PATH_PRECONNECT_TARGET_ABOVE_PACK_MV;
    if(target_mv > (uint32_t)BMS_BATTERY_FULL_CHARGE_VOLTAGE_MV) {
        target_mv = (uint32_t)BMS_BATTERY_FULL_CHARGE_VOLTAGE_MV;
    }

    return (uint16_t)target_mv;
}

static uint16_t Charge_Manager_Preconnect_Target_Mv(const bms_afe_data_t *afe,
                                                    const bms_power_sample_t *power_sample,
                                                    const bms_charge_parameters_t *params)
{
    uint16_t final_target_mv;

    (void)power_sample;
    final_target_mv = Charge_Manager_Calc_Preconnect_Target_Mv(afe, params);
    if(final_target_mv == 0U) {
        return 0U;
    }

    taskENTER_CRITICAL();
    s_preconnect_target_mv = final_target_mv;
    taskEXIT_CRITICAL();

    return final_target_mv;
}

static uint16_t Charge_Manager_Preconnect_Current_Limit_Ma(const bms_afe_data_t *afe,
                                                           const bms_power_sample_t *power_sample,
                                                           uint32_t period_ms)
{
    (void)afe;
    (void)power_sample;
    (void)period_ms;
    return BMS_PATH_PRECONNECT_CURRENT_MA;
}

static uint8_t Charge_Manager_Preconnect_Can_Drive(const bms_afe_data_t *afe,
                                                   const bms_power_sample_t *power_sample)
{
    if((afe == 0) || (power_sample == 0) || (afe->packVoltageMv == 0U)) {
        return 0U;
    }

    /*
     * Keep the preconnect power stage alive even when Vout is above the close
     * window. The fast loop will coast instead of driving higher; stopping here
     * clears preconnectActive and lets the output collapse before the path can
     * collect enough READY samples.
     */
    return 1U;
}

static uint8_t Charge_Manager_Cv_Entry_Ready(const bms_afe_data_t *afe,
                                             const bms_power_sample_t *power_sample,
                                             const bms_charge_parameters_t *params)
{
    if((afe == 0) || (power_sample == 0) || (params == 0) ||
       (params->targetVoltageMv == 0U)) {
        return 0U;
    }
    (void)power_sample;

    if(afe->packVoltageMv == 0U) {
        return 0U;
    }

    if((uint32_t)afe->packVoltageMv + CV_ENTRY_MARGIN_MV >=
       (uint32_t)params->targetVoltageMv) {
        return 1U;
    }

    return 0U;
}

static void Charge_Manager_Clear_Manual_Fet(void)
{
    s_manual_fet_active = 0U;
    s_manual_fet_mask = 0U;
}

static void Charge_Manager_Clear_Path_Settle_State_Unlocked(void)
{
    s_path_settle_remaining_ms = 0U;
    s_last_battery_path_enabled = 0U;
    s_path_settle_target_mv = 0U;
}

static void Charge_Manager_Clear_Path_Settle_Unlocked(void)
{
    Charge_Manager_Clear_Path_Settle_State_Unlocked();
    s_preconnect_target_mv = 0U;
}

static void Charge_Manager_Clear_Path_Settle(void)
{
    taskENTER_CRITICAL();
    Charge_Manager_Clear_Path_Settle_Unlocked();
    taskEXIT_CRITICAL();
}

static void Charge_Manager_Clear_Path_Settle_Only(void)
{
    taskENTER_CRITICAL();
    Charge_Manager_Clear_Path_Settle_State_Unlocked();
    taskEXIT_CRITICAL();
}

static uint8_t Charge_Manager_Path_Settle_Active(void)
{
    uint8_t active;

    taskENTER_CRITICAL();
    active = (s_path_settle_remaining_ms != 0U) ? 1U : 0U;
    taskEXIT_CRITICAL();

    return active;
}

static uint8_t Charge_Manager_Battery_Path_Rising_Edge(void)
{
    uint8_t rising_edge;

    taskENTER_CRITICAL();
    rising_edge = (s_last_battery_path_enabled == 0U) ? 1U : 0U;
    taskEXIT_CRITICAL();

    return rising_edge;
}

static void Charge_Manager_Update_Path_Settle(uint8_t battery_path_enabled,
                                              const bms_afe_data_t *afe,
                                              const bms_charge_parameters_t *params)
{
    uint16_t target_mv;

    if(battery_path_enabled == 0U) {
        Charge_Manager_Clear_Path_Settle_Only();
        return;
    }

    target_mv = Charge_Manager_Preconnect_Target_Mv(afe, 0, params);

    taskENTER_CRITICAL();
    if(s_last_battery_path_enabled == 0U) {
        s_path_settle_remaining_ms = CHARGE_PATH_SETTLE_MS;
        s_path_settle_target_mv = target_mv;
    }
    s_last_battery_path_enabled = 1U;
    taskEXIT_CRITICAL();
}

static void Charge_Manager_Decay_Path_Settle(uint32_t period_ms)
{
    if(period_ms == 0U) {
        period_ms = 1U;
    }

    taskENTER_CRITICAL();
    if(s_path_settle_remaining_ms != 0U) {
        if(period_ms >= (uint32_t)s_path_settle_remaining_ms) {
            s_path_settle_remaining_ms = 0U;
            s_path_settle_target_mv = 0U;
            s_preconnect_target_mv = 0U;
        } else {
            s_path_settle_remaining_ms =
                (uint16_t)((uint32_t)s_path_settle_remaining_ms - period_ms);
        }
    }
    taskEXIT_CRITICAL();
}

static bms_command_reply_t Charge_Manager_Handle_Fet_Mask_Command(uint8_t fet_mask)
{
    uint8_t fet_status;
    uint32_t blocking_faults;
    uint8_t preconnect_only_mask;

    if((fet_mask & ~(AFE_GD30BM2016_FET_STATUS_CHG |
                     AFE_GD30BM2016_FET_STATUS_PCHG |
                     AFE_GD30BM2016_FET_STATUS_DSG |
                     AFE_GD30BM2016_FET_STATUS_PDSG)) != 0U) {
        return Reply_Error((uint8_t)BMS_CMD_ERROR_INVALID_PARAM);
    }

    taskENTER_CRITICAL();
    if(s_work_mode != (uint8_t)BMS_WORK_MODE_BMS) {
        taskEXIT_CRITICAL();
        return Reply_Error((uint8_t)BMS_CMD_ERROR_INVALID_MODE);
    }
    blocking_faults = s_latched_faults | Safety_Manager_Get_Latched_Faults();
#if BMS_MANUAL_PRECONNECT_FET_ALLOW_WITH_FAULTS
    preconnect_only_mask = (AFE_GD30BM2016_FET_STATUS_PCHG |
                            AFE_GD30BM2016_FET_STATUS_DSG);
    if((fet_mask != 0U) &&
       ((fet_mask & (uint8_t)~preconnect_only_mask) == 0U)) {
        blocking_faults = 0U;
    }
#else
    preconnect_only_mask = 0U;
#endif
    (void)preconnect_only_mask;
    if((fet_mask != 0U) && (blocking_faults != 0U)) {
        taskEXIT_CRITICAL();
        return Reply_Error((uint8_t)BMS_CMD_ERROR_FAULT_ACTIVE);
    }

    s_digital_power_enabled = 0U;    Charge_Manager_Clear_Path_Settle_Unlocked();
    s_run_request = 0U;
    s_state = BMS_CHARGE_STATE_IDLE;
    if(s_mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER) {
        s_mode = (uint8_t)BMS_CHARGE_MODE_AUTO;
    }
    s_cv_done_counter = 0U;
    Charge_Manager_Reset_Output_Ovp_Counters();
    /*
     * Claim manual FET ownership before any I2C delay. Otherwise the 20 ms
     * control loop can see run_request == 0 and close the path again while the
     * command task is still waiting for BM2016 FET writes to finish.
     */
    s_manual_fet_active = 1U;
    s_manual_fet_mask = fet_mask;
    taskEXIT_CRITICAL();

    Power_Control_Stop();
    Power_Path_Manager_Force_Off();
    Safety_Manager_Set_Afe_Alert_Monitor(0U);

    /*
     * Let any control-loop iteration that already captured the old automatic
     * state finish its final Force_Off before the manual FET command opens
     * BM2016 outputs.
     */
    vTaskDelay(pdMS_TO_TICKS(MANUAL_FET_TAKEOVER_DELAY_MS));

    taskENTER_CRITICAL();
    s_manual_fet_active = 1U;
    s_manual_fet_mask = fet_mask;
    taskEXIT_CRITICAL();

    if(0U == Afe_Gd30bm2016_Set_Fet_Mask(fet_mask, &fet_status)) {
        taskENTER_CRITICAL();
        Charge_Manager_Clear_Manual_Fet();
        taskEXIT_CRITICAL();
        Power_Path_Manager_Force_Off();
        return Reply_Error((uint8_t)BMS_CMD_ERROR_FAULT_ACTIVE);
    }

    taskENTER_CRITICAL();
    s_manual_fet_active = 1U;
    s_manual_fet_mask = fet_mask;
    taskEXIT_CRITICAL();

    return Reply_Ok();
}

static uint8_t Charge_Manager_Output_Ovp_Confirmed(uint16_t output_voltage_mv,
                                                   uint16_t target_voltage_mv,
                                                   uint8_t run_request,
                                                   uint8_t digital_power)
{
    uint8_t *counter;
    uint16_t margin_mv;
    uint32_t ovp_limit_mv;
    uint32_t hard_limit_mv;

    if(run_request == 0U) {
        if(digital_power != 0U) {
            s_digital_output_ovp_confirm_count = 0U;
        } else {
            s_output_ovp_confirm_count = 0U;
        }
        return 0U;
    }

    counter = (digital_power != 0U) ? &s_digital_output_ovp_confirm_count :
                                      &s_output_ovp_confirm_count;
    margin_mv = (digital_power != 0U) ? BMS_DIGITAL_POWER_OUTPUT_OVP_MARGIN_MV :
                                        PACK_OVP_MARGIN_MV;
    ovp_limit_mv = (uint32_t)target_voltage_mv + margin_mv;
    hard_limit_mv = (uint32_t)target_voltage_mv + BMS_OUTPUT_OVP_HARD_MARGIN_MV;

    if((uint32_t)output_voltage_mv > hard_limit_mv) {
        *counter = BMS_OUTPUT_OVP_CONFIRM_CONTROL_COUNT;
        return 1U;
    }

    if((uint32_t)output_voltage_mv <= ovp_limit_mv) {
        *counter = 0U;
        return 0U;
    }

    if(*counter < BMS_OUTPUT_OVP_CONFIRM_CONTROL_COUNT) {
        (*counter)++;
    }

    return (*counter >= BMS_OUTPUT_OVP_CONFIRM_CONTROL_COUNT) ? 1U : 0U;
}

static void Charge_Manager_Record_Faults(uint32_t realtime_faults,
                                        uint32_t latchable_faults)
{
    if(latchable_faults != 0U) {
        taskENTER_CRITICAL();
        s_latched_faults |= latchable_faults;
        taskEXIT_CRITICAL();
        Safety_Manager_Report_Faults(latchable_faults);
    }

    taskENTER_CRITICAL();
    s_present_faults = realtime_faults;
    taskEXIT_CRITICAL();
}

static uint32_t Charge_Manager_Collect_Faults(const bms_afe_data_t *afe,
                                              const bms_power_sample_t *power_sample,
                                              uint8_t power_requested)
{
    uint32_t faults;
    uint32_t realtime_faults;
    uint32_t latchable_faults;
    uint32_t i;
    uint16_t current_limit_ma;
    uint16_t output_target_mv;
    uint8_t digital_power_enabled;
    uint8_t afeless_filter_active;
    uint8_t power_ocp_check_active;
    uint8_t preconnect_filter_active;
    uint8_t path_settle_filter_active;
    uint32_t afe_faults;
    uint32_t fast_faults;
    power_control_state_t power_state;
    power_path_manager_state_t path_state;

    /*
     * 锁存故障会一直保留到收到 CMD_CLEAR_FAULT。实时故障本轮重新采集，
     * 一旦出现就同步交给 safety_manager 立即关断 Q4/Q5。
     */
    taskENTER_CRITICAL();
    faults = s_latched_faults;
    digital_power_enabled = s_digital_power_enabled;
    output_target_mv = (digital_power_enabled != 0U) ?
                       s_digital_power_target_voltage_mv :
                       s_params.targetVoltageMv;
    taskEXIT_CRITICAL();
    faults |= Safety_Manager_Get_Latched_Faults();
    afeless_filter_active = Charge_Manager_Afeless_Filter_Active(digital_power_enabled,
                                                                 power_requested);
    if(afeless_filter_active != 0U) {
        faults = Charge_Manager_Filter_Digital_Power_Afeless_Faults(faults);
    }
    realtime_faults = 0U;
    current_limit_ma = s_params.targetCurrentMa;
    Power_Control_Get_State(&power_state);
    Power_Path_Manager_Get_State(&path_state);
    realtime_faults |= power_state.faultBitmap;
    path_settle_filter_active = Charge_Manager_Path_Settle_Active();
    power_ocp_check_active = Charge_Manager_Power_Ocp_Check_Active(power_requested,
                                                                    &power_state);
    if(path_settle_filter_active != 0U) {
        power_ocp_check_active = 0U;
    }
    preconnect_filter_active = ((digital_power_enabled == 0U) &&
                                (power_requested != 0U) &&
                                ((power_state.preconnectActive != 0U) ||
                                 (path_settle_filter_active != 0U))) ? 1U : 0U;
    if(power_state.enabled != 0U && power_state.targetCurrentMa != 0U) {
        current_limit_ma = power_state.targetCurrentMa;
        if(digital_power_enabled == 0U && power_state.targetVoltageMv != 0U) {
            if((power_state.preconnectActive != 0U) &&
               (power_state.preconnectOvpLimitMv != 0U)) {
                output_target_mv = power_state.preconnectOvpLimitMv;
            } else {
                output_target_mv = power_state.targetVoltageMv;
            }
        }
    }

    if(afe == 0) {
        realtime_faults |= BMS_FAULT_AFE_COMM;
    } else {
        afe_faults = afe->faultBitmap;
        if(preconnect_filter_active != 0U) {
            /*
             * Before the BM2016 main path closes, PACK/LD are intentionally
             * floating/precharged by the power board. Do not convert transient
             * AFE current/short recovery states into a latched charger OCP.
             * Cell voltage and temperature faults are kept.
             */
            afe_faults &= ~AFE_RECOVERABLE_LATCH_FAULT_MASK;
        }
        realtime_faults |= afe_faults;
        if(0U != Charge_Manager_Afe_Sample_Valid(afe)) {
            if(afe->cellMaxMv >= s_params.cellOvpMv) {
                realtime_faults |= BMS_FAULT_CELL_OVP;
            }
            if(afe->cellMinMv <= s_params.cellUvpMv) {
                realtime_faults |= BMS_FAULT_CELL_UVP;
            }
            if((digital_power_enabled == 0U) &&
               (power_requested != 0U) &&
               (afe->packVoltageMv >= (uint16_t)(output_target_mv + PACK_OVP_MARGIN_MV))) {
                realtime_faults |= BMS_FAULT_PACK_OVP;
            }
            for(i = 0U; i < BMS_AFE_TEMP_COUNT; i++) {
                if((0U != Charge_Manager_Temperature_Valid(afe->temperaturesX10[i])) &&
                   afe->temperaturesX10[i] >= s_params.tempOtpX10) {
                    realtime_faults |= BMS_FAULT_OTP;
                }
            }
        }
    }

    if(power_sample == 0) {
        realtime_faults |= BMS_FAULT_ADC;
    } else {
#if BMS_ENABLE_INPUT_UV_FAULT
        /*
         * 仅在需要外部输入供电的充电工况下，才把输入欠压当作故障。
         * 数字电源模式和板级调试阶段默认关闭此项。
         */
        if((power_requested != 0U) && (power_sample->inputVoltageMv < BMS_DEFAULT_INPUT_UV_THRESHOLD_MV)) {
            realtime_faults |= BMS_FAULT_INPUT_UV;
        }
#endif
        if(path_state.batteryPathEnabled != 0U) {
            if(Charge_Manager_Output_Ovp_Confirmed(power_sample->outputVoltageMv,
                                                   output_target_mv,
                                                   power_requested,
                                                   0U) != 0U) {
                realtime_faults |= BMS_FAULT_PACK_OVP;
            }
        } else {
            (void)Charge_Manager_Output_Ovp_Confirmed(power_sample->outputVoltageMv,
                                                      output_target_mv,
                                                      0U,
                                                      0U);
        }
        /* 硬件 FAULT_OC 和软件电流裕量超限都映射为充电过流故障。 */
        if((power_ocp_check_active != 0U) &&
           (power_sample->faultOcActive != 0U ||
            power_sample->outputCurrentMa > (int16_t)(current_limit_ma + 1500U))) {
            if(s_charge_ocp_confirm_count < BMS_OUTPUT_OVP_CONFIRM_CONTROL_COUNT) {
                s_charge_ocp_confirm_count++;
            }
            if(s_charge_ocp_confirm_count >= BMS_OUTPUT_OVP_CONFIRM_CONTROL_COUNT) {
                realtime_faults |= BMS_FAULT_CHARGE_OCP;
            }
        } else {
            Charge_Manager_Reset_Charge_Ocp_Counter();
        }
        if(((0U != Charge_Manager_Temperature_Valid(power_sample->mosTempX10)) &&
            power_sample->mosTempX10 >= s_params.tempOtpX10) ||
           ((0U != Charge_Manager_Temperature_Valid(power_sample->inductorTempX10)) &&
            power_sample->inductorTempX10 >= s_params.tempOtpX10)) {
            realtime_faults |= BMS_FAULT_OTP;
        }
        realtime_faults |= power_sample->faultBitmap;
    }

    fast_faults = Safety_Manager_Sample_Fast_Faults();
    if(preconnect_filter_active != 0U) {
        /*
         * While Vout is being aligned or the main path has just closed, PA12
         * and BM2016 ALERT can show short recovery spikes. Let the real fast
         * ISR/service paths handle a sustained hardware fault; do not relatch
         * those one-sample spikes through the 20 ms charge supervisor.
         */
        fast_faults &= ~AFE_RECOVERABLE_LATCH_FAULT_MASK;
    }
    realtime_faults |= fast_faults;
    if(afeless_filter_active != 0U) {
        realtime_faults = Charge_Manager_Filter_Digital_Power_Afeless_Faults(realtime_faults);
    }
    if(power_requested == 0U) {
        /*
         * Pre-existing AFE OCC/OCD history is recoverable while idle; do not
         * show charge OCP before the charger path has been requested.
         */
        realtime_faults &= ~BMS_FAULT_CHARGE_OCP;
    }
    if((digital_power_enabled == 0U) &&
       (power_requested != 0U) &&
       (power_state.enabled == 0U)) {
        /*
         * When PWM is off, old BM2016 OCD/SCD latch bits are recovery
         * conditions, not proof of a new power-stage fault. Do not relatch
         * them before Afe_Gd30bm2016_Fets_On() gets a chance to recover and
         * reopen the path.
         */
        realtime_faults &= ~AFE_RECOVERABLE_LATCH_FAULT_MASK;
    }

    taskENTER_CRITICAL();
    s_present_faults = realtime_faults;
    taskEXIT_CRITICAL();

    latchable_faults = realtime_faults;
    if(power_requested == 0U) {
        latchable_faults &= ~(BMS_FAULT_AFE_COMM |
                              BMS_FAULT_ADC |
                              BMS_FAULT_PACK_OVP |
                              BMS_FAULT_CHARGE_OCP);
    }

    if(latchable_faults != 0U) {
        taskENTER_CRITICAL();
        s_latched_faults |= latchable_faults;
        faults |= s_latched_faults;
        taskEXIT_CRITICAL();
        Safety_Manager_Report_Faults(latchable_faults);
    }

    faults |= realtime_faults | Safety_Manager_Get_Latched_Faults();
    if(afeless_filter_active != 0U) {
        faults = Charge_Manager_Filter_Digital_Power_Afeless_Faults(faults);
    }

    return faults;
}

static uint32_t Charge_Manager_Collect_Digital_Power_Faults(const bms_power_sample_t *power_sample,
                                                            uint16_t target_voltage_mv,
                                                            uint16_t current_limit_ma)
{
    uint32_t faults;
    uint32_t realtime_faults;
    power_control_state_t power_state;

    taskENTER_CRITICAL();
    faults = s_latched_faults;
    taskEXIT_CRITICAL();

    faults |= Safety_Manager_Get_Latched_Faults();
    faults = Charge_Manager_Filter_Digital_Power_Afeless_Faults(faults);
    realtime_faults = 0U;

    Power_Control_Get_State(&power_state);
    realtime_faults |= power_state.faultBitmap;

    if(power_sample == 0) {
        realtime_faults |= BMS_FAULT_ADC;
    } else {
        realtime_faults |= power_sample->faultBitmap;
        if(power_sample->faultOcActive != 0U) {
            realtime_faults |= BMS_FAULT_CHARGE_OCP;
        }
        if(Charge_Manager_Output_Ovp_Confirmed(power_sample->outputVoltageMv,
                                               target_voltage_mv,
                                               1U,
                                               1U) != 0U) {
            realtime_faults |= BMS_FAULT_PACK_OVP;
        }
        if(power_sample->outputCurrentMa >
           (int16_t)(current_limit_ma + BMS_DEFAULT_OUTPUT_OCP_MARGIN_MA)) {
            realtime_faults |= BMS_FAULT_CHARGE_OCP;
        }
        if(((0U != Charge_Manager_Temperature_Valid(power_sample->mosTempX10)) &&
            power_sample->mosTempX10 >= s_params.tempOtpX10) ||
           ((0U != Charge_Manager_Temperature_Valid(power_sample->inductorTempX10)) &&
            power_sample->inductorTempX10 >= s_params.tempOtpX10)) {
            realtime_faults |= BMS_FAULT_OTP;
        }
    }

    realtime_faults |= Safety_Manager_Sample_Fast_Faults();
    realtime_faults = Charge_Manager_Filter_Digital_Power_Afeless_Faults(realtime_faults);
    Charge_Manager_Record_Faults(realtime_faults, realtime_faults);

    taskENTER_CRITICAL();
    faults |= s_latched_faults;
    taskEXIT_CRITICAL();

    faults |= realtime_faults | Safety_Manager_Get_Latched_Faults();
    faults = Charge_Manager_Filter_Digital_Power_Afeless_Faults(faults);

    return faults;
}

static uint16_t Charge_Manager_Target_Current_For_State(bms_charge_state_t state)
{
    uint16_t target_current_ma;

    /* 涓流和恒压阶段本质上都是同一个功率级的限流工作模式。 */
    if(state == BMS_CHARGE_STATE_TRICKLE) {
        if(s_params.targetCurrentMa < TRICKLE_CURRENT_MA) {
            target_current_ma = s_params.targetCurrentMa;
        } else {
            target_current_ma = TRICKLE_CURRENT_MA;
        }
        return Power_Manager_Limit_Current(target_current_ma);
    }

    if(state == BMS_CHARGE_STATE_CV) {
        /*
         * 恒压阶段仍然把目标电流作为最大允许电流交给功率环。
         * 真正让电流自然下降的是电压环；cutoffCurrentMa 只用于判断充电完成。
         */
        return Power_Manager_Limit_Current(s_params.targetCurrentMa);
    }

    if(state == BMS_CHARGE_STATE_CC) {
        return Power_Manager_Limit_Current(s_params.targetCurrentMa);
    }

    return Power_Manager_Limit_Current(s_params.targetCurrentMa);
}

static uint8_t Charge_Manager_Control_Mode_For_State(uint8_t mode, bms_charge_state_t state)
{
    if(mode != (uint8_t)BMS_CHARGE_MODE_AUTO) {
        return mode;
    }

    if(state == BMS_CHARGE_STATE_CV) {
        return (uint8_t)BMS_CHARGE_MODE_CV;
    }

    if(state == BMS_CHARGE_STATE_TRICKLE) {
        return (uint8_t)BMS_CHARGE_MODE_TRICKLE;
    }

    return (uint8_t)BMS_CHARGE_MODE_CC;
}

static uint8_t Charge_Manager_Cv_Done_Allowed(const bms_power_sample_t *power_sample,
                                               const bms_charge_parameters_t *params)
{
    power_path_manager_state_t path_state;
    power_manager_state_t power_state;

    if((power_sample == 0) || (params == 0) || (params->targetVoltageMv == 0U)) {
        return 0U;
    }

    Power_Path_Manager_Get_State(&path_state);
    if(path_state.batteryPathEnabled == 0U) {
        return 0U;
    }

    if(Charge_Manager_Path_Settle_Active() != 0U) {
        return 0U;
    }

    if(((uint32_t)power_sample->outputVoltageMv + CV_ENTRY_MARGIN_MV) <
       (uint32_t)params->targetVoltageMv) {
        return 0U;
    }

    memset(&power_state, 0, sizeof(power_state));
    Power_Manager_Get_State(&power_state);
    if(power_state.deratingActive != 0U) {
        return 0U;
    }
    if((power_state.requestedCurrentMa != 0U) &&
       (power_state.limitedCurrentMa != 0U) &&
       ((uint32_t)power_state.limitedCurrentMa + CV_DONE_DPM_MARGIN_MA <
        (uint32_t)power_state.requestedCurrentMa)) {
        return 0U;
    }

    return 1U;
}

void Charge_Manager_Init(void)
{
    bms_charge_parameters_t stored_params;

    taskENTER_CRITICAL();
    if(Param_Storage_Get_Charge(&stored_params) != 0U) {
        Charge_Manager_Clamp_Parameters(&stored_params);
        if(Charge_Manager_Validate_Parameters(&stored_params) != 0U) {
            s_params = stored_params;
        } else {
            Charge_Manager_Default_Parameters(&s_params);
            Charge_Manager_Clamp_Parameters(&s_params);
        }
    } else {
        Charge_Manager_Default_Parameters(&s_params);
        Charge_Manager_Clamp_Parameters(&s_params);
    }
    s_state = BMS_CHARGE_STATE_IDLE;
    s_run_request = 0U;
    s_mode = (uint8_t)BMS_CHARGE_MODE_AUTO;
    s_work_mode = (uint8_t)BMS_WORK_MODE_BMS;
    s_latched_faults = 0U;
    s_present_faults = 0U;
    s_cv_done_counter = 0U;
    s_digital_power_enabled = 0U;
    s_digital_power_target_voltage_mv = 0U;
    s_digital_power_current_limit_ma = 0U;    Charge_Manager_Clear_Manual_Fet();
    Charge_Manager_Clear_Path_Settle_Unlocked();
    Charge_Manager_Reset_Output_Ovp_Counters();
    taskEXIT_CRITICAL();

    Power_Control_Stop();
    Safety_Manager_Set_Afe_Alert_Monitor(0U);
}

void Charge_Manager_Get_Parameters(bms_charge_parameters_t *parameters)
{
    if(parameters == 0) {
        return;
    }

    taskENTER_CRITICAL();
    *parameters = s_params;
    taskEXIT_CRITICAL();
}

uint8_t Charge_Manager_Is_Digital_Power_Active(void)
{
    uint8_t active;

    taskENTER_CRITICAL();
    active = ((s_work_mode == (uint8_t)BMS_WORK_MODE_DIGITAL_POWER) ||
              (s_digital_power_enabled != 0U) ||
              (s_state == BMS_CHARGE_STATE_DIGITAL_POWER) ||
              (s_mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER)) ? 1U : 0U;
    taskEXIT_CRITICAL();

    return active;
}

uint8_t Charge_Manager_Get_Work_Mode(void)
{
    uint8_t work_mode;

    taskENTER_CRITICAL();
    work_mode = s_work_mode;
    taskEXIT_CRITICAL();

    return work_mode;
}

bms_command_reply_t Charge_Manager_Set_Work_Mode(uint8_t work_mode)
{
    if(work_mode > (uint8_t)BMS_WORK_MODE_DIGITAL_POWER) {
        return Reply_Error((uint8_t)BMS_CMD_ERROR_INVALID_PARAM);
    }

    taskENTER_CRITICAL();
    s_work_mode = work_mode;
    s_digital_power_enabled = 0U;    Charge_Manager_Clear_Manual_Fet();
    Charge_Manager_Clear_Path_Settle_Unlocked();
    s_run_request = 0U;
    s_state = BMS_CHARGE_STATE_IDLE;
    s_mode = (work_mode == (uint8_t)BMS_WORK_MODE_DIGITAL_POWER) ?
             (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER :
             (uint8_t)BMS_CHARGE_MODE_AUTO;
    s_cv_done_counter = 0U;
    Charge_Manager_Reset_Output_Ovp_Counters();
    taskEXIT_CRITICAL();

    Power_Control_Stop();
    Power_Path_Manager_Force_Off();
    if(work_mode == (uint8_t)BMS_WORK_MODE_BMS) {
        (void)Afe_Gd30bm2016_Recover_Protections();
    }
    Safety_Manager_Set_Afe_Alert_Monitor(0U);
    return Reply_Ok();
}

bms_command_reply_t Charge_Manager_Set_Parameters(const bms_charge_parameters_t *parameters)
{
    bms_charge_parameters_t clamped_parameters;

    if(parameters == 0) {
        return Reply_Error((uint8_t)BMS_CMD_ERROR_INVALID_PARAM);
    }

    clamped_parameters = *parameters;
    Charge_Manager_Clamp_Parameters(&clamped_parameters);
    if(Charge_Manager_Validate_Parameters(&clamped_parameters) == 0U) {
        return Reply_Error((uint8_t)BMS_CMD_ERROR_INVALID_PARAM);
    }

    taskENTER_CRITICAL();
    if(s_work_mode != (uint8_t)BMS_WORK_MODE_BMS) {
        taskEXIT_CRITICAL();
        return Reply_Error((uint8_t)BMS_CMD_ERROR_INVALID_MODE);
    }
    s_params = clamped_parameters;
    taskEXIT_CRITICAL();
    (void)Param_Storage_Set_Charge(&clamped_parameters);
    return Reply_Ok();
}

bms_command_reply_t Charge_Manager_Handle_Command(uint8_t command_id, uint8_t argument, uint8_t has_argument)
{
    bms_command_reply_t reply;
    uint32_t present_faults;
    uint32_t power_faults;
    uint32_t fast_faults;
    uint8_t run_request_active;

    switch(command_id) {
    case CMD_START_CHARGE:
        taskENTER_CRITICAL();
        if(s_work_mode != (uint8_t)BMS_WORK_MODE_BMS) {
            reply = Reply_Error((uint8_t)BMS_CMD_ERROR_INVALID_MODE);
        } else if((s_latched_faults | Safety_Manager_Get_Latched_Faults()) != 0U) {
            reply = Reply_Error((uint8_t)BMS_CMD_ERROR_FAULT_ACTIVE);
        } else {
            s_digital_power_enabled = 0U;            Charge_Manager_Clear_Manual_Fet();
            Charge_Manager_Clear_Path_Settle_Unlocked();
            if(s_mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER) {
                s_mode = (uint8_t)BMS_CHARGE_MODE_AUTO;
            }
            s_run_request = 1U;
            s_state = BMS_CHARGE_STATE_PRECHECK;
            s_cv_done_counter = 0U;
            Charge_Manager_Reset_Output_Ovp_Counters();
            reply = Reply_Ok();
        }
        taskEXIT_CRITICAL();
        if(reply.result == (uint8_t)BMS_CMD_RESULT_OK) {
            Safety_Manager_Set_Afe_Alert_Monitor(0U);
            (void)Afe_Gd30bm2016_Recover_Protections();
        }
        break;

    case CMD_STOP_CHARGE:
        taskENTER_CRITICAL();
        s_digital_power_enabled = 0U;
        Charge_Manager_Clear_Manual_Fet();
        Charge_Manager_Clear_Path_Settle_Unlocked();
        if((s_work_mode != (uint8_t)BMS_WORK_MODE_DIGITAL_POWER) &&
           (s_mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER)) {
            s_mode = (uint8_t)BMS_CHARGE_MODE_AUTO;
        }
        s_run_request = 0U;
        s_state = BMS_CHARGE_STATE_IDLE;
        s_cv_done_counter = 0U;
        Charge_Manager_Reset_Output_Ovp_Counters();
        taskEXIT_CRITICAL();
        Power_Control_Stop();
        Power_Path_Manager_Force_Off();
        Safety_Manager_Set_Afe_Alert_Monitor(0U);
        reply = Reply_Ok();
        break;

    case CMD_EMERGENCY_STOP:
        taskENTER_CRITICAL();
        s_digital_power_enabled = 0U;
        Charge_Manager_Clear_Manual_Fet();
        Charge_Manager_Clear_Path_Settle_Unlocked();
        s_run_request = 0U;
        s_state = BMS_CHARGE_STATE_FAULT;
        s_latched_faults |= BMS_FAULT_EMERGENCY_STOP;
        Charge_Manager_Reset_Output_Ovp_Counters();
        taskEXIT_CRITICAL();
        Safety_Manager_Report_Faults(BMS_FAULT_EMERGENCY_STOP);
        Power_Path_Manager_Force_Off();
        Safety_Manager_Set_Afe_Alert_Monitor(0U);
        reply = Reply_Ok();
        break;

    case CMD_CLEAR_FAULT:
        (void)Afe_Gd30bm2016_Recover_Protections();
        taskENTER_CRITICAL();
        present_faults = s_present_faults;
        taskEXIT_CRITICAL();
        power_faults = Power_Control_Get_Fault_Status();
        present_faults &= ~power_faults;
        present_faults &= ~AFE_RECOVERABLE_LATCH_FAULT_MASK;
        fast_faults = Safety_Manager_Sample_Fast_Faults() & ~power_faults;
        fast_faults &= ~BMS_FAULT_AFE_PROTECTION;
#if BMS_DIGITAL_POWER_AFELESS_DEBUG
        present_faults = Charge_Manager_Filter_Digital_Power_Afeless_Faults(present_faults);
        fast_faults = Charge_Manager_Filter_Digital_Power_Afeless_Faults(fast_faults);
#endif

        if((present_faults | fast_faults) != 0U) {
            Power_Control_Stop();
            reply = Reply_Error((uint8_t)BMS_CMD_ERROR_FAULT_ACTIVE);
        } else {
            taskENTER_CRITICAL();
            s_digital_power_enabled = 0U;            Charge_Manager_Clear_Manual_Fet();
            Charge_Manager_Clear_Path_Settle_Unlocked();
            if((s_work_mode != (uint8_t)BMS_WORK_MODE_DIGITAL_POWER) &&
               (s_mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER)) {
                s_mode = (uint8_t)BMS_CHARGE_MODE_AUTO;
            }
            s_run_request = 0U;
            s_state = BMS_CHARGE_STATE_IDLE;
            s_latched_faults = 0U;
            s_present_faults = 0U;
            s_cv_done_counter = 0U;
            Charge_Manager_Reset_Output_Ovp_Counters();
            taskEXIT_CRITICAL();
            Safety_Manager_Clear_Latched_Faults();
            Power_Control_Clear_Fault_Lockout();
            Power_Control_Clear_Fault_Status();
            Power_Control_Stop();
            Power_Path_Manager_Force_Off();
            Safety_Manager_Set_Afe_Alert_Monitor(0U);
            reply = Reply_Ok();
        }
        break;

    case CMD_SWITCH_MODE:
        if(has_argument == 0U || argument > (uint8_t)BMS_CHARGE_MODE_CV) {
            reply = Reply_Error((uint8_t)BMS_CMD_ERROR_INVALID_PARAM);
        } else {
            taskENTER_CRITICAL();
            if(s_work_mode != (uint8_t)BMS_WORK_MODE_BMS) {
                reply = Reply_Error((uint8_t)BMS_CMD_ERROR_INVALID_MODE);
                run_request_active = 0U;
            } else {
            s_digital_power_enabled = 0U;            Charge_Manager_Clear_Manual_Fet();
            Charge_Manager_Clear_Path_Settle_Unlocked();
            s_mode = argument;
            run_request_active = s_run_request;
            if(s_run_request != 0U) {
                /*
                 * 运行中切换模式时，下一轮控制周期必须重新按新模式选择
                 * 涓流/恒流/恒压阶段，否则目标电流可能还沿用旧阶段。
                */
                s_state = BMS_CHARGE_STATE_PRECHECK;
                s_cv_done_counter = 0U;
                Charge_Manager_Reset_Output_Ovp_Counters();
            }
            reply = Reply_Ok();
            }
            taskEXIT_CRITICAL();
            if(reply.result == (uint8_t)BMS_CMD_RESULT_OK) {
                if(run_request_active != 0U) {
                    Safety_Manager_Set_Afe_Alert_Monitor(0U);
                } else {
                    Safety_Manager_Set_Afe_Alert_Monitor(0U);
                }
            }
        }
        break;

    case CMD_SET_WORK_MODE:
        if(has_argument == 0U) {
            reply = Reply_Error((uint8_t)BMS_CMD_ERROR_BAD_LENGTH);
        } else {
            reply = Charge_Manager_Set_Work_Mode(argument);
        }
        break;

    case CMD_SET_FET_MASK:
        if(has_argument == 0U) {
            reply = Reply_Error((uint8_t)BMS_CMD_ERROR_BAD_LENGTH);
        } else {
            reply = Charge_Manager_Handle_Fet_Mask_Command(argument);
        }
        break;

    default:
        reply = Reply_Error((uint8_t)BMS_CMD_ERROR_UNKNOWN_COMMAND);
        break;
    }

    return reply;
}

bms_command_reply_t Charge_Manager_Handle_Digital_Power_Command(uint8_t enable,
                                                               uint16_t target_voltage_mv,
                                                               uint16_t current_limit_ma)
{
    bms_command_reply_t reply;
    uint32_t raw_faults;
    uint32_t blocking_faults;

    if(enable == 0U) {
        taskENTER_CRITICAL();
        s_digital_power_enabled = 0U;
        Charge_Manager_Clear_Manual_Fet();
        Charge_Manager_Clear_Path_Settle_Unlocked();
        if((s_work_mode != (uint8_t)BMS_WORK_MODE_DIGITAL_POWER) &&
           (s_mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER)) {
            s_mode = (uint8_t)BMS_CHARGE_MODE_AUTO;
        }
        s_run_request = 0U;
        s_state = BMS_CHARGE_STATE_IDLE;
        s_cv_done_counter = 0U;
        Charge_Manager_Reset_Output_Ovp_Counters();
        taskEXIT_CRITICAL();
        Power_Control_Stop();
        Safety_Manager_Set_Afe_Alert_Monitor(0U);
        return Reply_Ok();
    }

    if(Charge_Manager_Validate_Digital_Power(target_voltage_mv, current_limit_ma) == 0U) {
        return Reply_Error((uint8_t)BMS_CMD_ERROR_INVALID_PARAM);
    }

    taskENTER_CRITICAL();
    if(s_work_mode != (uint8_t)BMS_WORK_MODE_DIGITAL_POWER) {
        taskEXIT_CRITICAL();
        return Reply_Error((uint8_t)BMS_CMD_ERROR_INVALID_MODE);
    }
    if(s_digital_power_enabled != 0U) {
        /*
         * 已在数字电源运行中，直接更新目标电压/电流。
         * 电压环会平滑跟踪到新设定值，不再停机泄放再重启。
         * 调低电压时由 OVP 软限幅兜底，而不是先掉电。
         */
        s_digital_power_target_voltage_mv = target_voltage_mv;
        s_digital_power_current_limit_ma = current_limit_ma;
        Charge_Manager_Clear_Manual_Fet();
        Charge_Manager_Clear_Path_Settle_Unlocked();
        s_state = BMS_CHARGE_STATE_DIGITAL_POWER;
        s_mode = (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER;
        s_cv_done_counter = 0U;
        Charge_Manager_Reset_Output_Ovp_Counters();
        taskEXIT_CRITICAL();
        Power_Control_Set(target_voltage_mv, current_limit_ma,
                          (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER);
        Safety_Manager_Set_Afe_Alert_Monitor(0U);
        return Reply_Ok();
    }
    raw_faults = s_latched_faults | Safety_Manager_Get_Latched_Faults();
    taskEXIT_CRITICAL();
    raw_faults |= Power_Control_Get_Fault_Status();

    blocking_faults = raw_faults & ~DIGITAL_POWER_STARTUP_IGNORE_MASK;
    if(blocking_faults != 0U) {
        return Reply_Error((uint8_t)BMS_CMD_ERROR_FAULT_ACTIVE);
    }

    Safety_Manager_Set_Afe_Alert_Monitor(0U);

#if BMS_DIGITAL_POWER_AFELESS_DEBUG
    if(raw_faults != 0U) {
        taskENTER_CRITICAL();
        s_latched_faults = 0U;
        taskEXIT_CRITICAL();
        Safety_Manager_Clear_Latched_Faults();
        Power_Control_Clear_Fault_Lockout();
        Power_Control_Clear_Fault_Status();
    }
#endif

    taskENTER_CRITICAL();
    s_digital_power_enabled = 1U;
    s_digital_power_target_voltage_mv = target_voltage_mv;
    s_digital_power_current_limit_ma = current_limit_ma;
    Charge_Manager_Clear_Manual_Fet();
    Charge_Manager_Clear_Path_Settle_Unlocked();
    s_run_request = 0U;
    s_state = BMS_CHARGE_STATE_DIGITAL_POWER;
    s_mode = (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER;
    s_cv_done_counter = 0U;
    Charge_Manager_Reset_Output_Ovp_Counters();
    reply = Reply_Ok();
    taskEXIT_CRITICAL();

    if(reply.result == (uint8_t)BMS_CMD_RESULT_OK) {
        Power_Control_Stop();
    }

    return reply;
}

void Charge_Manager_Update(uint32_t period_ms,
                           const bms_afe_data_t *afe,
                           const bms_power_sample_t *power_sample,
                           bms_status_t *status)
{
    bms_charge_parameters_t params;
    bms_charge_state_t state;
    uint8_t run_request;
    uint8_t mode;
    uint8_t control_mode;
    uint8_t work_mode;
    uint8_t digital_power_enabled;
    uint8_t manual_fet_active;
    uint8_t manual_fet_mask;
    uint16_t digital_power_target_voltage_mv;
    uint16_t digital_power_current_limit_ma;
    uint32_t faults;
    uint16_t target_current_ma;
    uint16_t preconnect_target_mv;
    uint16_t preconnect_ovp_limit_mv;
    uint8_t power_requested;
    uint8_t digital_power_fault_context;
    uint8_t path_manager_updated;
    uint8_t afe_handover_pending;
    uint32_t i;
    int16_t battery_current_ma;
    power_path_manager_state_t path_state;

    if(status == 0 || afe == 0 || power_sample == 0) {
        return;
    }

    taskENTER_CRITICAL();
    params = s_params;
    state = s_state;
    run_request = s_run_request;
    mode = s_mode;
    work_mode = s_work_mode;
    digital_power_enabled = s_digital_power_enabled;
    manual_fet_active = s_manual_fet_active;
    manual_fet_mask = s_manual_fet_mask;
    digital_power_target_voltage_mv = s_digital_power_target_voltage_mv;
    digital_power_current_limit_ma = s_digital_power_current_limit_ma;
    taskEXIT_CRITICAL();

    power_requested = ((run_request != 0U) || (digital_power_enabled != 0U)) ? 1U : 0U;
    battery_current_ma = afe->batteryCurrentMa;
    Power_Control_Set_Battery_Current_Feedback(0, 0U);
    Power_Control_Set_Battery_Voltage_Feedback(0U, 0U);
    path_manager_updated = 0U;
    afe_handover_pending = 0U;
    digital_power_fault_context = ((work_mode == (uint8_t)BMS_WORK_MODE_DIGITAL_POWER) ||
                                   (digital_power_enabled != 0U) ||
                                   (state == BMS_CHARGE_STATE_DIGITAL_POWER) ||
                                   (mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER)) ? 1U : 0U;
    if(digital_power_enabled != 0U) {
        faults = Charge_Manager_Collect_Digital_Power_Faults(power_sample,
                                                             digital_power_target_voltage_mv,
                                                             digital_power_current_limit_ma);
    } else {
        faults = Charge_Manager_Collect_Faults(afe, power_sample, power_requested);
    }

    /*
     * 这里的优先级不能随意调整：
     * 先处理故障并立即关 PWM；再处理停止/空闲；最后才允许自动充电曲线
     * 在涓流、恒流、恒压和完成状态之间切换。
     */
    if(faults != 0U) {
        run_request = 0U;
        Safety_Manager_Set_Afe_Alert_Monitor(0U);
        if(digital_power_fault_context != 0U) {
            mode = (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER;
        }
        digital_power_enabled = 0U;
        manual_fet_active = 0U;
        manual_fet_mask = 0U;
        state = BMS_CHARGE_STATE_FAULT;
        Safety_Manager_Report_Faults(faults);
        Charge_Manager_Clear_Path_Settle();
        Power_Path_Manager_Force_Off();
    } else if(manual_fet_active != 0U) {
        (void)manual_fet_mask;
        run_request = 0U;
        digital_power_enabled = 0U;
        state = BMS_CHARGE_STATE_IDLE;
        Charge_Manager_Clear_Path_Settle();
        Power_Control_Stop();
        Safety_Manager_Set_Afe_Alert_Monitor(0U);
    } else if(digital_power_enabled != 0U) {
        run_request = 0U;
        state = BMS_CHARGE_STATE_DIGITAL_POWER;
        mode = (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER;
        Charge_Manager_Clear_Path_Settle();
        Power_Control_Set(digital_power_target_voltage_mv,
                          digital_power_current_limit_ma,
                          mode);
    } else if(work_mode == (uint8_t)BMS_WORK_MODE_DIGITAL_POWER) {
        run_request = 0U;
        state = BMS_CHARGE_STATE_IDLE;
        mode = (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER;
        Charge_Manager_Clear_Path_Settle();
        Power_Control_Stop();
    } else if(run_request == 0U) {
        state = BMS_CHARGE_STATE_IDLE;
        Charge_Manager_Clear_Path_Settle();
        Power_Control_Stop();
        Safety_Manager_Set_Afe_Alert_Monitor(0U);
    } else {
        if(state == BMS_CHARGE_STATE_PRECHECK || state == BMS_CHARGE_STATE_IDLE) {
            if(mode == (uint8_t)BMS_CHARGE_MODE_TRICKLE) {
                state = BMS_CHARGE_STATE_TRICKLE;
            } else if(mode == (uint8_t)BMS_CHARGE_MODE_CC) {
                state = BMS_CHARGE_STATE_CC;
            } else if(mode == (uint8_t)BMS_CHARGE_MODE_CV) {
                state = BMS_CHARGE_STATE_CV;
            } else if(afe->cellMinMv < TRICKLE_EXIT_CELL_MV) {
                state = BMS_CHARGE_STATE_TRICKLE;
            } else {
                state = BMS_CHARGE_STATE_CC;
            }
        }

        if(mode == (uint8_t)BMS_CHARGE_MODE_AUTO) {
            /* 自动曲线：低压单体先涓流，正常后恒流，接近目标电压后进入恒压。 */
            if(state == BMS_CHARGE_STATE_TRICKLE && afe->cellMinMv >= TRICKLE_EXIT_CELL_MV) {
                state = BMS_CHARGE_STATE_CC;
            }
            if(state != BMS_CHARGE_STATE_TRICKLE && afe->cellMinMv <= TRICKLE_ENTER_CELL_MV) {
                state = BMS_CHARGE_STATE_TRICKLE;
            }
            if(state == BMS_CHARGE_STATE_CC &&
               Charge_Manager_Cv_Entry_Ready(afe, power_sample, &params) != 0U) {
                state = BMS_CHARGE_STATE_CV;
            }
            if((state == BMS_CHARGE_STATE_CV) &&
               (Charge_Manager_Cv_Done_Allowed(power_sample, &params) != 0U) &&
               (battery_current_ma >= 0) &&
               (battery_current_ma <= (int16_t)params.cutoffCurrentMa)) {
                /* 需要连续多次低电流采样，才判定充电完成，避免偶发抖动误触发。 */
                s_cv_done_counter++;
                if(s_cv_done_counter > CV_DONE_CONFIRM_COUNT) {
                    state = BMS_CHARGE_STATE_DONE;
                    run_request = 0U;
                    Charge_Manager_Clear_Path_Settle();
                    Power_Control_Stop();
                }
            } else {
                s_cv_done_counter = 0U;
            }
        }

        if(state != BMS_CHARGE_STATE_DONE) {
            Power_Path_Manager_Update(afe, power_sample, &params, state, faults);
            path_manager_updated = 1U;
            Power_Path_Manager_Get_State(&path_state);
            if(path_state.batteryPathEnabled != 0U) {
                afe_handover_pending = Charge_Manager_Battery_Path_Rising_Edge();
                Charge_Manager_Update_Path_Settle(path_state.batteryPathEnabled,
                                                  afe,
                                                  &params);
                Safety_Manager_Set_Afe_Alert_Monitor(
                    (Charge_Manager_Path_Settle_Active() != 0U) ? 0U : 1U);
                Power_Manager_Update(period_ms, afe, power_sample, &params, (uint8_t)state, faults);
                target_current_ma = Charge_Manager_Target_Current_For_State(state);
                control_mode = Charge_Manager_Control_Mode_For_State(mode, state);
                Power_Control_Set_Battery_Current_Feedback(battery_current_ma, 1U);
                Power_Control_Set_Battery_Voltage_Feedback(afe->packVoltageMv, 1U);
                if(afe_handover_pending != 0U) {
                    Power_Control_Set_Afe_Handover(params.targetVoltageMv,
                                                   target_current_ma,
                                                   control_mode,
                                                   power_sample);
                } else {
                    Power_Control_Set(params.targetVoltageMv, target_current_ma, control_mode);
                }
            } else {
                Charge_Manager_Update_Path_Settle(0U, afe, &params);
                /*
                 * 24V 输入下全程主动 Boost 预连接：先把 Vout 推到 Pack 附近，
                 * 再闭合 BM2016 主 CHG+DSG。预连接目标保持在闭合确认窗口内，
                 * 避免目标电压本身把 |Vout-Pack| 拉出 50 mV 阈值。
                 */
                Safety_Manager_Set_Afe_Alert_Monitor(0U);
                state = BMS_CHARGE_STATE_PRECHECK;
                if(Charge_Manager_Preconnect_Can_Drive(afe, power_sample) != 0U) {
                    preconnect_target_mv = Charge_Manager_Preconnect_Target_Mv(afe, power_sample, &params);
                    if(preconnect_target_mv != 0U) {
                        if(preconnect_target_mv >
                           (uint16_t)(0xFFFFU - BMS_PRECONNECT_OUTPUT_OVP_MARGIN_MV)) {
                            preconnect_ovp_limit_mv = 0xFFFFU;
                        } else {
                            preconnect_ovp_limit_mv =
                                (uint16_t)((uint32_t)preconnect_target_mv +
                                           (uint32_t)BMS_PRECONNECT_OUTPUT_OVP_MARGIN_MV);
                        }
                        Power_Control_Set_Preconnect(preconnect_target_mv,
                                                     Charge_Manager_Preconnect_Current_Limit_Ma(afe,
                                                                                                power_sample,
                                                                                                period_ms),
                                                     preconnect_ovp_limit_mv);
                    } else {
                        Power_Control_Stop();
                    }
                } else {
                    Power_Control_Stop();
                }
            }
        }
    }

    if(manual_fet_active != 0U) {
        /* Manual FET debug owns CHG/PCHG/DSG/PDSG until a normal command exits it. */
    } else if(work_mode == (uint8_t)BMS_WORK_MODE_DIGITAL_POWER || digital_power_enabled != 0U) {
        Charge_Manager_Clear_Path_Settle();
        Power_Path_Manager_Force_Off();
#if BMS_DIGITAL_POWER_AFELESS_DEBUG
    } else if((run_request == 0U) && (Charge_Manager_Afe_Sample_Valid(afe) == 0U)) {
        Charge_Manager_Clear_Path_Settle();
        Power_Path_Manager_Force_Off();
#endif
    } else if((run_request != 0U) && (path_manager_updated == 0U)) {
        Power_Path_Manager_Update(afe, power_sample, &params, state, faults);
    } else if(run_request == 0U) {
        Charge_Manager_Clear_Path_Settle();
        Power_Path_Manager_Force_Off();
    }

    memset(status, 0, sizeof(*status));
    /* status 是 SOC、均衡和 UART 上报共同使用的统一状态结构。 */
    status->packVoltageMv = afe->packVoltageMv;
    status->inputVoltageMv = power_sample->inputVoltageMv;
    status->outputVoltageMv = power_sample->outputVoltageMv;
    status->chargeState = (uint8_t)state;
    status->chargeMode = mode;
    status->workMode = work_mode;
    status->faultBitmap = faults;
    status->temperaturesX10[0] = afe->temperaturesX10[0];
    status->temperaturesX10[1] = afe->temperaturesX10[1];
    status->temperaturesX10[2] = afe->temperaturesX10[2];
    status->temperaturesX10[3] = afe->temperaturesX10[3];
    status->temperaturesX10[4] = power_sample->mosTempX10;
    status->temperaturesX10[5] = power_sample->inductorTempX10;
    status->cellMaxMv = afe->cellMaxMv;
    status->cellMinMv = afe->cellMinMv;
    status->cellDeltaMv = afe->cellDeltaMv;

    for(i = 0U; i < BMS_CELL_COUNT; i++) {
        status->cellMv[i] = afe->cellMv[i];
    }

    {
        power_control_state_t power_state;
        Power_Control_Get_State(&power_state);
        if((work_mode != (uint8_t)BMS_WORK_MODE_DIGITAL_POWER) &&
           (digital_power_enabled == 0U) &&
           (Charge_Manager_Afe_Sample_Valid(afe) != 0U)) {
            status->chargeCurrentMa = battery_current_ma;
        } else {
            status->chargeCurrentMa = 0;
        }
        status->dutyX100 = power_state.dutyX100;
    }

    taskENTER_CRITICAL();
    s_state = state;
    s_run_request = run_request;
    s_digital_power_enabled = digital_power_enabled;
    s_manual_fet_active = manual_fet_active;
    s_manual_fet_mask = manual_fet_mask;
    s_digital_power_target_voltage_mv = digital_power_target_voltage_mv;
    s_digital_power_current_limit_ma = digital_power_current_limit_ma;
    if((work_mode != (uint8_t)BMS_WORK_MODE_DIGITAL_POWER) &&
       (digital_power_enabled == 0U) &&
       (s_mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER) &&
       ((faults == 0U) || (digital_power_fault_context == 0U))) {
        s_mode = (uint8_t)BMS_CHARGE_MODE_AUTO;
    } else {
        s_mode = mode;
    }
    taskEXIT_CRITICAL();
}
