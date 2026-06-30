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


#include "charge_manager_params.inc"
#include "charge_manager_path.inc"
#include "charge_manager_faults.inc"
#include "charge_manager_control.inc"
#include "charge_manager_commands.inc"

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
