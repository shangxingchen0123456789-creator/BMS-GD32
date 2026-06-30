#include "charge_manager_internal.h"

charge_manager_context_t g_charge_manager;

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
    params = g_charge_manager.params;
    state = g_charge_manager.state;
    run_request = g_charge_manager.runRequest;
    mode = g_charge_manager.mode;
    work_mode = g_charge_manager.workMode;
    digital_power_enabled = g_charge_manager.digitalPowerEnabled;
    manual_fet_active = g_charge_manager.manualFetActive;
    manual_fet_mask = g_charge_manager.manualFetMask;
    digital_power_target_voltage_mv = g_charge_manager.digitalPowerTargetVoltageMv;
    digital_power_current_limit_ma = g_charge_manager.digitalPowerCurrentLimitMa;
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
     * 杩欓噷鐨勪紭鍏堢骇涓嶈兘闅忔剰璋冩暣锛?
     * 鍏堝鐞嗘晠闅滃苟绔嬪嵆鍏?PWM锛涘啀澶勭悊鍋滄/绌洪棽锛涙渶鍚庢墠鍏佽鑷姩鍏呯數鏇茬嚎
     * 鍦ㄦ稉娴併€佹亽娴併€佹亽鍘嬪拰瀹屾垚鐘舵€佷箣闂村垏鎹€?
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
            /* 鑷姩鏇茬嚎锛氫綆鍘嬪崟浣撳厛娑撴祦锛屾甯稿悗鎭掓祦锛屾帴杩戠洰鏍囩數鍘嬪悗杩涘叆鎭掑帇銆?*/
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
                /* 闇€瑕佽繛缁娆′綆鐢垫祦閲囨牱锛屾墠鍒ゅ畾鍏呯數瀹屾垚锛岄伩鍏嶅伓鍙戞姈鍔ㄨ瑙﹀彂銆?*/
                g_charge_manager.cvDoneCounter++;
                if(g_charge_manager.cvDoneCounter > CV_DONE_CONFIRM_COUNT) {
                    state = BMS_CHARGE_STATE_DONE;
                    run_request = 0U;
                    Charge_Manager_Clear_Path_Settle();
                    Power_Control_Stop();
                }
            } else {
                g_charge_manager.cvDoneCounter = 0U;
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
                 * 24V 杈撳叆涓嬪叏绋嬩富鍔?Boost 棰勮繛鎺ワ細鍏堟妸 Vout 鎺ㄥ埌 Pack 闄勮繎锛?
                 * 鍐嶉棴鍚?BM2016 涓?CHG+DSG銆傞杩炴帴鐩爣淇濇寔鍦ㄩ棴鍚堢‘璁ょ獥鍙ｅ唴锛?
                 * 閬垮厤鐩爣鐢靛帇鏈韩鎶?|Vout-Pack| 鎷夊嚭 50 mV 闃堝€笺€?
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
    /* status 鏄?SOC銆佸潎琛″拰 UART 涓婃姤鍏卞悓浣跨敤鐨勭粺涓€鐘舵€佺粨鏋勩€?*/
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
    g_charge_manager.state = state;
    g_charge_manager.runRequest = run_request;
    g_charge_manager.digitalPowerEnabled = digital_power_enabled;
    g_charge_manager.manualFetActive = manual_fet_active;
    g_charge_manager.manualFetMask = manual_fet_mask;
    g_charge_manager.digitalPowerTargetVoltageMv = digital_power_target_voltage_mv;
    g_charge_manager.digitalPowerCurrentLimitMa = digital_power_current_limit_ma;
    if((work_mode != (uint8_t)BMS_WORK_MODE_DIGITAL_POWER) &&
       (digital_power_enabled == 0U) &&
       (g_charge_manager.mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER) &&
       ((faults == 0U) || (digital_power_fault_context == 0U))) {
        g_charge_manager.mode = (uint8_t)BMS_CHARGE_MODE_AUTO;
    } else {
        g_charge_manager.mode = mode;
    }
    taskEXIT_CRITICAL();
}
