#include "charge_manager_internal.h"

uint32_t Charge_Manager_Filter_Digital_Power_Afeless_Faults(uint32_t faults)
{
#if BMS_DIGITAL_POWER_AFELESS_DEBUG
    return faults & ~DIGITAL_POWER_AFELESS_FAULT_MASK;
#else
    return faults;
#endif
}

uint8_t Charge_Manager_Afeless_Filter_Active(uint8_t digital_power_enabled,
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

uint8_t Charge_Manager_Afe_Sample_Valid(const bms_afe_data_t *afe)
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

uint8_t Charge_Manager_Temperature_Valid(int16_t temperature_x10)
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

uint8_t Charge_Manager_Power_Ocp_Check_Active(uint8_t power_requested,
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

void Charge_Manager_Reset_Output_Ovp_Counters(void)
{
    g_charge_manager.fault.outputOvpConfirmCount = 0U;
    g_charge_manager.fault.digitalOutputOvpConfirmCount = 0U;
}

void Charge_Manager_Reset_Charge_Ocp_Counter(void)
{
    g_charge_manager.fault.chargeOcpConfirmCount = 0U;
}


uint8_t Charge_Manager_Output_Ovp_Confirmed(uint16_t output_voltage_mv,
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
            g_charge_manager.fault.digitalOutputOvpConfirmCount = 0U;
        } else {
            g_charge_manager.fault.outputOvpConfirmCount = 0U;
        }
        return 0U;
    }

    counter = (digital_power != 0U) ? &g_charge_manager.fault.digitalOutputOvpConfirmCount :
                                      &g_charge_manager.fault.outputOvpConfirmCount;
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

void Charge_Manager_Record_Faults(uint32_t realtime_faults,
                                        uint32_t latchable_faults)
{
    if(latchable_faults != 0U) {
        taskENTER_CRITICAL();
        g_charge_manager.fault.latchedFaults |= latchable_faults;
        taskEXIT_CRITICAL();
        Safety_Manager_Report_Faults(latchable_faults);
    }

    taskENTER_CRITICAL();
    g_charge_manager.fault.presentFaults = realtime_faults;
    taskEXIT_CRITICAL();
}

uint32_t Charge_Manager_Collect_Faults(const bms_afe_data_t *afe,
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
     * 閿佸瓨鏁呴殰浼氫竴鐩翠繚鐣欏埌鏀跺埌 CMD_CLEAR_FAULT銆傚疄鏃舵晠闅滄湰杞噸鏂伴噰闆嗭紝
     * 涓€鏃﹀嚭鐜板氨鍚屾浜ょ粰 safety_manager 绔嬪嵆鍏虫柇 Q4/Q5銆?
     */
    taskENTER_CRITICAL();
    faults = g_charge_manager.fault.latchedFaults;
    digital_power_enabled = g_charge_manager.digital.enabled;
    output_target_mv = (digital_power_enabled != 0U) ?
                       g_charge_manager.digital.targetVoltageMv :
                       g_charge_manager.config.params.targetVoltageMv;
    taskEXIT_CRITICAL();
    faults |= Safety_Manager_Get_Latched_Faults();
    afeless_filter_active = Charge_Manager_Afeless_Filter_Active(digital_power_enabled,
                                                                 power_requested);
    if(afeless_filter_active != 0U) {
        faults = Charge_Manager_Filter_Digital_Power_Afeless_Faults(faults);
    }
    realtime_faults = 0U;
    current_limit_ma = g_charge_manager.config.params.targetCurrentMa;
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
            if(afe->cellMaxMv >= g_charge_manager.config.params.cellOvpMv) {
                realtime_faults |= BMS_FAULT_CELL_OVP;
            }
            if(afe->cellMinMv <= g_charge_manager.config.params.cellUvpMv) {
                realtime_faults |= BMS_FAULT_CELL_UVP;
            }
            if((digital_power_enabled == 0U) &&
               (power_requested != 0U) &&
               (afe->packVoltageMv >= (uint16_t)(output_target_mv + PACK_OVP_MARGIN_MV))) {
                realtime_faults |= BMS_FAULT_PACK_OVP;
            }
            for(i = 0U; i < BMS_AFE_TEMP_COUNT; i++) {
                if((0U != Charge_Manager_Temperature_Valid(afe->temperaturesX10[i])) &&
                   afe->temperaturesX10[i] >= g_charge_manager.config.params.tempOtpX10) {
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
         * 浠呭湪闇€瑕佸閮ㄨ緭鍏ヤ緵鐢电殑鍏呯數宸ュ喌涓嬶紝鎵嶆妸杈撳叆娆犲帇褰撲綔鏁呴殰銆?
         * 鏁板瓧鐢垫簮妯″紡鍜屾澘绾ц皟璇曢樁娈甸粯璁ゅ叧闂椤广€?
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
        /* 纭欢 FAULT_OC 鍜岃蒋浠剁數娴佽閲忚秴闄愰兘鏄犲皠涓哄厖鐢佃繃娴佹晠闅溿€?*/
        if((power_ocp_check_active != 0U) &&
           (power_sample->faultOcActive != 0U ||
            power_sample->outputCurrentMa > (int16_t)(current_limit_ma + 1500U))) {
            if(g_charge_manager.fault.chargeOcpConfirmCount < BMS_OUTPUT_OVP_CONFIRM_CONTROL_COUNT) {
                g_charge_manager.fault.chargeOcpConfirmCount++;
            }
            if(g_charge_manager.fault.chargeOcpConfirmCount >= BMS_OUTPUT_OVP_CONFIRM_CONTROL_COUNT) {
                realtime_faults |= BMS_FAULT_CHARGE_OCP;
            }
        } else {
            Charge_Manager_Reset_Charge_Ocp_Counter();
        }
        if(((0U != Charge_Manager_Temperature_Valid(power_sample->mosTempX10)) &&
            power_sample->mosTempX10 >= g_charge_manager.config.params.tempOtpX10) ||
           ((0U != Charge_Manager_Temperature_Valid(power_sample->inductorTempX10)) &&
            power_sample->inductorTempX10 >= g_charge_manager.config.params.tempOtpX10)) {
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
    g_charge_manager.fault.presentFaults = realtime_faults;
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
        g_charge_manager.fault.latchedFaults |= latchable_faults;
        faults |= g_charge_manager.fault.latchedFaults;
        taskEXIT_CRITICAL();
        Safety_Manager_Report_Faults(latchable_faults);
    }

    faults |= realtime_faults | Safety_Manager_Get_Latched_Faults();
    if(afeless_filter_active != 0U) {
        faults = Charge_Manager_Filter_Digital_Power_Afeless_Faults(faults);
    }

    return faults;
}

uint32_t Charge_Manager_Collect_Digital_Power_Faults(const bms_power_sample_t *power_sample,
                                                            uint16_t target_voltage_mv,
                                                            uint16_t current_limit_ma)
{
    uint32_t faults;
    uint32_t realtime_faults;
    power_control_state_t power_state;

    taskENTER_CRITICAL();
    faults = g_charge_manager.fault.latchedFaults;
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
            power_sample->mosTempX10 >= g_charge_manager.config.params.tempOtpX10) ||
           ((0U != Charge_Manager_Temperature_Valid(power_sample->inductorTempX10)) &&
            power_sample->inductorTempX10 >= g_charge_manager.config.params.tempOtpX10)) {
            realtime_faults |= BMS_FAULT_OTP;
        }
    }

    realtime_faults |= Safety_Manager_Sample_Fast_Faults();
    realtime_faults = Charge_Manager_Filter_Digital_Power_Afeless_Faults(realtime_faults);
    Charge_Manager_Record_Faults(realtime_faults, realtime_faults);

    taskENTER_CRITICAL();
    faults |= g_charge_manager.fault.latchedFaults;
    taskEXIT_CRITICAL();

    faults |= realtime_faults | Safety_Manager_Get_Latched_Faults();
    faults = Charge_Manager_Filter_Digital_Power_Afeless_Faults(faults);

    return faults;
}

