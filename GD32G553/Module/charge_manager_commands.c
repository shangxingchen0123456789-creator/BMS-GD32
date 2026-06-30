#include "charge_manager_internal.h"

bms_command_reply_t Charge_Manager_Handle_Fet_Mask_Command(uint8_t fet_mask)
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
    if(g_charge_manager.workMode != (uint8_t)BMS_WORK_MODE_BMS) {
        taskEXIT_CRITICAL();
        return Reply_Error((uint8_t)BMS_CMD_ERROR_INVALID_MODE);
    }
    blocking_faults = g_charge_manager.latchedFaults | Safety_Manager_Get_Latched_Faults();
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

    g_charge_manager.digitalPowerEnabled = 0U;
    Charge_Manager_Clear_Path_Settle_Unlocked();
    g_charge_manager.runRequest = 0U;
    g_charge_manager.state = BMS_CHARGE_STATE_IDLE;
    if(g_charge_manager.mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER) {
        g_charge_manager.mode = (uint8_t)BMS_CHARGE_MODE_AUTO;
    }
    g_charge_manager.cvDoneCounter = 0U;
    Charge_Manager_Reset_Output_Ovp_Counters();
    /*
     * Claim manual FET ownership before any I2C delay. Otherwise the 20 ms
     * control loop can see run_request == 0 and close the path again while the
     * command task is still waiting for BM2016 FET writes to finish.
     */
    g_charge_manager.manualFetActive = 1U;
    g_charge_manager.manualFetMask = fet_mask;
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
    g_charge_manager.manualFetActive = 1U;
    g_charge_manager.manualFetMask = fet_mask;
    taskEXIT_CRITICAL();

    if(0U == Afe_Gd30bm2016_Set_Fet_Mask(fet_mask, &fet_status)) {
        taskENTER_CRITICAL();
        Charge_Manager_Clear_Manual_Fet();
        taskEXIT_CRITICAL();
        Power_Path_Manager_Force_Off();
        return Reply_Error((uint8_t)BMS_CMD_ERROR_FAULT_ACTIVE);
    }

    taskENTER_CRITICAL();
    g_charge_manager.manualFetActive = 1U;
    g_charge_manager.manualFetMask = fet_mask;
    taskEXIT_CRITICAL();

    return Reply_Ok();
}


void Charge_Manager_Init(void)
{
    bms_charge_parameters_t stored_params;

    taskENTER_CRITICAL();
    if(Param_Storage_Get_Charge(&stored_params) != 0U) {
        Charge_Manager_Clamp_Parameters(&stored_params);
        if(Charge_Manager_Validate_Parameters(&stored_params) != 0U) {
            g_charge_manager.params = stored_params;
        } else {
            Charge_Manager_Default_Parameters(&g_charge_manager.params);
            Charge_Manager_Clamp_Parameters(&g_charge_manager.params);
        }
    } else {
        Charge_Manager_Default_Parameters(&g_charge_manager.params);
        Charge_Manager_Clamp_Parameters(&g_charge_manager.params);
    }
    g_charge_manager.state = BMS_CHARGE_STATE_IDLE;
    g_charge_manager.runRequest = 0U;
    g_charge_manager.mode = (uint8_t)BMS_CHARGE_MODE_AUTO;
    g_charge_manager.workMode = (uint8_t)BMS_WORK_MODE_BMS;
    g_charge_manager.latchedFaults = 0U;
    g_charge_manager.presentFaults = 0U;
    g_charge_manager.cvDoneCounter = 0U;
    g_charge_manager.digitalPowerEnabled = 0U;
    g_charge_manager.digitalPowerTargetVoltageMv = 0U;
    g_charge_manager.digitalPowerCurrentLimitMa = 0U;
    Charge_Manager_Clear_Manual_Fet();
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
    *parameters = g_charge_manager.params;
    taskEXIT_CRITICAL();
}

uint8_t Charge_Manager_Is_Digital_Power_Active(void)
{
    uint8_t active;

    taskENTER_CRITICAL();
    active = ((g_charge_manager.workMode == (uint8_t)BMS_WORK_MODE_DIGITAL_POWER) ||
              (g_charge_manager.digitalPowerEnabled != 0U) ||
              (g_charge_manager.state == BMS_CHARGE_STATE_DIGITAL_POWER) ||
              (g_charge_manager.mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER)) ? 1U : 0U;
    taskEXIT_CRITICAL();

    return active;
}

uint8_t Charge_Manager_Get_Work_Mode(void)
{
    uint8_t work_mode;

    taskENTER_CRITICAL();
    work_mode = g_charge_manager.workMode;
    taskEXIT_CRITICAL();

    return work_mode;
}

bms_command_reply_t Charge_Manager_Set_Work_Mode(uint8_t work_mode)
{
    if(work_mode > (uint8_t)BMS_WORK_MODE_DIGITAL_POWER) {
        return Reply_Error((uint8_t)BMS_CMD_ERROR_INVALID_PARAM);
    }

    taskENTER_CRITICAL();
    g_charge_manager.workMode = work_mode;
    g_charge_manager.digitalPowerEnabled = 0U;
    Charge_Manager_Clear_Manual_Fet();
    Charge_Manager_Clear_Path_Settle_Unlocked();
    g_charge_manager.runRequest = 0U;
    g_charge_manager.state = BMS_CHARGE_STATE_IDLE;
    g_charge_manager.mode = (work_mode == (uint8_t)BMS_WORK_MODE_DIGITAL_POWER) ?
             (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER :
             (uint8_t)BMS_CHARGE_MODE_AUTO;
    g_charge_manager.cvDoneCounter = 0U;
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
    if(g_charge_manager.workMode != (uint8_t)BMS_WORK_MODE_BMS) {
        taskEXIT_CRITICAL();
        return Reply_Error((uint8_t)BMS_CMD_ERROR_INVALID_MODE);
    }
    g_charge_manager.params = clamped_parameters;
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
        if(g_charge_manager.workMode != (uint8_t)BMS_WORK_MODE_BMS) {
            reply = Reply_Error((uint8_t)BMS_CMD_ERROR_INVALID_MODE);
        } else if((g_charge_manager.latchedFaults | Safety_Manager_Get_Latched_Faults()) != 0U) {
            reply = Reply_Error((uint8_t)BMS_CMD_ERROR_FAULT_ACTIVE);
        } else {
            g_charge_manager.digitalPowerEnabled = 0U;
            Charge_Manager_Clear_Manual_Fet();
            Charge_Manager_Clear_Path_Settle_Unlocked();
            if(g_charge_manager.mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER) {
                g_charge_manager.mode = (uint8_t)BMS_CHARGE_MODE_AUTO;
            }
            g_charge_manager.runRequest = 1U;
            g_charge_manager.state = BMS_CHARGE_STATE_PRECHECK;
            g_charge_manager.cvDoneCounter = 0U;
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
        g_charge_manager.digitalPowerEnabled = 0U;
        Charge_Manager_Clear_Manual_Fet();
        Charge_Manager_Clear_Path_Settle_Unlocked();
        if((g_charge_manager.workMode != (uint8_t)BMS_WORK_MODE_DIGITAL_POWER) &&
           (g_charge_manager.mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER)) {
            g_charge_manager.mode = (uint8_t)BMS_CHARGE_MODE_AUTO;
        }
        g_charge_manager.runRequest = 0U;
        g_charge_manager.state = BMS_CHARGE_STATE_IDLE;
        g_charge_manager.cvDoneCounter = 0U;
        Charge_Manager_Reset_Output_Ovp_Counters();
        taskEXIT_CRITICAL();
        Power_Control_Stop();
        Power_Path_Manager_Force_Off();
        Safety_Manager_Set_Afe_Alert_Monitor(0U);
        reply = Reply_Ok();
        break;

    case CMD_EMERGENCY_STOP:
        taskENTER_CRITICAL();
        g_charge_manager.digitalPowerEnabled = 0U;
        Charge_Manager_Clear_Manual_Fet();
        Charge_Manager_Clear_Path_Settle_Unlocked();
        g_charge_manager.runRequest = 0U;
        g_charge_manager.state = BMS_CHARGE_STATE_FAULT;
        g_charge_manager.latchedFaults |= BMS_FAULT_EMERGENCY_STOP;
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
        present_faults = g_charge_manager.presentFaults;
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
            g_charge_manager.digitalPowerEnabled = 0U;
            Charge_Manager_Clear_Manual_Fet();
            Charge_Manager_Clear_Path_Settle_Unlocked();
            if((g_charge_manager.workMode != (uint8_t)BMS_WORK_MODE_DIGITAL_POWER) &&
               (g_charge_manager.mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER)) {
                g_charge_manager.mode = (uint8_t)BMS_CHARGE_MODE_AUTO;
            }
            g_charge_manager.runRequest = 0U;
            g_charge_manager.state = BMS_CHARGE_STATE_IDLE;
            g_charge_manager.latchedFaults = 0U;
            g_charge_manager.presentFaults = 0U;
            g_charge_manager.cvDoneCounter = 0U;
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
            if(g_charge_manager.workMode != (uint8_t)BMS_WORK_MODE_BMS) {
                reply = Reply_Error((uint8_t)BMS_CMD_ERROR_INVALID_MODE);
                run_request_active = 0U;
            } else {
            g_charge_manager.digitalPowerEnabled = 0U;
            Charge_Manager_Clear_Manual_Fet();
            Charge_Manager_Clear_Path_Settle_Unlocked();
            g_charge_manager.mode = argument;
            run_request_active = g_charge_manager.runRequest;
            if(g_charge_manager.runRequest != 0U) {
                /*
                 * 杩愯涓垏鎹㈡ā寮忔椂锛屼笅涓€杞帶鍒跺懆鏈熷繀椤婚噸鏂版寜鏂版ā寮忛€夋嫨
                 * 娑撴祦/鎭掓祦/鎭掑帇闃舵锛屽惁鍒欑洰鏍囩數娴佸彲鑳借繕娌跨敤鏃ч樁娈点€?
                */
                g_charge_manager.state = BMS_CHARGE_STATE_PRECHECK;
                g_charge_manager.cvDoneCounter = 0U;
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
        g_charge_manager.digitalPowerEnabled = 0U;
        Charge_Manager_Clear_Manual_Fet();
        Charge_Manager_Clear_Path_Settle_Unlocked();
        if((g_charge_manager.workMode != (uint8_t)BMS_WORK_MODE_DIGITAL_POWER) &&
           (g_charge_manager.mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER)) {
            g_charge_manager.mode = (uint8_t)BMS_CHARGE_MODE_AUTO;
        }
        g_charge_manager.runRequest = 0U;
        g_charge_manager.state = BMS_CHARGE_STATE_IDLE;
        g_charge_manager.cvDoneCounter = 0U;
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
    if(g_charge_manager.workMode != (uint8_t)BMS_WORK_MODE_DIGITAL_POWER) {
        taskEXIT_CRITICAL();
        return Reply_Error((uint8_t)BMS_CMD_ERROR_INVALID_MODE);
    }
    if(g_charge_manager.digitalPowerEnabled != 0U) {
        /*
         * 宸插湪鏁板瓧鐢垫簮杩愯涓紝鐩存帴鏇存柊鐩爣鐢靛帇/鐢垫祦銆?
         * 鐢靛帇鐜細骞虫粦璺熻釜鍒版柊璁惧畾鍊硷紝涓嶅啀鍋滄満娉勬斁鍐嶉噸鍚€?
         * 璋冧綆鐢靛帇鏃剁敱 OVP 杞檺骞呭厹搴曪紝鑰屼笉鏄厛鎺夌數銆?
         */
        g_charge_manager.digitalPowerTargetVoltageMv = target_voltage_mv;
        g_charge_manager.digitalPowerCurrentLimitMa = current_limit_ma;
        Charge_Manager_Clear_Manual_Fet();
        Charge_Manager_Clear_Path_Settle_Unlocked();
        g_charge_manager.state = BMS_CHARGE_STATE_DIGITAL_POWER;
        g_charge_manager.mode = (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER;
        g_charge_manager.cvDoneCounter = 0U;
        Charge_Manager_Reset_Output_Ovp_Counters();
        taskEXIT_CRITICAL();
        Power_Control_Set(target_voltage_mv, current_limit_ma,
                          (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER);
        Safety_Manager_Set_Afe_Alert_Monitor(0U);
        return Reply_Ok();
    }
    raw_faults = g_charge_manager.latchedFaults | Safety_Manager_Get_Latched_Faults();
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
        g_charge_manager.latchedFaults = 0U;
        taskEXIT_CRITICAL();
        Safety_Manager_Clear_Latched_Faults();
        Power_Control_Clear_Fault_Lockout();
        Power_Control_Clear_Fault_Status();
    }
#endif

    taskENTER_CRITICAL();
    g_charge_manager.digitalPowerEnabled = 1U;
    g_charge_manager.digitalPowerTargetVoltageMv = target_voltage_mv;
    g_charge_manager.digitalPowerCurrentLimitMa = current_limit_ma;
    Charge_Manager_Clear_Manual_Fet();
    Charge_Manager_Clear_Path_Settle_Unlocked();
    g_charge_manager.runRequest = 0U;
    g_charge_manager.state = BMS_CHARGE_STATE_DIGITAL_POWER;
    g_charge_manager.mode = (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER;
    g_charge_manager.cvDoneCounter = 0U;
    Charge_Manager_Reset_Output_Ovp_Counters();
    reply = Reply_Ok();
    taskEXIT_CRITICAL();

    if(reply.result == (uint8_t)BMS_CMD_RESULT_OK) {
        Power_Control_Stop();
    }

    return reply;
}

