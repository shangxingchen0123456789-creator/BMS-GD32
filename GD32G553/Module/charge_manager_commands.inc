/* Internal implementation fragment for charge_manager.c only. */

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

    s_digital_power_enabled = 0U;
    Charge_Manager_Clear_Path_Settle_Unlocked();
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
    s_digital_power_current_limit_ma = 0U;
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
    s_digital_power_enabled = 0U;
    Charge_Manager_Clear_Manual_Fet();
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
            s_digital_power_enabled = 0U;
            Charge_Manager_Clear_Manual_Fet();
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
            s_digital_power_enabled = 0U;
            Charge_Manager_Clear_Manual_Fet();
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
            s_digital_power_enabled = 0U;
            Charge_Manager_Clear_Manual_Fet();
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

