void Power_Control_Init(void)
{
    pi_controller_config_t current_config;
    pi_controller_config_t voltage_config;

    memset(&s_power, 0, sizeof(s_power));
    s_fault_lockout = 0U;
    s_fault_status = 0U;
    Power_Control_Clear_Stall_Recover();

    current_config.kpDiv = (int32_t)POWER_CURRENT_KP_DIV;
    current_config.kiDiv = (int32_t)POWER_CURRENT_KI_DIV;
    current_config.integralLimit = POWER_INTEGRAL_LIMIT;
    current_config.stepLimit = (int32_t)POWER_LOOP_STEP_MAX_X100;
    voltage_config.kpDiv = (int32_t)POWER_VOLTAGE_KP_DIV;
    voltage_config.kiDiv = (int32_t)POWER_VOLTAGE_KI_DIV;
    voltage_config.integralLimit = POWER_INTEGRAL_LIMIT;
    voltage_config.stepLimit = (int32_t)POWER_LOOP_STEP_MAX_X100;
    Pi_Controller_Init(&s_current_pi, &current_config);
    Pi_Controller_Init(&s_voltage_pi, &voltage_config);

    Power_Control_Reset_Loop();

    Power_Pwm_Init();
}

void Power_Control_Stop(void)
{
    /*
     * 统一停止入口。
     * 故障、上位机停止命令、充电完成都会走这里，确保软件状态和硬件输出一致。
     */
    s_power.enabled = 0U;
    s_power.powerStageMode = (uint8_t)POWER_STAGE_MODE_OFF;
    s_power.targetCurrentMa = 0U;
    s_power.dutyX100 = 0U;
    Power_Control_Set_Battery_Current_Feedback(0, 0U);
    Power_Control_Set_Battery_Voltage_Feedback(0U, 0U);
    s_preconnect_active = 0U;
    s_preconnect_ovp_limit_mv = 0U;
    s_afe_handover_guard_count = 0U;
    Power_Control_Clear_Stall_Recover();
    Power_Control_Reset_Loop();

    Power_Pwm_Outputs_Disable();
}

void Power_Control_Fault_Lockout(void)
{
    s_fault_lockout = 1U;
    Power_Control_Stop();
}

void Power_Control_Clear_Fault_Lockout(void)
{
    s_fault_lockout = 0U;
}

uint32_t Power_Control_Get_Fault_Status(void)
{
    return s_fault_status;
}

void Power_Control_Clear_Fault_Status(void)
{
    s_fault_status = 0U;
    s_power.tripReason = (uint8_t)POWER_TRIP_REASON_NONE;
    s_power.tripFaults = 0U;
    s_power.tripIoutMa = 0;
    s_power.tripCurrentRefMa = 0U;
    s_power.tripOcpLimitMa = 0U;
    s_power.tripVoutMv = 0U;
    s_power.tripVinMv = 0U;
    s_power.tripDutyX100 = 0U;
    s_power.tripFaultOcActive = 0U;
}

void Power_Control_Set(uint16_t target_voltage_mv, uint16_t target_current_ma, uint8_t mode)
{
    uint8_t was_enabled;
    uint8_t was_preconnect;
    uint8_t target_changed;
    uint8_t allow_ovp_blank;
    uint16_t previous_target_voltage_mv;

    /*
     * 写入新的功率级目标。
     * 从停止状态首次进入运行状态时，重置 PI 和软启动电流；运行过程中状态机
     * 每 20 ms 会重复调用本函数，此时只更新目标，不清空环路历史。
     */
    target_voltage_mv = Power_Control_Clamp_Charge_Target_Mv(target_voltage_mv, mode);
    if(s_fault_lockout != 0U || s_fault_status != 0U) {
        Power_Control_Stop();
        return;
    }
    if(target_current_ma == 0U) {
        Power_Control_Stop();
        return;
    }

    was_enabled = s_power.enabled;
    was_preconnect = s_preconnect_active;
    previous_target_voltage_mv = s_power.targetVoltageMv;
    target_changed = ((was_enabled != 0U) &&
                      ((s_power.targetVoltageMv != target_voltage_mv) ||
                       (s_power.mode != mode))) ? 1U : 0U;
    if((was_enabled != 0U) &&
       (was_preconnect != 0U) &&
       (target_voltage_mv >= previous_target_voltage_mv)) {
        target_changed = 0U;
    }
    /*
     * 数字电源在运行中改设定值时不复位环路。
     * 保持当前 duty 和 PI 历史，让电压环平滑跟踪到新目标，
     * 而不是把 duty 砸回起始值造成 Vout 先掉再爬。
     */
    if((was_enabled != 0U) &&
       (mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER) &&
       (s_power.mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER)) {
        target_changed = 0U;
    }
    allow_ovp_blank = ((was_enabled == 0U) ||
                       (s_power.mode != mode) ||
                       (target_voltage_mv > s_power.targetVoltageMv)) ? 1U : 0U;
    s_preconnect_active = 0U;
    s_preconnect_ovp_limit_mv = 0U;
    if(mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER) {
        s_afe_handover_guard_count = 0U;
    }
    s_power.mode = mode;
    s_power.targetVoltageMv = target_voltage_mv;
    s_power.targetCurrentMa = target_current_ma;

    if((was_enabled == 0U) || (target_changed != 0U)) {
        Power_Control_Reset_Loop();
        if(allow_ovp_blank == 0U) {
            s_output_ovp_blank_count = 0U;
        }
        s_power.dutyX100 = POWER_START_DUTY_X100;
    }
    s_power.enabled = 1U;

    if(s_fault_lockout == 0U && s_fault_status == 0U) {
        Power_Control_Pwm_Apply();
        Power_Control_Pwm_Enable();
    } else {
        Power_Control_Stop();
    }
}

void Power_Control_Set_Preconnect(uint16_t target_voltage_mv,
                                  uint16_t target_current_ma,
                                  uint16_t ovp_limit_mv)
{
    Power_Control_Set(target_voltage_mv, target_current_ma, (uint8_t)BMS_CHARGE_MODE_CV);
    if(s_power.enabled != 0U) {
        s_preconnect_active = 1U;
        s_preconnect_ovp_limit_mv = ovp_limit_mv;
        s_afe_handover_guard_count = 0U;
    }
}

void Power_Control_Set_Afe_Handover(uint16_t target_voltage_mv,
                                    uint16_t target_current_ma,
                                    uint8_t mode,
                                    const bms_power_sample_t *sample)
{
    uint16_t previous_boost_duty_x100;

    previous_boost_duty_x100 = s_boost_duty_x100;
    Power_Control_Set(target_voltage_mv, target_current_ma, mode);
    if(s_power.enabled != 0U) {
        s_preconnect_active = 0U;
        s_preconnect_ovp_limit_mv = 0U;
        s_afe_handover_guard_count = BMS_POWER_AFE_HANDOVER_GUARD_CYCLES;
        Power_Control_Clear_Stall_Recover();
        Power_Control_Reset_Loop();
        Power_Control_Afe_Handover_Start_Output(sample, previous_boost_duty_x100);
        Power_Control_Pwm_Apply();
        Power_Control_Pwm_Enable();
    }
}

void Power_Control_Set_Battery_Current_Feedback(int16_t current_ma, uint8_t valid)
{
    if(valid != 0U) {
        s_battery_current_feedback_ma = current_ma;
        s_battery_current_feedback_valid = 1U;
    } else {
        s_battery_current_feedback_valid = 0U;
        s_battery_current_feedback_ma = 0;
    }
}

void Power_Control_Set_Battery_Voltage_Feedback(uint16_t pack_voltage_mv, uint8_t valid)
{
    if(valid != 0U && pack_voltage_mv != 0U) {
        s_battery_voltage_feedback_mv = pack_voltage_mv;
        s_battery_voltage_feedback_valid = 1U;
    } else {
        s_battery_voltage_feedback_valid = 0U;
        s_battery_voltage_feedback_mv = 0U;
    }
}

static void Power_Control_Latch_Fault(uint8_t reason,
                                      uint32_t faults,
                                      const bms_power_sample_t *sample,
                                      uint16_t current_ref_ma)
{
    if(faults == 0U) {
        return;
    }

    Power_Control_Record_Trip(reason, faults, sample, current_ref_ma);
    s_fault_status |= faults;
    Power_Control_Fault_Lockout();
}

void Power_Control_Fast_Loop(const bms_power_sample_t *sample)
{
    uint16_t current_ref_ma;
    uint16_t measured_current_ma;
    uint16_t output_current_ma;
    uint8_t voltage_loop_active;
    uint8_t light_load_guard_active;
    int32_t error;
    int32_t step;
    int32_t duty;
    uint16_t light_load_duty_limit;

    if(sample == 0) {
        return;
    }

    if(s_fault_lockout != 0U) {
        Power_Control_Stop();
        return;
    }

    if(s_power.enabled == 0U) {
        s_power.dutyX100 = 0U;
        Power_Pwm_Outputs_Disable();
        return;
    }

    if(sample->faultBitmap != 0U) {
        Power_Control_Latch_Fault((uint8_t)POWER_TRIP_REASON_SAMPLE_FAULT,
                                  sample->faultBitmap,
                                  sample,
                                  s_soft_current_ma);
        return;
    }

#if BMS_POWER_FAULT_PIN_FAST_LATCH_ENABLE
    if(sample->faultOcActive != 0U) {
        Power_Control_Latch_Fault((uint8_t)POWER_TRIP_REASON_FAULT_OC_PIN,
                                  BMS_FAULT_CHARGE_OCP,
                                  sample,
                                  s_soft_current_ma);
        return;
    }
#endif

    if(s_power.targetCurrentMa == 0U) {
        Power_Control_Stop();
        return;
    }

#if BMS_ENABLE_INPUT_UV_FAULT
    if(sample->inputVoltageMv < BMS_DEFAULT_INPUT_UV_THRESHOLD_MV) {
        Power_Control_Latch_Fault((uint8_t)POWER_TRIP_REASON_INPUT_UV,
                                  BMS_FAULT_INPUT_UV,
                                  sample,
                                  s_soft_current_ma);
        return;
    }
#endif

    if(Power_Control_Output_Ovp_Confirmed(sample->outputVoltageMv) != 0U) {
        if(s_preconnect_active != 0U) {
            Power_Control_Preconnect_Coast();
            Power_Control_Pwm_Apply();
            Power_Control_Pwm_Enable();
            Power_Control_Service_Stall_Recover(sample);
            return;
        }
        /*
         * 数字电源调低输出时 Vout 会暂时高于新目标。软 OVP 区只滑行泄放，
         * 让电压环把 Vout 拉回新设定值；只有超过硬限才锁故障。
         */
        if((s_power.mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER) &&
           (Power_Control_Output_Over_Hard_Limit(sample->outputVoltageMv) == 0U)) {
            Power_Control_Preconnect_Coast();
            Power_Control_Pwm_Apply();
            Power_Control_Pwm_Enable();
            return;
        }
        Power_Control_Latch_Fault((uint8_t)POWER_TRIP_REASON_OUTPUT_OVP,
                                  BMS_FAULT_PACK_OVP,
                                  sample,
                                  s_soft_current_ma);
        return;
    }

    if(sample->inputVoltageMv == 0U) {
        Power_Control_Stop();
        return;
    }

    if(Power_Control_Input_Guard_Active(sample) != 0U) {
        Power_Control_Reduce_Duty_For_Input_Guard(sample);
        Power_Control_Pwm_Apply();
        Power_Control_Pwm_Enable();
        return;
    }

    output_current_ma = Power_Control_Positive_Output_Current_Ma(sample);
    measured_current_ma = Power_Control_Current_Feedback_Ma(sample);
    current_ref_ma = Power_Control_Ramp_Current(s_power.targetCurrentMa);
    if(Power_Control_Output_Ocp_Confirmed(output_current_ma, current_ref_ma) != 0U) {
        Power_Control_Latch_Fault((uint8_t)POWER_TRIP_REASON_OUTPUT_OCP_SOFTWARE,
                                  BMS_FAULT_CHARGE_OCP,
                                  sample,
                                  current_ref_ma);
        return;
    }

    if(Power_Control_Preconnect_Coast_Should_Run(sample) != 0U) {
        Power_Control_Preconnect_Coast();
        Power_Control_Pwm_Apply();
        Power_Control_Pwm_Enable();
        Power_Control_Service_Stall_Recover(sample);
        return;
    }

    voltage_loop_active = Power_Voltage_Loop_Should_Run(sample, current_ref_ma, measured_current_ma);
    light_load_guard_active =
        Power_Control_Light_Load_Near_Target(sample, current_ref_ma, measured_current_ma);

    if(voltage_loop_active != 0U) {
        error = (int32_t)s_power.targetVoltageMv - (int32_t)sample->outputVoltageMv;
        step = Pi_Controller_Update(&s_voltage_pi, error);
        if(light_load_guard_active != 0U) {
            Pi_Controller_Decay(&s_current_pi, 1U, 2U);
        } else {
            Pi_Controller_Decay(&s_current_pi, 7U, 8U);
        }
    } else {
        error = (int32_t)current_ref_ma - (int32_t)measured_current_ma;
        step = Pi_Controller_Update(&s_current_pi, error);
        Pi_Controller_Decay(&s_voltage_pi, 7U, 8U);
    }

    step = Power_Control_Limit_Light_Load_Step(sample,
                                               current_ref_ma,
                                               measured_current_ma,
                                               step);
    duty = (int32_t)s_power.dutyX100 + step;
    if(duty < (int32_t)POWER_DUTY_MIN_X100) {
        duty = (int32_t)POWER_DUTY_MIN_X100;
    } else if(duty > (int32_t)POWER_LOOP_DUTY_MAX_X100) {
        duty = (int32_t)POWER_LOOP_DUTY_MAX_X100;
    }
    if(light_load_guard_active != 0U) {
        light_load_duty_limit = Power_Control_Light_Load_Duty_Max(sample);
        if(duty > (int32_t)light_load_duty_limit) {
            duty = (int32_t)light_load_duty_limit;
        }
    }
    duty = Power_Control_Limit_Preconnect_Duty(sample, duty);
    duty = Power_Control_Limit_Battery_Boost_Duty(sample, duty);
    duty = Power_Control_Limit_Afe_Handover_Duty(sample, duty);

    s_power.dutyX100 = (uint16_t)duty;
    s_power.dutyX100 = Clamp_U16(s_power.dutyX100, POWER_DUTY_MIN_X100, POWER_LOOP_DUTY_MAX_X100);
    Power_Map_Control_To_Pwm(sample, s_power.dutyX100);
    s_async_boost_rectifier =
        Power_Control_Async_Boost_Should_Run(current_ref_ma, measured_current_ma);

    if(s_fault_lockout != 0U) {
        Power_Control_Stop();
        return;
    }

    Power_Control_Pwm_Apply();
    Power_Control_Pwm_Enable();
    Power_Control_Afe_Handover_Decay();
    Power_Control_Service_Stall_Recover(sample);
}

void Power_Control_Apply(const bms_power_sample_t *sample)
{
    Power_Control_Fast_Loop(sample);
}

void Power_Control_Get_State(power_control_state_t *state)
{
    if(state == 0) {
        return;
    }

    *state = s_power;
    state->buckDutyX100 = s_buck_duty_x100;
    state->boostLowDutyX100 = s_boost_duty_x100;
    state->faultLockout = s_fault_lockout;
    state->faultBitmap = s_fault_status;
    state->preconnectActive = s_preconnect_active;
    state->preconnectOvpLimitMv = s_preconnect_ovp_limit_mv;
    state->softCurrentMa = s_soft_current_ma;
    state->asyncBoostRectifier = s_async_boost_rectifier;
    {
        power_pwm_state_t pwm_state;

        Power_Pwm_Get_State(&pwm_state);
        state->hardwareReady = pwm_state.ready;
        state->hardwareOutputsOn = pwm_state.outputsOn;
        state->periodTicks = pwm_state.periodTicks;
    }
}

uint8_t Power_Control_Wait_Adc_Sample_Point(uint32_t timeout)
{
    return Power_Pwm_Wait_Adc_Sample_Point(timeout);
}
