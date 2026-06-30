static uint16_t Clamp_U16(uint16_t value, uint16_t min_value, uint16_t max_value)
{
    if(value < min_value) {
        return min_value;
    }
    if(value > max_value) {
        return max_value;
    }
    return value;
}

static uint16_t Power_Control_Clamp_Charge_Target_Mv(uint16_t target_voltage_mv, uint8_t mode)
{
    if(mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER) {
        return target_voltage_mv;
    }

    if(target_voltage_mv > BMS_BATTERY_FULL_CHARGE_VOLTAGE_MV) {
        return BMS_BATTERY_FULL_CHARGE_VOLTAGE_MV;
    }

    return target_voltage_mv;
}

static void Power_Control_Reset_Loop(void)
{
    Pi_Controller_Reset(&s_current_pi);
    Pi_Controller_Reset(&s_voltage_pi);
    s_soft_current_ma = 0U;
    s_buck_duty_x100 = POWER_START_DUTY_X100;
    s_boost_duty_x100 = 0U;
    s_async_boost_rectifier = 0U;
    s_output_ovp_count = 0U;
    s_output_ovp_blank_count = BMS_OUTPUT_OVP_STARTUP_BLANK_SAMPLES;
    s_output_ocp_count = 0U;
}

static void Power_Control_Clear_Stall_Recover(void)
{
    s_power_stall_recover_count = 0U;
}

static uint8_t Power_Control_Output_Stalled(const bms_power_sample_t *sample)
{
    if(sample == 0) {
        return 0U;
    }

    if(s_power.targetVoltageMv <= POWER_STALL_RECOVER_VOUT_GAP_MV) {
        return 0U;
    }

    if(s_power.dutyX100 < POWER_STALL_RECOVER_DUTY_X100) {
        return 0U;
    }

    if((uint32_t)sample->outputVoltageMv + POWER_STALL_RECOVER_VOUT_GAP_MV >=
       (uint32_t)s_power.targetVoltageMv) {
        return 0U;
    }

    if(sample->inputCurrentMa > POWER_STALL_RECOVER_IIN_MAX_MA) {
        return 0U;
    }

    return 1U;
}

static void Power_Control_Service_Stall_Recover(const bms_power_sample_t *sample)
{
    if(s_preconnect_active != 0U || Power_Control_Afe_Handover_Active() != 0U) {
        s_power_stall_recover_count = 0U;
        return;
    }

    if(Power_Control_Output_Stalled(sample) == 0U) {
        s_power_stall_recover_count = 0U;
        return;
    }

    if(s_power_stall_recover_count < POWER_STALL_RECOVER_CONFIRM_COUNT) {
        s_power_stall_recover_count++;
        return;
    }

    s_power_stall_recover_count = 0U;
    Power_Pwm_Outputs_Disable();
    Power_Control_Reset_Loop();
    s_power.dutyX100 = POWER_START_DUTY_X100;
    Power_Map_Control_To_Pwm(sample, s_power.dutyX100);
    Power_Control_Pwm_Apply();
    Power_Control_Pwm_Enable();
}

static uint16_t Power_Control_Output_Ovp_Margin_Mv(void)
{
    if(s_power.mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER) {
        return BMS_DIGITAL_POWER_OUTPUT_OVP_MARGIN_MV;
    }

    return BMS_DEFAULT_OUTPUT_OVP_MARGIN_MV;
}

static uint8_t Power_Control_Output_Ovp_Confirmed(uint16_t output_voltage_mv)
{
    uint32_t soft_limit_mv;
    uint32_t hard_limit_mv;

    if(s_output_ovp_blank_count > 0U) {
        s_output_ovp_blank_count--;
    }

    if(s_preconnect_active != 0U) {
        if(s_preconnect_ovp_limit_mv != 0U) {
            soft_limit_mv = (uint32_t)s_preconnect_ovp_limit_mv;
        } else {
            soft_limit_mv = (uint32_t)s_power.targetVoltageMv +
                            BMS_PRECONNECT_OUTPUT_OVP_MARGIN_MV;
        }
        hard_limit_mv = (uint32_t)s_power.targetVoltageMv +
                        BMS_PRECONNECT_OUTPUT_OVP_HARD_MARGIN_MV;
    } else {
        soft_limit_mv = (uint32_t)s_power.targetVoltageMv + Power_Control_Output_Ovp_Margin_Mv();
        hard_limit_mv = (uint32_t)s_power.targetVoltageMv + BMS_OUTPUT_OVP_HARD_MARGIN_MV;
    }

    if((uint32_t)output_voltage_mv > hard_limit_mv) {
        s_output_ovp_count = BMS_OUTPUT_OVP_CONFIRM_SAMPLES;
        return 1U;
    }

    if((uint32_t)output_voltage_mv <= soft_limit_mv) {
        s_output_ovp_count = 0U;
        return 0U;
    }

    if(s_output_ovp_blank_count > 0U) {
        s_output_ovp_count = 0U;
        return 0U;
    }

    if(s_output_ovp_count < BMS_OUTPUT_OVP_CONFIRM_SAMPLES) {
        s_output_ovp_count++;
    }

    return (s_output_ovp_count >= BMS_OUTPUT_OVP_CONFIRM_SAMPLES) ? 1U : 0U;
}

static uint8_t Power_Control_Output_Over_Hard_Limit(uint16_t output_voltage_mv)
{
    uint32_t hard_limit_mv;

    if(s_preconnect_active != 0U) {
        hard_limit_mv = (uint32_t)s_power.targetVoltageMv +
                        BMS_PRECONNECT_OUTPUT_OVP_HARD_MARGIN_MV;
    } else {
        hard_limit_mv = (uint32_t)s_power.targetVoltageMv + BMS_OUTPUT_OVP_HARD_MARGIN_MV;
    }

    return ((uint32_t)output_voltage_mv > hard_limit_mv) ? 1U : 0U;
}

static uint16_t Power_Control_Ramp_Current(uint16_t target_current_ma)
{
    if(s_preconnect_active != 0U) {
        s_soft_current_ma = target_current_ma;
        return s_soft_current_ma;
    }

    if(s_soft_current_ma < target_current_ma) {
        if((uint16_t)(target_current_ma - s_soft_current_ma) > POWER_SOFTSTART_STEP_MA) {
            s_soft_current_ma = (uint16_t)(s_soft_current_ma + POWER_SOFTSTART_STEP_MA);
        } else {
            s_soft_current_ma = target_current_ma;
        }
    } else if(s_soft_current_ma > target_current_ma) {
        if((uint16_t)(s_soft_current_ma - target_current_ma) > POWER_SOFTSTART_STEP_MA) {
            s_soft_current_ma = (uint16_t)(s_soft_current_ma - POWER_SOFTSTART_STEP_MA);
        } else {
            s_soft_current_ma = target_current_ma;
        }
    }

    return s_soft_current_ma;
}

static uint16_t Power_Control_Positive_Output_Current_Ma(const bms_power_sample_t *sample)
{
    if((sample != 0) && (sample->outputCurrentMa > 0)) {
        return (uint16_t)sample->outputCurrentMa;
    }

    return 0U;
}

static uint8_t Power_Control_Output_Ocp_Confirmed(uint16_t output_current_ma,
                                                  uint16_t current_ref_ma)
{
    uint32_t limit_ma;

    limit_ma = (uint32_t)current_ref_ma + (uint32_t)BMS_DEFAULT_OUTPUT_OCP_MARGIN_MA;
    if(output_current_ma <= limit_ma) {
        s_output_ocp_count = 0U;
        return 0U;
    }

    if((s_preconnect_active == 0U) && (Power_Control_Afe_Handover_Active() == 0U)) {
        return 1U;
    }

    if(s_output_ocp_count < POWER_TRANSIENT_OCP_CONFIRM_COUNT) {
        s_output_ocp_count++;
    }

    return (s_output_ocp_count >= POWER_TRANSIENT_OCP_CONFIRM_COUNT) ? 1U : 0U;
}

static uint16_t Power_Control_Current_Feedback_Ma(const bms_power_sample_t *sample)
{
    int16_t battery_current_ma;

    if((s_preconnect_active == 0U) &&
       (s_power.mode != (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER) &&
       (s_battery_current_feedback_valid != 0U)) {
        battery_current_ma = s_battery_current_feedback_ma;
        return (battery_current_ma > 0) ? (uint16_t)battery_current_ma : 0U;
    }

    return Power_Control_Positive_Output_Current_Ma(sample);
}
