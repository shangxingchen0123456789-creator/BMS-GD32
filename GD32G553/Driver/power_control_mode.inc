static uint8_t Power_Voltage_Loop_Should_Run(const bms_power_sample_t *sample,
                                             uint16_t current_ref_ma,
                                             uint16_t measured_current_ma)
{
    if(sample == 0) {
        return 0U;
    }

    if(Power_Control_Afe_Handover_Active() != 0U) {
        return 1U;
    }

    if(measured_current_ma > current_ref_ma) {
        return 0U;
    }

    if(s_power.mode == (uint8_t)BMS_CHARGE_MODE_CV) {
        return 1U;
    }

    if(s_power.mode == (uint8_t)BMS_CHARGE_MODE_CC ||
       s_power.mode == (uint8_t)BMS_CHARGE_MODE_TRICKLE) {
        return 0U;
    }

    if(Power_Control_Light_Load_Near_Target(sample, current_ref_ma, measured_current_ma) != 0U) {
        return 1U;
    }

    if((uint32_t)sample->outputVoltageMv + POWER_VOLTAGE_CV_MARGIN_MV >= s_power.targetVoltageMv) {
        return 1U;
    }

    return 0U;
}

static uint8_t Power_Control_Async_Boost_Should_Run(uint16_t current_ref_ma,
                                                    uint16_t measured_current_ma)
{
    if(s_power.powerStageMode != (uint8_t)POWER_STAGE_MODE_BOOST) {
        return 0U;
    }

    if(s_power.mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER) {
        return 0U;
    }

    if(s_preconnect_active != 0U) {
#if BMS_POWER_PRECONNECT_ASYNC_BOOST_RECTIFIER
        return 1U;
#else
        return 0U;
#endif
    }

    if(Power_Control_Afe_Handover_Active() != 0U) {
        return 1U;
    }

#if BMS_POWER_LIGHT_LOAD_ASYNC_BOOST_RECTIFIER
    if(s_power.mode == (uint8_t)BMS_CHARGE_MODE_CC) {
        (void)current_ref_ma;
        (void)measured_current_ma;
        return 1U;
    }

    if(current_ref_ma <= POWER_ASYNC_BOOST_CURRENT_MAX_MA) {
        return 1U;
    }

    if(s_async_boost_rectifier != 0U) {
        return (measured_current_ma <= POWER_ASYNC_BOOST_EXIT_CURRENT_MA) ? 1U : 0U;
    }

    return (measured_current_ma <= POWER_ASYNC_BOOST_CURRENT_MAX_MA) ? 1U : 0U;
#else
    (void)current_ref_ma;
    (void)measured_current_ma;
    return 0U;
#endif
}

static uint16_t Power_Ocp_Limit_From_Ref(uint16_t current_ref_ma)
{
    uint32_t limit;

    limit = (uint32_t)current_ref_ma + (uint32_t)BMS_DEFAULT_OUTPUT_OCP_MARGIN_MA;
    if(limit > 65535U) {
        limit = 65535U;
    }

    return (uint16_t)limit;
}

static void Power_Control_Record_Trip(uint8_t reason,
                                      uint32_t faults,
                                      const bms_power_sample_t *sample,
                                      uint16_t current_ref_ma)
{
    s_power.tripReason = reason;
    s_power.tripFaults = faults;
    s_power.tripCurrentRefMa = current_ref_ma;
    s_power.tripOcpLimitMa = Power_Ocp_Limit_From_Ref(current_ref_ma);
    s_power.tripDutyX100 = s_power.dutyX100;

    if(sample != 0) {
        s_power.tripIoutMa = sample->outputCurrentMa;
        s_power.tripVoutMv = sample->outputVoltageMv;
        s_power.tripVinMv = sample->inputVoltageMv;
        s_power.tripFaultOcActive = sample->faultOcActive;
    } else {
        s_power.tripIoutMa = 0;
        s_power.tripVoutMv = 0U;
        s_power.tripVinMv = 0U;
        s_power.tripFaultOcActive = 0U;
    }
}

static void Power_Control_Pwm_Output_Context(power_pwm_output_context_t *context)
{
    if(context == 0) {
        return;
    }

    context->faultLockout = s_fault_lockout;
    context->faultStatus = s_fault_status;
    context->asyncBoostRectifier = s_async_boost_rectifier;
    context->boostStageActive =
        (s_power.powerStageMode == (uint8_t)POWER_STAGE_MODE_BOOST) ? 1U : 0U;
    context->boostLowDutyX100 = s_boost_duty_x100;
    context->preconnectActive = s_preconnect_active;
}

static void Power_Control_Pwm_Apply(void)
{
    Power_Pwm_Apply(s_buck_duty_x100, s_boost_duty_x100, s_fault_lockout);
}

static void Power_Control_Pwm_Enable(void)
{
    power_pwm_output_context_t context;

    Power_Control_Pwm_Output_Context(&context);
    Power_Pwm_Outputs_Enable(&context);
}

static void Power_Map_Control_To_Pwm(const bms_power_sample_t *sample, uint16_t control_x100)
{
    uint16_t buck_duty;
    uint16_t boost_duty;
    uint16_t input_voltage_mv;
    uint16_t output_voltage_mv;
    uint32_t region_target_mv;

    /*
     * 四开关非反相 Buck-Boost 的低功率调试映射：
     * - Buck 区：输入侧 PWM，输出侧高边常通；
     * - Boost 区：输入侧高边常通，输出侧低边 PWM；
     * - 过渡区：两侧同时按同一控制量动作。
     *
     * control_x100 增大时，三种模式下传递到输出侧的能量都增大。
     */
    control_x100 = Clamp_U16(control_x100, POWER_DUTY_MIN_X100, POWER_LOOP_DUTY_MAX_X100);

    input_voltage_mv = (sample != 0) ? sample->inputVoltageMv : 0U;
    output_voltage_mv = (sample != 0) ? sample->outputVoltageMv : 0U;
    region_target_mv = Power_Control_Cc_Duty_Target_Mv(sample);

    /*
     * During standalone power bring-up, Vin may be sanitized to 0 for a sample.
     * Do not force Buck in that case; if Vout is still below target, keep the
     * boost side active so a step-up request can actually start.
     */
    if(input_voltage_mv == 0U &&
       ((uint32_t)output_voltage_mv + POWER_REGION_MARGIN_MV < region_target_mv)) {
        buck_duty = POWER_DUTY_MAX_X100;
        boost_duty = control_x100;
        s_power.powerStageMode = (uint8_t)POWER_STAGE_MODE_BOOST;
    } else if(input_voltage_mv == 0U) {
        buck_duty = control_x100;
        boost_duty = 0U;
        s_power.powerStageMode = (uint8_t)POWER_STAGE_MODE_BUCK;
    } else if((uint32_t)input_voltage_mv > region_target_mv + POWER_REGION_MARGIN_MV) {
        buck_duty = control_x100;
        boost_duty = 0U;
        s_power.powerStageMode = (uint8_t)POWER_STAGE_MODE_BUCK;
    } else if((uint32_t)input_voltage_mv + POWER_REGION_MARGIN_MV < region_target_mv) {
        buck_duty = POWER_DUTY_MAX_X100;
        boost_duty = control_x100;
        s_power.powerStageMode = (uint8_t)POWER_STAGE_MODE_BOOST;
    } else {
        buck_duty = control_x100;
        boost_duty = control_x100;
        s_power.powerStageMode = (uint8_t)POWER_STAGE_MODE_BUCK_BOOST;
    }

    s_buck_duty_x100 = buck_duty;
    s_boost_duty_x100 = boost_duty;
}
