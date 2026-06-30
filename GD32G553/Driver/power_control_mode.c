#include "power_control_internal.h"

uint8_t Power_Voltage_Loop_Should_Run(const bms_power_sample_t *sample,
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

    if(g_power_control.power.mode == (uint8_t)BMS_CHARGE_MODE_CV) {
        return 1U;
    }

    if(g_power_control.power.mode == (uint8_t)BMS_CHARGE_MODE_CC ||
       g_power_control.power.mode == (uint8_t)BMS_CHARGE_MODE_TRICKLE) {
        return 0U;
    }

    if(Power_Control_Light_Load_Near_Target(sample, current_ref_ma, measured_current_ma) != 0U) {
        return 1U;
    }

    if((uint32_t)sample->outputVoltageMv + POWER_VOLTAGE_CV_MARGIN_MV >= g_power_control.power.targetVoltageMv) {
        return 1U;
    }

    return 0U;
}

uint8_t Power_Control_Async_Boost_Should_Run(uint16_t current_ref_ma,
                                                    uint16_t measured_current_ma)
{
    if(g_power_control.power.powerStageMode != (uint8_t)POWER_STAGE_MODE_BOOST) {
        return 0U;
    }

    if(g_power_control.power.mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER) {
        return 0U;
    }

    if(g_power_control.preconnectActive != 0U) {
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
    if(g_power_control.power.mode == (uint8_t)BMS_CHARGE_MODE_CC) {
        (void)current_ref_ma;
        (void)measured_current_ma;
        return 1U;
    }

    if(current_ref_ma <= POWER_ASYNC_BOOST_CURRENT_MAX_MA) {
        return 1U;
    }

    if(g_power_control.asyncBoostRectifier != 0U) {
        return (measured_current_ma <= POWER_ASYNC_BOOST_EXIT_CURRENT_MA) ? 1U : 0U;
    }

    return (measured_current_ma <= POWER_ASYNC_BOOST_CURRENT_MAX_MA) ? 1U : 0U;
#else
    (void)current_ref_ma;
    (void)measured_current_ma;
    return 0U;
#endif
}

uint16_t Power_Ocp_Limit_From_Ref(uint16_t current_ref_ma)
{
    uint32_t limit;

    limit = (uint32_t)current_ref_ma + (uint32_t)BMS_DEFAULT_OUTPUT_OCP_MARGIN_MA;
    if(limit > 65535U) {
        limit = 65535U;
    }

    return (uint16_t)limit;
}

void Power_Control_Record_Trip(uint8_t reason,
                                      uint32_t faults,
                                      const bms_power_sample_t *sample,
                                      uint16_t current_ref_ma)
{
    g_power_control.power.tripReason = reason;
    g_power_control.power.tripFaults = faults;
    g_power_control.power.tripCurrentRefMa = current_ref_ma;
    g_power_control.power.tripOcpLimitMa = Power_Ocp_Limit_From_Ref(current_ref_ma);
    g_power_control.power.tripDutyX100 = g_power_control.power.dutyX100;

    if(sample != 0) {
        g_power_control.power.tripIoutMa = sample->outputCurrentMa;
        g_power_control.power.tripVoutMv = sample->outputVoltageMv;
        g_power_control.power.tripVinMv = sample->inputVoltageMv;
        g_power_control.power.tripFaultOcActive = sample->faultOcActive;
    } else {
        g_power_control.power.tripIoutMa = 0;
        g_power_control.power.tripVoutMv = 0U;
        g_power_control.power.tripVinMv = 0U;
        g_power_control.power.tripFaultOcActive = 0U;
    }
}

void Power_Control_Pwm_Output_Context(power_pwm_output_context_t *context)
{
    if(context == 0) {
        return;
    }

    context->faultLockout = g_power_control.faultLockout;
    context->faultStatus = g_power_control.faultStatus;
    context->asyncBoostRectifier = g_power_control.asyncBoostRectifier;
    context->boostStageActive =
        (g_power_control.power.powerStageMode == (uint8_t)POWER_STAGE_MODE_BOOST) ? 1U : 0U;
    context->boostLowDutyX100 = g_power_control.boostDutyX100;
    context->preconnectActive = g_power_control.preconnectActive;
}

void Power_Control_Pwm_Apply(void)
{
    Power_Pwm_Apply(g_power_control.buckDutyX100, g_power_control.boostDutyX100, g_power_control.faultLockout);
}

void Power_Control_Pwm_Enable(void)
{
    power_pwm_output_context_t context;

    Power_Control_Pwm_Output_Context(&context);
    Power_Pwm_Outputs_Enable(&context);
}

void Power_Map_Control_To_Pwm(const bms_power_sample_t *sample, uint16_t control_x100)
{
    uint16_t buck_duty;
    uint16_t boost_duty;
    uint16_t input_voltage_mv;
    uint16_t output_voltage_mv;
    uint32_t region_target_mv;

    /*
     * 鍥涘紑鍏抽潪鍙嶇浉 Buck-Boost 鐨勪綆鍔熺巼璋冭瘯鏄犲皠锛?
     * - Buck 鍖猴細杈撳叆渚?PWM锛岃緭鍑轰晶楂樿竟甯搁€氾紱
     * - Boost 鍖猴細杈撳叆渚ч珮杈瑰父閫氾紝杈撳嚭渚т綆杈?PWM锛?
     * - 杩囨浮鍖猴細涓や晶鍚屾椂鎸夊悓涓€鎺у埗閲忓姩浣溿€?
     *
     * control_x100 澧炲ぇ鏃讹紝涓夌妯″紡涓嬩紶閫掑埌杈撳嚭渚х殑鑳介噺閮藉澶с€?
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
        g_power_control.power.powerStageMode = (uint8_t)POWER_STAGE_MODE_BOOST;
    } else if(input_voltage_mv == 0U) {
        buck_duty = control_x100;
        boost_duty = 0U;
        g_power_control.power.powerStageMode = (uint8_t)POWER_STAGE_MODE_BUCK;
    } else if((uint32_t)input_voltage_mv > region_target_mv + POWER_REGION_MARGIN_MV) {
        buck_duty = control_x100;
        boost_duty = 0U;
        g_power_control.power.powerStageMode = (uint8_t)POWER_STAGE_MODE_BUCK;
    } else if((uint32_t)input_voltage_mv + POWER_REGION_MARGIN_MV < region_target_mv) {
        buck_duty = POWER_DUTY_MAX_X100;
        boost_duty = control_x100;
        g_power_control.power.powerStageMode = (uint8_t)POWER_STAGE_MODE_BOOST;
    } else {
        buck_duty = control_x100;
        boost_duty = control_x100;
        g_power_control.power.powerStageMode = (uint8_t)POWER_STAGE_MODE_BUCK_BOOST;
    }

    g_power_control.buckDutyX100 = buck_duty;
    g_power_control.boostDutyX100 = boost_duty;
}
