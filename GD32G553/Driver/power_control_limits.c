#include "power_control_internal.h"

uint16_t Power_Control_Light_Load_Current_Threshold(uint16_t current_ref_ma)
{
    uint16_t threshold;

    threshold = (uint16_t)(current_ref_ma / POWER_LIGHT_LOAD_CURRENT_DIV);
    if(threshold < POWER_LIGHT_LOAD_CURRENT_MIN_MA) {
        threshold = POWER_LIGHT_LOAD_CURRENT_MIN_MA;
    }
    if(threshold > POWER_LIGHT_LOAD_CURRENT_MAX_MA) {
        threshold = POWER_LIGHT_LOAD_CURRENT_MAX_MA;
    }

    return threshold;
}

uint16_t Power_Control_Light_Load_Cv_Margin_Mv(void)
{
    if(g_power_control.preconnectActive != 0U) {
        return POWER_PRECONNECT_LIGHT_LOAD_MARGIN_MV;
    }

    return POWER_LIGHT_LOAD_CV_MARGIN_MV;
}

uint8_t Power_Control_Light_Load_Near_Target(const bms_power_sample_t *sample,
                                                    uint16_t current_ref_ma,
                                                    uint16_t measured_current_ma)
{
    if(sample == 0) {
        return 0U;
    }

    if(g_power_control.power.mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER) {
        return 0U;
    }

    if(g_power_control.power.mode == (uint8_t)BMS_CHARGE_MODE_CC ||
       g_power_control.power.mode == (uint8_t)BMS_CHARGE_MODE_TRICKLE) {
        return (current_ref_ma <= POWER_LIGHT_LOAD_CURRENT_MAX_MA) ? 1U : 0U;
    }

    if(measured_current_ma >
       Power_Control_Light_Load_Current_Threshold(current_ref_ma)) {
        return 0U;
    }

    if((uint32_t)sample->outputVoltageMv + Power_Control_Light_Load_Cv_Margin_Mv() <
       (uint32_t)g_power_control.power.targetVoltageMv) {
        return 0U;
    }

    return 1U;
}

uint16_t Power_Control_Light_Load_Duty_Max(const bms_power_sample_t *sample)
{
    uint32_t ideal_boost_duty;
    uint32_t target_for_limit_mv;

    if(sample == 0 ||
       sample->inputVoltageMv == 0U ||
       g_power_control.power.targetVoltageMv == 0U) {
        return POWER_LOOP_DUTY_MAX_X100;
    }

    target_for_limit_mv = (uint32_t)sample->outputVoltageMv + POWER_LIGHT_LOAD_VOUT_ADVANCE_MV;
    if(target_for_limit_mv > (uint32_t)g_power_control.power.targetVoltageMv) {
        target_for_limit_mv = (uint32_t)g_power_control.power.targetVoltageMv;
    }
    if(target_for_limit_mv <= (uint32_t)sample->inputVoltageMv) {
        return POWER_LOOP_DUTY_MAX_X100;
    }

    ideal_boost_duty =
        ((target_for_limit_mv - (uint32_t)sample->inputVoltageMv) * 10000UL) /
        target_for_limit_mv;
    ideal_boost_duty += POWER_LIGHT_LOAD_BOOST_HEADROOM_X100;
    if(ideal_boost_duty > POWER_LOOP_DUTY_MAX_X100) {
        ideal_boost_duty = POWER_LOOP_DUTY_MAX_X100;
    }

    return Clamp_U16((uint16_t)ideal_boost_duty,
                     POWER_DUTY_MIN_X100,
                     POWER_LOOP_DUTY_MAX_X100);
}

uint16_t Power_Control_Preconnect_Duty_Max(const bms_power_sample_t *sample)
{
    uint32_t ideal_boost_duty;

    if(sample == 0 ||
       sample->inputVoltageMv == 0U ||
       g_power_control.power.targetVoltageMv == 0U) {
        return POWER_PRECONNECT_BOOST_DUTY_MAX_X100;
    }

    if((uint32_t)sample->inputVoltageMv >= (uint32_t)g_power_control.power.targetVoltageMv) {
        /*
         * The shared duty command also drives the Buck/Buck-Boost startup path.
         * When Vin is already above the preconnect target, clamping this value
         * to the minimum duty leaves both bridges at 2% and Vout never rises.
         * Let the voltage loop ramp the Buck/Buck-Boost duty like digital-power
         * mode; the normal OVP/current checks still limit the output.
         */
        return POWER_PRECONNECT_BOOST_DUTY_MAX_X100;
    }

    ideal_boost_duty =
        (((uint32_t)g_power_control.power.targetVoltageMv - (uint32_t)sample->inputVoltageMv) * 10000UL) /
        (uint32_t)g_power_control.power.targetVoltageMv;
    ideal_boost_duty += POWER_PRECONNECT_BOOST_HEADROOM_X100;
    if(ideal_boost_duty > POWER_PRECONNECT_BOOST_DUTY_MAX_X100) {
        ideal_boost_duty = POWER_PRECONNECT_BOOST_DUTY_MAX_X100;
    }

    return Clamp_U16((uint16_t)ideal_boost_duty,
                     POWER_DUTY_MIN_X100,
                     POWER_PRECONNECT_BOOST_DUTY_MAX_X100);
}

uint16_t Power_Control_Preconnect_Duty_Min(const bms_power_sample_t *sample)
{
    uint32_t ideal_boost_duty;

    if(sample == 0 ||
       sample->inputVoltageMv == 0U ||
       g_power_control.power.targetVoltageMv == 0U) {
        return POWER_DUTY_MIN_X100;
    }

    if((uint32_t)sample->inputVoltageMv >= (uint32_t)g_power_control.power.targetVoltageMv) {
        return POWER_DUTY_MIN_X100;
    }

    if((uint32_t)sample->outputVoltageMv >=
       (uint32_t)g_power_control.power.targetVoltageMv + POWER_PRECONNECT_COAST_MARGIN_MV) {
        return POWER_DUTY_MIN_X100;
    }

    ideal_boost_duty =
        (((uint32_t)g_power_control.power.targetVoltageMv - (uint32_t)sample->inputVoltageMv) * 10000UL) /
        (uint32_t)g_power_control.power.targetVoltageMv;

    return Clamp_U16((uint16_t)ideal_boost_duty,
                     POWER_DUTY_MIN_X100,
                     POWER_PRECONNECT_BOOST_DUTY_MAX_X100);
}

uint8_t Power_Control_Preconnect_Coast_Should_Run(const bms_power_sample_t *sample)
{
    if(sample == 0) {
        return 0U;
    }

    if(g_power_control.preconnectActive == 0U) {
        return 0U;
    }

    /*
     * Only enter the preconnect hold path on real overshoot. The hold path
     * keeps a minimum boost pulse alive so Vout does not collapse before
     * CHG/DSG can hand over.
     */
    if((uint32_t)sample->outputVoltageMv <=
       (uint32_t)g_power_control.power.targetVoltageMv + POWER_PRECONNECT_COAST_MARGIN_MV) {
        return 0U;
    }

    return 1U;
}

void Power_Control_Preconnect_Coast(void)
{
    g_power_control.power.dutyX100 = POWER_DUTY_MIN_X100;
    g_power_control.buckDutyX100 = POWER_DUTY_MAX_X100;
    g_power_control.boostDutyX100 = POWER_DUTY_MIN_X100;
    g_power_control.power.powerStageMode = (uint8_t)POWER_STAGE_MODE_BOOST;
    g_power_control.asyncBoostRectifier = 1U;
    Pi_Controller_Decay(&g_power_control.currentPi, 1U, 2U);
    Pi_Controller_Decay(&g_power_control.voltagePi, 1U, 2U);
}

uint8_t Power_Control_Afe_Handover_Active(void)
{
    return (g_power_control.afeHandoverGuardCount != 0U) ? 1U : 0U;
}

uint16_t Power_Control_Handover_Boost_Duty(const bms_power_sample_t *sample,
                                                  uint16_t previous_boost_duty_x100)
{
    uint32_t ideal_boost_duty;
    uint16_t target_mv;
    uint16_t vin_mv;
    uint16_t duty;

    duty = previous_boost_duty_x100;
    if(duty < POWER_START_DUTY_X100) {
        duty = POWER_START_DUTY_X100;
    }

    if(sample == 0 || sample->inputVoltageMv == 0U || g_power_control.power.targetVoltageMv == 0U) {
        return Clamp_U16(duty, POWER_DUTY_MIN_X100, BMS_POWER_AFE_HANDOVER_BOOST_DUTY_MAX_X100);
    }

    vin_mv = sample->inputVoltageMv;
    target_mv = g_power_control.power.targetVoltageMv;
    if(target_mv > BMS_BATTERY_FULL_CHARGE_VOLTAGE_MV) {
        target_mv = BMS_BATTERY_FULL_CHARGE_VOLTAGE_MV;
    }
    if(vin_mv < target_mv) {
        ideal_boost_duty =
            (((uint32_t)target_mv - (uint32_t)vin_mv) * 10000UL) /
            (uint32_t)target_mv;
        ideal_boost_duty += BMS_POWER_BATTERY_BOOST_DUTY_HEADROOM_X100;
        if(ideal_boost_duty > (uint32_t)BMS_POWER_AFE_HANDOVER_BOOST_DUTY_MAX_X100) {
            ideal_boost_duty = (uint32_t)BMS_POWER_AFE_HANDOVER_BOOST_DUTY_MAX_X100;
        }
        if((uint16_t)ideal_boost_duty > duty) {
            duty = (uint16_t)ideal_boost_duty;
        }
    }

    return Clamp_U16(duty, POWER_DUTY_MIN_X100, BMS_POWER_AFE_HANDOVER_BOOST_DUTY_MAX_X100);
}

void Power_Control_Afe_Handover_Start_Output(const bms_power_sample_t *sample,
                                                    uint16_t previous_boost_duty_x100)
{
    g_power_control.power.dutyX100 = Power_Control_Handover_Boost_Duty(sample, previous_boost_duty_x100);
    g_power_control.buckDutyX100 = POWER_DUTY_MAX_X100;
    g_power_control.boostDutyX100 = g_power_control.power.dutyX100;
    g_power_control.power.powerStageMode = (uint8_t)POWER_STAGE_MODE_BOOST;
    g_power_control.asyncBoostRectifier = 1U;
}

void Power_Control_Afe_Handover_Decay(void)
{
    if(g_power_control.afeHandoverGuardCount > 0U) {
        g_power_control.afeHandoverGuardCount--;
    }
}

int32_t Power_Control_Limit_Afe_Handover_Duty(const bms_power_sample_t *sample,
                                                     int32_t duty)
{
    uint16_t duty_floor;

    if(Power_Control_Afe_Handover_Active() == 0U) {
        return duty;
    }

    duty_floor = Power_Control_Handover_Boost_Duty(sample, POWER_START_DUTY_X100);
    if(duty < (int32_t)duty_floor) {
        duty = (int32_t)duty_floor;
        Pi_Controller_Decay(&g_power_control.currentPi, 1U, 2U);
        Pi_Controller_Decay(&g_power_control.voltagePi, 1U, 2U);
    }

    if(duty > (int32_t)BMS_POWER_AFE_HANDOVER_BOOST_DUTY_MAX_X100) {
        duty = (int32_t)BMS_POWER_AFE_HANDOVER_BOOST_DUTY_MAX_X100;
        Pi_Controller_Decay(&g_power_control.currentPi, 1U, 2U);
        Pi_Controller_Decay(&g_power_control.voltagePi, 1U, 2U);
    }

    return duty;
}

int32_t Power_Control_Limit_Preconnect_Duty(const bms_power_sample_t *sample,
                                                   int32_t duty)
{
    uint16_t duty_limit;
    uint16_t duty_floor;

    if(g_power_control.preconnectActive == 0U) {
        return duty;
    }

    duty_floor = Power_Control_Preconnect_Duty_Min(sample);
    if(duty < (int32_t)duty_floor) {
        duty = (int32_t)duty_floor;
        Pi_Controller_Decay(&g_power_control.currentPi, 3U, 4U);
        Pi_Controller_Decay(&g_power_control.voltagePi, 3U, 4U);
    }

    duty_limit = Power_Control_Preconnect_Duty_Max(sample);
    if(duty > (int32_t)duty_limit) {
        duty = (int32_t)duty_limit;
        Pi_Controller_Decay(&g_power_control.currentPi, 3U, 4U);
        Pi_Controller_Decay(&g_power_control.voltagePi, 3U, 4U);
    }

    return duty;
}

uint32_t Power_Control_Cc_Duty_Target_Mv(const bms_power_sample_t *sample)
{
    uint32_t target_mv;
    uint32_t candidate_mv;

    target_mv = (uint32_t)g_power_control.power.targetVoltageMv;
    if(g_power_control.power.mode != (uint8_t)BMS_CHARGE_MODE_CC &&
       g_power_control.power.mode != (uint8_t)BMS_CHARGE_MODE_TRICKLE) {
        return target_mv;
    }

    if(g_power_control.power.mode == (uint8_t)BMS_CHARGE_MODE_CC) {
        return target_mv;
    }

    candidate_mv = 0UL;
    if(g_power_control.batteryVoltageFeedbackValid != 0U &&
       g_power_control.batteryVoltageFeedbackMv != 0U) {
        candidate_mv = (uint32_t)g_power_control.batteryVoltageFeedbackMv +
                       (uint32_t)POWER_CC_DUTY_PACK_MARGIN_MV;
    }
    if(sample != 0 && sample->outputVoltageMv != 0U) {
        uint32_t vout_target_mv =
            (uint32_t)sample->outputVoltageMv +
            (uint32_t)POWER_CC_DUTY_VOUT_ADVANCE_MV;
        if(vout_target_mv > candidate_mv) {
            candidate_mv = vout_target_mv;
        }
    }
    if(candidate_mv == 0UL || candidate_mv > target_mv) {
        candidate_mv = target_mv;
    }

    return candidate_mv;
}

uint32_t Power_Control_Battery_Boost_Headroom_X100(void)
{
    if(g_power_control.power.mode == (uint8_t)BMS_CHARGE_MODE_CC &&
       g_power_control.asyncBoostRectifier != 0U) {
        return (uint32_t)POWER_CC_ASYNC_BOOST_DUTY_HEADROOM_X100;
    }

    if((g_power_control.power.mode == (uint8_t)BMS_CHARGE_MODE_CC ||
        g_power_control.power.mode == (uint8_t)BMS_CHARGE_MODE_TRICKLE) &&
       g_power_control.power.targetCurrentMa <= POWER_LIGHT_LOAD_CURRENT_MAX_MA) {
        return (uint32_t)POWER_CC_LIGHT_LOAD_HEADROOM_X100;
    }

    return (uint32_t)BMS_POWER_BATTERY_BOOST_DUTY_HEADROOM_X100;
}

uint16_t Power_Control_Battery_Boost_Duty_Max(const bms_power_sample_t *sample)
{
#if BMS_POWER_BATTERY_BOOST_DUTY_LIMIT_ENABLE
    uint32_t ideal_boost_duty;
    uint32_t vin_mv;
    uint32_t target_mv;

    if(sample == 0 ||
       sample->inputVoltageMv == 0U ||
       g_power_control.power.targetVoltageMv == 0U ||
       g_power_control.preconnectActive != 0U ||
       g_power_control.power.mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER) {
        return POWER_LOOP_DUTY_MAX_X100;
    }

    vin_mv = (uint32_t)sample->inputVoltageMv;
    target_mv = Power_Control_Cc_Duty_Target_Mv(sample);
    if(target_mv > (uint32_t)BMS_BATTERY_FULL_CHARGE_VOLTAGE_MV) {
        target_mv = (uint32_t)BMS_BATTERY_FULL_CHARGE_VOLTAGE_MV;
    }
    if(vin_mv + POWER_REGION_MARGIN_MV >= target_mv) {
        return POWER_LOOP_DUTY_MAX_X100;
    }

    ideal_boost_duty = ((target_mv - vin_mv) * 10000UL) / target_mv;
    ideal_boost_duty += Power_Control_Battery_Boost_Headroom_X100();
    if(ideal_boost_duty > (uint32_t)BMS_POWER_BATTERY_BOOST_DUTY_MAX_X100) {
        ideal_boost_duty = (uint32_t)BMS_POWER_BATTERY_BOOST_DUTY_MAX_X100;
    }
    if(ideal_boost_duty > (uint32_t)POWER_LOOP_DUTY_MAX_X100) {
        ideal_boost_duty = (uint32_t)POWER_LOOP_DUTY_MAX_X100;
    }

    return Clamp_U16((uint16_t)ideal_boost_duty,
                     POWER_DUTY_MIN_X100,
                     POWER_LOOP_DUTY_MAX_X100);
#else
    (void)sample;
    return POWER_LOOP_DUTY_MAX_X100;
#endif
}

int32_t Power_Control_Limit_Battery_Boost_Duty(const bms_power_sample_t *sample,
                                                      int32_t duty)
{
#if BMS_POWER_BATTERY_BOOST_DUTY_LIMIT_ENABLE
    uint16_t duty_limit;

    duty_limit = Power_Control_Battery_Boost_Duty_Max(sample);
    if(duty > (int32_t)duty_limit) {
        duty = (int32_t)duty_limit;
        Pi_Controller_Decay(&g_power_control.currentPi, 3U, 4U);
        Pi_Controller_Decay(&g_power_control.voltagePi, 3U, 4U);
    }
#else
    (void)sample;
#endif

    return duty;
}

uint8_t Power_Control_Input_Guard_Active(const bms_power_sample_t *sample)
{
    if(sample == 0) {
        return 0U;
    }

    if(g_power_control.power.targetVoltageMv < POWER_INPUT_GUARD_TARGET_MIN_MV) {
        return 0U;
    }

    if(sample->inputVoltageMv != 0U &&
       sample->inputVoltageMv < POWER_INPUT_GUARD_VIN_MIN_MV) {
        return 1U;
    }

    if(sample->inputCurrentMa > POWER_INPUT_GUARD_IIN_MAX_MA) {
        return 1U;
    }

    return 0U;
}

void Power_Control_Reduce_Duty_For_Input_Guard(const bms_power_sample_t *sample)
{
    uint16_t measured_current_ma;

    if(g_power_control.power.dutyX100 > (uint16_t)(POWER_DUTY_MIN_X100 + POWER_INPUT_GUARD_DUTY_STEP_X100)) {
        g_power_control.power.dutyX100 =
            (uint16_t)(g_power_control.power.dutyX100 - POWER_INPUT_GUARD_DUTY_STEP_X100);
    } else {
        g_power_control.power.dutyX100 = POWER_DUTY_MIN_X100;
    }

    Pi_Controller_Decay(&g_power_control.currentPi, 3U, 4U);
    Pi_Controller_Decay(&g_power_control.voltagePi, 3U, 4U);
    Power_Map_Control_To_Pwm(sample, g_power_control.power.dutyX100);
    measured_current_ma = Power_Control_Current_Feedback_Ma(sample);
    g_power_control.asyncBoostRectifier =
        Power_Control_Async_Boost_Should_Run(g_power_control.softCurrentMa,
                                             measured_current_ma);
}

int32_t Power_Control_Limit_Light_Load_Step(const bms_power_sample_t *sample,
                                                   uint16_t current_ref_ma,
                                                   uint16_t measured_current_ma,
                                                   int32_t step)
{
    uint16_t duty_limit;

    if(Power_Control_Light_Load_Near_Target(sample, current_ref_ma, measured_current_ma) == 0U) {
        return step;
    }

    if(step > (int32_t)POWER_LIGHT_LOAD_STEP_MAX_X100) {
        step = (int32_t)POWER_LIGHT_LOAD_STEP_MAX_X100;
    }

    if(step > 0) {
        duty_limit = Power_Control_Light_Load_Duty_Max(sample);
        if(g_power_control.power.dutyX100 >= duty_limit) {
            step = 0;
        } else if((uint32_t)g_power_control.power.dutyX100 + (uint32_t)step > (uint32_t)duty_limit) {
            step = (int32_t)duty_limit - (int32_t)g_power_control.power.dutyX100;
        }
    }

    return step;
}
