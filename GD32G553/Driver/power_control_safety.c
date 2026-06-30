#include "power_control_internal.h"

uint16_t Clamp_U16(uint16_t value, uint16_t min_value, uint16_t max_value)
{
    if(value < min_value) {
        return min_value;
    }
    if(value > max_value) {
        return max_value;
    }
    return value;
}

uint16_t Power_Control_Clamp_Charge_Target_Mv(uint16_t target_voltage_mv, uint8_t mode)
{
    if(mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER) {
        return target_voltage_mv;
    }

    if(target_voltage_mv > BMS_BATTERY_FULL_CHARGE_VOLTAGE_MV) {
        return BMS_BATTERY_FULL_CHARGE_VOLTAGE_MV;
    }

    return target_voltage_mv;
}

void Power_Control_Reset_Loop(void)
{
    Pi_Controller_Reset(&g_power_control.currentPi);
    Pi_Controller_Reset(&g_power_control.voltagePi);
    g_power_control.softCurrentMa = 0U;
    g_power_control.buckDutyX100 = POWER_START_DUTY_X100;
    g_power_control.boostDutyX100 = 0U;
    g_power_control.asyncBoostRectifier = 0U;
    g_power_control.outputOvpCount = 0U;
    g_power_control.outputOvpBlankCount = BMS_OUTPUT_OVP_STARTUP_BLANK_SAMPLES;
    g_power_control.outputOcpCount = 0U;
}

void Power_Control_Clear_Stall_Recover(void)
{
    g_power_control.stallRecoverCount = 0U;
}

uint8_t Power_Control_Output_Stalled(const bms_power_sample_t *sample)
{
    if(sample == 0) {
        return 0U;
    }

    if(g_power_control.power.targetVoltageMv <= POWER_STALL_RECOVER_VOUT_GAP_MV) {
        return 0U;
    }

    if(g_power_control.power.dutyX100 < POWER_STALL_RECOVER_DUTY_X100) {
        return 0U;
    }

    if((uint32_t)sample->outputVoltageMv + POWER_STALL_RECOVER_VOUT_GAP_MV >=
       (uint32_t)g_power_control.power.targetVoltageMv) {
        return 0U;
    }

    if(sample->inputCurrentMa > POWER_STALL_RECOVER_IIN_MAX_MA) {
        return 0U;
    }

    return 1U;
}

void Power_Control_Service_Stall_Recover(const bms_power_sample_t *sample)
{
    if(g_power_control.preconnectActive != 0U || Power_Control_Afe_Handover_Active() != 0U) {
        g_power_control.stallRecoverCount = 0U;
        return;
    }

    if(Power_Control_Output_Stalled(sample) == 0U) {
        g_power_control.stallRecoverCount = 0U;
        return;
    }

    if(g_power_control.stallRecoverCount < POWER_STALL_RECOVER_CONFIRM_COUNT) {
        g_power_control.stallRecoverCount++;
        return;
    }

    g_power_control.stallRecoverCount = 0U;
    Power_Pwm_Outputs_Disable();
    Power_Control_Reset_Loop();
    g_power_control.power.dutyX100 = POWER_START_DUTY_X100;
    Power_Map_Control_To_Pwm(sample, g_power_control.power.dutyX100);
    Power_Control_Pwm_Apply();
    Power_Control_Pwm_Enable();
}

uint16_t Power_Control_Output_Ovp_Margin_Mv(void)
{
    if(g_power_control.power.mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER) {
        return BMS_DIGITAL_POWER_OUTPUT_OVP_MARGIN_MV;
    }

    return BMS_DEFAULT_OUTPUT_OVP_MARGIN_MV;
}

uint8_t Power_Control_Output_Ovp_Confirmed(uint16_t output_voltage_mv)
{
    uint32_t soft_limit_mv;
    uint32_t hard_limit_mv;

    if(g_power_control.outputOvpBlankCount > 0U) {
        g_power_control.outputOvpBlankCount--;
    }

    if(g_power_control.preconnectActive != 0U) {
        if(g_power_control.preconnectOvpLimitMv != 0U) {
            soft_limit_mv = (uint32_t)g_power_control.preconnectOvpLimitMv;
        } else {
            soft_limit_mv = (uint32_t)g_power_control.power.targetVoltageMv +
                            BMS_PRECONNECT_OUTPUT_OVP_MARGIN_MV;
        }
        hard_limit_mv = (uint32_t)g_power_control.power.targetVoltageMv +
                        BMS_PRECONNECT_OUTPUT_OVP_HARD_MARGIN_MV;
    } else {
        soft_limit_mv = (uint32_t)g_power_control.power.targetVoltageMv + Power_Control_Output_Ovp_Margin_Mv();
        hard_limit_mv = (uint32_t)g_power_control.power.targetVoltageMv + BMS_OUTPUT_OVP_HARD_MARGIN_MV;
    }

    if((uint32_t)output_voltage_mv > hard_limit_mv) {
        g_power_control.outputOvpCount = BMS_OUTPUT_OVP_CONFIRM_SAMPLES;
        return 1U;
    }

    if((uint32_t)output_voltage_mv <= soft_limit_mv) {
        g_power_control.outputOvpCount = 0U;
        return 0U;
    }

    if(g_power_control.outputOvpBlankCount > 0U) {
        g_power_control.outputOvpCount = 0U;
        return 0U;
    }

    if(g_power_control.outputOvpCount < BMS_OUTPUT_OVP_CONFIRM_SAMPLES) {
        g_power_control.outputOvpCount++;
    }

    return (g_power_control.outputOvpCount >= BMS_OUTPUT_OVP_CONFIRM_SAMPLES) ? 1U : 0U;
}

uint8_t Power_Control_Output_Over_Hard_Limit(uint16_t output_voltage_mv)
{
    uint32_t hard_limit_mv;

    if(g_power_control.preconnectActive != 0U) {
        hard_limit_mv = (uint32_t)g_power_control.power.targetVoltageMv +
                        BMS_PRECONNECT_OUTPUT_OVP_HARD_MARGIN_MV;
    } else {
        hard_limit_mv = (uint32_t)g_power_control.power.targetVoltageMv + BMS_OUTPUT_OVP_HARD_MARGIN_MV;
    }

    return ((uint32_t)output_voltage_mv > hard_limit_mv) ? 1U : 0U;
}

uint16_t Power_Control_Ramp_Current(uint16_t target_current_ma)
{
    if(g_power_control.preconnectActive != 0U) {
        g_power_control.softCurrentMa = target_current_ma;
        return g_power_control.softCurrentMa;
    }

    if(g_power_control.softCurrentMa < target_current_ma) {
        if((uint16_t)(target_current_ma - g_power_control.softCurrentMa) > POWER_SOFTSTART_STEP_MA) {
            g_power_control.softCurrentMa = (uint16_t)(g_power_control.softCurrentMa + POWER_SOFTSTART_STEP_MA);
        } else {
            g_power_control.softCurrentMa = target_current_ma;
        }
    } else if(g_power_control.softCurrentMa > target_current_ma) {
        if((uint16_t)(g_power_control.softCurrentMa - target_current_ma) > POWER_SOFTSTART_STEP_MA) {
            g_power_control.softCurrentMa = (uint16_t)(g_power_control.softCurrentMa - POWER_SOFTSTART_STEP_MA);
        } else {
            g_power_control.softCurrentMa = target_current_ma;
        }
    }

    return g_power_control.softCurrentMa;
}

uint16_t Power_Control_Positive_Output_Current_Ma(const bms_power_sample_t *sample)
{
    if((sample != 0) && (sample->outputCurrentMa > 0)) {
        return (uint16_t)sample->outputCurrentMa;
    }

    return 0U;
}

uint8_t Power_Control_Output_Ocp_Confirmed(uint16_t output_current_ma,
                                                  uint16_t current_ref_ma)
{
    uint32_t limit_ma;

    limit_ma = (uint32_t)current_ref_ma + (uint32_t)BMS_DEFAULT_OUTPUT_OCP_MARGIN_MA;
    if(output_current_ma <= limit_ma) {
        g_power_control.outputOcpCount = 0U;
        return 0U;
    }

    if((g_power_control.preconnectActive == 0U) && (Power_Control_Afe_Handover_Active() == 0U)) {
        return 1U;
    }

    if(g_power_control.outputOcpCount < POWER_TRANSIENT_OCP_CONFIRM_COUNT) {
        g_power_control.outputOcpCount++;
    }

    return (g_power_control.outputOcpCount >= POWER_TRANSIENT_OCP_CONFIRM_COUNT) ? 1U : 0U;
}

uint16_t Power_Control_Current_Feedback_Ma(const bms_power_sample_t *sample)
{
    int16_t battery_current_ma;

    if((g_power_control.preconnectActive == 0U) &&
       (g_power_control.power.mode != (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER) &&
       (g_power_control.batteryCurrentFeedbackValid != 0U)) {
        battery_current_ma = g_power_control.batteryCurrentFeedbackMa;
        return (battery_current_ma > 0) ? (uint16_t)battery_current_ma : 0U;
    }

    return Power_Control_Positive_Output_Current_Ma(sample);
}
