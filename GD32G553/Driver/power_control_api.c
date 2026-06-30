#include "power_control_internal.h"

void Power_Control_Init(void)
{
    pi_controller_config_t current_config;
    pi_controller_config_t voltage_config;

    memset(&g_power_control.loop.power, 0, sizeof(g_power_control.loop.power));
    g_power_control.fault.lockout = 0U;
    g_power_control.fault.status = 0U;
    Power_Control_Clear_Stall_Recover();

    current_config.kpDiv = (int32_t)POWER_CURRENT_KP_DIV;
    current_config.kiDiv = (int32_t)POWER_CURRENT_KI_DIV;
    current_config.integralLimit = POWER_INTEGRAL_LIMIT;
    current_config.stepLimit = (int32_t)POWER_LOOP_STEP_MAX_X100;
    voltage_config.kpDiv = (int32_t)POWER_VOLTAGE_KP_DIV;
    voltage_config.kiDiv = (int32_t)POWER_VOLTAGE_KI_DIV;
    voltage_config.integralLimit = POWER_INTEGRAL_LIMIT;
    voltage_config.stepLimit = (int32_t)POWER_LOOP_STEP_MAX_X100;
    Pi_Controller_Init(&g_power_control.loop.currentPi, &current_config);
    Pi_Controller_Init(&g_power_control.loop.voltagePi, &voltage_config);

    Power_Control_Reset_Loop();

    Power_Pwm_Init();
}

void Power_Control_Stop(void)
{
    /*
     * 缁熶竴鍋滄鍏ュ彛銆?
     * 鏁呴殰銆佷笂浣嶆満鍋滄鍛戒护銆佸厖鐢靛畬鎴愰兘浼氳蛋杩欓噷锛岀‘淇濊蒋浠剁姸鎬佸拰纭欢杈撳嚭涓€鑷淬€?
     */
    g_power_control.loop.power.enabled = 0U;
    g_power_control.loop.power.powerStageMode = (uint8_t)POWER_STAGE_MODE_OFF;
    g_power_control.loop.power.targetCurrentMa = 0U;
    g_power_control.loop.power.dutyX100 = 0U;
    Power_Control_Set_Battery_Current_Feedback(0, 0U);
    Power_Control_Set_Battery_Voltage_Feedback(0U, 0U);
    g_power_control.transition.preconnectActive = 0U;
    g_power_control.transition.preconnectOvpLimitMv = 0U;
    g_power_control.transition.afeHandoverGuardCount = 0U;
    Power_Control_Clear_Stall_Recover();
    Power_Control_Reset_Loop();

    Power_Pwm_Outputs_Disable();
}

void Power_Control_Fault_Lockout(void)
{
    g_power_control.fault.lockout = 1U;
    Power_Control_Stop();
}

void Power_Control_Clear_Fault_Lockout(void)
{
    g_power_control.fault.lockout = 0U;
}

uint32_t Power_Control_Get_Fault_Status(void)
{
    return g_power_control.fault.status;
}

void Power_Control_Clear_Fault_Status(void)
{
    g_power_control.fault.status = 0U;
    g_power_control.loop.power.tripReason = (uint8_t)POWER_TRIP_REASON_NONE;
    g_power_control.loop.power.tripFaults = 0U;
    g_power_control.loop.power.tripIoutMa = 0;
    g_power_control.loop.power.tripCurrentRefMa = 0U;
    g_power_control.loop.power.tripOcpLimitMa = 0U;
    g_power_control.loop.power.tripVoutMv = 0U;
    g_power_control.loop.power.tripVinMv = 0U;
    g_power_control.loop.power.tripDutyX100 = 0U;
    g_power_control.loop.power.tripFaultOcActive = 0U;
}

void Power_Control_Set(uint16_t target_voltage_mv, uint16_t target_current_ma, uint8_t mode)
{
    uint8_t was_enabled;
    uint8_t was_preconnect;
    uint8_t target_changed;
    uint8_t allow_ovp_blank;
    uint16_t previous_target_voltage_mv;

    /*
     * 鍐欏叆鏂扮殑鍔熺巼绾х洰鏍囥€?
     * 浠庡仠姝㈢姸鎬侀娆¤繘鍏ヨ繍琛岀姸鎬佹椂锛岄噸缃?PI 鍜岃蒋鍚姩鐢垫祦锛涜繍琛岃繃绋嬩腑鐘舵€佹満
     * 姣?20 ms 浼氶噸澶嶈皟鐢ㄦ湰鍑芥暟锛屾鏃跺彧鏇存柊鐩爣锛屼笉娓呯┖鐜矾鍘嗗彶銆?
     */
    target_voltage_mv = Power_Control_Clamp_Charge_Target_Mv(target_voltage_mv, mode);
    if(g_power_control.fault.lockout != 0U || g_power_control.fault.status != 0U) {
        Power_Control_Stop();
        return;
    }
    if(target_current_ma == 0U) {
        Power_Control_Stop();
        return;
    }

    was_enabled = g_power_control.loop.power.enabled;
    was_preconnect = g_power_control.transition.preconnectActive;
    previous_target_voltage_mv = g_power_control.loop.power.targetVoltageMv;
    target_changed = ((was_enabled != 0U) &&
                      ((g_power_control.loop.power.targetVoltageMv != target_voltage_mv) ||
                       (g_power_control.loop.power.mode != mode))) ? 1U : 0U;
    if((was_enabled != 0U) &&
       (was_preconnect != 0U) &&
       (target_voltage_mv >= previous_target_voltage_mv)) {
        target_changed = 0U;
    }
    /*
     * 鏁板瓧鐢垫簮鍦ㄨ繍琛屼腑鏀硅瀹氬€兼椂涓嶅浣嶇幆璺€?
     * 淇濇寔褰撳墠 duty 鍜?PI 鍘嗗彶锛岃鐢靛帇鐜钩婊戣窡韪埌鏂扮洰鏍囷紝
     * 鑰屼笉鏄妸 duty 鐮稿洖璧峰鍊奸€犳垚 Vout 鍏堟帀鍐嶇埇銆?
     */
    if((was_enabled != 0U) &&
       (mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER) &&
       (g_power_control.loop.power.mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER)) {
        target_changed = 0U;
    }
    allow_ovp_blank = ((was_enabled == 0U) ||
                       (g_power_control.loop.power.mode != mode) ||
                       (target_voltage_mv > g_power_control.loop.power.targetVoltageMv)) ? 1U : 0U;
    g_power_control.transition.preconnectActive = 0U;
    g_power_control.transition.preconnectOvpLimitMv = 0U;
    if(mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER) {
        g_power_control.transition.afeHandoverGuardCount = 0U;
    }
    g_power_control.loop.power.mode = mode;
    g_power_control.loop.power.targetVoltageMv = target_voltage_mv;
    g_power_control.loop.power.targetCurrentMa = target_current_ma;

    if((was_enabled == 0U) || (target_changed != 0U)) {
        Power_Control_Reset_Loop();
        if(allow_ovp_blank == 0U) {
            g_power_control.fault.outputOvpBlankCount = 0U;
        }
        g_power_control.loop.power.dutyX100 = POWER_START_DUTY_X100;
    }
    g_power_control.loop.power.enabled = 1U;

    if(g_power_control.fault.lockout == 0U && g_power_control.fault.status == 0U) {
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
    if(g_power_control.loop.power.enabled != 0U) {
        g_power_control.transition.preconnectActive = 1U;
        g_power_control.transition.preconnectOvpLimitMv = ovp_limit_mv;
        g_power_control.transition.afeHandoverGuardCount = 0U;
    }
}

void Power_Control_Set_Afe_Handover(uint16_t target_voltage_mv,
                                    uint16_t target_current_ma,
                                    uint8_t mode,
                                    const bms_power_sample_t *sample)
{
    uint16_t previous_boost_duty_x100;

    previous_boost_duty_x100 = g_power_control.loop.boostDutyX100;
    Power_Control_Set(target_voltage_mv, target_current_ma, mode);
    if(g_power_control.loop.power.enabled != 0U) {
        g_power_control.transition.preconnectActive = 0U;
        g_power_control.transition.preconnectOvpLimitMv = 0U;
        g_power_control.transition.afeHandoverGuardCount = BMS_POWER_AFE_HANDOVER_GUARD_CYCLES;
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
        g_power_control.feedback.batteryCurrentFeedbackMa = current_ma;
        g_power_control.feedback.batteryCurrentFeedbackValid = 1U;
    } else {
        g_power_control.feedback.batteryCurrentFeedbackValid = 0U;
        g_power_control.feedback.batteryCurrentFeedbackMa = 0;
    }
}

void Power_Control_Set_Battery_Voltage_Feedback(uint16_t pack_voltage_mv, uint8_t valid)
{
    if(valid != 0U && pack_voltage_mv != 0U) {
        g_power_control.feedback.batteryVoltageFeedbackMv = pack_voltage_mv;
        g_power_control.feedback.batteryVoltageFeedbackValid = 1U;
    } else {
        g_power_control.feedback.batteryVoltageFeedbackValid = 0U;
        g_power_control.feedback.batteryVoltageFeedbackMv = 0U;
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
    g_power_control.fault.status |= faults;
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

    if(g_power_control.fault.lockout != 0U) {
        Power_Control_Stop();
        return;
    }

    if(g_power_control.loop.power.enabled == 0U) {
        g_power_control.loop.power.dutyX100 = 0U;
        Power_Pwm_Outputs_Disable();
        return;
    }

    if(sample->faultBitmap != 0U) {
        Power_Control_Latch_Fault((uint8_t)POWER_TRIP_REASON_SAMPLE_FAULT,
                                  sample->faultBitmap,
                                  sample,
                                  g_power_control.loop.softCurrentMa);
        return;
    }

#if BMS_POWER_FAULT_PIN_FAST_LATCH_ENABLE
    if(sample->faultOcActive != 0U) {
        Power_Control_Latch_Fault((uint8_t)POWER_TRIP_REASON_FAULT_OC_PIN,
                                  BMS_FAULT_CHARGE_OCP,
                                  sample,
                                  g_power_control.loop.softCurrentMa);
        return;
    }
#endif

    if(g_power_control.loop.power.targetCurrentMa == 0U) {
        Power_Control_Stop();
        return;
    }

#if BMS_ENABLE_INPUT_UV_FAULT
    if(sample->inputVoltageMv < BMS_DEFAULT_INPUT_UV_THRESHOLD_MV) {
        Power_Control_Latch_Fault((uint8_t)POWER_TRIP_REASON_INPUT_UV,
                                  BMS_FAULT_INPUT_UV,
                                  sample,
                                  g_power_control.loop.softCurrentMa);
        return;
    }
#endif

    if(Power_Control_Output_Ovp_Confirmed(sample->outputVoltageMv) != 0U) {
        if(g_power_control.transition.preconnectActive != 0U) {
            Power_Control_Preconnect_Coast();
            Power_Control_Pwm_Apply();
            Power_Control_Pwm_Enable();
            Power_Control_Service_Stall_Recover(sample);
            return;
        }
        /*
         * 鏁板瓧鐢垫簮璋冧綆杈撳嚭鏃?Vout 浼氭殏鏃堕珮浜庢柊鐩爣銆傝蒋 OVP 鍖哄彧婊戣娉勬斁锛?
         * 璁╃數鍘嬬幆鎶?Vout 鎷夊洖鏂拌瀹氬€硷紱鍙湁瓒呰繃纭檺鎵嶉攣鏁呴殰銆?
         */
        if((g_power_control.loop.power.mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER) &&
           (Power_Control_Output_Over_Hard_Limit(sample->outputVoltageMv) == 0U)) {
            Power_Control_Preconnect_Coast();
            Power_Control_Pwm_Apply();
            Power_Control_Pwm_Enable();
            return;
        }
        Power_Control_Latch_Fault((uint8_t)POWER_TRIP_REASON_OUTPUT_OVP,
                                  BMS_FAULT_PACK_OVP,
                                  sample,
                                  g_power_control.loop.softCurrentMa);
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
    current_ref_ma = Power_Control_Ramp_Current(g_power_control.loop.power.targetCurrentMa);
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
        error = (int32_t)g_power_control.loop.power.targetVoltageMv - (int32_t)sample->outputVoltageMv;
        step = Pi_Controller_Update(&g_power_control.loop.voltagePi, error);
        if(light_load_guard_active != 0U) {
            Pi_Controller_Decay(&g_power_control.loop.currentPi, 1U, 2U);
        } else {
            Pi_Controller_Decay(&g_power_control.loop.currentPi, 7U, 8U);
        }
    } else {
        error = (int32_t)current_ref_ma - (int32_t)measured_current_ma;
        step = Pi_Controller_Update(&g_power_control.loop.currentPi, error);
        Pi_Controller_Decay(&g_power_control.loop.voltagePi, 7U, 8U);
    }

    step = Power_Control_Limit_Light_Load_Step(sample,
                                               current_ref_ma,
                                               measured_current_ma,
                                               step);
    duty = (int32_t)g_power_control.loop.power.dutyX100 + step;
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

    g_power_control.loop.power.dutyX100 = (uint16_t)duty;
    g_power_control.loop.power.dutyX100 = Clamp_U16(g_power_control.loop.power.dutyX100, POWER_DUTY_MIN_X100, POWER_LOOP_DUTY_MAX_X100);
    Power_Map_Control_To_Pwm(sample, g_power_control.loop.power.dutyX100);
    g_power_control.loop.asyncBoostRectifier =
        Power_Control_Async_Boost_Should_Run(current_ref_ma, measured_current_ma);

    if(g_power_control.fault.lockout != 0U) {
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

    *state = g_power_control.loop.power;
    state->buckDutyX100 = g_power_control.loop.buckDutyX100;
    state->boostLowDutyX100 = g_power_control.loop.boostDutyX100;
    state->faultLockout = g_power_control.fault.lockout;
    state->faultBitmap = g_power_control.fault.status;
    state->preconnectActive = g_power_control.transition.preconnectActive;
    state->preconnectOvpLimitMv = g_power_control.transition.preconnectOvpLimitMv;
    state->softCurrentMa = g_power_control.loop.softCurrentMa;
    state->asyncBoostRectifier = g_power_control.loop.asyncBoostRectifier;
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
