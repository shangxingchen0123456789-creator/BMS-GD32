#ifndef POWER_CONTROL_INTERNAL_H
#define POWER_CONTROL_INTERNAL_H

#include "power_control.h"

#include "bms_board_config.h"
#include "pi_controller.h"
#include "power_pwm.h"

#include <string.h>

#define POWER_DUTY_MIN_X100                    BMS_PWM_DUTY_MIN_X100
#define POWER_DUTY_MAX_X100                    BMS_PWM_DUTY_MAX_X100
#define POWER_LOOP_DUTY_MAX_X100               BMS_PWM_DUTY_MAX_X100
#define POWER_START_DUTY_X100                  300U
#define POWER_STALL_RECOVER_DUTY_X100          5500U
#define POWER_STALL_RECOVER_VOUT_GAP_MV        5000U
#define POWER_STALL_RECOVER_IIN_MAX_MA         80
#define POWER_STALL_RECOVER_CONFIRM_COUNT      25U
#define POWER_SOFTSTART_STEP_MA                50U
#define POWER_LOOP_STEP_MAX_X100               BMS_DEFAULT_LOOP_STEP_MAX_X100
#define POWER_VOLTAGE_CV_MARGIN_MV             100U
#define POWER_LIGHT_LOAD_CV_MARGIN_MV          1200U
#define POWER_PRECONNECT_LIGHT_LOAD_MARGIN_MV  500U
#define POWER_LIGHT_LOAD_CURRENT_MIN_MA        80U
#define POWER_LIGHT_LOAD_CURRENT_MAX_MA        300U
#define POWER_LIGHT_LOAD_CURRENT_DIV           4U
#define POWER_LIGHT_LOAD_STEP_MAX_X100         10U
#define POWER_LIGHT_LOAD_BOOST_HEADROOM_X100   500U
#define POWER_LIGHT_LOAD_VOUT_ADVANCE_MV       1600U
#define POWER_PRECONNECT_BOOST_DUTY_MAX_X100   POWER_LOOP_DUTY_MAX_X100
#define POWER_PRECONNECT_BOOST_HEADROOM_X100   1000U
#define POWER_PRECONNECT_COAST_MARGIN_MV       1000U
#define POWER_TRANSIENT_OCP_CONFIRM_COUNT      3U
#define POWER_INPUT_GUARD_TARGET_MIN_MV        30000U
#define POWER_INPUT_GUARD_VIN_MIN_MV           22000U
#define POWER_INPUT_GUARD_IIN_MAX_MA           1800
#define POWER_INPUT_GUARD_DUTY_STEP_X100       120U
#define POWER_CC_DUTY_PACK_MARGIN_MV           800U
#define POWER_CC_DUTY_VOUT_ADVANCE_MV          800U
#define POWER_CC_ASYNC_BOOST_DUTY_HEADROOM_X100 1000U
#define POWER_CC_LIGHT_LOAD_HEADROOM_X100      300U
#define POWER_ASYNC_BOOST_CURRENT_MAX_MA       300U
#define POWER_ASYNC_BOOST_EXIT_CURRENT_MA      500U
#define POWER_REGION_MARGIN_MV                 1500U
#define POWER_INTEGRAL_LIMIT                   200000L
#define POWER_CURRENT_KP_DIV                   BMS_DEFAULT_CURRENT_KP_DIV
#define POWER_CURRENT_KI_DIV                   BMS_DEFAULT_CURRENT_KI_DIV
#define POWER_VOLTAGE_KP_DIV                   BMS_DEFAULT_VOLTAGE_KP_DIV
#define POWER_VOLTAGE_KI_DIV                   BMS_DEFAULT_VOLTAGE_KI_DIV

typedef struct {
    power_control_state_t power;
    pi_controller_t currentPi;
    pi_controller_t voltagePi;
    uint16_t softCurrentMa;
    uint16_t buckDutyX100;
    uint16_t boostDutyX100;
    uint8_t asyncBoostRectifier;
    uint8_t stallRecoverCount;
    uint16_t outputOvpCount;
    uint16_t outputOvpBlankCount;
    uint8_t outputOcpCount;
    uint8_t preconnectActive;
    uint16_t preconnectOvpLimitMv;
    uint16_t afeHandoverGuardCount;
    volatile int16_t batteryCurrentFeedbackMa;
    volatile uint8_t batteryCurrentFeedbackValid;
    volatile uint16_t batteryVoltageFeedbackMv;
    volatile uint8_t batteryVoltageFeedbackValid;
    volatile uint8_t faultLockout;
    volatile uint32_t faultStatus;
} power_control_context_t;

extern power_control_context_t g_power_control;

#define s_power                             (g_power_control.power)
#define s_current_pi                        (g_power_control.currentPi)
#define s_voltage_pi                        (g_power_control.voltagePi)
#define s_soft_current_ma                   (g_power_control.softCurrentMa)
#define s_buck_duty_x100                    (g_power_control.buckDutyX100)
#define s_boost_duty_x100                   (g_power_control.boostDutyX100)
#define s_async_boost_rectifier             (g_power_control.asyncBoostRectifier)
#define s_power_stall_recover_count         (g_power_control.stallRecoverCount)
#define s_output_ovp_count                  (g_power_control.outputOvpCount)
#define s_output_ovp_blank_count            (g_power_control.outputOvpBlankCount)
#define s_output_ocp_count                  (g_power_control.outputOcpCount)
#define s_preconnect_active                 (g_power_control.preconnectActive)
#define s_preconnect_ovp_limit_mv           (g_power_control.preconnectOvpLimitMv)
#define s_afe_handover_guard_count          (g_power_control.afeHandoverGuardCount)
#define s_battery_current_feedback_ma       (g_power_control.batteryCurrentFeedbackMa)
#define s_battery_current_feedback_valid    (g_power_control.batteryCurrentFeedbackValid)
#define s_battery_voltage_feedback_mv       (g_power_control.batteryVoltageFeedbackMv)
#define s_battery_voltage_feedback_valid    (g_power_control.batteryVoltageFeedbackValid)
#define s_fault_lockout                     (g_power_control.faultLockout)
#define s_fault_status                      (g_power_control.faultStatus)

uint16_t Clamp_U16(uint16_t value, uint16_t min_value, uint16_t max_value);
uint16_t Power_Control_Clamp_Charge_Target_Mv(uint16_t target_voltage_mv, uint8_t mode);
void Power_Control_Reset_Loop(void);
void Power_Control_Clear_Stall_Recover(void);
uint8_t Power_Control_Output_Stalled(const bms_power_sample_t *sample);
void Power_Control_Service_Stall_Recover(const bms_power_sample_t *sample);
uint16_t Power_Control_Output_Ovp_Margin_Mv(void);
uint8_t Power_Control_Output_Ovp_Confirmed(uint16_t output_voltage_mv);
uint8_t Power_Control_Output_Over_Hard_Limit(uint16_t output_voltage_mv);
uint16_t Power_Control_Ramp_Current(uint16_t target_current_ma);
uint16_t Power_Control_Positive_Output_Current_Ma(const bms_power_sample_t *sample);
uint8_t Power_Control_Output_Ocp_Confirmed(uint16_t output_current_ma, uint16_t current_ref_ma);
uint16_t Power_Control_Current_Feedback_Ma(const bms_power_sample_t *sample);

uint16_t Power_Control_Light_Load_Current_Threshold(uint16_t current_ref_ma);
uint16_t Power_Control_Light_Load_Cv_Margin_Mv(void);
uint8_t Power_Control_Light_Load_Near_Target(const bms_power_sample_t *sample,
                                             uint16_t current_ref_ma,
                                             uint16_t measured_current_ma);
uint16_t Power_Control_Light_Load_Duty_Max(const bms_power_sample_t *sample);
uint16_t Power_Control_Preconnect_Duty_Max(const bms_power_sample_t *sample);
uint16_t Power_Control_Preconnect_Duty_Min(const bms_power_sample_t *sample);
uint8_t Power_Control_Preconnect_Coast_Should_Run(const bms_power_sample_t *sample);
void Power_Control_Preconnect_Coast(void);
uint8_t Power_Control_Afe_Handover_Active(void);
uint16_t Power_Control_Handover_Boost_Duty(const bms_power_sample_t *sample,
                                           uint16_t previous_boost_duty_x100);
void Power_Control_Afe_Handover_Start_Output(const bms_power_sample_t *sample,
                                             uint16_t previous_boost_duty_x100);
void Power_Control_Afe_Handover_Decay(void);
int32_t Power_Control_Limit_Afe_Handover_Duty(const bms_power_sample_t *sample, int32_t duty);
int32_t Power_Control_Limit_Preconnect_Duty(const bms_power_sample_t *sample, int32_t duty);
uint32_t Power_Control_Cc_Duty_Target_Mv(const bms_power_sample_t *sample);
uint32_t Power_Control_Battery_Boost_Headroom_X100(void);
uint16_t Power_Control_Battery_Boost_Duty_Max(const bms_power_sample_t *sample);
int32_t Power_Control_Limit_Battery_Boost_Duty(const bms_power_sample_t *sample, int32_t duty);
uint8_t Power_Control_Input_Guard_Active(const bms_power_sample_t *sample);
void Power_Control_Reduce_Duty_For_Input_Guard(const bms_power_sample_t *sample);
int32_t Power_Control_Limit_Light_Load_Step(const bms_power_sample_t *sample,
                                            uint16_t current_ref_ma,
                                            uint16_t measured_current_ma,
                                            int32_t step);

uint8_t Power_Voltage_Loop_Should_Run(const bms_power_sample_t *sample,
                                      uint16_t current_ref_ma,
                                      uint16_t measured_current_ma);
uint8_t Power_Control_Async_Boost_Should_Run(uint16_t current_ref_ma,
                                             uint16_t measured_current_ma);
uint16_t Power_Ocp_Limit_From_Ref(uint16_t current_ref_ma);
void Power_Control_Record_Trip(uint8_t reason,
                               uint32_t faults,
                               const bms_power_sample_t *sample,
                               uint16_t current_ref_ma);
void Power_Control_Pwm_Output_Context(power_pwm_output_context_t *context);
void Power_Control_Pwm_Apply(void);
void Power_Control_Pwm_Enable(void);
void Power_Map_Control_To_Pwm(const bms_power_sample_t *sample, uint16_t control_x100);

#endif
