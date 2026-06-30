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
/*
 * 预连接 coast 阈值从 500mV 提高到 1000mV。
 * 轻载 boost 过冲幅度实测 500~600mV，500mV 阈值仍会触发 coast 停机。
 * 提高到 1000mV 给足过冲余量，让电压环有时间把 Vout 拉回目标。
 */
#define POWER_PRECONNECT_COAST_MARGIN_MV       1000U
#define POWER_TRANSIENT_OCP_CONFIRM_COUNT      3U
#define POWER_INPUT_GUARD_TARGET_MIN_MV        30000U
#define POWER_INPUT_GUARD_VIN_MIN_MV           22000U
#define POWER_INPUT_GUARD_IIN_MAX_MA           1800
#define POWER_INPUT_GUARD_DUTY_STEP_X100       120U
#define POWER_CC_DUTY_PACK_MARGIN_MV           800U
#define POWER_CC_DUTY_VOUT_ADVANCE_MV          800U
#define POWER_CC_ASYNC_BOOST_DUTY_HEADROOM_X100 1000U
#define POWER_CC_LIGHT_LOAD_HEADROOM_X100       300U
#define POWER_ASYNC_BOOST_CURRENT_MAX_MA       300U
#define POWER_ASYNC_BOOST_EXIT_CURRENT_MA      500U
#define POWER_REGION_MARGIN_MV                 1500U
#define POWER_INTEGRAL_LIMIT                   200000L
#define POWER_CURRENT_KP_DIV                   BMS_DEFAULT_CURRENT_KP_DIV
#define POWER_CURRENT_KI_DIV                   BMS_DEFAULT_CURRENT_KI_DIV
#define POWER_VOLTAGE_KP_DIV                   BMS_DEFAULT_VOLTAGE_KP_DIV
#define POWER_VOLTAGE_KI_DIV                   BMS_DEFAULT_VOLTAGE_KI_DIV

/*
 * 四开关 Buck-Boost 功率级控制。
 *
 * PWM 层把两个 HRTIMER slave timer 分别对应到两个半桥：
 * - PWM1H/PWM1L：输入侧半桥，记为 Buck 半桥；
 * - PWM2H/PWM2L：输出侧半桥，记为 Boost 半桥。
 *
 * 控制层使用保守的双环 PI：
 * - 低于目标电压时优先按电流环调节，实现涓流/恒流限流；
 * - 接近或高于目标电压时切到电压环，实现恒压；
 * - 任意阶段只要输出电流超过软启动后的电流上限，就强制电流环接管。
 *
 * 这不是最终高带宽数字电源环路，而是适合低压低流首轮上电的 20 ms
 * 慢速监督闭环。后续做 ADC 触发同步采样后，可以把 PI 移到 HRTIMER/ADC
 * 中断里提高动态响应。
 *
 * 对外单位约定：
 * - targetVoltageMv：目标输出电压，单位 mV；
 * - targetCurrentMa：目标充电电流，单位 mA；
 * - dutyX100：占空比百分数 * 100，例如 5000 表示 50.00%；
 * - mode：记录上层充电模式，当前主要用于状态回读，后续可扩展不同控制策略。
 *
 * 重要边界：
 * - Power_Control_Set() 只打开功率级并写目标；
 * - Power_Control_Apply() 根据最新 ADC 采样更新 PI 和 PWM；
 * - Power_Control_Stop() 是所有故障/停止路径的统一关断入口。
 */
static power_control_state_t s_power;
static pi_controller_t s_current_pi;
static pi_controller_t s_voltage_pi;
static uint16_t s_soft_current_ma;
static uint16_t s_buck_duty_x100;
static uint16_t s_boost_duty_x100;
static uint8_t s_async_boost_rectifier;
static uint8_t s_power_stall_recover_count;
static uint16_t s_output_ovp_count;
static uint16_t s_output_ovp_blank_count;
static uint8_t s_output_ocp_count;
static uint8_t s_preconnect_active;
static uint16_t s_preconnect_ovp_limit_mv;
static uint16_t s_afe_handover_guard_count;
static volatile int16_t s_battery_current_feedback_ma;
static volatile uint8_t s_battery_current_feedback_valid;
static volatile uint16_t s_battery_voltage_feedback_mv;
static volatile uint8_t s_battery_voltage_feedback_valid;
static volatile uint8_t s_fault_lockout;
static volatile uint32_t s_fault_status;

static uint16_t Clamp_U16(uint16_t value, uint16_t min_value, uint16_t max_value);
static uint16_t Power_Control_Clamp_Charge_Target_Mv(uint16_t target_voltage_mv, uint8_t mode);
static void Power_Map_Control_To_Pwm(const bms_power_sample_t *sample, uint16_t control_x100);
static void Power_Control_Pwm_Apply(void);
static void Power_Control_Pwm_Enable(void);
static uint8_t Power_Control_Afe_Handover_Active(void);
static uint8_t Power_Control_Async_Boost_Should_Run(uint16_t current_ref_ma,
                                                    uint16_t measured_current_ma);

#include "power_control_safety.inc"

#include "power_control_limits.inc"

#include "power_control_mode.inc"

#include "power_control_api.inc"
