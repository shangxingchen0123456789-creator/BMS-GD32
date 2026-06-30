#include "power_control.h"

#include "bms_board_config.h"
#include "pi_controller.h"
#include "system_gd32g5x3.h"

#include <string.h>

#define POWER_USE_HARDWARE_PWM                 BMS_ENABLE_HRTIMER_PWM
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
#define POWER_PWM_COMPARE_GUARD_TICKS          3U
#define POWER_PWM_DEADTIME_CLOCK_MUL           32U
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
static uint8_t Power_Control_Afe_Handover_Active(void);
static uint8_t Power_Control_Async_Boost_Should_Run(uint16_t current_ref_ma,
                                                    uint16_t measured_current_ma);

#if POWER_USE_HARDWARE_PWM
static uint8_t s_pwm_ready;
static uint8_t s_pwm_outputs_on;
static uint32_t s_pwm_period_ticks;
static uint32_t s_pwm_deadtime_ticks;

static uint32_t Power_Clamp_U32(uint32_t value, uint32_t min_value, uint32_t max_value)
{
    if(value < min_value) {
        return min_value;
    }
    if(value > max_value) {
        return max_value;
    }

    return value;
}

static uint32_t Power_Pwm_Period_Ticks(void)
{
    uint32_t period;

    /*
     * HRTIMER 使用 MUL32 预分频，因此计数频率约为 SystemCoreClock * 32。
     * 这里按目标 PWM 频率换算 ARR 周期值，并限制到寄存器可接受范围内。
     */
    period = ((SystemCoreClock / BMS_PWM_FREQUENCY_HZ) * 32U);
    return Power_Clamp_U32(period, 100U, 0xFF00U);
}

static uint32_t Power_Pwm_Deadtime_Ticks(void)
{
    uint32_t clock_mhz;
    uint32_t max_ticks;
    uint32_t ticks;

    /*
     * 将板级配置的 ns 死区时间换算为 HRTIMER tick。
     * 死区同时加在上升沿和下降沿，避免同一半桥上下管直通。
     */
    clock_mhz = SystemCoreClock / 1000000U;
    ticks = ((clock_mhz * POWER_PWM_DEADTIME_CLOCK_MUL * BMS_PWM_DEADTIME_NS) + 999U) / 1000U;
    max_ticks = (s_pwm_period_ticks > (POWER_PWM_COMPARE_GUARD_TICKS * 4U)) ?
                (s_pwm_period_ticks / 4U) :
                POWER_PWM_COMPARE_GUARD_TICKS;

    return Power_Clamp_U32(ticks, POWER_PWM_COMPARE_GUARD_TICKS, max_ticks);
}

static uint32_t Power_Pwm_Leading_Compare_Ticks(void)
{
    uint32_t max_compare;

    if(s_pwm_period_ticks <= (POWER_PWM_COMPARE_GUARD_TICKS * 2U)) {
        return POWER_PWM_COMPARE_GUARD_TICKS;
    }

    max_compare = s_pwm_period_ticks - POWER_PWM_COMPARE_GUARD_TICKS;
    return Power_Clamp_U32(s_pwm_deadtime_ticks, POWER_PWM_COMPARE_GUARD_TICKS, max_compare);
}

static uint32_t Power_Pwm_Duty_Compare_Min_Ticks(void)
{
    uint32_t min_compare;
    uint32_t max_compare;

    min_compare = Power_Pwm_Leading_Compare_Ticks() + POWER_PWM_COMPARE_GUARD_TICKS;
    max_compare = (s_pwm_period_ticks > POWER_PWM_COMPARE_GUARD_TICKS) ?
                  (s_pwm_period_ticks - POWER_PWM_COMPARE_GUARD_TICKS) :
                  POWER_PWM_COMPARE_GUARD_TICKS;

    return Power_Clamp_U32(min_compare, POWER_PWM_COMPARE_GUARD_TICKS, max_compare);
}

static uint32_t Power_Pwm_Duty_Compare_Max_Ticks(void)
{
    uint32_t min_compare;
    uint32_t max_compare;
    uint32_t trailing_guard;

    min_compare = Power_Pwm_Duty_Compare_Min_Ticks();
    trailing_guard = s_pwm_deadtime_ticks + POWER_PWM_COMPARE_GUARD_TICKS;
    if(s_pwm_period_ticks <= trailing_guard) {
        return min_compare;
    }

    max_compare = s_pwm_period_ticks - trailing_guard;
    if(max_compare < min_compare) {
        return min_compare;
    }

    return max_compare;
}

static uint32_t Power_Pwm_Low_Set_Compare_From_High_Compare(uint32_t high_compare)
{
    uint32_t low_set_compare;
    uint32_t min_compare;
    uint32_t max_compare;

    low_set_compare = high_compare + s_pwm_deadtime_ticks;
    max_compare = (s_pwm_period_ticks > POWER_PWM_COMPARE_GUARD_TICKS) ?
                  (s_pwm_period_ticks - POWER_PWM_COMPARE_GUARD_TICKS) :
                  POWER_PWM_COMPARE_GUARD_TICKS;
    min_compare = high_compare + POWER_PWM_COMPARE_GUARD_TICKS;
    if(min_compare > max_compare) {
        min_compare = max_compare;
    }

    return Power_Clamp_U32(low_set_compare, min_compare, max_compare);
}

static uint32_t Power_Pwm_Compare_From_Duty(uint16_t duty_x100)
{
    uint32_t compare;

    /*
     * duty_x100 表示百分比 * 100，例如 5000 代表 50.00%。
     * compare 不能等于 0 或 period，否则可能出现极窄脉冲或寄存器边界行为，
     * 因此最终再留出 3 tick 的保护余量。
     */
    duty_x100 = Clamp_U16(duty_x100, POWER_DUTY_MIN_X100, POWER_DUTY_MAX_X100);
    compare = (s_pwm_period_ticks * (uint32_t)duty_x100) / 10000U;
    compare = Power_Clamp_U32(compare,
                              Power_Pwm_Duty_Compare_Min_Ticks(),
                              Power_Pwm_Duty_Compare_Max_Ticks());

    return compare;
}

static uint32_t Power_Pwm_Sample_Compare_Ticks(void)
{
    uint32_t compare;

    compare = (s_pwm_period_ticks * (uint32_t)BMS_ADC_SYNC_PHASE_X10000) / 10000U;
    return Power_Clamp_U32(compare, 5U, s_pwm_period_ticks - 5U);
}

static void Power_Pwm_Gpio_Init(void)
{
    uint32_t pwm_pins;

    pwm_pins = BMS_PWM1H_PIN | BMS_PWM1L_PIN | BMS_PWM2H_PIN | BMS_PWM2L_PIN;

    /*
     * 四路 PWM 和 FAULT_OC 都在同一个 GPIO 端口上，先开 GPIO 时钟，
     * 再切换到 HRTIMER 复用功能。
     */
    rcu_periph_clock_enable(BMS_PWM_GPIO_CLK);

    gpio_af_set(BMS_PWM_GPIO_PORT, BMS_PWM_AF, pwm_pins);
#if BMS_ENABLE_POWER_FAULT_PIN
    gpio_af_set(BMS_PWM_GPIO_PORT, BMS_PWM_AF, BMS_PWM_FAULT_PIN);
#endif

    gpio_mode_set(BMS_PWM_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, pwm_pins);
    gpio_output_options_set(BMS_PWM_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, pwm_pins);

#if BMS_ENABLE_POWER_FAULT_PIN
    gpio_mode_set(BMS_PWM_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, BMS_PWM_FAULT_PIN);
    gpio_output_options_set(BMS_PWM_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, BMS_PWM_FAULT_PIN);
#endif
}

static void Power_Pwm_Config_Fault(void)
{
#if BMS_ENABLE_POWER_FAULT_PIN
    hrtimer_faultcfg_parameter_struct faultcfg;

    /*
     * 网表中 FAULT_OC 为高电平有效。
     * 一旦硬件过流比较器触发，HRTIMER fault 会把相关输出强制拉到 inactive。
     * automatic_resume 打开，故障脚恢复后 HRTIMER 输出可自动恢复；软件仍会
     * 通过 PA12 快速采样锁存真正持续的过流故障。
     */
    hrtimer_faultcfg_struct_para_init(&faultcfg);
    faultcfg.source = HRTIMER_FAULT_SOURCE_PIN;
    faultcfg.polarity = HRTIMER_FAULT_POLARITY_HIGH;
    faultcfg.filter = 4U;
    faultcfg.control = HRTIMER_FAULT_CHANNEL_ENABLE;
    faultcfg.protect = HRTIMER_FAULT_PROTECT_DISABLE;
    faultcfg.resetmode = HRTIMER_FAULT_COUNTER_RESET_UNCONDITIONAL;
    faultcfg.fault_recovery_control = HRTIMER_FAULT_RECOVERY_CONTROL_ENABLE;

    hrtimer_fault_config(BMS_HRTIMER_PERIPH, HRTIMER_FAULT_0, &faultcfg);
    hrtimer_fault_input_enable(BMS_HRTIMER_PERIPH, HRTIMER_FAULT_0);
#endif
}

static void Power_Pwm_Config_Slave(uint32_t timer_id, uint32_t channel_h, uint32_t channel_l)
{
    hrtimer_baseinit_parameter_struct base;
    hrtimer_timerinit_parameter_struct timerinit;
    hrtimer_timercfg_parameter_struct timercfg;
    hrtimer_comparecfg_parameter_struct compare;
    hrtimer_channel_outputcfg_parameter_struct channelcfg;
    uint32_t initial_compare;
    uint32_t initial_high_set_compare;
    uint32_t initial_low_set_compare;

    initial_compare = Power_Pwm_Compare_From_Duty(POWER_DUTY_MIN_X100);
    initial_high_set_compare = Power_Pwm_Leading_Compare_Ticks();
    initial_low_set_compare = Power_Pwm_Low_Set_Compare_From_High_Compare(initial_compare);

    /*
     * 每个 slave timer 驱动一组互补半桥：
     * - timer0 对应 PWM1H/PWM1L；
     * - timer1 对应 PWM2H/PWM2L。
     * 两组 timer 使用相同周期和比较值，便于调试四开关同步工作。
     */
    hrtimer_baseinit_struct_para_init(&base);
    base.period = s_pwm_period_ticks;
    base.repetitioncounter = 0U;
    base.prescaler = HRTIMER_PRESCALER_MUL32;
    base.counter_mode = HRTIMER_COUNTER_MODE_CONTINOUS;
    base.counterdirection = HRTIMER_COUNTER_UP;
    hrtimer_timers_base_init(BMS_HRTIMER_PERIPH, timer_id, &base);

    /*
     * 关闭 shadow，比较值写入后通过 software_update 立即生效。
     * 后续如果需要更严格的同步更新，可以改为开启 shadow 并在周期边界统一装载。
     */
    hrtimer_timerinit_struct_para_init(&timerinit);
    timerinit.shadow = HRTIMER_SHADOW_DISABLED;
    hrtimer_timers_waveform_init(BMS_HRTIMER_PERIPH, timer_id, &timerinit);

    /*
     * Automatic HRTIMER deadtime is disabled here. Non-overlap is generated by
     * explicit compare events below; FAULT0 still handles hardware shutdown.
     */
    hrtimer_timercfg_struct_para_init(&timercfg);
    timercfg.deadtime_enable = HRTIMER_STXDEADTIME_DISABLED;
#if BMS_ENABLE_POWER_FAULT_PIN
    timercfg.fault_enable = HRTIMER_STXFAULTENABLE_FAULT0;
    timercfg.fault_automatic_resume = HRTIMER_STXFAULT_AUTOMATIC_RESUME_ENABLED;
#else
    timercfg.fault_enable = HRTIMER_STXFAULTENABLE_NONE;
#endif
    hrtimer_slavetimer_waveform_config(BMS_HRTIMER_PERIPH, timer_id, &timercfg);

    hrtimer_comparecfg_struct_para_init(&compare);
    compare.compare_value = initial_compare;
    compare.immediately_update_cmp0 = HRTIMER_IMMEDIATELY_UPDATE_CMP0_ENABLE;
    hrtimer_slavetimer_waveform_compare_config(BMS_HRTIMER_PERIPH, timer_id, HRTIMER_COMPARE0, &compare);

    hrtimer_comparecfg_struct_para_init(&compare);
    compare.compare_value = initial_high_set_compare;
    hrtimer_slavetimer_waveform_compare_config(BMS_HRTIMER_PERIPH, timer_id, HRTIMER_COMPARE1, &compare);

#if BMS_ENABLE_PWM_SYNC_ADC
    /*
     * CMP2 is not connected to any PWM output. It marks a fixed in-period
     * sampling phase for current ADC reads.
     */
    hrtimer_comparecfg_struct_para_init(&compare);
    compare.compare_value = Power_Pwm_Sample_Compare_Ticks();
    compare.immediately_update_cmp2 = HRTIMER_IMMEDIATELY_UPDATE_CMP2_ENABLE;
    hrtimer_slavetimer_waveform_compare_config(BMS_HRTIMER_PERIPH, timer_id, HRTIMER_COMPARE2, &compare);
#endif

    hrtimer_comparecfg_struct_para_init(&compare);
    compare.compare_value = initial_low_set_compare;
    hrtimer_slavetimer_waveform_compare_config(BMS_HRTIMER_PERIPH, timer_id, HRTIMER_COMPARE3, &compare);

    /* Explicit non-overlap: high side is CMP1..CMP0, low side is CMP3..PER. */
    hrtimer_channel_outputcfg_struct_para_init(&channelcfg);
    /*
     * High side is active from CMP1 to CMP0. Increasing duty moves CMP0 later.
     */
    channelcfg.polarity = HRTIMER_CHANNEL_POLARITY_HIGH;
    channelcfg.set_request = HRTIMER_CHANNEL_SET_CMP1;
    channelcfg.reset_request = HRTIMER_CHANNEL_RESET_CMP0;
    channelcfg.idle_state = HRTIMER_CHANNEL_IDLESTATE_INACTIVE;
    channelcfg.fault_state = HRTIMER_CHANNEL_FAULTSTATE_INACTIVE;
    hrtimer_slavetimer_waveform_channel_config(BMS_HRTIMER_PERIPH, timer_id, channel_h, &channelcfg);

    hrtimer_channel_outputcfg_struct_para_init(&channelcfg);
    /*
     * Low side is active from CMP3 to period end, after the CMP0 deadtime gap.
     */
    channelcfg.polarity = HRTIMER_CHANNEL_POLARITY_HIGH;
    channelcfg.set_request = HRTIMER_CHANNEL_SET_CMP3;
    channelcfg.reset_request = HRTIMER_CHANNEL_RESET_PER;
    channelcfg.idle_state = HRTIMER_CHANNEL_IDLESTATE_INACTIVE;
    channelcfg.fault_state = HRTIMER_CHANNEL_FAULTSTATE_INACTIVE;
    hrtimer_slavetimer_waveform_channel_config(BMS_HRTIMER_PERIPH, timer_id, channel_l, &channelcfg);
}

static uint16_t Power_Complementary_High_Duty(uint16_t low_duty_x100)
{
    if(low_duty_x100 >= 10000U) {
        return POWER_DUTY_MIN_X100;
    }

    return Clamp_U16((uint16_t)(10000U - low_duty_x100), POWER_DUTY_MIN_X100, POWER_DUTY_MAX_X100);
}

static void Power_Pwm_Apply(uint16_t buck_duty_x100, uint16_t boost_low_duty_x100)
{
    uint16_t buck_high_duty;
    uint16_t boost_high_duty;
    uint32_t buck_compare;
    uint32_t boost_compare;
    uint32_t buck_low_set_compare;
    uint32_t boost_low_set_compare;

    if(s_pwm_ready == 0U || s_fault_lockout != 0U) {
        return;
    }

    /*
     * HRTIMER 输出配置中 duty 表示“高边导通比例”。
     * Buck 半桥直接使用 buck_duty_x100；Boost 半桥的控制量表示低边
     * 储能占空比，因此这里取互补值写入高边比较寄存器。
     */
    buck_high_duty = Clamp_U16(buck_duty_x100, POWER_DUTY_MIN_X100, POWER_DUTY_MAX_X100);
    boost_high_duty = Power_Complementary_High_Duty(boost_low_duty_x100);

    buck_compare = Power_Pwm_Compare_From_Duty(buck_high_duty);
    boost_compare = Power_Pwm_Compare_From_Duty(boost_high_duty);
    buck_low_set_compare = Power_Pwm_Low_Set_Compare_From_High_Compare(buck_compare);
    boost_low_set_compare = Power_Pwm_Low_Set_Compare_From_High_Compare(boost_compare);

    hrtimer_slavetimer_compare_value_config(BMS_HRTIMER_PERIPH, HRTIMER_SLAVE_TIMER0, HRTIMER_COMPARE0, buck_compare);
    hrtimer_slavetimer_compare_value_config(BMS_HRTIMER_PERIPH, HRTIMER_SLAVE_TIMER0, HRTIMER_COMPARE3, buck_low_set_compare);
    hrtimer_slavetimer_compare_value_config(BMS_HRTIMER_PERIPH, HRTIMER_SLAVE_TIMER1, HRTIMER_COMPARE0, boost_compare);
    hrtimer_slavetimer_compare_value_config(BMS_HRTIMER_PERIPH, HRTIMER_SLAVE_TIMER1, HRTIMER_COMPARE3, boost_low_set_compare);
    hrtimer_software_update(BMS_HRTIMER_PERIPH, HRTIMER_UPDATE_SW_ST0 | HRTIMER_UPDATE_SW_ST1);
}

static uint32_t Power_Pwm_Active_Output_Channels(void)
{
    uint32_t channels;

    channels = BMS_PWM_OUTPUT_CHANNELS;
    if((s_async_boost_rectifier != 0U) &&
       (s_power.powerStageMode == (uint8_t)POWER_STAGE_MODE_BOOST) &&
       (s_boost_duty_x100 == 0U)) {
        if(s_preconnect_active == 0U) {
            return 0U;
        }
    }

    if(s_async_boost_rectifier != 0U) {
        channels &= ~((uint32_t)HRTIMER_ST1_CH0);
    }

    return channels;
}

static uint8_t Power_Pwm_Fault_Pin_Active(void)
{
#if BMS_ENABLE_POWER_FAULT_PIN
    return (RESET != gpio_input_bit_get(BMS_PWM_GPIO_PORT, BMS_PWM_FAULT_PIN)) ? 1U : 0U;
#else
    return 0U;
#endif
}

static void Power_Pwm_Clear_Recovered_Fault(void)
{
#if BMS_ENABLE_POWER_FAULT_PIN
    if((s_pwm_ready == 0U) || (s_fault_lockout != 0U) || (s_fault_status != 0U)) {
        return;
    }

    if(Power_Pwm_Fault_Pin_Active() != 0U) {
        return;
    }

    if(SET == hrtimer_common_flag_get(BMS_HRTIMER_PERIPH, HRTIMER_FLAG_FLT0)) {
        hrtimer_common_flag_clear(BMS_HRTIMER_PERIPH, HRTIMER_FLAG_FLT0);
        hrtimer_fault_counter_reset(BMS_HRTIMER_PERIPH, HRTIMER_FAULT_0);
    }
#endif
}

static void Power_Pwm_Outputs_Enable(void)
{
    uint32_t active_channels;
    uint32_t inactive_channels;

    /* 初始化完成后才允许打开输出，防止 GPIO 复用未就绪时出现毛刺。 */
    if((s_pwm_ready != 0U) && (s_fault_lockout == 0U)) {
        Power_Pwm_Clear_Recovered_Fault();
    }
    if((s_pwm_ready != 0U) && (s_fault_lockout == 0U)) {
        active_channels = Power_Pwm_Active_Output_Channels();
        inactive_channels = BMS_PWM_OUTPUT_CHANNELS & ~active_channels;
        if(active_channels != 0U) {
            hrtimer_output_channel_enable(BMS_HRTIMER_PERIPH, active_channels);
        }
        if(inactive_channels != 0U) {
            hrtimer_output_channel_disable(BMS_HRTIMER_PERIPH, inactive_channels);
        }
        s_pwm_outputs_on = (active_channels != 0U) ? 1U : 0U;
    }
}

static void Power_Pwm_Outputs_Disable(void)
{
    /* 关闭输出只影响引脚驱动，HRTIMER 计数器仍可继续运行，便于快速恢复。 */
    if(s_pwm_ready != 0U) {
        hrtimer_output_channel_disable(BMS_HRTIMER_PERIPH, BMS_PWM_OUTPUT_CHANNELS);
        s_pwm_outputs_on = 0U;
    }
}

uint8_t Power_Control_Wait_Adc_Sample_Point(uint32_t timeout)
{
#if BMS_ENABLE_PWM_SYNC_ADC
    if(s_pwm_ready == 0U) {
        return 0U;
    }

    hrtimer_timers_flag_clear(BMS_HRTIMER_PERIPH,
                              HRTIMER_SLAVE_TIMER0,
                              HRTIMER_MT_ST_FLAG_CMP2);
    while(timeout > 0U) {
        if(SET == hrtimer_timers_flag_get(BMS_HRTIMER_PERIPH,
                                          HRTIMER_SLAVE_TIMER0,
                                          HRTIMER_MT_ST_FLAG_CMP2)) {
            hrtimer_timers_flag_clear(BMS_HRTIMER_PERIPH,
                                      HRTIMER_SLAVE_TIMER0,
                                      HRTIMER_MT_ST_FLAG_CMP2);
            return 1U;
        }
        timeout--;
    }
#else
    (void)timeout;
#endif

    return 0U;
}

static void Power_Pwm_Init(void)
{
    uint32_t timeout;

    s_pwm_ready = 0U;
    s_pwm_outputs_on = 0U;
    s_pwm_period_ticks = Power_Pwm_Period_Ticks();
    s_pwm_deadtime_ticks = Power_Pwm_Deadtime_Ticks();

    Power_Pwm_Gpio_Init();

    rcu_periph_clock_enable(BMS_HRTIMER_CLK);
    rcu_hrtimer_clock_config(RCU_HRTIMERSRC_CKSYS);
    hrtimer_deinit(BMS_HRTIMER_PERIPH);

    /*
     * HRTIMER 高频工作前需要等待 DLL 校准完成。
     * 如果超时，s_pwm_ready 保持 0，后续 Power_Pwm_Apply()/enable() 会直接返回。
     */
    hrtimer_dll_calibration_start(BMS_HRTIMER_PERIPH, HRTIMER_CALIBRATION_ONCE);
    timeout = 200000U;
    while((RESET == hrtimer_common_flag_get(BMS_HRTIMER_PERIPH, HRTIMER_FLAG_DLLCAL)) && (timeout > 0U)) {
        timeout--;
    }
    if(timeout == 0U) {
        return;
    }
    hrtimer_common_flag_clear(BMS_HRTIMER_PERIPH, HRTIMER_FLAG_DLLCAL);

    /*
     * 初始化顺序：
     * 1. fault 输入；
     * 2. 两组互补输出 timer；
     * 3. 软件更新装载初值；
     * 4. 使能计数器但暂不打开输出通道。
     */
    Power_Pwm_Config_Fault();
    Power_Pwm_Config_Slave(HRTIMER_SLAVE_TIMER0, HRTIMER_ST0_CH0, HRTIMER_ST0_CH1);
    Power_Pwm_Config_Slave(HRTIMER_SLAVE_TIMER1, HRTIMER_ST1_CH0, HRTIMER_ST1_CH1);

    hrtimer_software_update(BMS_HRTIMER_PERIPH, HRTIMER_UPDATE_SW_ST0 | HRTIMER_UPDATE_SW_ST1);
    hrtimer_timers_counter_enable(BMS_HRTIMER_PERIPH, BMS_PWM_COUNTERS);
    hrtimer_output_channel_disable(BMS_HRTIMER_PERIPH, BMS_PWM_OUTPUT_CHANNELS);

    s_pwm_ready = 1U;
}
#else
uint8_t Power_Control_Wait_Adc_Sample_Point(uint32_t timeout)
{
    (void)timeout;
    return 0U;
}
#endif

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
#if POWER_USE_HARDWARE_PWM
    Power_Pwm_Outputs_Disable();
#endif
    Power_Control_Reset_Loop();
    s_power.dutyX100 = POWER_START_DUTY_X100;
    Power_Map_Control_To_Pwm(sample, s_power.dutyX100);
#if POWER_USE_HARDWARE_PWM
    Power_Pwm_Apply(s_buck_duty_x100, s_boost_duty_x100);
    Power_Pwm_Outputs_Enable();
#endif
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

static uint16_t Power_Control_Light_Load_Current_Threshold(uint16_t current_ref_ma)
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

static uint16_t Power_Control_Light_Load_Cv_Margin_Mv(void)
{
    if(s_preconnect_active != 0U) {
        return POWER_PRECONNECT_LIGHT_LOAD_MARGIN_MV;
    }

    return POWER_LIGHT_LOAD_CV_MARGIN_MV;
}

static uint8_t Power_Control_Light_Load_Near_Target(const bms_power_sample_t *sample,
                                                    uint16_t current_ref_ma,
                                                    uint16_t measured_current_ma)
{
    if(sample == 0) {
        return 0U;
    }

    if(s_power.mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER) {
        return 0U;
    }

    if(s_power.mode == (uint8_t)BMS_CHARGE_MODE_CC ||
       s_power.mode == (uint8_t)BMS_CHARGE_MODE_TRICKLE) {
        return (current_ref_ma <= POWER_LIGHT_LOAD_CURRENT_MAX_MA) ? 1U : 0U;
    }

    if(measured_current_ma >
       Power_Control_Light_Load_Current_Threshold(current_ref_ma)) {
        return 0U;
    }

    if((uint32_t)sample->outputVoltageMv + Power_Control_Light_Load_Cv_Margin_Mv() <
       (uint32_t)s_power.targetVoltageMv) {
        return 0U;
    }

    return 1U;
}

static uint16_t Power_Control_Light_Load_Duty_Max(const bms_power_sample_t *sample)
{
    uint32_t ideal_boost_duty;
    uint32_t target_for_limit_mv;

    if(sample == 0 ||
       sample->inputVoltageMv == 0U ||
       s_power.targetVoltageMv == 0U) {
        return POWER_LOOP_DUTY_MAX_X100;
    }

    target_for_limit_mv = (uint32_t)sample->outputVoltageMv + POWER_LIGHT_LOAD_VOUT_ADVANCE_MV;
    if(target_for_limit_mv > (uint32_t)s_power.targetVoltageMv) {
        target_for_limit_mv = (uint32_t)s_power.targetVoltageMv;
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

static uint16_t Power_Control_Preconnect_Duty_Max(const bms_power_sample_t *sample)
{
    uint32_t ideal_boost_duty;

    if(sample == 0 ||
       sample->inputVoltageMv == 0U ||
       s_power.targetVoltageMv == 0U) {
        return POWER_PRECONNECT_BOOST_DUTY_MAX_X100;
    }

    if((uint32_t)sample->inputVoltageMv >= (uint32_t)s_power.targetVoltageMv) {
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
        (((uint32_t)s_power.targetVoltageMv - (uint32_t)sample->inputVoltageMv) * 10000UL) /
        (uint32_t)s_power.targetVoltageMv;
    ideal_boost_duty += POWER_PRECONNECT_BOOST_HEADROOM_X100;
    if(ideal_boost_duty > POWER_PRECONNECT_BOOST_DUTY_MAX_X100) {
        ideal_boost_duty = POWER_PRECONNECT_BOOST_DUTY_MAX_X100;
    }

    return Clamp_U16((uint16_t)ideal_boost_duty,
                     POWER_DUTY_MIN_X100,
                     POWER_PRECONNECT_BOOST_DUTY_MAX_X100);
}

static uint16_t Power_Control_Preconnect_Duty_Min(const bms_power_sample_t *sample)
{
    uint32_t ideal_boost_duty;

    if(sample == 0 ||
       sample->inputVoltageMv == 0U ||
       s_power.targetVoltageMv == 0U) {
        return POWER_DUTY_MIN_X100;
    }

    if((uint32_t)sample->inputVoltageMv >= (uint32_t)s_power.targetVoltageMv) {
        return POWER_DUTY_MIN_X100;
    }

    if((uint32_t)sample->outputVoltageMv >=
       (uint32_t)s_power.targetVoltageMv + POWER_PRECONNECT_COAST_MARGIN_MV) {
        return POWER_DUTY_MIN_X100;
    }

    ideal_boost_duty =
        (((uint32_t)s_power.targetVoltageMv - (uint32_t)sample->inputVoltageMv) * 10000UL) /
        (uint32_t)s_power.targetVoltageMv;

    return Clamp_U16((uint16_t)ideal_boost_duty,
                     POWER_DUTY_MIN_X100,
                     POWER_PRECONNECT_BOOST_DUTY_MAX_X100);
}

static uint8_t Power_Control_Preconnect_Coast_Should_Run(const bms_power_sample_t *sample)
{
    if(sample == 0) {
        return 0U;
    }

    if(s_preconnect_active == 0U) {
        return 0U;
    }

    /*
     * Only enter the preconnect hold path on real overshoot. The hold path
     * keeps a minimum boost pulse alive so Vout does not collapse before
     * CHG/DSG can hand over.
     */
    if((uint32_t)sample->outputVoltageMv <=
       (uint32_t)s_power.targetVoltageMv + POWER_PRECONNECT_COAST_MARGIN_MV) {
        return 0U;
    }

    return 1U;
}

static void Power_Control_Preconnect_Coast(void)
{
    s_power.dutyX100 = POWER_DUTY_MIN_X100;
    s_buck_duty_x100 = POWER_DUTY_MAX_X100;
    s_boost_duty_x100 = POWER_DUTY_MIN_X100;
    s_power.powerStageMode = (uint8_t)POWER_STAGE_MODE_BOOST;
    s_async_boost_rectifier = 1U;
    Pi_Controller_Decay(&s_current_pi, 1U, 2U);
    Pi_Controller_Decay(&s_voltage_pi, 1U, 2U);
}

static uint8_t Power_Control_Afe_Handover_Active(void)
{
    return (s_afe_handover_guard_count != 0U) ? 1U : 0U;
}

static uint16_t Power_Control_Handover_Boost_Duty(const bms_power_sample_t *sample,
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

    if(sample == 0 || sample->inputVoltageMv == 0U || s_power.targetVoltageMv == 0U) {
        return Clamp_U16(duty, POWER_DUTY_MIN_X100, BMS_POWER_AFE_HANDOVER_BOOST_DUTY_MAX_X100);
    }

    vin_mv = sample->inputVoltageMv;
    target_mv = s_power.targetVoltageMv;
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

static void Power_Control_Afe_Handover_Start_Output(const bms_power_sample_t *sample,
                                                    uint16_t previous_boost_duty_x100)
{
    s_power.dutyX100 = Power_Control_Handover_Boost_Duty(sample, previous_boost_duty_x100);
    s_buck_duty_x100 = POWER_DUTY_MAX_X100;
    s_boost_duty_x100 = s_power.dutyX100;
    s_power.powerStageMode = (uint8_t)POWER_STAGE_MODE_BOOST;
    s_async_boost_rectifier = 1U;
}

static void Power_Control_Afe_Handover_Decay(void)
{
    if(s_afe_handover_guard_count > 0U) {
        s_afe_handover_guard_count--;
    }
}

static int32_t Power_Control_Limit_Afe_Handover_Duty(const bms_power_sample_t *sample,
                                                     int32_t duty)
{
    uint16_t duty_floor;

    if(Power_Control_Afe_Handover_Active() == 0U) {
        return duty;
    }

    duty_floor = Power_Control_Handover_Boost_Duty(sample, POWER_START_DUTY_X100);
    if(duty < (int32_t)duty_floor) {
        duty = (int32_t)duty_floor;
        Pi_Controller_Decay(&s_current_pi, 1U, 2U);
        Pi_Controller_Decay(&s_voltage_pi, 1U, 2U);
    }

    if(duty > (int32_t)BMS_POWER_AFE_HANDOVER_BOOST_DUTY_MAX_X100) {
        duty = (int32_t)BMS_POWER_AFE_HANDOVER_BOOST_DUTY_MAX_X100;
        Pi_Controller_Decay(&s_current_pi, 1U, 2U);
        Pi_Controller_Decay(&s_voltage_pi, 1U, 2U);
    }

    return duty;
}

static int32_t Power_Control_Limit_Preconnect_Duty(const bms_power_sample_t *sample,
                                                   int32_t duty)
{
    uint16_t duty_limit;
    uint16_t duty_floor;

    if(s_preconnect_active == 0U) {
        return duty;
    }

    duty_floor = Power_Control_Preconnect_Duty_Min(sample);
    if(duty < (int32_t)duty_floor) {
        duty = (int32_t)duty_floor;
        Pi_Controller_Decay(&s_current_pi, 3U, 4U);
        Pi_Controller_Decay(&s_voltage_pi, 3U, 4U);
    }

    duty_limit = Power_Control_Preconnect_Duty_Max(sample);
    if(duty > (int32_t)duty_limit) {
        duty = (int32_t)duty_limit;
        Pi_Controller_Decay(&s_current_pi, 3U, 4U);
        Pi_Controller_Decay(&s_voltage_pi, 3U, 4U);
    }

    return duty;
}

static uint32_t Power_Control_Cc_Duty_Target_Mv(const bms_power_sample_t *sample)
{
    uint32_t target_mv;
    uint32_t candidate_mv;

    target_mv = (uint32_t)s_power.targetVoltageMv;
    if(s_power.mode != (uint8_t)BMS_CHARGE_MODE_CC &&
       s_power.mode != (uint8_t)BMS_CHARGE_MODE_TRICKLE) {
        return target_mv;
    }

    if(s_power.mode == (uint8_t)BMS_CHARGE_MODE_CC) {
        return target_mv;
    }

    candidate_mv = 0UL;
    if(s_battery_voltage_feedback_valid != 0U &&
       s_battery_voltage_feedback_mv != 0U) {
        candidate_mv = (uint32_t)s_battery_voltage_feedback_mv +
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

static uint32_t Power_Control_Battery_Boost_Headroom_X100(void)
{
    if(s_power.mode == (uint8_t)BMS_CHARGE_MODE_CC &&
       s_async_boost_rectifier != 0U) {
        return (uint32_t)POWER_CC_ASYNC_BOOST_DUTY_HEADROOM_X100;
    }

    if((s_power.mode == (uint8_t)BMS_CHARGE_MODE_CC ||
        s_power.mode == (uint8_t)BMS_CHARGE_MODE_TRICKLE) &&
       s_power.targetCurrentMa <= POWER_LIGHT_LOAD_CURRENT_MAX_MA) {
        return (uint32_t)POWER_CC_LIGHT_LOAD_HEADROOM_X100;
    }

    return (uint32_t)BMS_POWER_BATTERY_BOOST_DUTY_HEADROOM_X100;
}

static uint16_t Power_Control_Battery_Boost_Duty_Max(const bms_power_sample_t *sample)
{
#if BMS_POWER_BATTERY_BOOST_DUTY_LIMIT_ENABLE
    uint32_t ideal_boost_duty;
    uint32_t vin_mv;
    uint32_t target_mv;

    if(sample == 0 ||
       sample->inputVoltageMv == 0U ||
       s_power.targetVoltageMv == 0U ||
       s_preconnect_active != 0U ||
       s_power.mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER) {
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

static int32_t Power_Control_Limit_Battery_Boost_Duty(const bms_power_sample_t *sample,
                                                      int32_t duty)
{
#if BMS_POWER_BATTERY_BOOST_DUTY_LIMIT_ENABLE
    uint16_t duty_limit;

    duty_limit = Power_Control_Battery_Boost_Duty_Max(sample);
    if(duty > (int32_t)duty_limit) {
        duty = (int32_t)duty_limit;
        Pi_Controller_Decay(&s_current_pi, 3U, 4U);
        Pi_Controller_Decay(&s_voltage_pi, 3U, 4U);
    }
#else
    (void)sample;
#endif

    return duty;
}

static uint8_t Power_Control_Input_Guard_Active(const bms_power_sample_t *sample)
{
    if(sample == 0) {
        return 0U;
    }

    if(s_power.targetVoltageMv < POWER_INPUT_GUARD_TARGET_MIN_MV) {
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

static void Power_Control_Reduce_Duty_For_Input_Guard(const bms_power_sample_t *sample)
{
    uint16_t measured_current_ma;

    if(s_power.dutyX100 > (uint16_t)(POWER_DUTY_MIN_X100 + POWER_INPUT_GUARD_DUTY_STEP_X100)) {
        s_power.dutyX100 =
            (uint16_t)(s_power.dutyX100 - POWER_INPUT_GUARD_DUTY_STEP_X100);
    } else {
        s_power.dutyX100 = POWER_DUTY_MIN_X100;
    }

    Pi_Controller_Decay(&s_current_pi, 3U, 4U);
    Pi_Controller_Decay(&s_voltage_pi, 3U, 4U);
    Power_Map_Control_To_Pwm(sample, s_power.dutyX100);
    measured_current_ma = Power_Control_Current_Feedback_Ma(sample);
    s_async_boost_rectifier =
        Power_Control_Async_Boost_Should_Run(s_soft_current_ma,
                                             measured_current_ma);
}

static int32_t Power_Control_Limit_Light_Load_Step(const bms_power_sample_t *sample,
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
        if(s_power.dutyX100 >= duty_limit) {
            step = 0;
        } else if((uint32_t)s_power.dutyX100 + (uint32_t)step > (uint32_t)duty_limit) {
            step = (int32_t)duty_limit - (int32_t)s_power.dutyX100;
        }
    }

    return step;
}

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

#if POWER_USE_HARDWARE_PWM
    Power_Pwm_Init();
#endif
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

#if POWER_USE_HARDWARE_PWM
    Power_Pwm_Outputs_Disable();
#endif
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

#if POWER_USE_HARDWARE_PWM
    if(s_fault_lockout == 0U && s_fault_status == 0U) {
        Power_Pwm_Apply(s_buck_duty_x100, s_boost_duty_x100);
        Power_Pwm_Outputs_Enable();
    } else {
        Power_Control_Stop();
    }
#endif
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
#if POWER_USE_HARDWARE_PWM
        Power_Pwm_Apply(s_buck_duty_x100, s_boost_duty_x100);
        Power_Pwm_Outputs_Enable();
#endif
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
#if POWER_USE_HARDWARE_PWM
        Power_Pwm_Outputs_Disable();
#endif
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
#if POWER_USE_HARDWARE_PWM
            Power_Pwm_Apply(s_buck_duty_x100, s_boost_duty_x100);
            Power_Pwm_Outputs_Enable();
#endif
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
#if POWER_USE_HARDWARE_PWM
            Power_Pwm_Apply(s_buck_duty_x100, s_boost_duty_x100);
            Power_Pwm_Outputs_Enable();
#endif
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
#if POWER_USE_HARDWARE_PWM
        Power_Pwm_Apply(s_buck_duty_x100, s_boost_duty_x100);
        Power_Pwm_Outputs_Enable();
#endif
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
#if POWER_USE_HARDWARE_PWM
        Power_Pwm_Apply(s_buck_duty_x100, s_boost_duty_x100);
        Power_Pwm_Outputs_Enable();
#endif
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

#if POWER_USE_HARDWARE_PWM
    Power_Pwm_Apply(s_buck_duty_x100, s_boost_duty_x100);
    Power_Pwm_Outputs_Enable();
#endif
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
#if POWER_USE_HARDWARE_PWM
    state->hardwareReady = s_pwm_ready;
    state->hardwareOutputsOn = s_pwm_outputs_on;
    state->periodTicks = s_pwm_period_ticks;
#else
    state->hardwareReady = 0U;
    state->hardwareOutputsOn = 0U;
    state->periodTicks = 0U;
#endif
}
