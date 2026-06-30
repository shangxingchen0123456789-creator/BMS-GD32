#include "power_pwm.h"

#include "bms_board_config.h"
#include "system_gd32g5x3.h"

#define POWER_USE_HARDWARE_PWM                 BMS_ENABLE_HRTIMER_PWM
#define POWER_DUTY_MIN_X100                    BMS_PWM_DUTY_MIN_X100
#define POWER_DUTY_MAX_X100                    BMS_PWM_DUTY_MAX_X100
#define POWER_PWM_COMPARE_GUARD_TICKS          3U
#define POWER_PWM_DEADTIME_CLOCK_MUL           32U

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

static uint16_t Power_Clamp_U16(uint16_t value, uint16_t min_value, uint16_t max_value)
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
    duty_x100 = Power_Clamp_U16(duty_x100, POWER_DUTY_MIN_X100, POWER_DUTY_MAX_X100);
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

    return Power_Clamp_U16((uint16_t)(10000U - low_duty_x100), POWER_DUTY_MIN_X100, POWER_DUTY_MAX_X100);
}

void Power_Pwm_Apply(uint16_t buck_duty_x100,
                     uint16_t boost_low_duty_x100,
                     uint8_t fault_lockout)
{
    uint16_t buck_high_duty;
    uint16_t boost_high_duty;
    uint32_t buck_compare;
    uint32_t boost_compare;
    uint32_t buck_low_set_compare;
    uint32_t boost_low_set_compare;

    if(s_pwm_ready == 0U || fault_lockout != 0U) {
        return;
    }

    /*
     * HRTIMER 输出配置中 duty 表示“高边导通比例”。
     * Buck 半桥直接使用 buck_duty_x100；Boost 半桥的控制量表示低边
     * 储能占空比，因此这里取互补值写入高边比较寄存器。
     */
    buck_high_duty = Power_Clamp_U16(buck_duty_x100, POWER_DUTY_MIN_X100, POWER_DUTY_MAX_X100);
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

static uint32_t Power_Pwm_Active_Output_Channels(const power_pwm_output_context_t *context)
{
    uint32_t channels;

    channels = BMS_PWM_OUTPUT_CHANNELS;
    if(context == 0) {
        return 0U;
    }

    if((context->asyncBoostRectifier != 0U) &&
       (context->boostStageActive != 0U) &&
       (context->boostLowDutyX100 == 0U)) {
        if(context->preconnectActive == 0U) {
            return 0U;
        }
    }

    if(context->asyncBoostRectifier != 0U) {
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

static void Power_Pwm_Clear_Recovered_Fault(const power_pwm_output_context_t *context)
{
#if BMS_ENABLE_POWER_FAULT_PIN
    if(context == 0) {
        return;
    }

    if((s_pwm_ready == 0U) || (context->faultLockout != 0U) || (context->faultStatus != 0U)) {
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

void Power_Pwm_Outputs_Enable(const power_pwm_output_context_t *context)
{
    uint32_t active_channels;
    uint32_t inactive_channels;

    /* 初始化完成后才允许打开输出，防止 GPIO 复用未就绪时出现毛刺。 */
    if(context == 0) {
        return;
    }

    if((s_pwm_ready != 0U) && (context->faultLockout == 0U)) {
        Power_Pwm_Clear_Recovered_Fault(context);
    }
    if((s_pwm_ready != 0U) && (context->faultLockout == 0U)) {
        active_channels = Power_Pwm_Active_Output_Channels(context);
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

void Power_Pwm_Outputs_Disable(void)
{
    /* 关闭输出只影响引脚驱动，HRTIMER 计数器仍可继续运行，便于快速恢复。 */
    if(s_pwm_ready != 0U) {
        hrtimer_output_channel_disable(BMS_HRTIMER_PERIPH, BMS_PWM_OUTPUT_CHANNELS);
        s_pwm_outputs_on = 0U;
    }
}

uint8_t Power_Pwm_Wait_Adc_Sample_Point(uint32_t timeout)
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

void Power_Pwm_Init(void)
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
void Power_Pwm_Init(void)
{
}

void Power_Pwm_Apply(uint16_t buck_duty_x100,
                     uint16_t boost_low_duty_x100,
                     uint8_t fault_lockout)
{
    (void)buck_duty_x100;
    (void)boost_low_duty_x100;
    (void)fault_lockout;
}

void Power_Pwm_Outputs_Enable(const power_pwm_output_context_t *context)
{
    (void)context;
}

void Power_Pwm_Outputs_Disable(void)
{
}

uint8_t Power_Pwm_Wait_Adc_Sample_Point(uint32_t timeout)
{
    (void)timeout;
    return 0U;
}
#endif

void Power_Pwm_Get_State(power_pwm_state_t *state)
{
    if(state == 0) {
        return;
    }

#if POWER_USE_HARDWARE_PWM
    state->ready = s_pwm_ready;
    state->outputsOn = s_pwm_outputs_on;
    state->periodTicks = s_pwm_period_ticks;
#else
    state->ready = 0U;
    state->outputsOn = 0U;
    state->periodTicks = 0U;
#endif
}
