#include "safety_manager.h"

#include "afe_gd30bm2016.h"
#include "bms_board_config.h"
#include "power_control.h"

/*
 * 最高优先级安全关断路径。
 *
 * ALERT 和 FAULT_OC 均可通过 EXTI 触发，ISR 内只做寄存器级快速关断：
 * - 关闭 HRTIMER/PWM；
 * - 拉高 DFETOFF，要求 BM2016 立即关闭放电 FET；
 * - 锁存故障位。
 *
 * BM2016 FET Control all-off 由最高优先级 safety task 在任务上下文补发，
 * 避免在中断里访问通信总线，同时把充电、放电、预充、预放路径都关断。
 */
static volatile uint32_t s_latched_faults;
static volatile uint8_t s_path_off_pending;
static volatile uint8_t s_afe_alert_monitor_disabled;
static volatile uint32_t s_last_trip_faults;
static volatile uint8_t s_last_trip_source;
static volatile uint8_t s_last_power_fault_pin_level;

static uint8_t Safety_Power_Fault_Monitor_Enabled(void)
{
    power_control_state_t state;

    Power_Control_Get_State(&state);
    return (state.enabled != 0U) ? 1U : 0U;
}

uint8_t Safety_Manager_Afe_Alert_Monitor_Enabled(void)
{
#if BMS_DIGITAL_POWER_AFELESS_DEBUG
    if(s_afe_alert_monitor_disabled != 0U) {
        return 0U;
    }
#endif

    return 1U;
}

void Safety_Manager_Set_Afe_Alert_Monitor(uint8_t enabled)
{
#if BMS_DIGITAL_POWER_AFELESS_DEBUG
    s_afe_alert_monitor_disabled = (enabled == 0U) ? 1U : 0U;
    if(enabled == 0U) {
        exti_interrupt_flag_clear(BMS_AFE_ALERT_EXTI_LINE);
    }
#else
    (void)enabled;
#endif
}

static uint8_t Safety_Read_Power_Fault_Pin(void)
{
#if BMS_ENABLE_POWER_FAULT_PIN
    return (RESET != gpio_input_bit_get(BMS_PWM_GPIO_PORT, BMS_PWM_FAULT_PIN)) ? 1U : 0U;
#else
    return 0U;
#endif
}

static void Safety_Latch_Faults(uint32_t faults, uint8_t source)
{
    uint32_t primask;

    if(faults == 0U) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if((faults & ~s_latched_faults) != 0U) {
        s_path_off_pending = 1U;
    }
    s_last_trip_faults = faults;
    s_last_trip_source = source;
    s_last_power_fault_pin_level = Safety_Read_Power_Fault_Pin();
    s_latched_faults |= faults;
    if(primask == 0U) {
        __enable_irq();
    }
}

static void Safety_Record_Diagnostic(uint32_t faults, uint8_t source)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    s_last_trip_faults = faults;
    s_last_trip_source = source;
    s_last_power_fault_pin_level = Safety_Read_Power_Fault_Pin();
    if(primask == 0U) {
        __enable_irq();
    }
}

static void Safety_Fast_Shutdown(void)
{
    Power_Control_Fault_Lockout();
    Afe_Gd30bm2016_Force_Path_Off_Fast();
}

static uint8_t Safety_Take_Path_Off_Pending(void)
{
    uint8_t pending;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    pending = s_path_off_pending;
    s_path_off_pending = 0U;
    if(primask == 0U) {
        __enable_irq();
    }

    return pending;
}

void Safety_Manager_Init(void)
{
    s_latched_faults = 0U;
    s_path_off_pending = 0U;
    s_last_trip_faults = 0U;
    s_last_trip_source = (uint8_t)SAFETY_TRIP_SOURCE_NONE;
    s_last_power_fault_pin_level = 0U;
#if BMS_DIGITAL_POWER_AFELESS_DEBUG
    s_afe_alert_monitor_disabled = 1U;
#else
    s_afe_alert_monitor_disabled = 0U;
#endif

    rcu_periph_clock_enable(RCU_SYSCFG);

    syscfg_exti_line_config(BMS_AFE_ALERT_EXTI_PORT_SOURCE, BMS_AFE_ALERT_EXTI_PIN_SOURCE);
    exti_init(BMS_AFE_ALERT_EXTI_LINE, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(BMS_AFE_ALERT_EXTI_LINE);

#if BMS_ENABLE_POWER_FAULT_PIN
    syscfg_exti_line_config(BMS_PWM_FAULT_EXTI_PORT_SOURCE, BMS_PWM_FAULT_EXTI_PIN_SOURCE);
    exti_init(BMS_PWM_FAULT_EXTI_LINE, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    exti_interrupt_flag_clear(BMS_PWM_FAULT_EXTI_LINE);
#endif

    nvic_irq_enable(EXTI10_15_IRQn, 0U, 0U);
}

uint32_t Safety_Manager_Sample_Fast_Faults(void)
{
    uint32_t faults;

    faults = 0U;
    faults |= Power_Control_Get_Fault_Status();

    if((Safety_Manager_Afe_Alert_Monitor_Enabled() != 0U) &&
       (Afe_Gd30bm2016_Alert_Active() != 0U)) {
        faults |= BMS_FAULT_AFE_PROTECTION;
    }

#if BMS_ENABLE_POWER_FAULT_PIN && BMS_POWER_FAULT_PIN_FAST_LATCH_ENABLE
    if((Safety_Power_Fault_Monitor_Enabled() != 0U) &&
       (RESET != gpio_input_bit_get(BMS_PWM_GPIO_PORT, BMS_PWM_FAULT_PIN))) {
        faults |= BMS_FAULT_CHARGE_OCP;
    }
#endif

    return faults;
}

void Safety_Manager_Service(void)
{
    uint32_t fast_faults;

    fast_faults = Safety_Manager_Sample_Fast_Faults();
    if(fast_faults != 0U) {
        Safety_Latch_Faults(fast_faults, (uint8_t)SAFETY_TRIP_SOURCE_SERVICE_FAST);
        Safety_Fast_Shutdown();
    }

    if(Safety_Take_Path_Off_Pending() != 0U) {
        Afe_Gd30bm2016_Fets_Off();
    }
}

void Safety_Manager_Report_Faults(uint32_t faults)
{
    if(faults == 0U) {
        return;
    }

    Safety_Latch_Faults(faults, (uint8_t)SAFETY_TRIP_SOURCE_REPORT);
    Safety_Fast_Shutdown();
    if(Safety_Take_Path_Off_Pending() != 0U) {
        Afe_Gd30bm2016_Fets_Off();
    }
}

void Safety_Manager_Clear_Latched_Faults(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    s_latched_faults = 0U;
    s_path_off_pending = 0U;
    s_last_trip_faults = 0U;
    s_last_trip_source = (uint8_t)SAFETY_TRIP_SOURCE_NONE;
    s_last_power_fault_pin_level = 0U;
    if(primask == 0U) {
        __enable_irq();
    }
}

uint32_t Safety_Manager_Get_Latched_Faults(void)
{
    return s_latched_faults;
}

void Safety_Manager_Get_Debug(safety_manager_debug_t *debug)
{
    if(debug == 0) {
        return;
    }

    debug->faults = s_last_trip_faults;
    debug->source = s_last_trip_source;
    debug->powerFaultPinLevel = s_last_power_fault_pin_level;
}

void Safety_Manager_Handle_External_Fault_Isr(uint32_t faults)
{
    if(faults == 0U) {
        return;
    }

    Safety_Latch_Faults(faults, (uint8_t)SAFETY_TRIP_SOURCE_EXTERNAL_ISR);
    Safety_Fast_Shutdown();
}

void Safety_Manager_Handle_Power_Fault_Isr(void)
{
#if BMS_ENABLE_POWER_FAULT_PIN
    if(Safety_Power_Fault_Monitor_Enabled() == 0U) {
        return;
    }
    Safety_Record_Diagnostic(0U, (uint8_t)SAFETY_TRIP_SOURCE_POWER_FAULT_ISR);
#if BMS_POWER_FAULT_PIN_FAST_LATCH_ENABLE
    if(Safety_Read_Power_Fault_Pin() == 0U) {
        return;
    }

    Safety_Latch_Faults(BMS_FAULT_CHARGE_OCP, (uint8_t)SAFETY_TRIP_SOURCE_POWER_FAULT_ISR);
    Safety_Fast_Shutdown();
#endif
#endif
}
