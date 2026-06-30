#include "board_support.h"

#include "FreeRTOS.h"
#include "task.h"
#include "gd32g553q_eval.h"

/* 板级辅助函数集中在这里，避免应用层直接依赖评估板命名。 */
static void Board_Support_Led_Init(uint32_t gpio_periph, uint32_t pin, rcu_periph_enum clock)
{
    rcu_periph_clock_enable(clock);
    gpio_mode_set(gpio_periph, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, pin);
    gpio_output_options_set(gpio_periph, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, pin);
    GPIO_BC(gpio_periph) = pin;
}

void Board_Support_Init(void)
{
    /* 调试阶段用评估板 LED 指示心跳和故障状态。 */
    Board_Support_Led_Init(LED1_GPIO_PORT, LED1_PIN, LED1_GPIO_CLK);
    Board_Support_Led_Init(LED2_GPIO_PORT, LED2_PIN, LED2_GPIO_CLK);
    Board_Support_Led_Init(LED3_GPIO_PORT, LED3_PIN, LED3_GPIO_CLK);
    Board_Support_Led_Init(LED4_GPIO_PORT, LED4_PIN, LED4_GPIO_CLK);
}

uint32_t Board_Support_Millis(void)
{
    /* 协议时间戳统一使用 FreeRTOS tick 换算出的毫秒值。 */
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

void Board_Support_Led_Toggle(void)
{
    GPIO_TG(LED1_GPIO_PORT) = LED1_PIN;
}

void Board_Support_Fault_Led(uint8_t on)
{
    if(on != 0U) {
        GPIO_BOP(LED2_GPIO_PORT) = LED2_PIN;
    } else {
        GPIO_BC(LED2_GPIO_PORT) = LED2_PIN;
    }
}
