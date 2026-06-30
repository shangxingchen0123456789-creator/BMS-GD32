#ifndef POWER_PWM_H
#define POWER_PWM_H

#include <stdint.h>

typedef struct {
    uint8_t faultLockout;
    uint32_t faultStatus;
    uint8_t asyncBoostRectifier;
    uint8_t boostStageActive;
    uint16_t boostLowDutyX100;
    uint8_t preconnectActive;
} power_pwm_output_context_t;

typedef struct {
    uint8_t ready;
    uint8_t outputsOn;
    uint32_t periodTicks;
} power_pwm_state_t;

void Power_Pwm_Init(void);
void Power_Pwm_Apply(uint16_t buck_duty_x100,
                     uint16_t boost_low_duty_x100,
                     uint8_t fault_lockout);
void Power_Pwm_Outputs_Enable(const power_pwm_output_context_t *context);
void Power_Pwm_Outputs_Disable(void);
uint8_t Power_Pwm_Wait_Adc_Sample_Point(uint32_t timeout);
void Power_Pwm_Get_State(power_pwm_state_t *state);

#endif
