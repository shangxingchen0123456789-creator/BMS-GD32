#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

#include <stdint.h>

typedef struct {
    int32_t kpDiv;
    int32_t kiDiv;
    int32_t integralLimit;
    int32_t stepLimit;
} pi_controller_config_t;

typedef struct {
    pi_controller_config_t config;
    int32_t integral;
} pi_controller_t;

void Pi_Controller_Init(pi_controller_t *controller, const pi_controller_config_t *config);
void Pi_Controller_Reset(pi_controller_t *controller);
int32_t Pi_Controller_Update(pi_controller_t *controller, int32_t error);
void Pi_Controller_Decay(pi_controller_t *controller, uint8_t keep_num, uint8_t keep_den);

#endif
