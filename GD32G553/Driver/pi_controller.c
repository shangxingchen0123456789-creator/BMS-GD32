#include "pi_controller.h"

static int32_t Pi_Clamp_I32(int32_t value, int32_t min_value, int32_t max_value)
{
    if(value < min_value) {
        return min_value;
    }
    if(value > max_value) {
        return max_value;
    }
    return value;
}

void Pi_Controller_Init(pi_controller_t *controller, const pi_controller_config_t *config)
{
    if(controller == 0 || config == 0) {
        return;
    }

    controller->config = *config;
    controller->integral = 0;
}

void Pi_Controller_Reset(pi_controller_t *controller)
{
    if(controller == 0) {
        return;
    }

    controller->integral = 0;
}

int32_t Pi_Controller_Update(pi_controller_t *controller, int32_t error)
{
    int32_t kp_div;
    int32_t ki_div;
    int32_t step;

    if(controller == 0) {
        return 0;
    }

    kp_div = controller->config.kpDiv;
    ki_div = controller->config.kiDiv;
    if(kp_div == 0) {
        kp_div = 1;
    }
    if(ki_div == 0) {
        ki_div = 1;
    }

    controller->integral = Pi_Clamp_I32(controller->integral + error,
                                        -controller->config.integralLimit,
                                        controller->config.integralLimit);

    step = (error / kp_div) + (controller->integral / ki_div);
    return Pi_Clamp_I32(step, -controller->config.stepLimit, controller->config.stepLimit);
}

void Pi_Controller_Decay(pi_controller_t *controller, uint8_t keep_num, uint8_t keep_den)
{
    if(controller == 0 || keep_den == 0U) {
        return;
    }

    controller->integral = (controller->integral * (int32_t)keep_num) / (int32_t)keep_den;
}
