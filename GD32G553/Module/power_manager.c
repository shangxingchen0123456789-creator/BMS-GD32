#include "power_manager.h"

#include <string.h>

#define POWER_MANAGER_EFFICIENCY_X100         90U
#define POWER_MANAGER_INPUT_FOLDBACK_TARGET_MV 30000U
#define POWER_MANAGER_INPUT_FOLDBACK_VIN_MIN_MV 22000U
#define POWER_MANAGER_INPUT_FOLDBACK_VIN_RECOVER_MV 23000U
#define POWER_MANAGER_INPUT_FOLDBACK_IIN_MAX_MA 1800
#define POWER_MANAGER_INPUT_FOLDBACK_IIN_RECOVER_MA 1400
#define POWER_MANAGER_INPUT_FOLDBACK_DROP_MA  100U
#define POWER_MANAGER_INPUT_FOLDBACK_RECOVER_MA 25U
#define POWER_MANAGER_INPUT_FOLDBACK_FLOOR_MA 50U
#define POWER_MANAGER_CONVERSION_TARGET_MIN_MV 30000U
#define POWER_MANAGER_CONVERSION_IIN_MIN_MA  1000
#define POWER_MANAGER_CONVERSION_INPUT_MIN_MW 12000UL
#define POWER_MANAGER_CONVERSION_MIN_EFF_X100 35U

static bms_system_config_t s_config;
static power_manager_state_t s_state;
static uint16_t s_input_foldback_current_ma;
static uint8_t s_thermal_suspend_active;

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

static void Power_Manager_Clamp_Config(bms_system_config_t *config)
{
    if(config == 0) {
        return;
    }

    config->trickleCurrentMa =
        Power_Clamp_U16(config->trickleCurrentMa, 100U, 300U);
    config->minChargeCurrentMa =
        Power_Clamp_U16(config->minChargeCurrentMa, 100U, 200U);
}

void Power_Manager_Init(void)
{
    Param_Storage_Restore_Default(&s_config);
    (void)Param_Storage_Get_Config(&s_config);
    Power_Manager_Clamp_Config(&s_config);
    memset(&s_state, 0, sizeof(s_state));
    s_input_foldback_current_ma = 0U;
    s_thermal_suspend_active = 0U;
}

void Power_Manager_Configure(const bms_system_config_t *config)
{
    if(config == 0) {
        return;
    }

    s_config = *config;
    Power_Manager_Clamp_Config(&s_config);
}

void Power_Manager_Set_Input_Limit(uint16_t current_limit_ma, uint16_t power_limit_w)
{
    s_config.inputCurrentLimitMa = current_limit_ma;
    s_config.inputPowerLimitW = power_limit_w;
    s_input_foldback_current_ma = 0U;
}

static uint16_t Power_Manager_Calc_Input_Current_Limit(const bms_power_sample_t *sample,
                                                       const bms_charge_parameters_t *parameters)
{
    uint32_t power_mw;
    uint32_t power_limited_current_ma;
    uint32_t voltage_mv;

    if(sample == 0 || parameters == 0 || sample->inputVoltageMv == 0U) {
        return 0U;
    }

    power_mw = ((uint32_t)sample->inputVoltageMv * (uint32_t)s_config.inputCurrentLimitMa) / 1000U;
    power_mw = (power_mw * POWER_MANAGER_EFFICIENCY_X100) / 100U;

    if(s_config.inputPowerLimitW != 0U) {
        if(power_mw > ((uint32_t)s_config.inputPowerLimitW * 1000U)) {
            power_mw = (uint32_t)s_config.inputPowerLimitW * 1000U;
        }
    }

    voltage_mv = sample->outputVoltageMv;
    if(voltage_mv < parameters->targetVoltageMv) {
        voltage_mv = parameters->targetVoltageMv;
    }
    if(voltage_mv == 0U) {
        return 0U;
    }

    power_limited_current_ma = (power_mw * 1000U) / voltage_mv;
    if(power_limited_current_ma > 65535U) {
        power_limited_current_ma = 65535U;
    }

    s_state.availablePowerW = (uint16_t)(power_mw / 1000U);
    return (uint16_t)power_limited_current_ma;
}

static uint16_t Power_Manager_Calc_Thermal_Limit(uint16_t requested_current_ma,
                                                 const bms_power_sample_t *sample)
{
    int16_t temp_x10;
    uint32_t span;
    uint32_t over;
    uint32_t current_span;
    uint32_t limited;

    if(sample == 0) {
        return requested_current_ma;
    }

    temp_x10 = sample->mosTempX10;
    if(sample->inductorTempX10 != (int16_t)BMS_TEMP_UNAVAILABLE_X10 &&
       (temp_x10 == (int16_t)BMS_TEMP_UNAVAILABLE_X10 || sample->inductorTempX10 > temp_x10)) {
        temp_x10 = sample->inductorTempX10;
    }

    if(temp_x10 == (int16_t)BMS_TEMP_UNAVAILABLE_X10 ||
       temp_x10 <= s_config.derateStartTempX10) {
        s_state.deratingActive = 0U;
        s_thermal_suspend_active = 0U;
        return requested_current_ma;
    }

    s_state.deratingActive = 1U;
    if(temp_x10 >= s_config.derateStopTempX10) {
        s_thermal_suspend_active = 1U;
    }
    if(s_thermal_suspend_active != 0U) {
        return 0U;
    }

    span = (uint32_t)((int32_t)s_config.derateStopTempX10 - (int32_t)s_config.derateStartTempX10);
    over = (uint32_t)((int32_t)temp_x10 - (int32_t)s_config.derateStartTempX10);
    current_span = (requested_current_ma > s_config.minChargeCurrentMa) ?
                   ((uint32_t)requested_current_ma - (uint32_t)s_config.minChargeCurrentMa) :
                   0U;
    if(span == 0U) {
        return s_config.minChargeCurrentMa;
    }

    limited = (uint32_t)requested_current_ma - ((current_span * over) / span);
    return Power_Clamp_U16((uint16_t)limited, s_config.minChargeCurrentMa, requested_current_ma);
}

static uint8_t Power_Manager_Conversion_Stressed(const bms_power_sample_t *sample,
                                                 const bms_afe_data_t *afe,
                                                 const bms_charge_parameters_t *parameters)
{
    uint32_t input_power_mw;
    uint32_t output_power_mw;
    uint16_t measured_output_current_ma;

    if(sample == 0 || parameters == 0) {
        return 0U;
    }
    if(parameters->targetVoltageMv < POWER_MANAGER_CONVERSION_TARGET_MIN_MV) {
        return 0U;
    }
    if(sample->inputVoltageMv == 0U ||
       sample->outputVoltageMv == 0U ||
       sample->inputCurrentMa < POWER_MANAGER_CONVERSION_IIN_MIN_MA) {
        return 0U;
    }

    measured_output_current_ma =
        (sample->outputCurrentMa > 0) ? (uint16_t)sample->outputCurrentMa : 0U;
    input_power_mw =
        ((uint32_t)sample->inputVoltageMv * (uint32_t)sample->inputCurrentMa) / 1000UL;
    if(input_power_mw < POWER_MANAGER_CONVERSION_INPUT_MIN_MW) {
        return 0U;
    }

    if((afe != 0) && (afe->packVoltageMv != 0U) && (afe->batteryCurrentMa > 0)) {
        output_power_mw =
            ((uint32_t)afe->packVoltageMv * (uint32_t)afe->batteryCurrentMa) / 1000UL;
    } else {
        output_power_mw =
            ((uint32_t)sample->outputVoltageMv * (uint32_t)measured_output_current_ma) / 1000UL;
    }
    if((output_power_mw * 100UL) >=
       (input_power_mw * (uint32_t)POWER_MANAGER_CONVERSION_MIN_EFF_X100)) {
        return 0U;
    }

    return 1U;
}

static uint8_t Power_Manager_Input_Stressed(const bms_power_sample_t *sample,
                                            const bms_afe_data_t *afe,
                                            const bms_charge_parameters_t *parameters)
{
    if(sample == 0 || parameters == 0) {
        return 0U;
    }
    if(parameters->targetVoltageMv < POWER_MANAGER_INPUT_FOLDBACK_TARGET_MV) {
        return 0U;
    }
    if(sample->inputVoltageMv != 0U &&
       sample->inputVoltageMv < POWER_MANAGER_INPUT_FOLDBACK_VIN_MIN_MV) {
        return 1U;
    }
    if(sample->inputCurrentMa > POWER_MANAGER_INPUT_FOLDBACK_IIN_MAX_MA) {
        return 1U;
    }
    if(Power_Manager_Conversion_Stressed(sample, afe, parameters) != 0U) {
        return 1U;
    }

    return 0U;
}

static uint8_t Power_Manager_Input_Recovered(const bms_power_sample_t *sample,
                                             const bms_afe_data_t *afe,
                                             const bms_charge_parameters_t *parameters)
{
    if(sample == 0 || parameters == 0) {
        return 0U;
    }
    if(parameters->targetVoltageMv < POWER_MANAGER_INPUT_FOLDBACK_TARGET_MV) {
        return 1U;
    }
    if(sample->inputVoltageMv < POWER_MANAGER_INPUT_FOLDBACK_VIN_RECOVER_MV) {
        return 0U;
    }
    if(sample->inputCurrentMa > POWER_MANAGER_INPUT_FOLDBACK_IIN_RECOVER_MA) {
        return 0U;
    }
    if(Power_Manager_Conversion_Stressed(sample, afe, parameters) != 0U) {
        return 0U;
    }

    return 1U;
}

static uint16_t Power_Manager_Apply_Input_Foldback(uint16_t requested_ma,
                                                   uint16_t input_limit_ma,
                                                   const bms_afe_data_t *afe,
                                                   const bms_power_sample_t *sample,
                                                   const bms_charge_parameters_t *parameters)
{
    uint16_t ceiling_ma;
    uint16_t base_ma;

    if(requested_ma == 0U || sample == 0 || parameters == 0) {
        s_input_foldback_current_ma = 0U;
        return input_limit_ma;
    }

    ceiling_ma = requested_ma;
    if(input_limit_ma != 0U && input_limit_ma < ceiling_ma) {
        ceiling_ma = input_limit_ma;
    }

    if(Power_Manager_Input_Stressed(sample, afe, parameters) != 0U) {
        base_ma = s_state.limitedCurrentMa;
        if(base_ma == 0U || base_ma > ceiling_ma) {
            base_ma = ceiling_ma;
        }
        if(base_ma > POWER_MANAGER_INPUT_FOLDBACK_DROP_MA) {
            s_input_foldback_current_ma =
                (uint16_t)(base_ma - POWER_MANAGER_INPUT_FOLDBACK_DROP_MA);
        } else {
            s_input_foldback_current_ma = POWER_MANAGER_INPUT_FOLDBACK_FLOOR_MA;
        }
        if(s_input_foldback_current_ma < POWER_MANAGER_INPUT_FOLDBACK_FLOOR_MA) {
            s_input_foldback_current_ma = POWER_MANAGER_INPUT_FOLDBACK_FLOOR_MA;
        }
    } else if(Power_Manager_Input_Recovered(sample, afe, parameters) != 0U) {
        if(s_input_foldback_current_ma != 0U) {
            if((uint16_t)(ceiling_ma - s_input_foldback_current_ma) >
               POWER_MANAGER_INPUT_FOLDBACK_RECOVER_MA) {
                s_input_foldback_current_ma =
                    (uint16_t)(s_input_foldback_current_ma +
                               POWER_MANAGER_INPUT_FOLDBACK_RECOVER_MA);
            } else {
                s_input_foldback_current_ma = 0U;
            }
        }
    }

    if(s_input_foldback_current_ma != 0U &&
       (input_limit_ma == 0U || s_input_foldback_current_ma < input_limit_ma)) {
        return s_input_foldback_current_ma;
    }

    return input_limit_ma;
}

void Power_Manager_Update(uint32_t period_ms,
                          const bms_afe_data_t *afe,
                          const bms_power_sample_t *sample,
                          const bms_charge_parameters_t *parameters,
                          uint8_t charge_state,
                          uint32_t faults)
{
    uint16_t input_limit_ma;
    uint16_t thermal_limit_ma;
    uint16_t requested_ma;
    uint16_t target_ma;

    (void)period_ms;
    if(parameters == 0 || sample == 0) {
        return;
    }

    requested_ma = parameters->targetCurrentMa;
    if(charge_state == (uint8_t)BMS_CHARGE_STATE_TRICKLE) {
        requested_ma = (s_config.trickleCurrentMa < requested_ma) ? s_config.trickleCurrentMa : requested_ma;
    }

    if(faults != 0U ||
       charge_state == (uint8_t)BMS_CHARGE_STATE_IDLE ||
       charge_state == (uint8_t)BMS_CHARGE_STATE_DONE ||
       charge_state == (uint8_t)BMS_CHARGE_STATE_FAULT) {
        requested_ma = 0U;
    }

    if(charge_state == (uint8_t)BMS_CHARGE_STATE_CC) {
        /*
         * CC 是功率/发热最高的阶段，必须参与 MOS/电感温度降流：
         * 温度逼近时平滑回退充电电流，而不是一路全速直到 60°C 硬跳闸停充。
         * 输入限流仍交给 power_control 的 input guard 处理，这里只叠加热限制。
         */
        input_limit_ma = requested_ma;
        thermal_limit_ma = Power_Manager_Calc_Thermal_Limit(requested_ma, sample);
        target_ma = requested_ma;
        if(thermal_limit_ma < target_ma) {
            target_ma = thermal_limit_ma;
        }
    } else {
        input_limit_ma = Power_Manager_Calc_Input_Current_Limit(sample, parameters);
        input_limit_ma =
            Power_Manager_Apply_Input_Foldback(requested_ma, input_limit_ma, afe, sample, parameters);
        thermal_limit_ma = Power_Manager_Calc_Thermal_Limit(requested_ma, sample);
        target_ma = requested_ma;
        if(input_limit_ma != 0U && input_limit_ma < target_ma) {
            target_ma = input_limit_ma;
        }
        if(thermal_limit_ma < target_ma) {
            target_ma = thermal_limit_ma;
        }
    }

    /*
     * Power_Manager only calculates the allowed current ceiling. The actual
     * current soft-start is handled once in power_control.c, so CC does not
     * appear as a 100 mA target on the supervisor/debug path.
     */
    s_state.limitedCurrentMa = target_ma;

    s_state.requestedCurrentMa = requested_ma;
    s_state.inputLimitedCurrentMa = input_limit_ma;
    s_state.thermalLimitedCurrentMa = thermal_limit_ma;
}

uint16_t Power_Manager_Limit_Current(uint16_t requested_current_ma)
{
    if(s_state.limitedCurrentMa == 0U) {
        return 0U;
    }
    if(s_state.limitedCurrentMa < requested_current_ma) {
        return s_state.limitedCurrentMa;
    }
    return requested_current_ma;
}

void Power_Manager_Get_State(power_manager_state_t *state)
{
    if(state == 0) {
        return;
    }

    *state = s_state;
}
