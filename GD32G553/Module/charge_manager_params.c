#include "charge_manager_internal.h"

bms_command_reply_t Reply_Ok(void)
{
    bms_command_reply_t reply;

    reply.result = (uint8_t)BMS_CMD_RESULT_OK;
    reply.errorCode = (uint8_t)BMS_CMD_ERROR_NONE;
    return reply;
}

bms_command_reply_t Reply_Error(uint8_t error_code)
{
    bms_command_reply_t reply;

    reply.result = (uint8_t)BMS_CMD_RESULT_ERROR;
    reply.errorCode = error_code;
    return reply;
}

void Charge_Manager_Default_Parameters(bms_charge_parameters_t *parameters)
{
    /* 榛樿鍙傛暟瀵瑰簲 bms_types.h 涓殑 9 涓查攤鐢垫睜鏂规銆?*/
    parameters->targetVoltageMv = BMS_DEFAULT_TARGET_VOLTAGE_MV;
    parameters->targetCurrentMa = BMS_DEFAULT_TARGET_CURRENT_MA;
    parameters->cutoffCurrentMa = BMS_DEFAULT_CUTOFF_CURRENT_MA;
    parameters->cellOvpMv = BMS_DEFAULT_CELL_OVP_MV;
    parameters->cellUvpMv = BMS_DEFAULT_CELL_UVP_MV;
    parameters->tempOtpX10 = BMS_DEFAULT_TEMP_OTP_X10;
    parameters->balanceDeltaMv = BMS_DEFAULT_BALANCE_DELTA_MV;
}

void Charge_Manager_Clamp_Parameters(bms_charge_parameters_t *parameters)
{
    if(parameters == 0) {
        return;
    }

    if(parameters->targetCurrentMa < 600U) {
        parameters->targetCurrentMa = 600U;
    } else if(parameters->targetCurrentMa > 1000U) {
        parameters->targetCurrentMa = 1000U;
    }
    if(parameters->cutoffCurrentMa < 100U) {
        parameters->cutoffCurrentMa = 100U;
    } else if(parameters->cutoffCurrentMa > 200U) {
        parameters->cutoffCurrentMa = 200U;
    }
    if(parameters->cutoffCurrentMa >= parameters->targetCurrentMa) {
        parameters->cutoffCurrentMa = 150U;
    }
    if(parameters->targetVoltageMv > BMS_BATTERY_FULL_CHARGE_VOLTAGE_MV) {
        parameters->targetVoltageMv = BMS_BATTERY_FULL_CHARGE_VOLTAGE_MV;
    }
}

uint8_t Charge_Manager_Validate_Parameters(const bms_charge_parameters_t *parameters)
{
    uint32_t pack_limit_mv;

    if(parameters == 0) {
        return 0U;
    }

    pack_limit_mv = (uint32_t)parameters->cellOvpMv * BMS_CELL_COUNT;

    /*
     * 鍗充娇涓婁綅鏈哄凡缁忓仛浜嗘牎楠岋紝鍥轰欢浠嶇劧瑕佸啀娆￠檺鍒跺弬鏁拌寖鍥淬€?
     * 鍥轰欢鏄槻姝㈠嵄闄╁懡浠ゆ垨鎹熷潖甯ц繘鍏ュ姛鐜囩骇鐨勬渶鍚庝竴閬撲繚鎶ゃ€?
     */
    if(parameters->targetVoltageMv < 25000U ||
       parameters->targetVoltageMv > BMS_BATTERY_FULL_CHARGE_VOLTAGE_MV ||
       parameters->targetVoltageMv > pack_limit_mv) {
        return 0U;
    }
    if(parameters->targetCurrentMa < 600U || parameters->targetCurrentMa > 1000U) {
        return 0U;
    }
    if(parameters->cutoffCurrentMa < 100U ||
       parameters->cutoffCurrentMa > 200U ||
       parameters->cutoffCurrentMa > parameters->targetCurrentMa) {
        return 0U;
    }
    if(parameters->cellOvpMv < 4100U || parameters->cellOvpMv > 4300U) {
        return 0U;
    }
    if(parameters->cellUvpMv < 2500U || parameters->cellUvpMv > 3300U) {
        return 0U;
    }
    if(parameters->tempOtpX10 < 450 || parameters->tempOtpX10 > 800) {
        return 0U;
    }
    if(parameters->balanceDeltaMv < 5U || parameters->balanceDeltaMv > 200U) {
        return 0U;
    }

    return 1U;
}

uint8_t Charge_Manager_Validate_Digital_Power(uint16_t target_voltage_mv,
                                                     uint16_t current_limit_ma)
{
    if(target_voltage_mv < BMS_DIGITAL_POWER_MIN_OUTPUT_MV ||
       target_voltage_mv > BMS_DIGITAL_POWER_MAX_OUTPUT_MV) {
        return 0U;
    }
    if(current_limit_ma < BMS_DIGITAL_POWER_MIN_CURRENT_MA ||
       current_limit_ma > BMS_DIGITAL_POWER_MAX_CURRENT_MA) {
        return 0U;
    }

    return 1U;
}

