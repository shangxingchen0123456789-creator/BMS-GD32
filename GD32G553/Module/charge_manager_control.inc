/* Internal implementation fragment for charge_manager.c only. */

static uint16_t Charge_Manager_Target_Current_For_State(bms_charge_state_t state)
{
    uint16_t target_current_ma;

    /* 涓流和恒压阶段本质上都是同一个功率级的限流工作模式。 */
    if(state == BMS_CHARGE_STATE_TRICKLE) {
        if(s_params.targetCurrentMa < TRICKLE_CURRENT_MA) {
            target_current_ma = s_params.targetCurrentMa;
        } else {
            target_current_ma = TRICKLE_CURRENT_MA;
        }
        return Power_Manager_Limit_Current(target_current_ma);
    }

    if(state == BMS_CHARGE_STATE_CV) {
        /*
         * 恒压阶段仍然把目标电流作为最大允许电流交给功率环。
         * 真正让电流自然下降的是电压环；cutoffCurrentMa 只用于判断充电完成。
         */
        return Power_Manager_Limit_Current(s_params.targetCurrentMa);
    }

    if(state == BMS_CHARGE_STATE_CC) {
        return Power_Manager_Limit_Current(s_params.targetCurrentMa);
    }

    return Power_Manager_Limit_Current(s_params.targetCurrentMa);
}

static uint8_t Charge_Manager_Control_Mode_For_State(uint8_t mode, bms_charge_state_t state)
{
    if(mode != (uint8_t)BMS_CHARGE_MODE_AUTO) {
        return mode;
    }

    if(state == BMS_CHARGE_STATE_CV) {
        return (uint8_t)BMS_CHARGE_MODE_CV;
    }

    if(state == BMS_CHARGE_STATE_TRICKLE) {
        return (uint8_t)BMS_CHARGE_MODE_TRICKLE;
    }

    return (uint8_t)BMS_CHARGE_MODE_CC;
}

static uint8_t Charge_Manager_Cv_Done_Allowed(const bms_power_sample_t *power_sample,
                                               const bms_charge_parameters_t *params)
{
    power_path_manager_state_t path_state;
    power_manager_state_t power_state;

    if((power_sample == 0) || (params == 0) || (params->targetVoltageMv == 0U)) {
        return 0U;
    }

    Power_Path_Manager_Get_State(&path_state);
    if(path_state.batteryPathEnabled == 0U) {
        return 0U;
    }

    if(Charge_Manager_Path_Settle_Active() != 0U) {
        return 0U;
    }

    if(((uint32_t)power_sample->outputVoltageMv + CV_ENTRY_MARGIN_MV) <
       (uint32_t)params->targetVoltageMv) {
        return 0U;
    }

    memset(&power_state, 0, sizeof(power_state));
    Power_Manager_Get_State(&power_state);
    if(power_state.deratingActive != 0U) {
        return 0U;
    }
    if((power_state.requestedCurrentMa != 0U) &&
       (power_state.limitedCurrentMa != 0U) &&
       ((uint32_t)power_state.limitedCurrentMa + CV_DONE_DPM_MARGIN_MA <
        (uint32_t)power_state.requestedCurrentMa)) {
        return 0U;
    }

    return 1U;
}

