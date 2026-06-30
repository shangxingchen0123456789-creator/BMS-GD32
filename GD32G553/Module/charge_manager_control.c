#include "charge_manager_internal.h"

uint16_t Charge_Manager_Target_Current_For_State(bms_charge_state_t state)
{
    uint16_t target_current_ma;

    /* 娑撴祦鍜屾亽鍘嬮樁娈垫湰璐ㄤ笂閮芥槸鍚屼竴涓姛鐜囩骇鐨勯檺娴佸伐浣滄ā寮忋€?*/
    if(state == BMS_CHARGE_STATE_TRICKLE) {
        if(g_charge_manager.config.params.targetCurrentMa < TRICKLE_CURRENT_MA) {
            target_current_ma = g_charge_manager.config.params.targetCurrentMa;
        } else {
            target_current_ma = TRICKLE_CURRENT_MA;
        }
        return Power_Manager_Limit_Current(target_current_ma);
    }

    if(state == BMS_CHARGE_STATE_CV) {
        /*
         * 鎭掑帇闃舵浠嶇劧鎶婄洰鏍囩數娴佷綔涓烘渶澶у厑璁哥數娴佷氦缁欏姛鐜囩幆銆?
         * 鐪熸璁╃數娴佽嚜鐒朵笅闄嶇殑鏄數鍘嬬幆锛沜utoffCurrentMa 鍙敤浜庡垽鏂厖鐢靛畬鎴愩€?
         */
        return Power_Manager_Limit_Current(g_charge_manager.config.params.targetCurrentMa);
    }

    if(state == BMS_CHARGE_STATE_CC) {
        return Power_Manager_Limit_Current(g_charge_manager.config.params.targetCurrentMa);
    }

    return Power_Manager_Limit_Current(g_charge_manager.config.params.targetCurrentMa);
}

uint8_t Charge_Manager_Control_Mode_For_State(uint8_t mode, bms_charge_state_t state)
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

uint8_t Charge_Manager_Cv_Done_Allowed(const bms_power_sample_t *power_sample,
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

