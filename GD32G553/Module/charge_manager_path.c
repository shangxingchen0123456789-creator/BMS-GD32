#include "charge_manager_internal.h"

uint16_t Charge_Manager_Calc_Preconnect_Target_Mv(const bms_afe_data_t *afe,
                                                         const bms_charge_parameters_t *params)
{
    uint32_t target_mv;

    if(params == 0) {
        return 0U;
    }

    (void)params;
    if((afe == 0) || (afe->packVoltageMv == 0U)) {
        return 0U;
    }

    /*
     * 24V 杈撳叆鏂规涓嬶紝棰勮繛鎺ョ洰鏍囧彧鐣ラ珮浜?Pack锛屽苟淇濇寔鍦ㄤ富 FET 闂悎纭
     * 绐楀彛鍐呫€傝繖鏍?Boost 鑳芥妸 Vout 鎺ㄥ埌鐢垫睜鐢靛帇闄勮繎锛屽悓鏃堕伩鍏嶇洰鏍囨湰韬?
     * 楂樺嚭 Pack 瓒呰繃 |Vout-Pack| < 50 mV 鐨?handover 鍒ゆ嵁銆?
     */
    target_mv = (uint32_t)afe->packVoltageMv +
                (uint32_t)BMS_PATH_PRECONNECT_TARGET_ABOVE_PACK_MV;
    if(target_mv > (uint32_t)BMS_BATTERY_FULL_CHARGE_VOLTAGE_MV) {
        target_mv = (uint32_t)BMS_BATTERY_FULL_CHARGE_VOLTAGE_MV;
    }

    return (uint16_t)target_mv;
}

uint16_t Charge_Manager_Preconnect_Target_Mv(const bms_afe_data_t *afe,
                                                    const bms_power_sample_t *power_sample,
                                                    const bms_charge_parameters_t *params)
{
    uint16_t final_target_mv;

    (void)power_sample;
    final_target_mv = Charge_Manager_Calc_Preconnect_Target_Mv(afe, params);
    if(final_target_mv == 0U) {
        return 0U;
    }

    taskENTER_CRITICAL();
    s_preconnect_target_mv = final_target_mv;
    taskEXIT_CRITICAL();

    return final_target_mv;
}

uint16_t Charge_Manager_Preconnect_Current_Limit_Ma(const bms_afe_data_t *afe,
                                                           const bms_power_sample_t *power_sample,
                                                           uint32_t period_ms)
{
    (void)afe;
    (void)power_sample;
    (void)period_ms;
    return BMS_PATH_PRECONNECT_CURRENT_MA;
}

uint8_t Charge_Manager_Preconnect_Can_Drive(const bms_afe_data_t *afe,
                                                   const bms_power_sample_t *power_sample)
{
    if((afe == 0) || (power_sample == 0) || (afe->packVoltageMv == 0U)) {
        return 0U;
    }

    /*
     * Keep the preconnect power stage alive even when Vout is above the close
     * window. The fast loop will coast instead of driving higher; stopping here
     * clears preconnectActive and lets the output collapse before the path can
     * collect enough READY samples.
     */
    return 1U;
}

uint8_t Charge_Manager_Cv_Entry_Ready(const bms_afe_data_t *afe,
                                             const bms_power_sample_t *power_sample,
                                             const bms_charge_parameters_t *params)
{
    if((afe == 0) || (power_sample == 0) || (params == 0) ||
       (params->targetVoltageMv == 0U)) {
        return 0U;
    }
    (void)power_sample;

    if(afe->packVoltageMv == 0U) {
        return 0U;
    }

    if((uint32_t)afe->packVoltageMv + CV_ENTRY_MARGIN_MV >=
       (uint32_t)params->targetVoltageMv) {
        return 1U;
    }

    return 0U;
}

void Charge_Manager_Clear_Manual_Fet(void)
{
    s_manual_fet_active = 0U;
    s_manual_fet_mask = 0U;
}

void Charge_Manager_Clear_Path_Settle_State_Unlocked(void)
{
    s_path_settle_remaining_ms = 0U;
    s_last_battery_path_enabled = 0U;
    s_path_settle_target_mv = 0U;
}

void Charge_Manager_Clear_Path_Settle_Unlocked(void)
{
    Charge_Manager_Clear_Path_Settle_State_Unlocked();
    s_preconnect_target_mv = 0U;
}

void Charge_Manager_Clear_Path_Settle(void)
{
    taskENTER_CRITICAL();
    Charge_Manager_Clear_Path_Settle_Unlocked();
    taskEXIT_CRITICAL();
}

void Charge_Manager_Clear_Path_Settle_Only(void)
{
    taskENTER_CRITICAL();
    Charge_Manager_Clear_Path_Settle_State_Unlocked();
    taskEXIT_CRITICAL();
}

uint8_t Charge_Manager_Path_Settle_Active(void)
{
    uint8_t active;

    taskENTER_CRITICAL();
    active = (s_path_settle_remaining_ms != 0U) ? 1U : 0U;
    taskEXIT_CRITICAL();

    return active;
}

uint8_t Charge_Manager_Battery_Path_Rising_Edge(void)
{
    uint8_t rising_edge;

    taskENTER_CRITICAL();
    rising_edge = (s_last_battery_path_enabled == 0U) ? 1U : 0U;
    taskEXIT_CRITICAL();

    return rising_edge;
}

void Charge_Manager_Update_Path_Settle(uint8_t battery_path_enabled,
                                              const bms_afe_data_t *afe,
                                              const bms_charge_parameters_t *params)
{
    uint16_t target_mv;

    if(battery_path_enabled == 0U) {
        Charge_Manager_Clear_Path_Settle_Only();
        return;
    }

    target_mv = Charge_Manager_Preconnect_Target_Mv(afe, 0, params);

    taskENTER_CRITICAL();
    if(s_last_battery_path_enabled == 0U) {
        s_path_settle_remaining_ms = CHARGE_PATH_SETTLE_MS;
        s_path_settle_target_mv = target_mv;
    }
    s_last_battery_path_enabled = 1U;
    taskEXIT_CRITICAL();
}

void Charge_Manager_Decay_Path_Settle(uint32_t period_ms)
{
    if(period_ms == 0U) {
        period_ms = 1U;
    }

    taskENTER_CRITICAL();
    if(s_path_settle_remaining_ms != 0U) {
        if(period_ms >= (uint32_t)s_path_settle_remaining_ms) {
            s_path_settle_remaining_ms = 0U;
            s_path_settle_target_mv = 0U;
            s_preconnect_target_mv = 0U;
        } else {
            s_path_settle_remaining_ms =
                (uint16_t)((uint32_t)s_path_settle_remaining_ms - period_ms);
        }
    }
    taskEXIT_CRITICAL();
}

