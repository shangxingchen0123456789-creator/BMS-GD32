#include "power_path_manager.h"

#include "afe_gd30bm2016.h"
#include "bms_board_config.h"

#define POWER_PATH_FET_ON_FAILED_HOLD_COUNT 20U
#if BMS_PATH_HARDWARE_PRECONNECT_USE_PCHG
#define POWER_PATH_HARDWARE_PRECONNECT_PCHG_MASK AFE_GD30BM2016_FET_STATUS_PCHG
#else
#define POWER_PATH_HARDWARE_PRECONNECT_PCHG_MASK 0U
#endif
#if BMS_PATH_HARDWARE_PRECONNECT_USE_PDSG
#define POWER_PATH_HARDWARE_PRECONNECT_PDSG_MASK AFE_GD30BM2016_FET_STATUS_PDSG
#else
#define POWER_PATH_HARDWARE_PRECONNECT_PDSG_MASK 0U
#endif
#if BMS_PATH_HARDWARE_PRECONNECT_USE_DSG
#define POWER_PATH_HARDWARE_PRECONNECT_DSG_MASK AFE_GD30BM2016_FET_STATUS_DSG
#else
#define POWER_PATH_HARDWARE_PRECONNECT_DSG_MASK 0U
#endif
#define POWER_PATH_HARDWARE_PRECONNECT_FET_MASK (POWER_PATH_HARDWARE_PRECONNECT_PCHG_MASK | \
                                                 POWER_PATH_HARDWARE_PRECONNECT_PDSG_MASK | \
                                                 POWER_PATH_HARDWARE_PRECONNECT_DSG_MASK)

/*
 * 电池充/放电路径管理。
 *
 * 主控网表中 Q4/Q5 所在路径由 BM2016 的 FET/PCHG/PDSG 相关引脚参与控制。
 * 本模块只决策“电池路径是否允许接通”，实际预充、预放和 FET 关断命令
 * 由 afe_gd30bm2016.c 下发给 BM2016。
 */
static uint8_t s_external_power_present;
static uint8_t s_battery_path_enabled;
static uint8_t s_path_off_commanded;
static uint8_t s_preconnect_ready;
static uint8_t s_preconnect_confirm_count;
static uint8_t s_preconnect_flags;
static uint8_t s_preconnect_reason;
static uint8_t s_preconnect_fets_active;
static uint8_t s_preconnect_fet_mask;
static uint16_t s_preconnect_delta_mv;
static uint16_t s_preconnect_allowed_delta_mv;
static uint8_t s_fet_on_failed_hold_count;

static uint8_t Path_External_Power_Present(uint16_t input_voltage_mv)
{
    uint16_t threshold_mv;

    threshold_mv = BMS_POWER_EXTERNAL_PRESENT_MV;
    if(s_external_power_present != 0U) {
        if(threshold_mv > BMS_POWER_EXTERNAL_HYSTERESIS_MV) {
            threshold_mv = (uint16_t)(threshold_mv - BMS_POWER_EXTERNAL_HYSTERESIS_MV);
        }
    }

    if(input_voltage_mv >= threshold_mv) {
        s_external_power_present = 1U;
    } else {
        s_external_power_present = 0U;
    }

    return s_external_power_present;
}

static uint16_t Path_Abs_Diff_U16(uint16_t a, uint16_t b)
{
    return (a >= b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

static void Path_Mark_Fet_On_Failed(void)
{
    Afe_Gd30bm2016_Force_Path_Off_Fast();
    s_battery_path_enabled = 0U;
    s_preconnect_fets_active = 0U;
    s_preconnect_fet_mask = 0U;
    s_path_off_commanded = 1U;
    s_preconnect_ready = 0U;
    s_preconnect_confirm_count = 0U;
    s_preconnect_flags &= (uint8_t)~POWER_PATH_PRECONNECT_FLAG_READY;
    s_preconnect_reason = POWER_PATH_PRECONNECT_REASON_FET_ON_FAILED;
    s_fet_on_failed_hold_count = POWER_PATH_FET_ON_FAILED_HOLD_COUNT;
}

static void Path_Preconnect_Reset_Debug(uint8_t reason)
{
    s_preconnect_ready = 0U;
    s_preconnect_confirm_count = 0U;
    s_preconnect_flags = (s_preconnect_fets_active != 0U) ?
                         POWER_PATH_PRECONNECT_FLAG_ACTIVE : 0U;
    s_preconnect_delta_mv = 0U;
    s_preconnect_allowed_delta_mv = BMS_PATH_PRECONNECT_TARGET_DELTA_MV;
    s_preconnect_reason = reason;
    s_fet_on_failed_hold_count = 0U;
}

static uint8_t Path_Preconnect_Ready(const bms_afe_data_t *afe,
                                     const bms_power_sample_t *power_sample)
{
    uint16_t delta_mv;
    uint32_t close_threshold_mv;
    uint16_t target_mv;

    s_preconnect_flags = (s_preconnect_fets_active != 0U) ?
                         POWER_PATH_PRECONNECT_FLAG_ACTIVE : 0U;
    s_preconnect_delta_mv = 0U;
    s_preconnect_allowed_delta_mv = 0U;

    if((afe == 0) || (power_sample == 0) ||
       (afe->packVoltageMv == 0U)) {
        Path_Preconnect_Reset_Debug(POWER_PATH_PRECONNECT_REASON_NO_SAMPLE);
        return 0U;
    }

    target_mv = afe->packVoltageMv;
    delta_mv = Path_Abs_Diff_U16(power_sample->outputVoltageMv, target_mv);
    s_preconnect_delta_mv = delta_mv;
    s_preconnect_allowed_delta_mv = BMS_PATH_PRECONNECT_DELTA_MV;

    /*
     * Vout 达到电池电压附近（Pack-50 mV）即闭合主 FET，让电池立刻把输出钳在
     * Pack，避免无电池钳位时 Boost 在轻载下过冲撞硬 OVP。下沿保留 50 mV 防止
     * Vout 明显低于 Pack 时闭合产生反灌；Vout 偏高时同样闭合，由电池吸收拉回。
     */
    close_threshold_mv = (uint32_t)target_mv - (uint32_t)BMS_PATH_PRECONNECT_DELTA_MV;
    if((uint32_t)power_sample->outputVoltageMv >= close_threshold_mv) {
        if(s_preconnect_confirm_count < BMS_PATH_PRECONNECT_CONFIRM_COUNT) {
            s_preconnect_confirm_count++;
        }
        s_preconnect_reason = POWER_PATH_PRECONNECT_REASON_OK;
    } else {
        s_preconnect_confirm_count = 0U;
        s_preconnect_flags |= POWER_PATH_PRECONNECT_FLAG_VOUT_LOW;
        s_preconnect_reason = POWER_PATH_PRECONNECT_REASON_VOUT_LOW;
    }

    s_preconnect_ready =
        (s_preconnect_confirm_count >= BMS_PATH_PRECONNECT_CONFIRM_COUNT) ? 1U : 0U;
    if(s_preconnect_ready != 0U) {
        s_preconnect_flags |= POWER_PATH_PRECONNECT_FLAG_READY;
    }
    if(s_fet_on_failed_hold_count != 0U) {
        s_fet_on_failed_hold_count--;
        s_preconnect_reason = POWER_PATH_PRECONNECT_REASON_FET_ON_FAILED;
    }
    return s_preconnect_ready;
}

static uint8_t Path_Hardware_Preconnect_Enable(void)
{
#if BMS_PATH_HARDWARE_PRECONNECT_ENABLE
    uint8_t fet_status;
    uint8_t fet_mask;

    fet_mask = POWER_PATH_HARDWARE_PRECONNECT_FET_MASK;
    if((s_preconnect_fets_active != 0U) &&
       (s_preconnect_fet_mask == fet_mask)) {
        return 1U;
    }

    if(fet_mask == 0U) {
        s_preconnect_fets_active = 0U;
        s_preconnect_fet_mask = 0U;
        s_preconnect_flags &= (uint8_t)~POWER_PATH_PRECONNECT_FLAG_ACTIVE;
        s_preconnect_reason = POWER_PATH_PRECONNECT_REASON_PRECHARGE_FET_FAILED;
        return 0U;
    }

    if(0U != Afe_Gd30bm2016_Set_Fet_Mask(fet_mask, &fet_status)) {
        s_preconnect_fets_active = 1U;
        s_preconnect_fet_mask = fet_mask;
        s_path_off_commanded = 0U;
        s_preconnect_flags |= POWER_PATH_PRECONNECT_FLAG_ACTIVE;
        return 1U;
    }

    s_preconnect_fets_active = 0U;
    s_preconnect_fet_mask = 0U;
    s_preconnect_flags &= (uint8_t)~POWER_PATH_PRECONNECT_FLAG_ACTIVE;
    s_preconnect_reason = POWER_PATH_PRECONNECT_REASON_PRECHARGE_FET_FAILED;
    return 0U;
#else
    return 0U;
#endif
}

static void Path_Set_Battery_Enabled(uint8_t enable)
{
    uint8_t fet_status;

    if(enable != 0U) {
        if(s_battery_path_enabled == 0U) {
            if(0U != Afe_Gd30bm2016_Fets_On(&fet_status)) {
                s_battery_path_enabled = 1U;
                s_preconnect_fets_active = 0U;
                s_preconnect_fet_mask = 0U;
                s_preconnect_flags &= (uint8_t)~POWER_PATH_PRECONNECT_FLAG_ACTIVE;
                s_path_off_commanded = 0U;
                s_fet_on_failed_hold_count = 0U;
            } else {
                Path_Mark_Fet_On_Failed();
            }
        }
    } else {
        if((s_battery_path_enabled != 0U) ||
           (s_preconnect_fets_active != 0U) ||
           (s_path_off_commanded == 0U)) {
            Afe_Gd30bm2016_Fets_Off();
            s_battery_path_enabled = 0U;
            s_preconnect_fets_active = 0U;
            s_preconnect_fet_mask = 0U;
            s_preconnect_flags &= (uint8_t)~POWER_PATH_PRECONNECT_FLAG_ACTIVE;
            s_path_off_commanded = 1U;
            s_fet_on_failed_hold_count = 0U;
        }
    }
}

void Power_Path_Manager_Init(void)
{
    s_external_power_present = 0U;
    s_battery_path_enabled = 0U;
    s_path_off_commanded = 0U;
    s_preconnect_ready = 0U;
    s_preconnect_confirm_count = 0U;
    s_preconnect_flags = 0U;
    s_preconnect_reason = POWER_PATH_PRECONNECT_REASON_NO_SAMPLE;
    s_preconnect_fets_active = 0U;
    s_preconnect_fet_mask = 0U;
    s_preconnect_delta_mv = 0U;
    s_preconnect_allowed_delta_mv = BMS_PATH_PRECONNECT_DELTA_MV;
    s_fet_on_failed_hold_count = 0U;
    Power_Path_Manager_Force_Off();
}

void Power_Path_Manager_Get_State(power_path_manager_state_t *state)
{
    if(state == 0) {
        return;
    }

    state->externalPowerPresent = s_external_power_present;
    state->batteryPathEnabled = s_battery_path_enabled;
    state->pathOffCommanded = s_path_off_commanded;
    state->preconnectFlags = s_preconnect_flags;
    if(s_preconnect_fets_active != 0U) {
        state->preconnectFlags |= POWER_PATH_PRECONNECT_FLAG_ACTIVE;
    }
    state->preconnectReason = s_preconnect_reason;
    state->preconnectConfirmCount = s_preconnect_confirm_count;
    state->preconnectDeltaMv = s_preconnect_delta_mv;
    state->preconnectThresholdMv = s_preconnect_allowed_delta_mv;
}

void Power_Path_Manager_Force_Off(void)
{
    if(s_path_off_commanded == 0U) {
        Afe_Gd30bm2016_Fets_Off();
        s_path_off_commanded = 1U;
    } else {
        Afe_Gd30bm2016_Force_Path_Off_Fast();
    }
    s_battery_path_enabled = 0U;
    s_preconnect_fets_active = 0U;
    s_preconnect_fet_mask = 0U;
    s_preconnect_ready = 0U;
    s_preconnect_confirm_count = 0U;
    s_preconnect_flags = 0U;
    s_preconnect_delta_mv = 0U;
    s_preconnect_allowed_delta_mv = BMS_PATH_PRECONNECT_DELTA_MV;
    s_preconnect_reason = POWER_PATH_PRECONNECT_REASON_IDLE;
    s_fet_on_failed_hold_count = 0U;
}

void Power_Path_Manager_Update(const bms_afe_data_t *afe,
                               const bms_power_sample_t *power_sample,
                               const bms_charge_parameters_t *parameters,
                               bms_charge_state_t charge_state,
                               uint32_t fault_bitmap)
{
    uint8_t external_present;
    uint8_t preconnect_ready;

    (void)parameters;
    (void)charge_state;

    if(fault_bitmap != 0U) {
        Power_Path_Manager_Force_Off();
        s_preconnect_reason = POWER_PATH_PRECONNECT_REASON_FAULT;
        return;
    }

    if(power_sample == 0) {
        Power_Path_Manager_Force_Off();
        s_preconnect_reason = POWER_PATH_PRECONNECT_REASON_NO_SAMPLE;
        return;
    }

    external_present = Path_External_Power_Present(power_sample->inputVoltageMv);
    preconnect_ready = 0U;

    if(external_present == 0U) {
        Path_Preconnect_Reset_Debug(POWER_PATH_PRECONNECT_REASON_NO_INPUT);
        Path_Set_Battery_Enabled(0U);
        return;
    }

#if BMS_PATH_BYPASS_PRECONNECT_FOR_TEST
    (void)preconnect_ready;
    /*
     * Lab-only bypass: close the BM2016 main path without waiting for the
     * Vout-Stack confirmation window. Keep the debug fields explicit so
     * Power Debug shows that the path was forced ready for this test.
     */
    s_preconnect_confirm_count = BMS_PATH_PRECONNECT_CONFIRM_COUNT;
    s_preconnect_ready = 1U;
    s_preconnect_flags = POWER_PATH_PRECONNECT_FLAG_READY;
    s_preconnect_reason = POWER_PATH_PRECONNECT_REASON_OK;
    Path_Set_Battery_Enabled(1U);
#else
    if(s_battery_path_enabled != 0U) {
        /*
         * Once BM2016 CHG+DSG have closed, the preconnect comparator is no
         * longer allowed to chatter the main path. Vout/PACK/LD and the BM2016
         * status readback can move for a few control samples during handover;
         * only explicit faults, missing input, or stop/force-off open the path.
         */
        if((afe != 0) && (power_sample != 0) &&
           (afe->packVoltageMv != 0U) && (power_sample->outputVoltageMv != 0U)) {
            s_preconnect_delta_mv =
                Path_Abs_Diff_U16(power_sample->outputVoltageMv, afe->packVoltageMv);
        } else {
            s_preconnect_delta_mv = 0U;
        }
        s_preconnect_confirm_count = BMS_PATH_PRECONNECT_CONFIRM_COUNT;
        s_preconnect_ready = 1U;
        s_preconnect_flags = POWER_PATH_PRECONNECT_FLAG_READY;
        s_preconnect_reason = POWER_PATH_PRECONNECT_REASON_OK;
    } else {
#if BMS_PATH_HARDWARE_PRECONNECT_ENABLE
        if(Path_Hardware_Preconnect_Enable() == 0U) {
            preconnect_ready = Path_Preconnect_Ready(afe, power_sample);
            if(preconnect_ready != 0U) {
                Path_Set_Battery_Enabled(1U);
            }
            return;
        }
        preconnect_ready = Path_Preconnect_Ready(afe, power_sample);
        /*
         * READY 只是软件判据通过，BM2016 芯片有 100 mV 预放电门闩，可能拒绝闭合。
         * 必须确认主 FET 真正闭合（Vout-Stack 压差 < 50 mV），才通知上层切 CC。
         * 否则上层收到 batteryPathEnabled=1 就把 boost 目标从 Pack-20 改成 37.8V，
         * 无电池钳位时 boost 过冲撞硬 OVP。
         */
        if(preconnect_ready != 0U) {
            if((afe != 0) && (power_sample != 0) &&
               (afe->packVoltageMv != 0U) && (power_sample->outputVoltageMv != 0U)) {
                uint16_t delta_mv = Path_Abs_Diff_U16(power_sample->outputVoltageMv,
                                                      afe->packVoltageMv);
                if(delta_mv < 50U) {
                    Path_Set_Battery_Enabled(1U);
                }
            }
        }
#else
        preconnect_ready = Path_Preconnect_Ready(afe, power_sample);
        if(preconnect_ready == 0U) {
            Path_Set_Battery_Enabled(0U);
        } else {
            Path_Set_Battery_Enabled(1U);
        }
#endif
    }
#endif
}
