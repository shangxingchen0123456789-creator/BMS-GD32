#include "balance_manager.h"

/*
 * 被动均衡策略。
 *
 * 只在充电后段允许均衡，此时单体压差更有意义，
 * 对高电压电芯放电也不会明显干扰前面的恒流充电阶段。
 */
static uint16_t s_balance_bitmap;

void Balance_Manager_Init(void)
{
    s_balance_bitmap = 0U;
}

uint16_t Balance_Manager_Update(const bms_afe_data_t *afe,
                                const bms_charge_parameters_t *parameters,
                                uint8_t charge_state)
{
    uint32_t i;
    uint16_t threshold_mv;
    uint16_t bitmap;

    if(afe == 0 || parameters == 0) {
        s_balance_bitmap = 0U;
        return s_balance_bitmap;
    }

    /* 预检、涓流、恒流阶段不做均衡，优先把充电能量送入电池包。 */
    if(charge_state != (uint8_t)BMS_CHARGE_STATE_CV &&
       charge_state != (uint8_t)BMS_CHARGE_STATE_DONE) {
        s_balance_bitmap = 0U;
        return s_balance_bitmap;
    }

    if(afe->cellDeltaMv < parameters->balanceDeltaMv) {
        s_balance_bitmap = 0U;
        return s_balance_bitmap;
    }

    /* 高于最低单体 balanceDelta 的电芯会被选入均衡位图。 */
    threshold_mv = (uint16_t)(afe->cellMinMv + parameters->balanceDeltaMv);
    bitmap = 0U;

    for(i = 0U; i < BMS_CELL_COUNT; i++) {
        if(afe->cellMv[i] >= threshold_mv) {
            bitmap |= (uint16_t)(1U << i);
        }
    }

    s_balance_bitmap = (uint16_t)(bitmap & 0x01FFU);
    return s_balance_bitmap;
}
