#include "soc_estimator.h"

#include "battery_model.h"
#include "bms_board_config.h"
#include "param_storage.h"

#include "FreeRTOS.h"
#include "task.h"

#define SOC_REST_CURRENT_MA                   100
#define SOC_OCV_CORRECT_PERIOD_MS             1000U
#define SOC_FULL_CELL_MV                      4180U
#define SOC_EMPTY_CELL_MV                     3000U
#define SOC_CHARGE_EFF_X1000                  995LL

static uint16_t s_soc_x10;
static uint16_t s_capacity_mah;
static int64_t s_soc_residual_ma_ms;
static uint32_t s_rest_ms;
static uint8_t s_initialized;

static int16_t Soc_Primary_Temperature_X10(const bms_afe_data_t *afe)
{
    uint32_t i;

    if(afe == 0) {
        return 250;
    }

    for(i = 0U; i < BMS_AFE_TEMP_COUNT; i++) {
        if(afe->temperaturesX10[i] != (int16_t)BMS_TEMP_UNAVAILABLE_X10) {
            return afe->temperaturesX10[i];
        }
    }

    return 250;
}

static uint8_t Soc_Voltage_Valid(const bms_afe_data_t *afe)
{
    uint32_t i;

    if(afe == 0) {
        return 0U;
    }
    if((afe->faultBitmap & BMS_FAULT_AFE_COMM) != 0U) {
        return 0U;
    }
    if(afe->cellMinMv == 0U || afe->cellMaxMv == 0U) {
        return 0U;
    }
    for(i = 0U; i < BMS_CELL_COUNT; i++) {
        if(afe->cellMv[i] == 0U) {
            return 0U;
        }
    }

    return 1U;
}

static uint16_t Soc_Ocv_From_Afe_X10(const bms_afe_data_t *afe)
{
    uint32_t sum_mv;
    uint32_t i;
    uint16_t cell_mv;

    if(Soc_Voltage_Valid(afe) == 0U) {
        return s_soc_x10;
    }

    sum_mv = 0U;
    for(i = 0U; i < BMS_CELL_COUNT; i++) {
        sum_mv += afe->cellMv[i];
    }
    cell_mv = (uint16_t)(sum_mv / BMS_CELL_COUNT);
    if(afe->cellMinMv < cell_mv) {
        cell_mv = afe->cellMinMv;
    }

    return Battery_Model_Ocv_To_Soc_X10(cell_mv);
}

static void Soc_Step_Towards(uint16_t target_x10, uint16_t step_x10)
{
    if(s_soc_x10 < target_x10) {
        if((uint16_t)(target_x10 - s_soc_x10) > step_x10) {
            s_soc_x10 = (uint16_t)(s_soc_x10 + step_x10);
        } else {
            s_soc_x10 = target_x10;
        }
    } else if(s_soc_x10 > target_x10) {
        if((uint16_t)(s_soc_x10 - target_x10) > step_x10) {
            s_soc_x10 = (uint16_t)(s_soc_x10 - step_x10);
        } else {
            s_soc_x10 = target_x10;
        }
    }
}

void Soc_Estimator_Init(void)
{
    bms_system_config_t config;

    s_capacity_mah = BMS_DEFAULT_SOC_CAPACITY_MAH;
    if(Param_Storage_Get_Config(&config) != 0U && config.socCapacityMah != 0U) {
        s_capacity_mah = config.socCapacityMah;
    }

    taskENTER_CRITICAL();
    s_soc_x10 = 0U;
    s_soc_residual_ma_ms = 0;
    s_rest_ms = 0U;
    s_initialized = 0U;
    taskEXIT_CRITICAL();
}

void Soc_Estimator_Set_Capacity(uint16_t capacity_mah)
{
    if(capacity_mah == 0U) {
        return;
    }

    taskENTER_CRITICAL();
    s_capacity_mah = capacity_mah;
    taskEXIT_CRITICAL();
}

void Soc_Estimator_Ocv_Correct(uint16_t cell_voltage_mv)
{
    uint16_t target_soc;

    target_soc = Battery_Model_Ocv_To_Soc_X10(cell_voltage_mv);

    taskENTER_CRITICAL();
    Soc_Step_Towards(target_soc, 2U);
    taskEXIT_CRITICAL();
}

void Soc_Estimator_Update(uint32_t period_ms, const bms_afe_data_t *afe, int16_t battery_current_ma)
{
    int64_t one_soc_x10_ma_ms;
    int64_t delta_ma_ms;
    uint16_t ocv_soc;
    uint16_t capacity_factor;
    int16_t temp_x10;
    int16_t abs_current_ma;

    if(period_ms == 0U) {
        return;
    }

    temp_x10 = Soc_Primary_Temperature_X10(afe);
    capacity_factor = Battery_Model_Capacity_Factor_X1000(temp_x10);
    if(capacity_factor == 0U) {
        capacity_factor = 1000U;
    }

    one_soc_x10_ma_ms = ((int64_t)s_capacity_mah * 3600LL * (int64_t)capacity_factor) / 1000LL;
    if(one_soc_x10_ma_ms <= 0) {
        one_soc_x10_ma_ms = (int64_t)BMS_DEFAULT_SOC_CAPACITY_MAH * 3600LL;
    }

    ocv_soc = Soc_Ocv_From_Afe_X10(afe);
    abs_current_ma = (battery_current_ma >= 0) ? battery_current_ma : (int16_t)(-battery_current_ma);

    taskENTER_CRITICAL();

    if(s_initialized == 0U) {
        if(Soc_Voltage_Valid(afe) != 0U) {
            s_soc_x10 = ocv_soc;
            s_initialized = 1U;
        }
        s_soc_residual_ma_ms = 0;
        s_rest_ms = 0U;
        taskEXIT_CRITICAL();
        return;
    }

    if(battery_current_ma != 0) {
        delta_ma_ms = (int64_t)battery_current_ma * (int64_t)period_ms;
        if(delta_ma_ms > 0) {
            delta_ma_ms = (delta_ma_ms * SOC_CHARGE_EFF_X1000) / 1000LL;
        }
        s_soc_residual_ma_ms += delta_ma_ms;

        while(s_soc_residual_ma_ms >= one_soc_x10_ma_ms) {
            if(s_soc_x10 < 1000U) {
                s_soc_x10++;
            }
            s_soc_residual_ma_ms -= one_soc_x10_ma_ms;
        }
        while(s_soc_residual_ma_ms <= -one_soc_x10_ma_ms) {
            if(s_soc_x10 > 0U) {
                s_soc_x10--;
            }
            s_soc_residual_ma_ms += one_soc_x10_ma_ms;
        }
    }

    if(abs_current_ma <= SOC_REST_CURRENT_MA && Soc_Voltage_Valid(afe) != 0U) {
        s_rest_ms += period_ms;
        if(s_rest_ms >= SOC_OCV_CORRECT_PERIOD_MS) {
            s_rest_ms = 0U;
            Soc_Step_Towards(ocv_soc, 1U);
            s_soc_residual_ma_ms = 0;
        }
    } else {
        s_rest_ms = 0U;
    }

    if(Soc_Voltage_Valid(afe) != 0U) {
        if(afe->cellMinMv <= SOC_EMPTY_CELL_MV) {
            s_soc_x10 = 0U;
            s_soc_residual_ma_ms = 0;
        } else if(afe->cellMinMv >= SOC_FULL_CELL_MV &&
                  battery_current_ma >= 0 &&
                  abs_current_ma <= (int16_t)BMS_DEFAULT_CUTOFF_CURRENT_MA) {
            s_soc_x10 = 1000U;
            s_soc_residual_ma_ms = 0;
        }
    }

    if(s_soc_x10 > 1000U) {
        s_soc_x10 = 1000U;
    }

    taskEXIT_CRITICAL();
}

uint16_t Soc_Estimator_Get_X10(void)
{
    uint16_t soc;

    taskENTER_CRITICAL();
    soc = s_soc_x10;
    taskEXIT_CRITICAL();

    return soc;
}
