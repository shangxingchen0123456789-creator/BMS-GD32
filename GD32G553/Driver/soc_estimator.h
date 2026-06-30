#ifndef SOC_ESTIMATOR_H
#define SOC_ESTIMATOR_H

#include "bms_types.h"

void Soc_Estimator_Init(void);
void Soc_Estimator_Update(uint32_t period_ms, const bms_afe_data_t *afe, int16_t battery_current_ma);
uint16_t Soc_Estimator_Get_X10(void);
void Soc_Estimator_Set_Capacity(uint16_t capacity_mah);
void Soc_Estimator_Ocv_Correct(uint16_t cell_voltage_mv);

#endif
