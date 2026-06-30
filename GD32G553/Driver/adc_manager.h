#ifndef ADC_MANAGER_H
#define ADC_MANAGER_H

#include "bms_types.h"

void Adc_Manager_Init(void);
void Adc_Manager_Sample(const bms_afe_data_t *afe, bms_power_sample_t *sample);
void Adc_Manager_Sample_Fast(bms_power_sample_t *sample);

#endif
