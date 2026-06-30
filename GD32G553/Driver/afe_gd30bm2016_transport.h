#ifndef AFE_GD30BM2016_TRANSPORT_H
#define AFE_GD30BM2016_TRANSPORT_H

#include "FreeRTOS.h"

#include <stdint.h>

void Afe_Gd30bm2016_Transport_Init(void);
uint8_t Afe_Gd30bm2016_Transport_Lock(TickType_t timeout);
void Afe_Gd30bm2016_Transport_Unlock(void);
void Afe_Gd30bm2016_Transport_Delay_Ms(uint32_t ms);
void Afe_Gd30bm2016_Transport_Gpio_Init(void);
uint8_t Afe_Gd30bm2016_Transport_Write_Raw(uint8_t address,
                                           uint8_t reg_addr,
                                           const uint8_t *data,
                                           uint8_t length);
uint8_t Afe_Gd30bm2016_Transport_Read_Raw(uint8_t address,
                                          uint8_t reg_addr,
                                          uint8_t *data,
                                          uint8_t length);

#endif
