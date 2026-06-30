#ifndef BMS_UART_H
#define BMS_UART_H

#include <stdint.h>

void Bms_Uart_Init(void);
uint16_t Bms_Uart_Read(uint8_t *data, uint16_t max_length);
void Bms_Uart_Send(const uint8_t *data, uint16_t length);
void Bms_Uart_Irq_Handler(void);

#endif
