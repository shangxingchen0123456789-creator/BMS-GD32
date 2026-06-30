#ifndef BOARD_SUPPORT_H
#define BOARD_SUPPORT_H

#include <stdint.h>

void Board_Support_Init(void);
uint32_t Board_Support_Millis(void);
void Board_Support_Led_Toggle(void);
void Board_Support_Fault_Led(uint8_t on);

#endif
