#ifndef BMS_COMM_SERVICE_H
#define BMS_COMM_SERVICE_H

#include <stdint.h>

void Bms_Comm_Service_Init(void);
void Bms_Comm_Service_Poll(uint32_t now_ms);

#endif
