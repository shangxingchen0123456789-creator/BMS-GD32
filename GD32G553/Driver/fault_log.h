#ifndef FAULT_LOG_H
#define FAULT_LOG_H

#include "bms_types.h"

#include <stdint.h>

#define FAULT_LOG_VERSION                     1U

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t length;
    uint32_t sequence;
    uint32_t runtimeMs;
    uint32_t faultBitmap;
    uint16_t vinMv;
    uint16_t voutMv;
    int16_t ioutMa;
    int16_t mosTempX10;
    int16_t inductorTempX10;
    uint16_t socX10;
    uint8_t chargeState;
    uint8_t chargeMode;
    uint16_t dutyX100;
    uint16_t cellMinMv;
    uint16_t cellMaxMv;
    uint16_t reserved;
    uint32_t crc32;
} fault_log_record_t;

void Fault_Log_Init(void);
void Fault_Log_Service(const bms_status_t *status, const bms_power_sample_t *sample);
uint16_t Fault_Log_Count(void);
uint8_t Fault_Log_Read(uint16_t newest_index, fault_log_record_t *record);
void Fault_Log_Clear(void);

#endif
