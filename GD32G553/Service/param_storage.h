#ifndef PARAM_STORAGE_H
#define PARAM_STORAGE_H

#include "bms_types.h"

#include <stdint.h>

#define BMS_PARAM_STORAGE_VERSION             1U

typedef struct {
    bms_charge_parameters_t charge;
    uint16_t trickleCurrentMa;
    uint16_t inputUvThresholdMv;
    uint16_t outputOvpMarginMv;
    uint16_t outputOcpMarginMa;
    uint16_t inputCurrentLimitMa;
    uint16_t inputPowerLimitW;
    uint16_t minChargeCurrentMa;
    int16_t derateStartTempX10;
    int16_t derateStopTempX10;
    uint16_t socCapacityMah;
    uint16_t socInitialX10;
    uint16_t currentKpDiv;
    uint16_t currentKiDiv;
    uint16_t voltageKpDiv;
    uint16_t voltageKiDiv;
    uint16_t loopStepMaxX100;
    uint16_t adcVinGainX1000;
    uint16_t adcVoutGainX1000;
    uint16_t adcIinOffsetRaw;
    uint16_t adcIoutOffsetRaw;
    uint16_t reserved[8];
} bms_system_config_t;

void Param_Storage_Init(void);
void Param_Storage_Service(uint32_t now_ms);
void Param_Storage_Restore_Default(bms_system_config_t *config);
uint8_t Param_Storage_Get_Config(bms_system_config_t *config);
uint8_t Param_Storage_Set_Config(const bms_system_config_t *config);
uint8_t Param_Storage_Get_Charge(bms_charge_parameters_t *parameters);
uint8_t Param_Storage_Set_Charge(const bms_charge_parameters_t *parameters);
void Param_Storage_Request_Save(void);
uint8_t Param_Storage_Save_Now(void);

#endif
