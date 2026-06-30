#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H

#include "bms_types.h"

typedef enum {
    POWER_STAGE_MODE_OFF = 0,
    POWER_STAGE_MODE_BUCK = 1,
    POWER_STAGE_MODE_BOOST = 2,
    POWER_STAGE_MODE_BUCK_BOOST = 3,
    POWER_STAGE_MODE_BOOST_ASYNC = 4
} power_stage_mode_t;

typedef enum {
    POWER_TRIP_REASON_NONE = 0,
    POWER_TRIP_REASON_SAMPLE_FAULT = 1,
    POWER_TRIP_REASON_FAULT_OC_PIN = 2,
    POWER_TRIP_REASON_INPUT_UV = 3,
    POWER_TRIP_REASON_OUTPUT_OVP = 4,
    POWER_TRIP_REASON_OUTPUT_OCP_SOFTWARE = 5
} power_trip_reason_t;

typedef struct {
    uint8_t enabled;
    uint8_t mode;
    uint8_t powerStageMode;
    uint16_t targetVoltageMv;
    uint16_t targetCurrentMa;
    uint16_t dutyX100;
    uint16_t buckDutyX100;
    uint16_t boostLowDutyX100;
    uint8_t faultLockout;
    uint8_t hardwareReady;
    uint8_t hardwareOutputsOn;
    uint8_t asyncBoostRectifier;
    uint32_t faultBitmap;
    uint32_t periodTicks;
    uint8_t preconnectActive;
    uint16_t preconnectOvpLimitMv;
    uint16_t softCurrentMa;
    uint8_t tripReason;
    uint8_t tripFaultOcActive;
    uint32_t tripFaults;
    int16_t tripIoutMa;
    uint16_t tripCurrentRefMa;
    uint16_t tripOcpLimitMa;
    uint16_t tripVoutMv;
    uint16_t tripVinMv;
    uint16_t tripDutyX100;
} power_control_state_t;

void Power_Control_Init(void);
void Power_Control_Stop(void);
void Power_Control_Fault_Lockout(void);
void Power_Control_Clear_Fault_Lockout(void);
uint32_t Power_Control_Get_Fault_Status(void);
void Power_Control_Clear_Fault_Status(void);
void Power_Control_Set(uint16_t target_voltage_mv, uint16_t target_current_ma, uint8_t mode);
void Power_Control_Set_Preconnect(uint16_t target_voltage_mv, uint16_t target_current_ma, uint16_t ovp_limit_mv);
void Power_Control_Set_Afe_Handover(uint16_t target_voltage_mv,
                                    uint16_t target_current_ma,
                                    uint8_t mode,
                                    const bms_power_sample_t *sample);
void Power_Control_Set_Battery_Current_Feedback(int16_t current_ma, uint8_t valid);
void Power_Control_Set_Battery_Voltage_Feedback(uint16_t pack_voltage_mv, uint8_t valid);
void Power_Control_Apply(const bms_power_sample_t *sample);
void Power_Control_Fast_Loop(const bms_power_sample_t *sample);
uint8_t Power_Control_Wait_Adc_Sample_Point(uint32_t timeout);
void Power_Control_Get_State(power_control_state_t *state);

#endif
