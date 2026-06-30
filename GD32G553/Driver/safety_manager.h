#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include "bms_types.h"

typedef enum {
    SAFETY_TRIP_SOURCE_NONE = 0,
    SAFETY_TRIP_SOURCE_SERVICE_FAST = 1,
    SAFETY_TRIP_SOURCE_REPORT = 2,
    SAFETY_TRIP_SOURCE_EXTERNAL_ISR = 3,
    SAFETY_TRIP_SOURCE_POWER_FAULT_ISR = 4
} safety_trip_source_t;

typedef struct {
    uint32_t faults;
    uint8_t source;
    uint8_t powerFaultPinLevel;
} safety_manager_debug_t;

void Safety_Manager_Init(void);
void Safety_Manager_Service(void);
void Safety_Manager_Report_Faults(uint32_t faults);
void Safety_Manager_Clear_Latched_Faults(void);
uint32_t Safety_Manager_Get_Latched_Faults(void);
uint32_t Safety_Manager_Sample_Fast_Faults(void);
void Safety_Manager_Get_Debug(safety_manager_debug_t *debug);
void Safety_Manager_Set_Afe_Alert_Monitor(uint8_t enabled);
uint8_t Safety_Manager_Afe_Alert_Monitor_Enabled(void);
void Safety_Manager_Handle_External_Fault_Isr(uint32_t faults);
void Safety_Manager_Handle_Power_Fault_Isr(void);

#endif
