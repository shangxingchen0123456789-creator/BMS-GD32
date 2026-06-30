#ifndef BATTERY_MODEL_H
#define BATTERY_MODEL_H

#include <stdint.h>

uint16_t Battery_Model_Ocv_To_Soc_X10(uint16_t cell_voltage_mv);
uint16_t Battery_Model_Pack_To_Cell_Mv(uint16_t pack_voltage_mv, uint8_t cell_count);
uint16_t Battery_Model_Capacity_Factor_X1000(int16_t temperature_x10);

#endif
