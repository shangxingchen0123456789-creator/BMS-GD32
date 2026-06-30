#include "battery_model.h"

typedef struct {
    uint16_t mv;
    uint16_t socX10;
} battery_ocv_point_t;

static const battery_ocv_point_t s_ocv_table[] = {
    {3000U, 0U},
    {3300U, 50U},
    {3500U, 120U},
    {3650U, 250U},
    {3750U, 400U},
    {3850U, 550U},
    {3950U, 700U},
    {4050U, 850U},
    {4150U, 950U},
    {4200U, 1000U}
};

uint16_t Battery_Model_Ocv_To_Soc_X10(uint16_t cell_voltage_mv)
{
    uint32_t i;
    uint32_t delta_mv;
    uint32_t delta_soc;
    uint32_t soc;

    if(cell_voltage_mv <= s_ocv_table[0].mv) {
        return s_ocv_table[0].socX10;
    }

    for(i = 1U; i < (sizeof(s_ocv_table) / sizeof(s_ocv_table[0])); i++) {
        if(cell_voltage_mv <= s_ocv_table[i].mv) {
            delta_mv = (uint32_t)s_ocv_table[i].mv - (uint32_t)s_ocv_table[i - 1U].mv;
            delta_soc = (uint32_t)s_ocv_table[i].socX10 - (uint32_t)s_ocv_table[i - 1U].socX10;
            soc = (uint32_t)s_ocv_table[i - 1U].socX10;
            if(delta_mv != 0U) {
                soc += (((uint32_t)cell_voltage_mv - (uint32_t)s_ocv_table[i - 1U].mv) * delta_soc) / delta_mv;
            }
            return (uint16_t)soc;
        }
    }

    return 1000U;
}

uint16_t Battery_Model_Pack_To_Cell_Mv(uint16_t pack_voltage_mv, uint8_t cell_count)
{
    if(cell_count == 0U) {
        return 0U;
    }

    return (uint16_t)((uint32_t)pack_voltage_mv / (uint32_t)cell_count);
}

uint16_t Battery_Model_Capacity_Factor_X1000(int16_t temperature_x10)
{
    if(temperature_x10 <= -100) {
        return 650U;
    }
    if(temperature_x10 <= 0) {
        return 800U;
    }
    if(temperature_x10 >= 450) {
        return 900U;
    }

    return 1000U;
}
