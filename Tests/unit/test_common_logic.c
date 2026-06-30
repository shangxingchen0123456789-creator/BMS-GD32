#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "battery_model.h"
#include "balance_manager.h"
#include "pi_controller.h"

static void Test_Pi_Controller(void)
{
    pi_controller_t controller;
    const pi_controller_config_t config = {
        10,
        10,
        50,
        12
    };

    Pi_Controller_Init(&controller, &config);
    assert(Pi_Controller_Update(&controller, 100) == 12);
    assert(controller.integral == 50);

    Pi_Controller_Decay(&controller, 1U, 2U);
    assert(controller.integral == 25);

    Pi_Controller_Reset(&controller);
    assert(controller.integral == 0);
    assert(Pi_Controller_Update(0, 100) == 0);
}

static void Test_Battery_Model(void)
{
    assert(Battery_Model_Ocv_To_Soc_X10(3000U) == 0U);
    assert(Battery_Model_Ocv_To_Soc_X10(3400U) == 85U);
    assert(Battery_Model_Ocv_To_Soc_X10(4200U) == 1000U);
    assert(Battery_Model_Pack_To_Cell_Mv(37800U, 9U) == 4200U);
    assert(Battery_Model_Pack_To_Cell_Mv(37800U, 0U) == 0U);
    assert(Battery_Model_Capacity_Factor_X1000(-100) == 650U);
    assert(Battery_Model_Capacity_Factor_X1000(450) == 900U);
}

static void Test_Balance_Manager(void)
{
    bms_afe_data_t afe;
    bms_charge_parameters_t parameters;

    memset(&afe, 0, sizeof(afe));
    memset(&parameters, 0, sizeof(parameters));

    Balance_Manager_Init();
    assert(Balance_Manager_Update(0, &parameters, (uint8_t)BMS_CHARGE_STATE_CV) == 0U);

    parameters.balanceDeltaMv = 30U;
    afe.cellMinMv = 3500U;
    afe.cellDeltaMv = 60U;
    afe.cellMv[0] = 3500U;
    afe.cellMv[1] = 3529U;
    afe.cellMv[2] = 3530U;
    afe.cellMv[3] = 3560U;

    assert(Balance_Manager_Update(&afe, &parameters, (uint8_t)BMS_CHARGE_STATE_CC) == 0U);
    assert(Balance_Manager_Update(&afe, &parameters, (uint8_t)BMS_CHARGE_STATE_CV) == 0x000CU);

    afe.cellDeltaMv = 20U;
    assert(Balance_Manager_Update(&afe, &parameters, (uint8_t)BMS_CHARGE_STATE_DONE) == 0U);
}

int main(void)
{
    Test_Pi_Controller();
    Test_Battery_Model();
    Test_Balance_Manager();
    return 0;
}

