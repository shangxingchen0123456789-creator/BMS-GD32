#include "charge_manager_internal.h"

charge_manager_context_t g_charge_manager;

typedef struct {
    bms_charge_parameters_t params;
    bms_charge_state_t state;
    uint8_t runRequest;
    uint8_t mode;
    uint8_t workMode;
    uint8_t digitalPowerEnabled;
    uint8_t manualFetActive;
    uint8_t manualFetMask;
    uint16_t digitalPowerTargetVoltageMv;
    uint16_t digitalPowerCurrentLimitMa;
    uint16_t cvDoneCounter;
    uint32_t faults;
    uint8_t powerRequested;
    uint8_t digitalPowerFaultContext;
    uint8_t pathManagerUpdated;
    int16_t batteryCurrentMa;
} charge_manager_update_context_t;

static void Charge_Manager_Load_Update_Context(charge_manager_update_context_t *context,
                                               const bms_afe_data_t *afe)
{
    taskENTER_CRITICAL();
    context->params = g_charge_manager.config.params;
    context->state = g_charge_manager.control.state;
    context->runRequest = g_charge_manager.control.runRequest;
    context->mode = g_charge_manager.control.mode;
    context->workMode = g_charge_manager.control.workMode;
    context->digitalPowerEnabled = g_charge_manager.digital.enabled;
    context->manualFetActive = g_charge_manager.manualFet.active;
    context->manualFetMask = g_charge_manager.manualFet.mask;
    context->digitalPowerTargetVoltageMv = g_charge_manager.digital.targetVoltageMv;
    context->digitalPowerCurrentLimitMa = g_charge_manager.digital.currentLimitMa;
    context->cvDoneCounter = g_charge_manager.control.cvDoneCounter;
    taskEXIT_CRITICAL();

    context->powerRequested =
        ((context->runRequest != 0U) || (context->digitalPowerEnabled != 0U)) ? 1U : 0U;
    context->batteryCurrentMa = afe->batteryCurrentMa;
    context->pathManagerUpdated = 0U;
    context->faults = 0U;
    context->digitalPowerFaultContext =
        ((context->workMode == (uint8_t)BMS_WORK_MODE_DIGITAL_POWER) ||
         (context->digitalPowerEnabled != 0U) ||
         (context->state == BMS_CHARGE_STATE_DIGITAL_POWER) ||
         (context->mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER)) ? 1U : 0U;

    Power_Control_Set_Battery_Current_Feedback(0, 0U);
    Power_Control_Set_Battery_Voltage_Feedback(0U, 0U);
}

static void Charge_Manager_Collect_Update_Faults(charge_manager_update_context_t *context,
                                                 const bms_afe_data_t *afe,
                                                 const bms_power_sample_t *power_sample)
{
    if(context->digitalPowerEnabled != 0U) {
        context->faults =
            Charge_Manager_Collect_Digital_Power_Faults(power_sample,
                                                        context->digitalPowerTargetVoltageMv,
                                                        context->digitalPowerCurrentLimitMa);
    } else {
        context->faults = Charge_Manager_Collect_Faults(afe,
                                                        power_sample,
                                                        context->powerRequested);
    }
}

static uint8_t Charge_Manager_Dispatch_Update_State(charge_manager_update_context_t *context)
{
    /*
     * Modes that own the power path stop the normal charger state machine here.
     * The final path ownership helper still runs after this dispatch.
     */
    if(context->faults != 0U) {
        context->runRequest = 0U;
        Safety_Manager_Set_Afe_Alert_Monitor(0U);
        if(context->digitalPowerFaultContext != 0U) {
            context->mode = (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER;
        }
        context->digitalPowerEnabled = 0U;
        context->manualFetActive = 0U;
        context->manualFetMask = 0U;
        context->state = BMS_CHARGE_STATE_FAULT;
        Safety_Manager_Report_Faults(context->faults);
        Charge_Manager_Clear_Path_Settle();
        Power_Path_Manager_Force_Off();
        return 0U;
    }

    if(context->manualFetActive != 0U) {
        (void)context->manualFetMask;
        context->runRequest = 0U;
        context->digitalPowerEnabled = 0U;
        context->state = BMS_CHARGE_STATE_IDLE;
        Charge_Manager_Clear_Path_Settle();
        Power_Control_Stop();
        Safety_Manager_Set_Afe_Alert_Monitor(0U);
        return 0U;
    }

    if(context->digitalPowerEnabled != 0U) {
        context->runRequest = 0U;
        context->state = BMS_CHARGE_STATE_DIGITAL_POWER;
        context->mode = (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER;
        Charge_Manager_Clear_Path_Settle();
        Power_Control_Set(context->digitalPowerTargetVoltageMv,
                          context->digitalPowerCurrentLimitMa,
                          context->mode);
        return 0U;
    }

    if(context->workMode == (uint8_t)BMS_WORK_MODE_DIGITAL_POWER) {
        context->runRequest = 0U;
        context->state = BMS_CHARGE_STATE_IDLE;
        context->mode = (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER;
        Charge_Manager_Clear_Path_Settle();
        Power_Control_Stop();
        return 0U;
    }

    if(context->runRequest == 0U) {
        context->state = BMS_CHARGE_STATE_IDLE;
        Charge_Manager_Clear_Path_Settle();
        Power_Control_Stop();
        Safety_Manager_Set_Afe_Alert_Monitor(0U);
        return 0U;
    }

    return 1U;
}

static void Charge_Manager_Select_Charge_State(charge_manager_update_context_t *context,
                                               const bms_afe_data_t *afe)
{
    if(context->state != BMS_CHARGE_STATE_PRECHECK &&
       context->state != BMS_CHARGE_STATE_IDLE) {
        return;
    }

    if(context->mode == (uint8_t)BMS_CHARGE_MODE_TRICKLE) {
        context->state = BMS_CHARGE_STATE_TRICKLE;
    } else if(context->mode == (uint8_t)BMS_CHARGE_MODE_CC) {
        context->state = BMS_CHARGE_STATE_CC;
    } else if(context->mode == (uint8_t)BMS_CHARGE_MODE_CV) {
        context->state = BMS_CHARGE_STATE_CV;
    } else if(afe->cellMinMv < TRICKLE_EXIT_CELL_MV) {
        context->state = BMS_CHARGE_STATE_TRICKLE;
    } else {
        context->state = BMS_CHARGE_STATE_CC;
    }
}

static void Charge_Manager_Update_Auto_State(charge_manager_update_context_t *context,
                                             const bms_afe_data_t *afe,
                                             const bms_power_sample_t *power_sample)
{
    if(context->mode != (uint8_t)BMS_CHARGE_MODE_AUTO) {
        return;
    }

    if(context->state == BMS_CHARGE_STATE_TRICKLE &&
       afe->cellMinMv >= TRICKLE_EXIT_CELL_MV) {
        context->state = BMS_CHARGE_STATE_CC;
    }
    if(context->state != BMS_CHARGE_STATE_TRICKLE &&
       afe->cellMinMv <= TRICKLE_ENTER_CELL_MV) {
        context->state = BMS_CHARGE_STATE_TRICKLE;
    }
    if(context->state == BMS_CHARGE_STATE_CC &&
       Charge_Manager_Cv_Entry_Ready(afe, power_sample, &context->params) != 0U) {
        context->state = BMS_CHARGE_STATE_CV;
    }
    if((context->state == BMS_CHARGE_STATE_CV) &&
       (Charge_Manager_Cv_Done_Allowed(power_sample, &context->params) != 0U) &&
       (context->batteryCurrentMa >= 0) &&
       (context->batteryCurrentMa <= (int16_t)context->params.cutoffCurrentMa)) {
        context->cvDoneCounter++;
        if(context->cvDoneCounter > CV_DONE_CONFIRM_COUNT) {
            context->state = BMS_CHARGE_STATE_DONE;
            context->runRequest = 0U;
            Charge_Manager_Clear_Path_Settle();
            Power_Control_Stop();
        }
    } else {
        context->cvDoneCounter = 0U;
    }
}

static uint16_t Charge_Manager_Preconnect_Ovp_Limit(uint16_t preconnect_target_mv)
{
    if(preconnect_target_mv >
       (uint16_t)(0xFFFFU - BMS_PRECONNECT_OUTPUT_OVP_MARGIN_MV)) {
        return 0xFFFFU;
    }

    return (uint16_t)((uint32_t)preconnect_target_mv +
                      (uint32_t)BMS_PRECONNECT_OUTPUT_OVP_MARGIN_MV);
}

static void Charge_Manager_Drive_Enabled_Path(charge_manager_update_context_t *context,
                                              uint32_t period_ms,
                                              const bms_afe_data_t *afe,
                                              const bms_power_sample_t *power_sample)
{
    uint8_t control_mode;
    uint8_t afe_handover_pending;
    uint16_t target_current_ma;

    afe_handover_pending = Charge_Manager_Battery_Path_Rising_Edge();
    Charge_Manager_Update_Path_Settle(1U, afe, &context->params);
    Safety_Manager_Set_Afe_Alert_Monitor(
        (Charge_Manager_Path_Settle_Active() != 0U) ? 0U : 1U);
    Power_Manager_Update(period_ms,
                         afe,
                         power_sample,
                         &context->params,
                         (uint8_t)context->state,
                         context->faults);
    target_current_ma = Charge_Manager_Target_Current_For_State(context->state);
    control_mode = Charge_Manager_Control_Mode_For_State(context->mode, context->state);
    Power_Control_Set_Battery_Current_Feedback(context->batteryCurrentMa, 1U);
    Power_Control_Set_Battery_Voltage_Feedback(afe->packVoltageMv, 1U);
    if(afe_handover_pending != 0U) {
        Power_Control_Set_Afe_Handover(context->params.targetVoltageMv,
                                       target_current_ma,
                                       control_mode,
                                       power_sample);
    } else {
        Power_Control_Set(context->params.targetVoltageMv, target_current_ma, control_mode);
    }
}

static void Charge_Manager_Drive_Preconnect_Path(charge_manager_update_context_t *context,
                                                 uint32_t period_ms,
                                                 const bms_afe_data_t *afe,
                                                 const bms_power_sample_t *power_sample)
{
    uint16_t preconnect_target_mv;

    Charge_Manager_Update_Path_Settle(0U, afe, &context->params);
    Safety_Manager_Set_Afe_Alert_Monitor(0U);
    context->state = BMS_CHARGE_STATE_PRECHECK;

    if(Charge_Manager_Preconnect_Can_Drive(afe, power_sample) == 0U) {
        Power_Control_Stop();
        return;
    }

    preconnect_target_mv = Charge_Manager_Preconnect_Target_Mv(afe,
                                                               power_sample,
                                                               &context->params);
    if(preconnect_target_mv == 0U) {
        Power_Control_Stop();
        return;
    }

    Power_Control_Set_Preconnect(
        preconnect_target_mv,
        Charge_Manager_Preconnect_Current_Limit_Ma(afe, power_sample, period_ms),
        Charge_Manager_Preconnect_Ovp_Limit(preconnect_target_mv));
}

static void Charge_Manager_Drive_Normal_Path(charge_manager_update_context_t *context,
                                             uint32_t period_ms,
                                             const bms_afe_data_t *afe,
                                             const bms_power_sample_t *power_sample)
{
    power_path_manager_state_t path_state;

    if(context->state == BMS_CHARGE_STATE_DONE) {
        return;
    }

    Power_Path_Manager_Update(afe,
                              power_sample,
                              &context->params,
                              context->state,
                              context->faults);
    context->pathManagerUpdated = 1U;
    Power_Path_Manager_Get_State(&path_state);
    if(path_state.batteryPathEnabled != 0U) {
        Charge_Manager_Drive_Enabled_Path(context, period_ms, afe, power_sample);
    } else {
        Charge_Manager_Drive_Preconnect_Path(context, period_ms, afe, power_sample);
    }
}

static void Charge_Manager_Handle_Normal_Charge(charge_manager_update_context_t *context,
                                                uint32_t period_ms,
                                                const bms_afe_data_t *afe,
                                                const bms_power_sample_t *power_sample)
{
    Charge_Manager_Select_Charge_State(context, afe);
    Charge_Manager_Update_Auto_State(context, afe, power_sample);
    Charge_Manager_Drive_Normal_Path(context, period_ms, afe, power_sample);
}

static void Charge_Manager_Apply_Final_Path_State(const charge_manager_update_context_t *context,
                                                  const bms_afe_data_t *afe,
                                                  const bms_power_sample_t *power_sample)
{
    if(context->manualFetActive != 0U) {
        /* Manual FET debug owns CHG/PCHG/DSG/PDSG until a normal command exits it. */
    } else if(context->workMode == (uint8_t)BMS_WORK_MODE_DIGITAL_POWER ||
              context->digitalPowerEnabled != 0U) {
        Charge_Manager_Clear_Path_Settle();
        Power_Path_Manager_Force_Off();
#if BMS_DIGITAL_POWER_AFELESS_DEBUG
    } else if((context->runRequest == 0U) && (Charge_Manager_Afe_Sample_Valid(afe) == 0U)) {
        Charge_Manager_Clear_Path_Settle();
        Power_Path_Manager_Force_Off();
#endif
    } else if((context->runRequest != 0U) && (context->pathManagerUpdated == 0U)) {
        Power_Path_Manager_Update(afe,
                                  power_sample,
                                  &context->params,
                                  context->state,
                                  context->faults);
    } else if(context->runRequest == 0U) {
        Charge_Manager_Clear_Path_Settle();
        Power_Path_Manager_Force_Off();
    }
}

static void Charge_Manager_Fill_Status(const charge_manager_update_context_t *context,
                                       const bms_afe_data_t *afe,
                                       const bms_power_sample_t *power_sample,
                                       bms_status_t *status)
{
    uint32_t i;
    power_control_state_t power_state;

    memset(status, 0, sizeof(*status));
    status->packVoltageMv = afe->packVoltageMv;
    status->inputVoltageMv = power_sample->inputVoltageMv;
    status->outputVoltageMv = power_sample->outputVoltageMv;
    status->chargeState = (uint8_t)context->state;
    status->chargeMode = context->mode;
    status->workMode = context->workMode;
    status->faultBitmap = context->faults;
    status->temperaturesX10[0] = afe->temperaturesX10[0];
    status->temperaturesX10[1] = afe->temperaturesX10[1];
    status->temperaturesX10[2] = afe->temperaturesX10[2];
    status->temperaturesX10[3] = afe->temperaturesX10[3];
    status->temperaturesX10[4] = power_sample->mosTempX10;
    status->temperaturesX10[5] = power_sample->inductorTempX10;
    status->cellMaxMv = afe->cellMaxMv;
    status->cellMinMv = afe->cellMinMv;
    status->cellDeltaMv = afe->cellDeltaMv;

    for(i = 0U; i < BMS_CELL_COUNT; i++) {
        status->cellMv[i] = afe->cellMv[i];
    }

    Power_Control_Get_State(&power_state);
    if((context->workMode != (uint8_t)BMS_WORK_MODE_DIGITAL_POWER) &&
       (context->digitalPowerEnabled == 0U) &&
       (Charge_Manager_Afe_Sample_Valid(afe) != 0U)) {
        status->chargeCurrentMa = context->batteryCurrentMa;
    } else {
        status->chargeCurrentMa = 0;
    }
    status->dutyX100 = power_state.dutyX100;
}

static void Charge_Manager_Store_Update_Context(const charge_manager_update_context_t *context)
{
    taskENTER_CRITICAL();
    g_charge_manager.control.state = context->state;
    g_charge_manager.control.runRequest = context->runRequest;
    g_charge_manager.control.cvDoneCounter = context->cvDoneCounter;
    g_charge_manager.digital.enabled = context->digitalPowerEnabled;
    g_charge_manager.manualFet.active = context->manualFetActive;
    g_charge_manager.manualFet.mask = context->manualFetMask;
    g_charge_manager.digital.targetVoltageMv = context->digitalPowerTargetVoltageMv;
    g_charge_manager.digital.currentLimitMa = context->digitalPowerCurrentLimitMa;
    if((context->workMode != (uint8_t)BMS_WORK_MODE_DIGITAL_POWER) &&
       (context->digitalPowerEnabled == 0U) &&
       (g_charge_manager.control.mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER) &&
       ((context->faults == 0U) || (context->digitalPowerFaultContext == 0U))) {
        g_charge_manager.control.mode = (uint8_t)BMS_CHARGE_MODE_AUTO;
    } else {
        g_charge_manager.control.mode = context->mode;
    }
    taskEXIT_CRITICAL();
}

void Charge_Manager_Update(uint32_t period_ms,
                           const bms_afe_data_t *afe,
                           const bms_power_sample_t *power_sample,
                           bms_status_t *status)
{
    charge_manager_update_context_t context;

    if(status == 0 || afe == 0 || power_sample == 0) {
        return;
    }

    Charge_Manager_Load_Update_Context(&context, afe);
    Charge_Manager_Collect_Update_Faults(&context, afe, power_sample);
    if(Charge_Manager_Dispatch_Update_State(&context) != 0U) {
        Charge_Manager_Handle_Normal_Charge(&context, period_ms, afe, power_sample);
    }
    Charge_Manager_Apply_Final_Path_State(&context, afe, power_sample);
    Charge_Manager_Fill_Status(&context, afe, power_sample, status);
    Charge_Manager_Store_Update_Context(&context);
}
