#include "bms_comm_service.h"

#include "bms_protocol.h"
#include "bms_uart.h"
#include "bms_state.h"
#include "board_support.h"
#include "charge_manager.h"
#include "afe_gd30bm2016.h"
#include "app_tasks.h"
#include "fault_log.h"
#include "power_control.h"
#include "power_manager.h"
#include "power_path_manager.h"
#include "safety_manager.h"

#include <string.h>

/*
 * UART 通信调度服务。
 *
 * 接收方向持续解析上位机字节流；发送方向按周期上报遥测：
 * - 实时状态每 100 ms 一帧；
 * - 单体电压每 500 ms 一帧。
 * 命令应答沿用请求帧中的 sequence，方便上位机对应请求和响应。
 */
static bms_proto_parser_t s_parser;
static uint8_t s_tx_sequence;
static uint32_t s_last_status_ms;
static uint32_t s_last_cells_ms;
static uint32_t s_last_fault_bitmap;

static void Comm_Send_Frame(uint8_t type, uint8_t sequence, const uint8_t *payload, uint8_t payload_length)
{
    uint8_t frame[BMS_PROTO_HEADER_SIZE + BMS_PROTO_MAX_PAYLOAD_SIZE + BMS_PROTO_CRC_SIZE];
    uint16_t frame_length;

    frame_length = Bms_Proto_Build_Frame(type, sequence, payload, payload_length, frame, sizeof(frame));
    if(frame_length != 0U) {
        Bms_Uart_Send(frame, frame_length);
    }
}

static void Comm_Send_Ack(uint8_t ack_type, uint8_t ack_sequence, uint8_t result, uint8_t error_code)
{
    uint8_t payload[BMS_PROTO_ACK_PAYLOAD_SIZE];
    uint8_t length;

    length = Bms_Proto_Encode_Ack(ack_type, ack_sequence, result, error_code, payload, sizeof(payload));
    Comm_Send_Frame((uint8_t)BMS_PROTO_FRAME_ACK, ack_sequence, payload, length);
}

static void Comm_Send_Parameters(uint8_t type, uint8_t sequence)
{
    bms_charge_parameters_t parameters;
    uint8_t payload[BMS_PROTO_PARAM_PAYLOAD_SIZE];
    uint8_t length;

    Charge_Manager_Get_Parameters(&parameters);
    length = Bms_Proto_Encode_Parameters(&parameters, payload, sizeof(payload));
    Comm_Send_Frame(type, sequence, payload, length);
}

static uint16_t Comm_Read_U16_Le(const uint8_t *payload, uint8_t offset)
{
    return (uint16_t)payload[offset] |
           (uint16_t)((uint16_t)payload[offset + 1U] << 8);
}

static void Comm_Handle_Charge_Command(const bms_proto_frame_t *frame)
{
    bms_command_reply_t reply;
    uint8_t command_id;
    uint8_t argument;
    uint8_t has_argument;

    if(frame->length < 1U) {
        Comm_Send_Ack(frame->type,
                      frame->sequence,
                      (uint8_t)BMS_CMD_RESULT_ERROR,
                      (uint8_t)BMS_CMD_ERROR_BAD_LENGTH);
        return;
    }

    command_id = frame->payload[0];
    if(command_id == BMS_CMD_DIGITAL_POWER_SET) {
        uint8_t enable;
        uint16_t target_voltage_mv;
        uint16_t current_limit_ma;

        if(frame->length < 6U) {
            Comm_Send_Ack(frame->type,
                          frame->sequence,
                          (uint8_t)BMS_CMD_RESULT_ERROR,
                          (uint8_t)BMS_CMD_ERROR_BAD_LENGTH);
            return;
        }

        enable = frame->payload[1];
        target_voltage_mv = Comm_Read_U16_Le(frame->payload, 2U);
        current_limit_ma = Comm_Read_U16_Le(frame->payload, 4U);
        reply = Charge_Manager_Handle_Digital_Power_Command(enable,
                                                            target_voltage_mv,
                                                            current_limit_ma);
        Comm_Send_Ack(frame->type, frame->sequence, reply.result, reply.errorCode);
        return;
    }

    argument = 0U;
    has_argument = 0U;
    if(frame->length >= 2U) {
        argument = frame->payload[1];
        has_argument = 1U;
    }

    reply = Charge_Manager_Handle_Command(command_id, argument, has_argument);
    Comm_Send_Ack(frame->type, frame->sequence, reply.result, reply.errorCode);
}

static void Comm_Handle_Param_Set(const bms_proto_frame_t *frame)
{
    bms_charge_parameters_t parameters;
    bms_command_reply_t reply;

    if(Bms_Proto_Decode_Parameters(frame->payload, frame->length, &parameters) == 0U) {
        Comm_Send_Ack(frame->type,
                      frame->sequence,
                      (uint8_t)BMS_CMD_RESULT_ERROR,
                      (uint8_t)BMS_CMD_ERROR_BAD_LENGTH);
        return;
    }

    reply = Charge_Manager_Set_Parameters(&parameters);
    Comm_Send_Ack(frame->type, frame->sequence, reply.result, reply.errorCode);

    /* 参数接收成功后回显一帧，让上位机显示固件最终接受的参数。 */
    if(reply.result == (uint8_t)BMS_CMD_RESULT_OK) {
        Comm_Send_Parameters((uint8_t)BMS_PROTO_FRAME_PARAM_SET, frame->sequence);
    }
}

static void Comm_Send_Afe_Debug(uint8_t sequence)
{
    afe_gd30bm2016_debug_t afe_debug;
    bms_proto_afe_debug_t debug;
    uint8_t payload[BMS_PROTO_AFE_DEBUG_PAYLOAD_SIZE];
    uint8_t length;

    Afe_Gd30bm2016_Get_Debug(&afe_debug);
    memset(&debug, 0, sizeof(debug));
    debug.configFailIndex = afe_debug.configFailIndex;
    debug.cfgupdateSeen = afe_debug.cfgupdateSeen;
    debug.i2cAddrWrite = afe_debug.i2cAddrWrite;
    debug.lastReg = afe_debug.lastReg;
    debug.lastCrcOk = afe_debug.lastCrcOk;
    debug.lastCrcRx = afe_debug.lastCrcRx;
    debug.lastCrcCalc = afe_debug.lastCrcCalc;
    debug.configFailReg = afe_debug.configFailReg;
    debug.lastBatteryStatus = afe_debug.lastBatteryStatus;
    debug.probeVcellMode = afe_debug.probeVcellMode;
    debug.rawCell9Mv = afe_debug.rawCell9Mv;
    debug.rawCell16Mv = afe_debug.rawCell16Mv;
    debug.stackMinusCell1_8Mv = afe_debug.stackMinusCell1_8Mv;
    debug.safetyStatusA = afe_debug.safetyStatusA;
    debug.safetyStatusB = afe_debug.safetyStatusB;
    debug.safetyStatusC = afe_debug.safetyStatusC;
    debug.manufacturingStatus = afe_debug.manufacturingStatus;
    debug.configFailStage = afe_debug.configFailStage;
    debug.configFailStep = afe_debug.configFailStep;
    debug.fetOptions = afe_debug.fetOptions;
    debug.alertActive = Afe_Gd30bm2016_Alert_Active();
    debug.fetStatusOk = Afe_Gd30bm2016_Read_Fet_Status(&debug.fetStatus);
    debug.pathVoltageOk = Afe_Gd30bm2016_Read_Path_Voltages(&debug.pathStackMv,
                                                            &debug.pathPackMv,
                                                            &debug.pathLdMv);

    length = Bms_Proto_Encode_Afe_Debug(&debug, Board_Support_Millis(), payload, sizeof(payload));
    Comm_Send_Frame((uint8_t)BMS_PROTO_FRAME_AFE_DEBUG, sequence, payload, length);
}

static void Comm_Send_Power_Debug(uint8_t sequence)
{
    power_control_state_t control;
    power_manager_state_t manager;
    power_path_manager_state_t path;
    safety_manager_debug_t safety;
    bms_proto_power_debug_t debug;
    bms_status_t status;
    uint8_t payload[BMS_PROTO_POWER_DEBUG_PAYLOAD_SIZE];
    uint8_t length;

    memset(&control, 0, sizeof(control));
    memset(&manager, 0, sizeof(manager));
    memset(&path, 0, sizeof(path));
    memset(&safety, 0, sizeof(safety));
    memset(&debug, 0, sizeof(debug));
    memset(&status, 0, sizeof(status));

    Power_Control_Get_State(&control);
    Power_Manager_Get_State(&manager);
    Power_Path_Manager_Get_State(&path);
    Safety_Manager_Get_Debug(&safety);
    Bms_State_Get_Status(&status);

    debug.sampleValid = App_Tasks_Get_Latest_Power_Sample(&debug.sample);
    debug.controlEnabled = control.enabled;
    debug.workMode = Charge_Manager_Get_Work_Mode();
    debug.chargeState = status.chargeState;
    debug.chargeMode = control.mode;
    debug.powerStageMode = (control.asyncBoostRectifier != 0U) ?
                           (uint8_t)POWER_STAGE_MODE_BOOST_ASYNC :
                           control.powerStageMode;
    debug.targetVoltageMv = control.targetVoltageMv;
    debug.targetCurrentMa = control.targetCurrentMa;
    debug.dutyX100 = control.dutyX100;
    debug.buckDutyX100 = control.buckDutyX100;
    debug.boostLowDutyX100 = control.boostLowDutyX100;
    debug.faultLockout = control.faultLockout;
    debug.hardwareReady = control.hardwareReady;
    debug.hardwareOutputsOn = control.hardwareOutputsOn;
    debug.controlFaultBitmap = control.faultBitmap;
    debug.safetyLatchedFaults = Safety_Manager_Get_Latched_Faults();
    debug.safetyFastFaults = Safety_Manager_Sample_Fast_Faults();
    if((control.mode == (uint8_t)BMS_CHARGE_MODE_DIGITAL_POWER) && (control.enabled != 0U)) {
        debug.availablePowerW = (uint16_t)(((uint32_t)control.targetVoltageMv *
                                            (uint32_t)control.targetCurrentMa) / 1000000U);
        debug.requestedCurrentMa = control.targetCurrentMa;
        debug.limitedCurrentMa = control.targetCurrentMa;
        debug.inputLimitedCurrentMa = control.targetCurrentMa;
        debug.thermalLimitedCurrentMa = control.targetCurrentMa;
        debug.deratingActive = 0U;
    } else {
        debug.availablePowerW = manager.availablePowerW;
        debug.requestedCurrentMa = manager.requestedCurrentMa;
        debug.limitedCurrentMa = manager.limitedCurrentMa;
        debug.inputLimitedCurrentMa = manager.inputLimitedCurrentMa;
        debug.thermalLimitedCurrentMa = manager.thermalLimitedCurrentMa;
        debug.deratingActive = manager.deratingActive;
    }
    debug.externalPowerPresent = path.externalPowerPresent;
    debug.batteryPathEnabled = path.batteryPathEnabled;
    debug.pathOffCommanded = path.pathOffCommanded;
    debug.preconnectFlags = path.preconnectFlags;
    if(control.preconnectActive != 0U) {
        debug.preconnectFlags |= POWER_PATH_PRECONNECT_FLAG_ACTIVE;
    }
    debug.preconnectReason = path.preconnectReason;
    debug.preconnectConfirmCount = path.preconnectConfirmCount;
    debug.preconnectDeltaMv = path.preconnectDeltaMv;
    debug.preconnectThresholdMv = path.preconnectThresholdMv;
    debug.preconnectTargetMv = (control.preconnectActive != 0U) ? control.targetVoltageMv : 0U;
    debug.softCurrentMa = control.softCurrentMa;
    debug.tripReason = control.tripReason;
    debug.tripFaults = control.tripFaults;
    debug.tripIoutMa = control.tripIoutMa;
    debug.tripCurrentRefMa = control.tripCurrentRefMa;
    debug.tripOcpLimitMa = control.tripOcpLimitMa;
    debug.tripVoutMv = control.tripVoutMv;
    debug.tripVinMv = control.tripVinMv;
    debug.tripDutyX100 = control.tripDutyX100;
    debug.tripFaultOcActive = control.tripFaultOcActive;
    debug.safetyTripSource = safety.source;
    debug.safetyTripPowerFaultPin = safety.powerFaultPinLevel;
    debug.safetyTripFaults = safety.faults;

    length = Bms_Proto_Encode_Power_Debug(&debug, Board_Support_Millis(), payload, sizeof(payload));
    Comm_Send_Frame((uint8_t)BMS_PROTO_FRAME_POWER_DEBUG, sequence, payload, length);
}

static void Comm_Send_Fault_Log_Data(uint8_t sequence, uint8_t op, uint8_t result, uint16_t newest_index, const fault_log_record_t *record, uint8_t valid)
{
    uint8_t payload[BMS_PROTO_FAULT_LOG_DATA_PAYLOAD_SIZE];
    uint8_t length;

    length = Bms_Proto_Encode_Fault_Log_Data(op,
                                             result,
                                             Fault_Log_Count(),
                                             newest_index,
                                             record,
                                             valid,
                                             payload,
                                             sizeof(payload));
    Comm_Send_Frame((uint8_t)BMS_PROTO_FRAME_FAULT_LOG_DATA, sequence, payload, length);
}

static void Comm_Handle_Fault_Log_Read(const bms_proto_frame_t *frame)
{
    fault_log_record_t record;
    uint8_t op;
    uint16_t newest_index;
    uint8_t valid;

    if(Bms_Proto_Decode_Fault_Log_Read(frame->payload, frame->length, &op, &newest_index) == 0U || op > 1U) {
        Comm_Send_Ack(frame->type,
                      frame->sequence,
                      (uint8_t)BMS_CMD_RESULT_ERROR,
                      (uint8_t)BMS_CMD_ERROR_BAD_LENGTH);
        return;
    }

    Comm_Send_Ack(frame->type,
                  frame->sequence,
                  (uint8_t)BMS_CMD_RESULT_OK,
                  (uint8_t)BMS_CMD_ERROR_NONE);

    valid = 0U;
    if(op != 0U) {
        valid = Fault_Log_Read(newest_index, &record);
    }
    Comm_Send_Fault_Log_Data(frame->sequence, op, (valid != 0U || op == 0U) ? 0U : 1U, newest_index, &record, valid);
}

static void Comm_Handle_Fault_Log_Clear(const bms_proto_frame_t *frame)
{
    if(Bms_Proto_Decode_Fault_Log_Clear(frame->payload, frame->length) == 0U) {
        Comm_Send_Ack(frame->type,
                      frame->sequence,
                      (uint8_t)BMS_CMD_RESULT_ERROR,
                      (uint8_t)BMS_CMD_ERROR_INVALID_PARAM);
        return;
    }

    Fault_Log_Clear();
    Comm_Send_Ack(frame->type,
                  frame->sequence,
                  (uint8_t)BMS_CMD_RESULT_OK,
                  (uint8_t)BMS_CMD_ERROR_NONE);
    Comm_Send_Fault_Log_Data(frame->sequence, 0U, 0U, 0U, 0, 0U);
}

static void Comm_Handle_Frame(const bms_proto_frame_t *frame)
{
    if(frame == 0) {
        return;
    }

    switch(frame->type) {
    case BMS_PROTO_FRAME_CHARGE_COMMAND:
        Comm_Handle_Charge_Command(frame);
        break;

    case BMS_PROTO_FRAME_PARAM_SET:
        Comm_Handle_Param_Set(frame);
        break;

    case BMS_PROTO_FRAME_PARAM_READ:
        Comm_Send_Ack(frame->type,
                      frame->sequence,
                      (uint8_t)BMS_CMD_RESULT_OK,
                      (uint8_t)BMS_CMD_ERROR_NONE);
        Comm_Send_Parameters((uint8_t)BMS_PROTO_FRAME_PARAM_READ, frame->sequence);
        break;

    case BMS_PROTO_FRAME_AFE_DEBUG_READ:
        Comm_Send_Ack(frame->type,
                      frame->sequence,
                      (uint8_t)BMS_CMD_RESULT_OK,
                      (uint8_t)BMS_CMD_ERROR_NONE);
        Comm_Send_Afe_Debug(frame->sequence);
        break;

    case BMS_PROTO_FRAME_POWER_DEBUG_READ:
        Comm_Send_Ack(frame->type,
                      frame->sequence,
                      (uint8_t)BMS_CMD_RESULT_OK,
                      (uint8_t)BMS_CMD_ERROR_NONE);
        Comm_Send_Power_Debug(frame->sequence);
        break;

    case BMS_PROTO_FRAME_FAULT_LOG_READ:
        Comm_Handle_Fault_Log_Read(frame);
        break;

    case BMS_PROTO_FRAME_FAULT_LOG_CLEAR:
        Comm_Handle_Fault_Log_Clear(frame);
        break;

    default:
        Comm_Send_Ack(frame->type,
                      frame->sequence,
                      (uint8_t)BMS_CMD_RESULT_ERROR,
                      (uint8_t)BMS_CMD_ERROR_UNKNOWN_COMMAND);
        break;
    }
}

static void Comm_Send_Realtime_Status(void)
{
    bms_status_t status;
    uint8_t payload[BMS_PROTO_REALTIME_PAYLOAD_SIZE];
    uint8_t length;

    Bms_State_Get_Status(&status);
    length = Bms_Proto_Encode_Realtime(&status, payload, sizeof(payload));
    Comm_Send_Frame((uint8_t)BMS_PROTO_FRAME_REAL_TIME_STATUS, s_tx_sequence, payload, length);
    s_tx_sequence++;
}

static void Comm_Send_Cell_Voltages(void)
{
    bms_status_t status;
    uint8_t payload[BMS_PROTO_CELL_PAYLOAD_SIZE];
    uint8_t length;

    Bms_State_Get_Status(&status);
    length = Bms_Proto_Encode_Cells(&status, payload, sizeof(payload));
    Comm_Send_Frame((uint8_t)BMS_PROTO_FRAME_CELL_VOLTAGES, s_tx_sequence, payload, length);
    s_tx_sequence++;
}

static void Comm_Send_Fault_Alarm(const bms_status_t *status)
{
    uint8_t payload[BMS_PROTO_REALTIME_PAYLOAD_SIZE];
    uint8_t length;

    if(status == 0) {
        return;
    }

    length = Bms_Proto_Encode_Realtime(status, payload, sizeof(payload));
    Comm_Send_Frame((uint8_t)BMS_PROTO_FRAME_FAULT_ALARM, s_tx_sequence, payload, length);
    s_tx_sequence++;
}

void Bms_Comm_Service_Init(void)
{
    Bms_Proto_Parser_Init(&s_parser);
    Bms_Uart_Init();
    s_tx_sequence = 0U;
    /*
     * Send one status frame immediately after UART init. This gives a serial
     * terminal a boot-time marker even if a later peripheral init stalls.
     */
    s_last_status_ms = Board_Support_Millis();
    s_last_cells_ms = s_last_status_ms;
    s_last_fault_bitmap = 0U;
    Comm_Send_Realtime_Status();
}

void Bms_Comm_Service_Poll(uint32_t now_ms)
{
    /* 115200bps 下 10ms 约 115 字节，128 字节可一次读走一个周期内的满速输入。 */
    uint8_t rx[128];
    uint16_t count;
    uint16_t i;
    bms_proto_frame_t frame;
    bms_proto_parse_result_t result;
    bms_status_t status;

    /* 读走当前 UART 中已到达的字节，未完成的半帧保留在 s_parser 中。 */
    count = Bms_Uart_Read(rx, sizeof(rx));
    for(i = 0U; i < count; i++) {
        result = Bms_Proto_Parse_Byte(&s_parser, rx[i], &frame);
        if(result == BMS_PROTO_PARSE_FRAME) {
            Comm_Handle_Frame(&frame);
        }
    }

    /* 使用无符号减法，保证 Board_Support_Millis() 回绕时周期判断仍然正确。 */
    if((uint32_t)(now_ms - s_last_status_ms) >= 100U) {
        s_last_status_ms = now_ms;
        Comm_Send_Realtime_Status();
    }

    if((uint32_t)(now_ms - s_last_cells_ms) >= 500U) {
        s_last_cells_ms = now_ms;
        Comm_Send_Cell_Voltages();
    }

    Bms_State_Get_Status(&status);
    if(status.faultBitmap != s_last_fault_bitmap) {
        s_last_fault_bitmap = status.faultBitmap;
        Comm_Send_Fault_Alarm(&status);
    }
}
