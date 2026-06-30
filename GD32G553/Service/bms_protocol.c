#include "bms_protocol.h"

#include <string.h>

/*
 * 与 BMSMonitorSrc 上位机共用的二进制 UART 协议。
 *
 * 帧格式：
 *   AA 55 | version | type | sequence | payload_length | payload | crc16_le
 * CRC16/MODBUS 从 version 计算到 payload，不包含前面的 AA55 帧头。
 * 负载中的多字节字段统一使用小端格式，便于与 GD32 内存布局对应。
 */
enum {
    PARSER_WAIT_SOF0 = 0,
    PARSER_WAIT_SOF1,
    PARSER_READ_VER,
    PARSER_READ_TYPE,
    PARSER_READ_SEQ,
    PARSER_READ_LEN,
    PARSER_READ_PAYLOAD,
    PARSER_READ_CRC_LO,
    PARSER_READ_CRC_HI
};

static void Write_U16_Le(uint8_t *payload, uint8_t *offset, uint16_t value)
{
    payload[*offset] = (uint8_t)(value & 0xFFU);
    (*offset)++;
    payload[*offset] = (uint8_t)((value >> 8) & 0xFFU);
    (*offset)++;
}

static void Write_I16_Le(uint8_t *payload, uint8_t *offset, int16_t value)
{
    Write_U16_Le(payload, offset, (uint16_t)value);
}

static void Write_U32_Le(uint8_t *payload, uint8_t *offset, uint32_t value)
{
    payload[*offset] = (uint8_t)(value & 0xFFUL);
    (*offset)++;
    payload[*offset] = (uint8_t)((value >> 8) & 0xFFUL);
    (*offset)++;
    payload[*offset] = (uint8_t)((value >> 16) & 0xFFUL);
    (*offset)++;
    payload[*offset] = (uint8_t)((value >> 24) & 0xFFUL);
    (*offset)++;
}

static uint16_t Read_U16_Le(const uint8_t *payload, uint8_t offset)
{
    return (uint16_t)payload[offset] | (uint16_t)((uint16_t)payload[offset + 1U] << 8);
}

static int16_t Read_I16_Le(const uint8_t *payload, uint8_t offset)
{
    return (int16_t)Read_U16_Le(payload, offset);
}

static uint32_t Read_U32_Le(const uint8_t *payload, uint8_t offset)
{
    return (uint32_t)payload[offset] |
           ((uint32_t)payload[offset + 1U] << 8) |
           ((uint32_t)payload[offset + 2U] << 16) |
           ((uint32_t)payload[offset + 3U] << 24);
}

void Bms_Proto_Parser_Init(bms_proto_parser_t *parser)
{
    if(parser == 0) {
        return;
    }

    memset(parser, 0, sizeof(*parser));
    parser->state = PARSER_WAIT_SOF0;
}

uint16_t Bms_Proto_Crc16_Modbus(const uint8_t *data, uint16_t length)
{
    uint16_t crc;
    uint16_t i;
    uint8_t bit;

    crc = 0xFFFFU;
    for(i = 0U; i < length; i++) {
        crc ^= data[i];
        for(bit = 0U; bit < 8U; bit++) {
            if((crc & 0x0001U) != 0U) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001U);
            } else {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }

    return crc;
}

bms_proto_parse_result_t Bms_Proto_Parse_Byte(bms_proto_parser_t *parser,
                                              uint8_t byte,
                                              bms_proto_frame_t *frame)
{
    uint8_t crc_data[4U + BMS_PROTO_MAX_PAYLOAD_SIZE];
    uint16_t crc;
    uint16_t received_crc;

    if(parser == 0 || frame == 0) {
        return BMS_PROTO_PARSE_ERROR;
    }

    switch(parser->state) {
    case PARSER_WAIT_SOF0:
        /* 流式解析器可以接收任意长度字节块，并通过 AA55 自动重新同步。 */
        if(byte == BMS_PROTO_SOF0) {
            parser->state = PARSER_WAIT_SOF1;
        }
        break;

    case PARSER_WAIT_SOF1:
        if(byte == BMS_PROTO_SOF1) {
            parser->state = PARSER_READ_VER;
        } else if(byte != BMS_PROTO_SOF0) {
            parser->state = PARSER_WAIT_SOF0;
        }
        break;

    case PARSER_READ_VER:
        parser->frame.version = byte;
        if(byte == BMS_PROTO_VERSION) {
            parser->state = PARSER_READ_TYPE;
        } else {
            parser->state = PARSER_WAIT_SOF0;
            return BMS_PROTO_PARSE_ERROR;
        }
        break;

    case PARSER_READ_TYPE:
        parser->frame.type = byte;
        parser->state = PARSER_READ_SEQ;
        break;

    case PARSER_READ_SEQ:
        parser->frame.sequence = byte;
        parser->state = PARSER_READ_LEN;
        break;

    case PARSER_READ_LEN:
        parser->frame.length = byte;
        parser->payloadIndex = 0U;
        /* 长度超限的帧立即丢弃，避免 payload 缓冲区溢出。 */
        if(byte > BMS_PROTO_MAX_PAYLOAD_SIZE) {
            parser->state = PARSER_WAIT_SOF0;
            return BMS_PROTO_PARSE_ERROR;
        }
        parser->state = (byte == 0U) ? PARSER_READ_CRC_LO : PARSER_READ_PAYLOAD;
        break;

    case PARSER_READ_PAYLOAD:
        parser->frame.payload[parser->payloadIndex] = byte;
        parser->payloadIndex++;
        if(parser->payloadIndex >= parser->frame.length) {
            parser->state = PARSER_READ_CRC_LO;
        }
        break;

    case PARSER_READ_CRC_LO:
        parser->receivedCrcLo = byte;
        parser->state = PARSER_READ_CRC_HI;
        break;

    case PARSER_READ_CRC_HI:
        /* 重新拼出 CRC 输入数据，不依赖已经被状态机消费掉的原始字节。 */
        crc_data[0] = parser->frame.version;
        crc_data[1] = parser->frame.type;
        crc_data[2] = parser->frame.sequence;
        crc_data[3] = parser->frame.length;
        if(parser->frame.length != 0U) {
            memcpy(&crc_data[4], parser->frame.payload, parser->frame.length);
        }

        crc = Bms_Proto_Crc16_Modbus(crc_data, (uint16_t)(4U + parser->frame.length));
        received_crc = (uint16_t)parser->receivedCrcLo | (uint16_t)((uint16_t)byte << 8);
        parser->state = PARSER_WAIT_SOF0;

        if(crc == received_crc) {
            *frame = parser->frame;
            return BMS_PROTO_PARSE_FRAME;
        }
        return BMS_PROTO_PARSE_ERROR;

    default:
        Bms_Proto_Parser_Init(parser);
        return BMS_PROTO_PARSE_ERROR;
    }

    return BMS_PROTO_PARSE_NONE;
}

uint16_t Bms_Proto_Build_Frame(uint8_t type,
                               uint8_t sequence,
                               const uint8_t *payload,
                               uint8_t payload_length,
                               uint8_t *out,
                               uint16_t out_size)
{
    uint16_t frame_size;
    uint16_t crc;

    frame_size = (uint16_t)(BMS_PROTO_HEADER_SIZE + payload_length + BMS_PROTO_CRC_SIZE);
    if(out == 0 ||
       out_size < frame_size ||
       payload_length > BMS_PROTO_MAX_PAYLOAD_SIZE ||
       (payload_length != 0U && payload == 0)) {
        return 0U;
    }

    /* 帧头固定长度，payload 长度字段按协议设计保持 1 字节。 */
    out[0] = BMS_PROTO_SOF0;
    out[1] = BMS_PROTO_SOF1;
    out[2] = BMS_PROTO_VERSION;
    out[3] = type;
    out[4] = sequence;
    out[5] = payload_length;

    if(payload_length != 0U && payload != 0) {
        memcpy(&out[6], payload, payload_length);
    }

    crc = Bms_Proto_Crc16_Modbus(&out[2], (uint16_t)(4U + payload_length));
    out[6U + payload_length] = (uint8_t)(crc & 0xFFU);
    out[7U + payload_length] = (uint8_t)((crc >> 8) & 0xFFU);

    return frame_size;
}

uint8_t Bms_Proto_Encode_Realtime(const bms_status_t *status, uint8_t *payload, uint8_t payload_size)
{
    uint8_t offset;
    uint32_t i;

    if(status == 0 || payload == 0 || payload_size < BMS_PROTO_REALTIME_PAYLOAD_SIZE) {
        return 0U;
    }

    /* 此处字段顺序必须与上位机 Protocol::parseRealTimeStatus() 保持一致。 */
    offset = 0U;
    Write_U32_Le(payload, &offset, status->timestampMs);
    Write_U16_Le(payload, &offset, status->packVoltageMv);
    Write_I16_Le(payload, &offset, status->chargeCurrentMa);
    Write_U16_Le(payload, &offset, status->inputVoltageMv);
    Write_U16_Le(payload, &offset, status->outputVoltageMv);
    Write_U16_Le(payload, &offset, status->dutyX100);
    Write_U16_Le(payload, &offset, status->socX10);
    payload[offset] = status->chargeState;
    offset++;
    Write_U32_Le(payload, &offset, status->faultBitmap);

    for(i = 0U; i < BMS_TEMP_COUNT; i++) {
        Write_I16_Le(payload, &offset, status->temperaturesX10[i]);
    }
    payload[offset] = status->chargeMode;
    offset++;
    payload[offset] = status->workMode;
    offset++;

    return offset;
}

uint8_t Bms_Proto_Encode_Cells(const bms_status_t *status, uint8_t *payload, uint8_t payload_size)
{
    uint8_t offset;
    uint8_t snapshot_length;
    uint32_t i;

    if(status == 0 || payload == 0 || payload_size < BMS_PROTO_CELL_BASE_PAYLOAD_SIZE) {
        return 0U;
    }

    /* 9 串电池包：先放 9 路单体电压，再放汇总值和均衡位图。 */
    offset = 0U;
    for(i = 0U; i < BMS_CELL_COUNT; i++) {
        Write_U16_Le(payload, &offset, status->cellMv[i]);
    }
    Write_U16_Le(payload, &offset, status->cellMaxMv);
    Write_U16_Le(payload, &offset, status->cellMinMv);
    Write_U16_Le(payload, &offset, status->cellDeltaMv);
    Write_U16_Le(payload, &offset, status->balanceBitmap);

    if(payload_size >= BMS_PROTO_CELL_PAYLOAD_SIZE) {
        snapshot_length = Bms_Proto_Encode_Realtime(status,
                                                    &payload[offset],
                                                    (uint8_t)(payload_size - offset));
        if(snapshot_length != 0U) {
            offset = (uint8_t)(offset + snapshot_length);
        }
    }

    return offset;
}

uint8_t Bms_Proto_Encode_Parameters(const bms_charge_parameters_t *parameters, uint8_t *payload, uint8_t payload_size)
{
    uint8_t offset;

    if(parameters == 0 || payload == 0 || payload_size < BMS_PROTO_PARAM_PAYLOAD_SIZE) {
        return 0U;
    }

    offset = 0U;
    Write_U16_Le(payload, &offset, parameters->targetVoltageMv);
    Write_U16_Le(payload, &offset, parameters->targetCurrentMa);
    Write_U16_Le(payload, &offset, parameters->cutoffCurrentMa);
    Write_U16_Le(payload, &offset, parameters->cellOvpMv);
    Write_U16_Le(payload, &offset, parameters->cellUvpMv);
    Write_I16_Le(payload, &offset, parameters->tempOtpX10);
    Write_U16_Le(payload, &offset, parameters->balanceDeltaMv);

    return offset;
}

uint8_t Bms_Proto_Decode_Parameters(const uint8_t *payload, uint8_t payload_length, bms_charge_parameters_t *parameters)
{
    uint8_t offset;

    if(payload == 0 || parameters == 0 || payload_length < BMS_PROTO_PARAM_PAYLOAD_SIZE) {
        return 0U;
    }

    /* 上位机直接发送物理单位：mV、mA、0.1 摄氏度和 mV 压差。 */
    offset = 0U;
    parameters->targetVoltageMv = Read_U16_Le(payload, offset);
    offset = (uint8_t)(offset + 2U);
    parameters->targetCurrentMa = Read_U16_Le(payload, offset);
    offset = (uint8_t)(offset + 2U);
    parameters->cutoffCurrentMa = Read_U16_Le(payload, offset);
    offset = (uint8_t)(offset + 2U);
    parameters->cellOvpMv = Read_U16_Le(payload, offset);
    offset = (uint8_t)(offset + 2U);
    parameters->cellUvpMv = Read_U16_Le(payload, offset);
    offset = (uint8_t)(offset + 2U);
    parameters->tempOtpX10 = Read_I16_Le(payload, offset);
    offset = (uint8_t)(offset + 2U);
    parameters->balanceDeltaMv = Read_U16_Le(payload, offset);

    return 1U;
}

uint8_t Bms_Proto_Encode_Afe_Debug(const bms_proto_afe_debug_t *debug,
                                   uint32_t timestamp_ms,
                                   uint8_t *payload,
                                   uint8_t payload_size)
{
    uint8_t offset;
    uint8_t flags;

    if(debug == 0 || payload == 0 || payload_size < BMS_PROTO_AFE_DEBUG_PAYLOAD_SIZE) {
        return 0U;
    }

    flags = 0U;
    if(debug->pathVoltageOk != 0U) {
        flags |= (1U << 0);
    }
    if(debug->fetStatusOk != 0U) {
        flags |= (1U << 1);
    }
    if(debug->alertActive != 0U) {
        flags |= (1U << 2);
    }
    if(debug->lastCrcOk == 1U) {
        flags |= (1U << 3);
    }
    if(debug->lastCrcOk == 2U) {
        flags |= (1U << 4);
    }
    if(debug->cfgupdateSeen != 0U) {
        flags |= (1U << 5);
    }

    offset = 0U;
    Write_U32_Le(payload, &offset, timestamp_ms);
    payload[offset++] = flags;
    payload[offset++] = debug->i2cAddrWrite;
    payload[offset++] = debug->lastReg;
    payload[offset++] = debug->lastCrcOk;
    payload[offset++] = debug->lastCrcRx;
    payload[offset++] = debug->lastCrcCalc;
    payload[offset++] = debug->configFailIndex;
    payload[offset++] = debug->fetStatus;
    Write_U16_Le(payload, &offset, debug->configFailReg);
    Write_U16_Le(payload, &offset, debug->lastBatteryStatus);
    Write_U16_Le(payload, &offset, debug->probeVcellMode);
    Write_U16_Le(payload, &offset, debug->rawCell9Mv);
    Write_U16_Le(payload, &offset, debug->rawCell16Mv);
    Write_U16_Le(payload, &offset, debug->stackMinusCell1_8Mv);
    Write_U16_Le(payload, &offset, debug->pathStackMv);
    Write_U16_Le(payload, &offset, debug->pathPackMv);
    Write_U16_Le(payload, &offset, debug->pathLdMv);
    payload[offset++] = debug->safetyStatusA;
    payload[offset++] = debug->safetyStatusB;
    payload[offset++] = debug->safetyStatusC;
    payload[offset++] = debug->manufacturingStatus;
    payload[offset++] = debug->configFailStage;
    payload[offset++] = debug->configFailStep;
    payload[offset++] = debug->fetOptions;

    return offset;
}

uint8_t Bms_Proto_Encode_Power_Debug(const bms_proto_power_debug_t *debug,
                                     uint32_t timestamp_ms,
                                     uint8_t *payload,
                                     uint8_t payload_size)
{
    uint8_t offset;
    uint8_t flags;

    if(debug == 0 || payload == 0 || payload_size < BMS_PROTO_POWER_DEBUG_PAYLOAD_SIZE) {
        return 0U;
    }

    flags = 0U;
    if(debug->faultLockout != 0U) {
        flags |= (1U << 0);
    }
    if(debug->hardwareReady != 0U) {
        flags |= (1U << 1);
    }
    if(debug->hardwareOutputsOn != 0U) {
        flags |= (1U << 2);
    }
    if(debug->deratingActive != 0U) {
        flags |= (1U << 3);
    }
    if(debug->sampleValid != 0U) {
        flags |= (1U << 4);
    }
    if(debug->externalPowerPresent != 0U) {
        flags |= (1U << 5);
    }
    if(debug->batteryPathEnabled != 0U) {
        flags |= (1U << 6);
    }
    if(debug->pathOffCommanded != 0U) {
        flags |= (1U << 7);
    }

    offset = 0U;
    Write_U32_Le(payload, &offset, timestamp_ms);
    payload[offset++] = flags;
    payload[offset++] = debug->controlEnabled;
    payload[offset++] = debug->chargeMode;
    payload[offset++] = debug->powerStageMode;
    Write_U16_Le(payload, &offset, debug->targetVoltageMv);
    Write_U16_Le(payload, &offset, debug->targetCurrentMa);
    Write_U16_Le(payload, &offset, debug->dutyX100);
    Write_U16_Le(payload, &offset, debug->buckDutyX100);
    Write_U16_Le(payload, &offset, debug->boostLowDutyX100);
    Write_U32_Le(payload, &offset, debug->controlFaultBitmap);
    Write_U32_Le(payload, &offset, debug->safetyLatchedFaults);
    Write_U32_Le(payload, &offset, debug->safetyFastFaults);
    Write_U16_Le(payload, &offset, debug->availablePowerW);
    Write_U16_Le(payload, &offset, debug->requestedCurrentMa);
    Write_U16_Le(payload, &offset, debug->limitedCurrentMa);
    Write_U16_Le(payload, &offset, debug->inputLimitedCurrentMa);
    Write_U16_Le(payload, &offset, debug->thermalLimitedCurrentMa);
    Write_U16_Le(payload, &offset, debug->sample.inputVoltageMv);
    Write_U16_Le(payload, &offset, debug->sample.outputVoltageMv);
    Write_I16_Le(payload, &offset, debug->sample.inputCurrentMa);
    Write_I16_Le(payload, &offset, debug->sample.outputCurrentMa);
    Write_I16_Le(payload, &offset, debug->sample.mosTempX10);
    Write_I16_Le(payload, &offset, debug->sample.inductorTempX10);
    Write_U32_Le(payload, &offset, debug->sample.faultBitmap);
    payload[offset++] = debug->sample.faultOcActive;
    Write_U16_Le(payload, &offset, debug->sample.iinRaw);
    Write_U16_Le(payload, &offset, debug->sample.vinRaw);
    Write_U16_Le(payload, &offset, debug->sample.voutRaw);
    Write_U16_Le(payload, &offset, debug->sample.ioutRaw);
    Write_U16_Le(payload, &offset, debug->sample.mosTempRaw);
    Write_U16_Le(payload, &offset, debug->sample.inductorTempRaw);
    payload[offset++] = debug->workMode;
    payload[offset++] = debug->chargeState;
    payload[offset++] = debug->preconnectFlags;
    payload[offset++] = debug->preconnectReason;
    payload[offset++] = debug->preconnectConfirmCount;
    Write_U16_Le(payload, &offset, debug->preconnectDeltaMv);
    Write_U16_Le(payload, &offset, debug->preconnectThresholdMv);
    Write_U16_Le(payload, &offset, debug->preconnectTargetMv);
    Write_U16_Le(payload, &offset, debug->softCurrentMa);
    payload[offset++] = debug->tripReason;
    Write_U32_Le(payload, &offset, debug->tripFaults);
    Write_I16_Le(payload, &offset, debug->tripIoutMa);
    Write_U16_Le(payload, &offset, debug->tripCurrentRefMa);
    Write_U16_Le(payload, &offset, debug->tripOcpLimitMa);
    Write_U16_Le(payload, &offset, debug->tripVoutMv);
    Write_U16_Le(payload, &offset, debug->tripVinMv);
    Write_U16_Le(payload, &offset, debug->tripDutyX100);
    payload[offset++] = debug->tripFaultOcActive;
    payload[offset++] = debug->safetyTripSource;
    payload[offset++] = debug->safetyTripPowerFaultPin;
    Write_U32_Le(payload, &offset, debug->safetyTripFaults);

    return offset;
}

uint8_t Bms_Proto_Decode_Fault_Log_Read(const uint8_t *payload,
                                        uint8_t payload_length,
                                        uint8_t *op,
                                        uint16_t *newest_index)
{
    if(payload == 0 || op == 0 || newest_index == 0 || payload_length < BMS_PROTO_FAULT_LOG_READ_PAYLOAD_SIZE) {
        return 0U;
    }

    *op = payload[0];
    *newest_index = Read_U16_Le(payload, 1U);
    return 1U;
}

uint8_t Bms_Proto_Decode_Fault_Log_Clear(const uint8_t *payload, uint8_t payload_length)
{
    uint32_t magic;

    if(payload == 0 || payload_length < BMS_PROTO_FAULT_LOG_CLEAR_PAYLOAD_SIZE) {
        return 0U;
    }

    magic = Read_U32_Le(payload, 0U);
    if(magic != 0x524C4346UL || payload[4] != 0xA5U) {
        return 0U;
    }

    return 1U;
}

uint8_t Bms_Proto_Encode_Fault_Log_Data(uint8_t op,
                                        uint8_t result,
                                        uint16_t total_count,
                                        uint16_t newest_index,
                                        const fault_log_record_t *record,
                                        uint8_t record_valid,
                                        uint8_t *payload,
                                        uint8_t payload_size)
{
    uint8_t offset;
    uint8_t valid;

    if(payload == 0 || payload_size < BMS_PROTO_FAULT_LOG_DATA_PAYLOAD_SIZE) {
        return 0U;
    }

    valid = (record != 0 && record_valid != 0U) ? 1U : 0U;
    offset = 0U;
    payload[offset++] = op;
    payload[offset++] = result;
    Write_U16_Le(payload, &offset, total_count);
    Write_U16_Le(payload, &offset, newest_index);
    payload[offset++] = valid;
    payload[offset++] = FAULT_LOG_VERSION;

    if(valid != 0U) {
        Write_U32_Le(payload, &offset, record->sequence);
        Write_U32_Le(payload, &offset, record->runtimeMs);
        Write_U32_Le(payload, &offset, record->faultBitmap);
        Write_U16_Le(payload, &offset, record->vinMv);
        Write_U16_Le(payload, &offset, record->voutMv);
        Write_I16_Le(payload, &offset, record->ioutMa);
        Write_I16_Le(payload, &offset, record->mosTempX10);
        Write_I16_Le(payload, &offset, record->inductorTempX10);
        Write_U16_Le(payload, &offset, record->socX10);
        payload[offset++] = record->chargeState;
        payload[offset++] = record->chargeMode;
        Write_U16_Le(payload, &offset, record->dutyX100);
        Write_U16_Le(payload, &offset, record->cellMinMv);
        Write_U16_Le(payload, &offset, record->cellMaxMv);
        Write_U32_Le(payload, &offset, record->crc32);
    } else {
        while(offset < BMS_PROTO_FAULT_LOG_DATA_PAYLOAD_SIZE) {
            payload[offset++] = 0U;
        }
    }

    return offset;
}

uint8_t Bms_Proto_Encode_Ack(uint8_t ack_type,
                             uint8_t ack_sequence,
                             uint8_t result,
                             uint8_t error_code,
                             uint8_t *payload,
                             uint8_t payload_size)
{
    if(payload == 0 || payload_size < BMS_PROTO_ACK_PAYLOAD_SIZE) {
        return 0U;
    }

    payload[0] = ack_type;
    payload[1] = ack_sequence;
    payload[2] = result;
    payload[3] = error_code;

    return BMS_PROTO_ACK_PAYLOAD_SIZE;
}
