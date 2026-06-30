#ifndef BMS_PROTOCOL_H
#define BMS_PROTOCOL_H

#include "bms_types.h"
#include "fault_log.h"

#include <stdint.h>

#define BMS_PROTO_VERSION                      0x01U
#define BMS_PROTO_SOF0                         0xAAU
#define BMS_PROTO_SOF1                         0x55U
#define BMS_PROTO_MAX_PAYLOAD_SIZE             112U
#define BMS_PROTO_HEADER_SIZE                  6U
#define BMS_PROTO_CRC_SIZE                     2U
#define BMS_PROTO_REALTIME_PAYLOAD_SIZE        (23U + (2U * BMS_TEMP_COUNT))
#define BMS_PROTO_CELL_BASE_PAYLOAD_SIZE       26U
#define BMS_PROTO_CELL_POWER_SNAPSHOT_SIZE     BMS_PROTO_REALTIME_PAYLOAD_SIZE
#define BMS_PROTO_CELL_PAYLOAD_SIZE            (BMS_PROTO_CELL_BASE_PAYLOAD_SIZE + \
                                                BMS_PROTO_CELL_POWER_SNAPSHOT_SIZE)
#define BMS_PROTO_PARAM_PAYLOAD_SIZE           14U
#define BMS_PROTO_ACK_PAYLOAD_SIZE             4U
#define BMS_PROTO_AFE_DEBUG_PAYLOAD_SIZE       37U
#define BMS_PROTO_POWER_DEBUG_PAYLOAD_SIZE     106U
#define BMS_PROTO_FAULT_LOG_READ_PAYLOAD_SIZE  4U
#define BMS_PROTO_FAULT_LOG_DATA_PAYLOAD_SIZE  44U
#define BMS_PROTO_FAULT_LOG_CLEAR_PAYLOAD_SIZE 8U

typedef enum {
    BMS_PROTO_FRAME_REAL_TIME_STATUS = 0x01,
    BMS_PROTO_FRAME_CELL_VOLTAGES = 0x02,
    BMS_PROTO_FRAME_FAULT_ALARM = 0x03,
    BMS_PROTO_FRAME_AFE_DEBUG = 0x04,
    BMS_PROTO_FRAME_POWER_DEBUG = 0x05,
    BMS_PROTO_FRAME_FAULT_LOG_DATA = 0x06,
    BMS_PROTO_FRAME_CHARGE_COMMAND = 0x10,
    BMS_PROTO_FRAME_PARAM_SET = 0x11,
    BMS_PROTO_FRAME_PARAM_READ = 0x12,
    BMS_PROTO_FRAME_AFE_DEBUG_READ = 0x13,
    BMS_PROTO_FRAME_POWER_DEBUG_READ = 0x14,
    BMS_PROTO_FRAME_FAULT_LOG_READ = 0x15,
    BMS_PROTO_FRAME_FAULT_LOG_CLEAR = 0x16,
    BMS_PROTO_FRAME_ACK = 0x7F
} bms_proto_frame_type_t;

typedef enum {
    BMS_PROTO_PARSE_NONE = 0,
    BMS_PROTO_PARSE_FRAME = 1,
    BMS_PROTO_PARSE_ERROR = 2
} bms_proto_parse_result_t;

typedef struct {
    uint8_t version;
    uint8_t type;
    uint8_t sequence;
    uint8_t length;
    uint8_t payload[BMS_PROTO_MAX_PAYLOAD_SIZE];
} bms_proto_frame_t;

typedef struct {
    uint8_t state;
    uint8_t payloadIndex;
    uint8_t receivedCrcLo;
    bms_proto_frame_t frame;
} bms_proto_parser_t;

typedef struct {
    uint8_t configFailIndex;
    uint8_t cfgupdateSeen;
    uint8_t i2cAddrWrite;
    uint8_t lastReg;
    uint8_t lastCrcOk;
    uint8_t lastCrcRx;
    uint8_t lastCrcCalc;
    uint16_t configFailReg;
    uint16_t lastBatteryStatus;
    uint16_t probeVcellMode;
    uint16_t rawCell9Mv;
    uint16_t rawCell16Mv;
    uint16_t stackMinusCell1_8Mv;
    uint8_t alertActive;
    uint8_t fetStatusOk;
    uint8_t fetStatus;
    uint8_t pathVoltageOk;
    uint16_t pathStackMv;
    uint16_t pathPackMv;
    uint16_t pathLdMv;
    uint8_t safetyStatusA;
    uint8_t safetyStatusB;
    uint8_t safetyStatusC;
    uint8_t manufacturingStatus;
    uint8_t configFailStage;
    uint8_t configFailStep;
    uint8_t fetOptions;
} bms_proto_afe_debug_t;

typedef struct {
    uint8_t sampleValid;
    bms_power_sample_t sample;
    uint8_t controlEnabled;
    uint8_t workMode;
    uint8_t chargeState;
    uint8_t chargeMode;
    uint8_t powerStageMode;
    uint16_t targetVoltageMv;
    uint16_t targetCurrentMa;
    uint16_t dutyX100;
    uint16_t buckDutyX100;
    uint16_t boostLowDutyX100;
    uint8_t faultLockout;
    uint8_t hardwareReady;
    uint8_t hardwareOutputsOn;
    uint32_t controlFaultBitmap;
    uint32_t safetyLatchedFaults;
    uint32_t safetyFastFaults;
    uint16_t availablePowerW;
    uint16_t requestedCurrentMa;
    uint16_t limitedCurrentMa;
    uint16_t inputLimitedCurrentMa;
    uint16_t thermalLimitedCurrentMa;
    uint8_t deratingActive;
    uint8_t externalPowerPresent;
    uint8_t batteryPathEnabled;
    uint8_t pathOffCommanded;
    uint8_t preconnectFlags;
    uint8_t preconnectReason;
    uint8_t preconnectConfirmCount;
    uint16_t preconnectDeltaMv;
    uint16_t preconnectThresholdMv;
    uint16_t preconnectTargetMv;
    uint16_t softCurrentMa;
    uint8_t tripReason;
    uint32_t tripFaults;
    int16_t tripIoutMa;
    uint16_t tripCurrentRefMa;
    uint16_t tripOcpLimitMa;
    uint16_t tripVoutMv;
    uint16_t tripVinMv;
    uint16_t tripDutyX100;
    uint8_t tripFaultOcActive;
    uint8_t safetyTripSource;
    uint8_t safetyTripPowerFaultPin;
    uint32_t safetyTripFaults;
} bms_proto_power_debug_t;

void Bms_Proto_Parser_Init(bms_proto_parser_t *parser);
bms_proto_parse_result_t Bms_Proto_Parse_Byte(bms_proto_parser_t *parser,
                                              uint8_t byte,
                                              bms_proto_frame_t *frame);
uint16_t Bms_Proto_Crc16_Modbus(const uint8_t *data, uint16_t length);
uint16_t Bms_Proto_Build_Frame(uint8_t type,
                               uint8_t sequence,
                               const uint8_t *payload,
                               uint8_t payload_length,
                               uint8_t *out,
                               uint16_t out_size);

uint8_t Bms_Proto_Encode_Realtime(const bms_status_t *status, uint8_t *payload, uint8_t payload_size);
uint8_t Bms_Proto_Encode_Cells(const bms_status_t *status, uint8_t *payload, uint8_t payload_size);
uint8_t Bms_Proto_Encode_Parameters(const bms_charge_parameters_t *parameters, uint8_t *payload, uint8_t payload_size);
uint8_t Bms_Proto_Decode_Parameters(const uint8_t *payload, uint8_t payload_length, bms_charge_parameters_t *parameters);
uint8_t Bms_Proto_Encode_Afe_Debug(const bms_proto_afe_debug_t *debug,
                                   uint32_t timestamp_ms,
                                   uint8_t *payload,
                                   uint8_t payload_size);
uint8_t Bms_Proto_Encode_Power_Debug(const bms_proto_power_debug_t *debug,
                                     uint32_t timestamp_ms,
                                     uint8_t *payload,
                                     uint8_t payload_size);
uint8_t Bms_Proto_Decode_Fault_Log_Read(const uint8_t *payload,
                                        uint8_t payload_length,
                                        uint8_t *op,
                                        uint16_t *newest_index);
uint8_t Bms_Proto_Decode_Fault_Log_Clear(const uint8_t *payload, uint8_t payload_length);
uint8_t Bms_Proto_Encode_Fault_Log_Data(uint8_t op,
                                        uint8_t result,
                                        uint16_t total_count,
                                        uint16_t newest_index,
                                        const fault_log_record_t *record,
                                        uint8_t record_valid,
                                        uint8_t *payload,
                                        uint8_t payload_size);
uint8_t Bms_Proto_Encode_Ack(uint8_t ack_type,
                             uint8_t ack_sequence,
                             uint8_t result,
                             uint8_t error_code,
                             uint8_t *payload,
                             uint8_t payload_size);

#endif
