#include "param_storage.h"

#include "bms_board_config.h"
#include "flash_storage.h"

#include <string.h>

#define PARAM_RECORD_MAGIC                    0x5041524DUL
#define PARAM_SAVE_DELAY_MS                   3000U

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t length;
    uint32_t sequence;
    bms_system_config_t config;
    uint32_t crc32;
} param_record_t;

static bms_system_config_t s_config;
static uint32_t s_sequence;
static uint8_t s_dirty;
static uint32_t s_dirty_since_ms;

static uint8_t Param_Record_Valid(const param_record_t *record)
{
    uint32_t crc;

    if(record == 0) {
        return 0U;
    }
    if(record->magic != PARAM_RECORD_MAGIC ||
       record->version != BMS_PARAM_STORAGE_VERSION ||
       record->length != sizeof(bms_system_config_t)) {
        return 0U;
    }

    crc = Flash_Storage_Crc32(record, sizeof(*record) - sizeof(record->crc32));
    return (crc == record->crc32) ? 1U : 0U;
}

static uint32_t Param_Record_Address(uint32_t index)
{
    return BMS_PARAM_STORAGE_BASE_ADDRESS + (index * sizeof(param_record_t));
}

static uint32_t Param_Record_Capacity(void)
{
    return BMS_PARAM_STORAGE_SIZE_BYTES / sizeof(param_record_t);
}

void Param_Storage_Restore_Default(bms_system_config_t *config)
{
    if(config == 0) {
        return;
    }

    memset(config, 0, sizeof(*config));
    config->charge.targetVoltageMv = BMS_DEFAULT_TARGET_VOLTAGE_MV;
    config->charge.targetCurrentMa = BMS_DEFAULT_TARGET_CURRENT_MA;
    config->charge.cutoffCurrentMa = BMS_DEFAULT_CUTOFF_CURRENT_MA;
    config->charge.cellOvpMv = BMS_DEFAULT_CELL_OVP_MV;
    config->charge.cellUvpMv = BMS_DEFAULT_CELL_UVP_MV;
    config->charge.tempOtpX10 = BMS_DEFAULT_TEMP_OTP_X10;
    config->charge.balanceDeltaMv = BMS_DEFAULT_BALANCE_DELTA_MV;
    config->trickleCurrentMa = BMS_DEFAULT_TRICKLE_CURRENT_MA;
    config->inputUvThresholdMv = BMS_DEFAULT_INPUT_UV_THRESHOLD_MV;
    config->outputOvpMarginMv = BMS_DEFAULT_OUTPUT_OVP_MARGIN_MV;
    config->outputOcpMarginMa = BMS_DEFAULT_OUTPUT_OCP_MARGIN_MA;
    config->inputCurrentLimitMa = BMS_DEFAULT_INPUT_CURRENT_LIMIT_MA;
    config->inputPowerLimitW = BMS_DEFAULT_INPUT_POWER_LIMIT_W;
    config->minChargeCurrentMa = BMS_DEFAULT_MIN_CHARGE_CURRENT_MA;
    config->derateStartTempX10 = BMS_DEFAULT_DERATE_START_TEMP_X10;
    config->derateStopTempX10 = BMS_DEFAULT_DERATE_STOP_TEMP_X10;
    config->socCapacityMah = BMS_DEFAULT_SOC_CAPACITY_MAH;
    config->socInitialX10 = 0U;
    config->currentKpDiv = BMS_DEFAULT_CURRENT_KP_DIV;
    config->currentKiDiv = BMS_DEFAULT_CURRENT_KI_DIV;
    config->voltageKpDiv = BMS_DEFAULT_VOLTAGE_KP_DIV;
    config->voltageKiDiv = BMS_DEFAULT_VOLTAGE_KI_DIV;
    config->loopStepMaxX100 = BMS_DEFAULT_LOOP_STEP_MAX_X100;
    config->adcVinGainX1000 = BMS_ADC_VIN_GAIN_X1000;
    config->adcVoutGainX1000 = BMS_ADC_VOUT_GAIN_X1000;
    config->adcIinOffsetRaw = BMS_ADC_IIN_OFFSET_RAW;
    config->adcIoutOffsetRaw = BMS_ADC_IOUT_OFFSET_RAW;
}

void Param_Storage_Init(void)
{
    param_record_t record;
    uint32_t i;
    uint32_t capacity;
    uint8_t found;

    Flash_Storage_Init();
    Param_Storage_Restore_Default(&s_config);
    s_sequence = 0U;
    found = 0U;
    capacity = Param_Record_Capacity();

    for(i = 0U; i < capacity; i++) {
        Flash_Storage_Read(Param_Record_Address(i), &record, sizeof(record));
        if(Param_Record_Valid(&record) != 0U) {
            if(found == 0U || record.sequence >= s_sequence) {
                s_config = record.config;
                s_sequence = record.sequence;
                found = 1U;
            }
        }
    }

    s_dirty = 0U;
    s_dirty_since_ms = 0U;
}

uint8_t Param_Storage_Get_Config(bms_system_config_t *config)
{
    if(config == 0) {
        return 0U;
    }

    *config = s_config;
    return 1U;
}

uint8_t Param_Storage_Set_Config(const bms_system_config_t *config)
{
    if(config == 0) {
        return 0U;
    }

    s_config = *config;
    Param_Storage_Request_Save();
    return 1U;
}

uint8_t Param_Storage_Get_Charge(bms_charge_parameters_t *parameters)
{
    if(parameters == 0) {
        return 0U;
    }

    *parameters = s_config.charge;
    return 1U;
}

uint8_t Param_Storage_Set_Charge(const bms_charge_parameters_t *parameters)
{
    if(parameters == 0) {
        return 0U;
    }

    s_config.charge = *parameters;
    Param_Storage_Request_Save();
    return 1U;
}

void Param_Storage_Request_Save(void)
{
    s_dirty = 1U;
}

uint8_t Param_Storage_Save_Now(void)
{
    param_record_t record;
    uint32_t i;
    uint32_t capacity;
    uint32_t address;

    capacity = Param_Record_Capacity();
    address = 0U;

    for(i = 0U; i < capacity; i++) {
        if(Flash_Storage_Is_Erased(Param_Record_Address(i), sizeof(param_record_t)) != 0U) {
            address = Param_Record_Address(i);
            break;
        }
    }

    if(address == 0U) {
        if(Flash_Storage_Erase(BMS_PARAM_STORAGE_BASE_ADDRESS, BMS_PARAM_STORAGE_SIZE_BYTES) != FLASH_STORAGE_OK) {
            return 0U;
        }
        address = BMS_PARAM_STORAGE_BASE_ADDRESS;
    }

    memset(&record, 0xFF, sizeof(record));
    record.magic = PARAM_RECORD_MAGIC;
    record.version = BMS_PARAM_STORAGE_VERSION;
    record.length = sizeof(bms_system_config_t);
    record.sequence = s_sequence + 1U;
    record.config = s_config;
    record.crc32 = Flash_Storage_Crc32(&record, sizeof(record) - sizeof(record.crc32));

    if(Flash_Storage_Write(address, &record, sizeof(record)) != FLASH_STORAGE_OK) {
        return 0U;
    }

    s_sequence = record.sequence;
    s_dirty = 0U;
    return 1U;
}

void Param_Storage_Service(uint32_t now_ms)
{
    if(s_dirty == 0U) {
        s_dirty_since_ms = now_ms;
        return;
    }

    if(s_dirty_since_ms == 0U) {
        s_dirty_since_ms = now_ms;
        return;
    }

    if((uint32_t)(now_ms - s_dirty_since_ms) >= PARAM_SAVE_DELAY_MS) {
        (void)Param_Storage_Save_Now();
        s_dirty_since_ms = now_ms;
    }
}
