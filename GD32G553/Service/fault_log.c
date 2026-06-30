#include "fault_log.h"

#include "bms_board_config.h"
#include "flash_storage.h"

#include <string.h>

#define FAULT_LOG_MAGIC                       0x464C4F47UL
#define FAULT_LOG_MIN_INTERVAL_MS             1000U

static uint32_t s_next_sequence;
static uint32_t s_last_fault_bitmap;
static uint32_t s_last_write_ms;
static uint16_t s_record_count;

static uint32_t Fault_Log_Record_Address(uint32_t index)
{
    return BMS_FAULT_LOG_BASE_ADDRESS + (index * sizeof(fault_log_record_t));
}

static uint32_t Fault_Log_Record_Capacity(void)
{
    return BMS_FAULT_LOG_SIZE_BYTES / sizeof(fault_log_record_t);
}

static uint8_t Fault_Log_Record_Valid(const fault_log_record_t *record)
{
    uint32_t crc;

    if(record == 0) {
        return 0U;
    }
    if(record->magic != FAULT_LOG_MAGIC ||
       record->version != FAULT_LOG_VERSION ||
       record->length != sizeof(fault_log_record_t)) {
        return 0U;
    }

    crc = Flash_Storage_Crc32(record, sizeof(*record) - sizeof(record->crc32));
    return (crc == record->crc32) ? 1U : 0U;
}

void Fault_Log_Init(void)
{
    fault_log_record_t record;
    uint32_t i;
    uint32_t capacity;

    s_next_sequence = 1U;
    s_last_fault_bitmap = 0U;
    s_last_write_ms = 0U;
    s_record_count = 0U;
    capacity = Fault_Log_Record_Capacity();

    for(i = 0U; i < capacity; i++) {
        Flash_Storage_Read(Fault_Log_Record_Address(i), &record, sizeof(record));
        if(Fault_Log_Record_Valid(&record) != 0U) {
            s_record_count++;
            if(record.sequence >= s_next_sequence) {
                s_next_sequence = record.sequence + 1U;
            }
        }
    }
}

void Fault_Log_Clear(void)
{
    (void)Flash_Storage_Erase(BMS_FAULT_LOG_BASE_ADDRESS, BMS_FAULT_LOG_SIZE_BYTES);
    s_next_sequence = 1U;
    s_last_fault_bitmap = 0U;
    s_last_write_ms = 0U;
    s_record_count = 0U;
}

static uint8_t Fault_Log_Write_Record(const fault_log_record_t *record)
{
    uint32_t capacity;
    uint32_t index;
    uint32_t address;

    capacity = Fault_Log_Record_Capacity();
    index = (s_next_sequence - 1U) % capacity;
    address = Fault_Log_Record_Address(index);

    if(Flash_Storage_Is_Erased(address, sizeof(fault_log_record_t)) == 0U) {
        if(index == 0U) {
            if(Flash_Storage_Erase(BMS_FAULT_LOG_BASE_ADDRESS, BMS_FAULT_LOG_SIZE_BYTES) != FLASH_STORAGE_OK) {
                return 0U;
            }
            s_record_count = 0U;
        } else {
            return 0U;
        }
    }

    if(Flash_Storage_Write(address, record, sizeof(*record)) != FLASH_STORAGE_OK) {
        return 0U;
    }

    if(s_record_count < capacity) {
        s_record_count++;
    }
    return 1U;
}

void Fault_Log_Service(const bms_status_t *status, const bms_power_sample_t *sample)
{
    fault_log_record_t record;
    uint32_t new_faults;

    if(status == 0 || sample == 0 || status->faultBitmap == 0U) {
        if(status != 0 && status->faultBitmap == 0U) {
            s_last_fault_bitmap = 0U;
        }
        return;
    }

    new_faults = status->faultBitmap & ~s_last_fault_bitmap;
    if(new_faults == 0U &&
       (uint32_t)(status->timestampMs - s_last_write_ms) < FAULT_LOG_MIN_INTERVAL_MS) {
        return;
    }

    memset(&record, 0xFF, sizeof(record));
    record.magic = FAULT_LOG_MAGIC;
    record.version = FAULT_LOG_VERSION;
    record.length = sizeof(fault_log_record_t);
    record.sequence = s_next_sequence;
    record.runtimeMs = status->timestampMs;
    record.faultBitmap = status->faultBitmap;
    record.vinMv = sample->inputVoltageMv;
    record.voutMv = sample->outputVoltageMv;
    record.ioutMa = sample->outputCurrentMa;
    record.mosTempX10 = sample->mosTempX10;
    record.inductorTempX10 = sample->inductorTempX10;
    record.socX10 = status->socX10;
    record.chargeState = status->chargeState;
    record.chargeMode = status->chargeMode;
    record.dutyX100 = status->dutyX100;
    record.cellMinMv = status->cellMinMv;
    record.cellMaxMv = status->cellMaxMv;
    record.reserved = 0U;
    record.crc32 = Flash_Storage_Crc32(&record, sizeof(record) - sizeof(record.crc32));

    if(Fault_Log_Write_Record(&record) != 0U) {
        s_next_sequence++;
        s_last_write_ms = status->timestampMs;
        s_last_fault_bitmap = status->faultBitmap;
    }
}

uint16_t Fault_Log_Count(void)
{
    return s_record_count;
}

uint8_t Fault_Log_Read(uint16_t newest_index, fault_log_record_t *record)
{
    fault_log_record_t candidate;
    uint32_t capacity;
    uint32_t newest_sequence;
    uint32_t target_sequence;
    uint32_t i;

    if(record == 0 || newest_index >= s_record_count) {
        return 0U;
    }

    capacity = Fault_Log_Record_Capacity();
    newest_sequence = s_next_sequence - 1U;
    target_sequence = newest_sequence - (uint32_t)newest_index;

    for(i = 0U; i < capacity; i++) {
        Flash_Storage_Read(Fault_Log_Record_Address(i), &candidate, sizeof(candidate));
        if(Fault_Log_Record_Valid(&candidate) != 0U && candidate.sequence == target_sequence) {
            *record = candidate;
            return 1U;
        }
    }

    return 0U;
}
