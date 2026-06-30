#include "flash_storage.h"

#include "bms_board_config.h"
#include "gd32g5x3_fmc.h"

#include <string.h>

#define FLASH_STORAGE_ERASED_BYTE             0xFFU

static uint8_t Flash_Range_Valid(uint32_t address, uint32_t length)
{
    uint32_t end;

    if(length == 0U) {
        return 0U;
    }
    end = address + length;
    if(end < address) {
        return 0U;
    }
    if(address < BMS_STORAGE_FLASH_BASE_ADDRESS) {
        return 0U;
    }
    if(end > (BMS_STORAGE_FLASH_BASE_ADDRESS + BMS_STORAGE_FLASH_SIZE_BYTES)) {
        return 0U;
    }

    return 1U;
}

static uint8_t Flash_Address_To_Page(uint32_t address, uint32_t *bank, uint32_t *page)
{
    uint32_t page_size;
    uint32_t offset;
    uint32_t bank_size_bytes;

    if(bank == 0 || page == 0 || address < MAIN_FLASH_BASE_ADDRESS) {
        return 0U;
    }

    page_size = fmc_page_size_get();
    if(page_size == 0U) {
        return 0U;
    }

    offset = address - MAIN_FLASH_BASE_ADDRESS;
    if(OB_DUAL_BANK_MODE == (FMC_OBCTL & FMC_OBCTL_DBS)) {
        bank_size_bytes = (uint32_t)MAIN_FLASH_BANK_SIZE * 1024U;
        if(offset >= bank_size_bytes) {
            *bank = FMC_BANK1;
            *page = (offset - bank_size_bytes) / page_size;
        } else {
            *bank = FMC_BANK0;
            *page = offset / page_size;
        }
    } else {
        *bank = FMC_BANK0;
        *page = offset / page_size;
    }

    return 1U;
}

void Flash_Storage_Init(void)
{
}

void Flash_Storage_Read(uint32_t address, void *data, uint32_t length)
{
    if(data == 0 || length == 0U) {
        return;
    }

    memcpy(data, (const void *)address, length);
}

flash_storage_status_t Flash_Storage_Erase(uint32_t address, uint32_t length)
{
    uint32_t page_size;
    uint32_t first;
    uint32_t last;
    uint32_t current;
    uint32_t bank;
    uint32_t page;
    uint32_t primask;
    fmc_state_enum state;

    if(Flash_Range_Valid(address, length) == 0U) {
        return FLASH_STORAGE_ERROR_RANGE;
    }

    page_size = fmc_page_size_get();
    if(page_size == 0U) {
        return FLASH_STORAGE_ERROR_ERASE;
    }

    first = address - (address % page_size);
    last = (address + length + page_size - 1U) - ((address + length + page_size - 1U) % page_size);

    primask = __get_PRIMASK();
    __disable_irq();
    fmc_unlock();

    for(current = first; current < last; current += page_size) {
        if(Flash_Address_To_Page(current, &bank, &page) == 0U) {
            fmc_lock();
            if(primask == 0U) {
                __enable_irq();
            }
            return FLASH_STORAGE_ERROR_RANGE;
        }
        state = fmc_page_erase(bank, page);
        if(state != FMC_READY) {
            fmc_lock();
            if(primask == 0U) {
                __enable_irq();
            }
            return FLASH_STORAGE_ERROR_ERASE;
        }
    }

    fmc_lock();
    if(primask == 0U) {
        __enable_irq();
    }

    return FLASH_STORAGE_OK;
}

flash_storage_status_t Flash_Storage_Write(uint32_t address, const void *data, uint32_t length)
{
    const uint8_t *bytes;
    uint8_t buffer[8];
    uint32_t offset;
    uint32_t chunk;
    uint32_t primask;
    uint64_t word;
    fmc_state_enum state;

    if(data == 0 || length == 0U) {
        return FLASH_STORAGE_ERROR_PARAM;
    }
    if(Flash_Range_Valid(address, length) == 0U || (address & 0x7U) != 0U) {
        return FLASH_STORAGE_ERROR_RANGE;
    }

    bytes = (const uint8_t *)data;
    primask = __get_PRIMASK();
    __disable_irq();
    fmc_unlock();

    for(offset = 0U; offset < length; offset += 8U) {
        memset(buffer, FLASH_STORAGE_ERASED_BYTE, sizeof(buffer));
        chunk = length - offset;
        if(chunk > 8U) {
            chunk = 8U;
        }
        memcpy(buffer, &bytes[offset], chunk);
        memcpy(&word, buffer, sizeof(word));
        state = fmc_doubleword_program(address + offset, word);
        if(state != FMC_READY) {
            fmc_lock();
            if(primask == 0U) {
                __enable_irq();
            }
            return FLASH_STORAGE_ERROR_PROGRAM;
        }
    }

    fmc_lock();
    if(primask == 0U) {
        __enable_irq();
    }

    return FLASH_STORAGE_OK;
}

uint8_t Flash_Storage_Is_Erased(uint32_t address, uint32_t length)
{
    const uint8_t *p;
    uint32_t i;

    if(Flash_Range_Valid(address, length) == 0U) {
        return 0U;
    }

    p = (const uint8_t *)address;
    for(i = 0U; i < length; i++) {
        if(p[i] != FLASH_STORAGE_ERASED_BYTE) {
            return 0U;
        }
    }

    return 1U;
}

uint32_t Flash_Storage_Crc32(const void *data, uint32_t length)
{
    const uint8_t *bytes;
    uint32_t crc;
    uint32_t i;
    uint8_t bit;

    if(data == 0) {
        return 0U;
    }

    bytes = (const uint8_t *)data;
    crc = 0xFFFFFFFFUL;
    for(i = 0U; i < length; i++) {
        crc ^= bytes[i];
        for(bit = 0U; bit < 8U; bit++) {
            if((crc & 1UL) != 0UL) {
                crc = (crc >> 1) ^ 0xEDB88320UL;
            } else {
                crc >>= 1;
            }
        }
    }

    return ~crc;
}
