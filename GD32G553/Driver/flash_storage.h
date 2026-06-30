#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include <stdint.h>

typedef enum {
    FLASH_STORAGE_OK = 0,
    FLASH_STORAGE_ERROR_PARAM = 1,
    FLASH_STORAGE_ERROR_RANGE = 2,
    FLASH_STORAGE_ERROR_ERASE = 3,
    FLASH_STORAGE_ERROR_PROGRAM = 4
} flash_storage_status_t;

void Flash_Storage_Init(void);
void Flash_Storage_Read(uint32_t address, void *data, uint32_t length);
flash_storage_status_t Flash_Storage_Erase(uint32_t address, uint32_t length);
flash_storage_status_t Flash_Storage_Write(uint32_t address, const void *data, uint32_t length);
uint8_t Flash_Storage_Is_Erased(uint32_t address, uint32_t length);
uint32_t Flash_Storage_Crc32(const void *data, uint32_t length);

#endif
