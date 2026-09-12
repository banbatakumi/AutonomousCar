#ifndef FLASH_H_
#define FLASH_H_

#include <string.h>

#include "main.h"

// STM32F446: Sector 7 (最終セクター, 128KB)
// アドレス: 0x08060000 - 0x0807FFFF
#define FLASH_USER_START_ADDR ((uint32_t)0x08060000)
#define FLASH_USER_SECTOR FLASH_SECTOR_7
#define FLASH_USER_VOLTAGE_RANGE FLASH_VOLTAGE_RANGE_3

// MPU6050 calibration data storage (Sector 6, 128KB)
#define FLASH_MPU_START_ADDR ((uint32_t)0x08040000)
#define FLASH_MPU_SECTOR FLASH_SECTOR_6
#define FLASH_MPU_VOLTAGE_RANGE FLASH_VOLTAGE_RANGE_3

static inline HAL_StatusTypeDef Flash_WriteDataToSector(uint32_t address,
                                                        uint32_t sector,
                                                        uint32_t voltage_range,
                                                        const void* data,
                                                        size_t size) {
  if (address == 0 || data == NULL || size == 0) {
    return HAL_ERROR;
  }

  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef eraseInit;
  uint32_t sectorError = 0;
  eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
  eraseInit.Sector = sector;
  eraseInit.NbSectors = 1;
  eraseInit.VoltageRange = voltage_range;

  if (HAL_FLASHEx_Erase(&eraseInit, &sectorError) != HAL_OK) {
    HAL_FLASH_Lock();
    return HAL_ERROR;
  }

  const uint8_t* bytes = (const uint8_t*)data;
  const uint32_t* p = (const uint32_t*)data;
  size_t full_words = size / 4;
  for (size_t i = 0; i < full_words; i++) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + i * 4, p[i]) != HAL_OK) {
      HAL_FLASH_Lock();
      return HAL_ERROR;
    }
  }

  // size が4の倍数でない端数分。p[full_words] をそのまま読むと呼び出し元バッファの
  // 末尾を最大3バイト超えて読むことになるため、一時ワードへ端数バイトだけコピーしてから
  // 書き込む。消去済みフラッシュは0xFFなので、端数に含まれない上位バイトは0xFFのままにして
  // 書き込んでも無害 (0xFF→0xFFのプログラムは実質no-op)
  size_t remainder = size - full_words * 4;
  if (remainder > 0) {
    uint32_t last_word = 0xFFFFFFFFu;
    memcpy(&last_word, bytes + full_words * 4, remainder);
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + full_words * 4, last_word) != HAL_OK) {
      HAL_FLASH_Lock();
      return HAL_ERROR;
    }
  }

  HAL_FLASH_Lock();
  return HAL_OK;
}

static inline HAL_StatusTypeDef Flash_WriteData(uint32_t address,
                                                const void* data,
                                                size_t size) {
  if (address < FLASH_USER_START_ADDR) {
    return HAL_ERROR;
  }
  return Flash_WriteDataToSector(address, FLASH_USER_SECTOR,
                                 FLASH_USER_VOLTAGE_RANGE, data, size);
}

static inline void Flash_ReadData(uint32_t address, void* buffer, size_t size) {
  uint8_t* bytes = (uint8_t*)buffer;
  uint32_t* p = (uint32_t*)buffer;
  size_t full_words = size / 4;
  for (size_t i = 0; i < full_words; i++) {
    p[i] = *(volatile uint32_t*)(address + i * 4);
  }

  // size が4の倍数でない端数分。p[full_words] へそのまま書くと呼び出し元バッファの
  // 末尾を最大3バイト超えて書き込むことになるため、一旦ワード単位で読んでから
  // 端数バイトだけをバッファへコピーする
  size_t remainder = size - full_words * 4;
  if (remainder > 0) {
    uint32_t last_word = *(volatile uint32_t*)(address + full_words * 4);
    memcpy(bytes + full_words * 4, &last_word, remainder);
  }
}

#endif  // FLASH_H_