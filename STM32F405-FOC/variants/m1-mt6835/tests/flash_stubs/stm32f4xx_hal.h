#ifndef TEST_FLASH_HAL_H
#define TEST_FLASH_HAL_H
#include <stdint.h>
typedef enum {HAL_OK,HAL_ERROR} HAL_StatusTypeDef;
typedef struct {uint32_t TypeErase,VoltageRange,Sector,NbSectors;} FLASH_EraseInitTypeDef;
#define FLASH_TYPEERASE_SECTORS 1
#define FLASH_VOLTAGE_RANGE_3 3
#define FLASH_SECTOR_11 11
#define FLASH_TYPEPROGRAM_WORD 2
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *,uint32_t *);
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t,uint32_t,uint32_t);
#endif
