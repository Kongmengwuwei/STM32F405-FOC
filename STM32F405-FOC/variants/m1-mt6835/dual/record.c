#include "dual.h"
#include "bsp_motor_record.h"
#include "stm32f4xx_hal.h"
#include <string.h>

#define RECORD_BASE 0x080e0000u
#ifndef FOC_M1_INSTALLATION_ID
#define FOC_M1_INSTALLATION_ID 2u
#endif
#if FOC_M1_INSTALLATION_ID < 1 || FOC_M1_INSTALLATION_ID > 15
#error FOC_M1_INSTALLATION_ID must be in 1..15
#endif
#define M1_CAL_ID ((1u << 12) | (FOC_MOTOR_ID << 8) | \
                   (FOC_ENCODER_ID << 4) | FOC_M1_INSTALLATION_ID)

static uint32_t identity(unsigned motor)
{
    return (((motor ? M1_CAL_ID : FOC_CALIBRATION_ID) << 16) | FOC_POLE_PAIRS);
}

static bool valid(const record_t *r, unsigned motor)
{
    /* Version 5 invalidates calibration from the original encoder mapping. */
    return r->version == 5u && r->poles == identity(motor) &&
           r->magic == 0x464f4331u && r->checksum == checksum(r) &&
           (r->cal.direction == 1 || r->cal.direction == -1) &&
           r->cal.zero >= 0.0f && r->cal.zero < 6.2831853072f;
}

bool dual_record_load(unsigned motor, foc_calibration_t *out)
{
    if (motor > 1u) return false;
    const record_t *r = (const record_t *)(RECORD_BASE + motor * sizeof(record_t));
    if (valid(r, motor)) {
        *out = r->cal;
        return true;
    }
    return false;
}

bool dual_record_save(unsigned motor, const foc_calibration_t *cal)
{
    if (motor > 1u) return false;
    record_t records[2];
    bool present[2];
    for (unsigned i = 0u; i < 2u; ++i) {
        foc_calibration_t previous;
        bool had = dual_record_load(i, &previous);
        present[i] = had || i == motor;
        records[i] = (record_t){.version = 5u, .poles = identity(i),
                                .cal = had ? previous : (foc_calibration_t){0},
                                .magic = 0x464f4331u};
    }
    records[motor].cal = *cal;
    records[motor].magic = 0x464f4331u;
    for (unsigned i = 0u; i < 2u; ++i) records[i].checksum = checksum(&records[i]);
    FLASH_EraseInitTypeDef erase = {.TypeErase = FLASH_TYPEERASE_SECTORS,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3, .Sector = FLASH_SECTOR_11, .NbSectors = 1u};
    uint32_t failed;
    if (HAL_FLASH_Unlock() != HAL_OK) return false;
    bool ok = HAL_FLASHEx_Erase(&erase, &failed) == HAL_OK;
    for (unsigned i = 0u; ok && i < 2u; ++i) {
        if (!present[i]) continue;
        for (unsigned j = 0u; ok && j < sizeof(record_t); j += 4u) {
            uint32_t word;
            memcpy(&word, (const uint8_t *)&records[i] + j, sizeof word);
            ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                RECORD_BASE + i * sizeof(record_t) + j, word) == HAL_OK;
        }
    }
    HAL_FLASH_Lock();
    foc_calibration_t readback;
    return ok && dual_record_load(motor, &readback) &&
           memcmp(&readback, cal, sizeof readback) == 0;
}
