#include "bsp_motor_record.h"
#include "dual.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#define LEG_RECORD_BASE (FOC_RECORD_ADDRESS + 4096u)
_Static_assert(FOC_COG_ADDRESS - FOC_RECORD_ADDRESS + sizeof(cog_record_t) <= 4096u,
               "Legacy single-axis records overlap the dual-leg area");
static uint32_t identity(unsigned i) {
  return (((i ? 1u : 2u) << 12) | (FOC_MOTOR_ID << 8) | (FOC_ENCODER_ID << 4) |
          (FOC_INSTALLATION_ID + i))
             << 16 |
         FOC_POLE_PAIRS;
}
static bool valid(const record_t *r, unsigned i) {
  return r->version == 6u && r->poles == identity(i) &&
         r->magic == 0x464f4331u && r->checksum == checksum(r) &&
         (r->cal.direction == 1 || r->cal.direction == -1) &&
         r->cal.zero >= 0 && r->cal.zero < 6.2831853072f;
}
bool dual_record_load(unsigned i, foc_calibration_t *c) {
  if (i > 1)
    return false;
  const record_t *r =
      (const record_t *)(LEG_RECORD_BASE + i * sizeof(record_t));
  if (!valid(r, i))
    return false;
  *c = (foc_calibration_t){r->cal.zero, r->cal.direction, FOC_POLE_PAIRS};
  return true;
}
bool dual_record_save(unsigned i, const foc_calibration_t *c) {
  if (i > 1 || c->pole_pairs != FOC_POLE_PAIRS)
    return false;
  static uint32_t prefix[4096 / 4];
  record_t records[2];
  bool present[2];
  memcpy(prefix, (const void *)FOC_RECORD_ADDRESS, sizeof prefix);
  for (unsigned n = 0; n < 2; ++n) {
    foc_calibration_t old = {0};
    present[n] = dual_record_load(n, &old) || n == i;
    records[n] =
        (record_t){.version = 6,
                   .poles = identity(n),
                   .cal = {present[n] && n != i ? old.zero : c->zero,
                           present[n] && n != i ? old.direction : c->direction},
                   .magic = 0x464f4331u};
    records[n].checksum = checksum(&records[n]);
    if (present[n] && !valid(&records[n], n))
      return false;
  }
  FLASH_EraseInitTypeDef erase = {.TypeErase = FLASH_TYPEERASE_SECTORS,
                                  .VoltageRange = FLASH_VOLTAGE_RANGE_3,
                                  .Sector = FLASH_SECTOR_11,
                                  .NbSectors = 1};
  uint32_t failed;
  if (HAL_FLASH_Unlock() != HAL_OK)
    return false;
  bool ok = HAL_FLASHEx_Erase(&erase, &failed) == HAL_OK;
  for (unsigned n = 0; ok && n < sizeof prefix / 4; ++n)
    if (prefix[n] != 0xffffffffu)
      ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                             FOC_RECORD_ADDRESS + n * 4u, prefix[n]) == HAL_OK;
  for (unsigned n = 0; ok && n < 2; ++n)
    if (present[n])
      for (unsigned j = 0; ok && j < sizeof(record_t); j += 4) {
        uint32_t word;
        memcpy(&word, (const uint8_t *)&records[n] + j, 4);
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                               LEG_RECORD_BASE + n * sizeof(record_t) + j,
                               word) == HAL_OK;
      }
  HAL_FLASH_Lock();
  foc_calibration_t back;
  return ok &&
         memcmp(prefix, (const void *)FOC_RECORD_ADDRESS, sizeof prefix) == 0 &&
         dual_record_load(i, &back) && memcmp(&back, c, sizeof back) == 0;
}
