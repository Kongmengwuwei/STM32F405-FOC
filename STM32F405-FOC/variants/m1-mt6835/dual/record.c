#include "bsp_motor_record.h"
#include "dual.h"
#include "fivebar_config.h"
#include "foot_record.h"
#include "leg_reference_config.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#define LEG_RECORD_BASE (FOC_RECORD_ADDRESS + 4096u)
_Static_assert(FOC_COG_ADDRESS - FOC_RECORD_ADDRESS + sizeof(cog_record_t) <=
                   4096u,
               "Legacy single-axis records overlap the dual-leg area");
#define FOOT_RECORD_BASE (LEG_RECORD_BASE + 2u * sizeof(record_t))
typedef struct {
  uint32_t version, binding;
  fb_geometry_t geometry;
  uint32_t checksum, magic;
} foot_record_t;
_Static_assert(sizeof(foot_record_t) % 4u == 0u, "Flash word alignment");
_Static_assert(FOOT_RECORD_BASE + sizeof(foot_record_t) <=
                   FOC_RECORD_ADDRESS + 128u * 1024u,
               "Foot calibration outside sector 11");
static uint32_t foot_binding(void) {
  fb_geometry_t reference = FIVEBAR_GEOMETRY_INITIALIZER;
  reference.zero_ccw[0] = reference.zero_ccw[1] = 0;
  const float home_reference[2] = {LEG_REFERENCE_RAW_M0, LEG_REFERENCE_RAW_M1};
  return record_checksum_bytes(&reference, sizeof reference) ^
         record_checksum_bytes(home_reference, sizeof home_reference) ^
         FOC_CALIBRATION_ID;
}
static uint32_t foot_checksum(const foot_record_t *r) {
  return record_checksum_bytes(r, offsetof(foot_record_t, checksum));
}
static bool foot_valid(const foot_record_t *r) {
  if (r->version != 1u || r->binding != foot_binding() ||
      r->magic != 0x46424b31u || r->checksum != foot_checksum(r) ||
      !fb_valid(&r->geometry))
    return false;
  fb_geometry_t a = r->geometry, b = FIVEBAR_GEOMETRY_INITIALIZER;
  a.zero_ccw[0] = a.zero_ccw[1] = b.zero_ccw[0] = b.zero_ccw[1] = 0;
  return memcmp(&a, &b, sizeof a) == 0;
}
bool foot_record_load(fb_geometry_t *g) {
  const foot_record_t *r = (const foot_record_t *)FOOT_RECORD_BASE;
  if (!g || !foot_valid(r))
    return false;
  *g = r->geometry;
  return true;
}
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
/* Shared sector: preserve legacy FOC/cog records, both electrical records,
 * and foot calibration on every dual-leg write. Magic is committed last. */
static bool write_records(const record_t records[2],
                          const foot_record_t *foot) {
  static uint32_t prefix[4096 / 4];
  memcpy(prefix, (const void *)FOC_RECORD_ADDRESS, sizeof prefix);
  FLASH_EraseInitTypeDef erase = {.TypeErase = FLASH_TYPEERASE_SECTORS,
                                  .VoltageRange = FLASH_VOLTAGE_RANGE_3,
                                  .Sector = FLASH_SECTOR_11,
                                  .NbSectors = 1};
  uint32_t failed;
  if (HAL_FLASH_Unlock() != HAL_OK)
    return false;
  bool ok = HAL_FLASHEx_Erase(&erase, &failed) == HAL_OK;
  for (unsigned n = 0; ok && n < sizeof prefix / 4; n++)
    if (prefix[n] != 0xffffffffu)
      ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                             FOC_RECORD_ADDRESS + n * 4u, prefix[n]) == HAL_OK;
  for (unsigned n = 0; ok && n < 2; n++) {
    if (!valid(&records[n], n))
      continue;
    for (unsigned j = 0; ok && j < sizeof(record_t); j += 4) {
      uint32_t word;
      memcpy(&word, (const uint8_t *)&records[n] + j, 4);
      ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                             LEG_RECORD_BASE + n * sizeof(record_t) + j,
                             word) == HAL_OK;
    }
  }
  if (foot_valid(foot))
    for (unsigned j = 0; ok && j < sizeof *foot; j += 4) {
      uint32_t word;
      memcpy(&word, (const uint8_t *)foot + j, 4);
      ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FOOT_RECORD_BASE + j,
                             word) == HAL_OK;
    }
  HAL_FLASH_Lock();
  return ok &&
         memcmp(prefix, (const void *)FOC_RECORD_ADDRESS, sizeof prefix) == 0 &&
         (!foot_valid(foot) ||
          memcmp(foot, (const void *)FOOT_RECORD_BASE, sizeof *foot) == 0);
}
bool dual_record_save(unsigned i, const foc_calibration_t *c) {
  if (i > 1 || !c || c->pole_pairs != FOC_POLE_PAIRS)
    return false;
  record_t records[2];
  memcpy(records, (const void *)LEG_RECORD_BASE, sizeof records);
  records[i] = (record_t){.version = 6,
                          .poles = identity(i),
                          .cal = {c->zero, c->direction},
                          .magic = 0x464f4331u};
  records[i].checksum = checksum(&records[i]);
  if (!valid(&records[i], i))
    return false;
  foot_record_t foot;
  memcpy(&foot, (const void *)FOOT_RECORD_BASE, sizeof foot);
  foc_calibration_t back;
  return write_records(records, &foot) && dual_record_load(i, &back) &&
         memcmp(&back, c, sizeof back) == 0;
}
bool foot_record_save(const fb_geometry_t *g) {
  if (!g || !fb_valid(g))
    return false;
  foot_record_t foot = {.version = 1,
                        .binding = foot_binding(),
                        .geometry = *g,
                        .magic = 0x46424b31u};
  foot.checksum = foot_checksum(&foot);
  if (!foot_valid(&foot))
    return false;
  record_t records[2];
  memcpy(records, (const void *)LEG_RECORD_BASE, sizeof records);
  fb_geometry_t back;
  return write_records(records, &foot) && foot_record_load(&back) &&
         memcmp(&back, g, sizeof back) == 0;
}
