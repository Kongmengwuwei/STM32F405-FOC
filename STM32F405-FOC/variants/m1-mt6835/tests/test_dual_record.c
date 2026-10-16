#include "bsp_motor_record.h"
#include "dual.h"
#include "fivebar_config.h"
#include "foot_record.h"
#include "stm32f4xx_hal.h"
#include <assert.h>
#include <stdio.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <sys/mman.h>
#endif
static bool unlocked, erase_fail;
static unsigned programs, fail_at;
HAL_StatusTypeDef HAL_FLASH_Unlock(void) {
  unlocked = true;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_FLASH_Lock(void) {
  unlocked = false;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *e,
                                    uint32_t *failed) {
  assert(unlocked && e->Sector == 11 && e->NbSectors == 1);
  *failed = 0;
  if (erase_fail)
    return HAL_ERROR;
  memset((void *)(uintptr_t)FOC_RECORD_ADDRESS, 0xff, 128u * 1024u);
  programs = 0;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t address,
                                    uint32_t word) {
  assert(unlocked && type == FLASH_TYPEPROGRAM_WORD &&
         address >= FOC_RECORD_ADDRESS &&
         address < FOC_RECORD_ADDRESS + 128u * 1024u && !(address & 3));
  if (++programs == fail_at)
    return HAL_ERROR;
  uint32_t *p = (uint32_t *)(uintptr_t)address;
  assert((*p & word) == word);
  *p = word;
  return HAL_OK;
}
int main(void) {
#ifdef _WIN32
  void *memory =
      VirtualAlloc((void *)(uintptr_t)FOC_RECORD_ADDRESS, 128u * 1024u,
                   MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE);
#else
  void *memory = mmap((void *)(uintptr_t)FOC_RECORD_ADDRESS, 128u * 1024u,
                      PROT_READ | PROT_WRITE,
                      MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
#endif
  assert(memory == (void *)(uintptr_t)FOC_RECORD_ADDRESS);
  memset(memory, 0xff, 128u * 1024u);
  uint32_t prefix[1024];
  for (unsigned i = 0; i < 1024; i++)
    prefix[i] = 0x12340000u + i;
  memcpy(memory, prefix, sizeof prefix);
  fb_geometry_t geometry = FIVEBAR_GEOMETRY_INITIALIZER, back;
  assert(!foot_record_load(&back));
  foc_calibration_t c0 = {.zero = .23f, .direction = 1, .pole_pairs = 7};
  foc_calibration_t c1 = {.zero = 4.34f, .direction = 1, .pole_pairs = 7}, cal;
  assert(dual_record_save(0, &c0) && dual_record_save(1, &c1));
  geometry.zero_ccw[0] = 23.6f;
  geometry.zero_ccw[1] = 154.4f;
  assert(foot_record_save(&geometry) && foot_record_load(&back));
  assert(!memcmp(&geometry, &back, sizeof back));
  assert(!memcmp(prefix, memory, sizeof prefix));
  assert(dual_record_load(0, &cal) && !memcmp(&cal, &c0, sizeof cal));
  assert(dual_record_load(1, &cal) && !memcmp(&cal, &c1, sizeof cal));
  c0.zero = .4f;
  assert(dual_record_save(0, &c0));
  assert(foot_record_load(&back) && !memcmp(&geometry, &back, sizeof back));
  fb_geometry_t wrong = geometry;
  wrong.active[0] = 81;
  assert(!foot_record_save(&wrong));
  assert(foot_record_load(&back));
  erase_fail = true;
  assert(!foot_record_save(&geometry));
  erase_fail = false;
  assert(foot_record_load(&back));
  /* Failure at the final magic word must not expose an incomplete geometry. */
  fail_at = 1024 + 2 * sizeof(record_t) / 4 + (sizeof(fb_geometry_t) + 16) / 4;
  assert(!foot_record_save(&geometry));
  assert(!foot_record_load(&back));
  fail_at = 0;
  assert(foot_record_save(&geometry));
  assert(dual_record_load(0, &cal) && !memcmp(&cal, &c0, sizeof cal));
  assert(dual_record_load(1, &cal) && !memcmp(&cal, &c1, sizeof cal));
  assert(!memcmp(prefix, memory, sizeof prefix));
  uint32_t *foot =
      (uint32_t *)((uint8_t *)memory + 4096 + 2 * sizeof(record_t));
  /* A different home/geometry binding must fail even with a valid checksum. */
  foot[1] ^= 1u;
  foot[(8 + sizeof(fb_geometry_t)) / 4] =
      record_checksum_bytes(foot, 8 + sizeof(fb_geometry_t));
  assert(!foot_record_load(&back));
  assert(foot_record_save(&geometry) && foot_record_load(&back));
  foot[2] ^= 1;
  assert(!foot_record_load(&back));
#ifdef _WIN32
  VirtualFree(memory, 0, MEM_RELEASE);
#else
  munmap(memory, 128u * 1024u);
#endif
  puts("PASS: shared Flash preservation, motor re-save, geometry binding, CRC "
       "and incomplete-write rejection");
}
