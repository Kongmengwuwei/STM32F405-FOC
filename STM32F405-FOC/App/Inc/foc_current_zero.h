#ifndef FOC_CURRENT_ZERO_H
#define FOC_CURRENT_ZERO_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  FOC_CURRENT_ZERO_COLLECTING,
  FOC_CURRENT_ZERO_READY,
  FOC_CURRENT_ZERO_FAULT
} foc_current_zero_status_t;

typedef struct
{
  uint32_t sum_b;
  uint32_t sum_c;
  uint16_t min_b;
  uint16_t max_b;
  uint16_t min_c;
  uint16_t max_c;
  uint16_t count;
  float offset_b;
  float offset_c;
  foc_current_zero_status_t status;
} foc_current_zero_t;

enum
{
  FOC_CURRENT_ZERO_SAMPLES = 256,
  FOC_CURRENT_ZERO_MIN_RAW = 1400,
  FOC_CURRENT_ZERO_MAX_RAW = 2700,
  FOC_CURRENT_ZERO_MAX_SPAN = 64
};

void foc_current_zero_init(foc_current_zero_t *calibration);
foc_current_zero_status_t foc_current_zero_add(foc_current_zero_t *calibration,
                                               uint16_t raw_b,
                                               uint16_t raw_c);
void foc_current_zero_abort(foc_current_zero_t *calibration);

#endif
