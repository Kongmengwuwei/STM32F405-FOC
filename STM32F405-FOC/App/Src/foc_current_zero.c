#include "foc_current_zero.h"

#include <stddef.h>

void foc_current_zero_init(foc_current_zero_t *calibration)
{
  if (calibration == NULL)
  {
    return;
  }
  *calibration = (foc_current_zero_t){
      .min_b = UINT16_MAX,
      .min_c = UINT16_MAX,
      .status = FOC_CURRENT_ZERO_COLLECTING};
}

foc_current_zero_status_t foc_current_zero_add(foc_current_zero_t *calibration,
                                               uint16_t raw_b,
                                               uint16_t raw_c)
{
  if (calibration == NULL)
  {
    return FOC_CURRENT_ZERO_FAULT;
  }
  if (calibration->status != FOC_CURRENT_ZERO_COLLECTING)
  {
    return calibration->status;
  }
  if (raw_b < FOC_CURRENT_ZERO_MIN_RAW || raw_b > FOC_CURRENT_ZERO_MAX_RAW ||
      raw_c < FOC_CURRENT_ZERO_MIN_RAW || raw_c > FOC_CURRENT_ZERO_MAX_RAW)
  {
    calibration->status = FOC_CURRENT_ZERO_FAULT;
    return calibration->status;
  }

  if (raw_b < calibration->min_b) calibration->min_b = raw_b;
  if (raw_b > calibration->max_b) calibration->max_b = raw_b;
  if (raw_c < calibration->min_c) calibration->min_c = raw_c;
  if (raw_c > calibration->max_c) calibration->max_c = raw_c;
  if (calibration->max_b - calibration->min_b > FOC_CURRENT_ZERO_MAX_SPAN ||
      calibration->max_c - calibration->min_c > FOC_CURRENT_ZERO_MAX_SPAN)
  {
    calibration->status = FOC_CURRENT_ZERO_FAULT;
    return calibration->status;
  }

  calibration->sum_b += raw_b;
  calibration->sum_c += raw_c;
  ++calibration->count;
  if (calibration->count == FOC_CURRENT_ZERO_SAMPLES)
  {
    calibration->offset_b =
        (float)calibration->sum_b / (float)FOC_CURRENT_ZERO_SAMPLES;
    calibration->offset_c =
        (float)calibration->sum_c / (float)FOC_CURRENT_ZERO_SAMPLES;
    calibration->status = FOC_CURRENT_ZERO_READY;
  }
  return calibration->status;
}

void foc_current_zero_abort(foc_current_zero_t *calibration)
{
  if (calibration != NULL &&
      calibration->status == FOC_CURRENT_ZERO_COLLECTING)
  {
    calibration->status = FOC_CURRENT_ZERO_FAULT;
  }
}
