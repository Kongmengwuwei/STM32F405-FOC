#include "foc_current_zero.h"

#include <assert.h>

int main(void)
{
  foc_current_zero_t zero;
  foc_current_zero_init(&zero);
  for (unsigned int i = 0u; i < FOC_CURRENT_ZERO_SAMPLES; ++i)
  {
    const foc_current_zero_status_t status =
        foc_current_zero_add(&zero, (uint16_t)(2020u + (i & 1u)),
                             (uint16_t)(2030u + (i & 1u)));
    assert(status == (i + 1u == FOC_CURRENT_ZERO_SAMPLES
                          ? FOC_CURRENT_ZERO_READY
                          : FOC_CURRENT_ZERO_COLLECTING));
  }
  assert(zero.count == FOC_CURRENT_ZERO_SAMPLES);
  assert(zero.offset_b == 2020.5f);
  assert(zero.offset_c == 2030.5f);
  assert(foc_current_zero_add(&zero, 4000u, 4000u) == FOC_CURRENT_ZERO_READY);

  foc_current_zero_init(&zero);
  assert(foc_current_zero_add(&zero, 2020u, 2030u) ==
         FOC_CURRENT_ZERO_COLLECTING);
  assert(foc_current_zero_add(&zero, 2100u, 2030u) ==
         FOC_CURRENT_ZERO_FAULT);
  assert(zero.count == 1u);

  foc_current_zero_init(&zero);
  assert(foc_current_zero_add(&zero, 0u, 2030u) == FOC_CURRENT_ZERO_FAULT);
  foc_current_zero_init(&zero);
  foc_current_zero_abort(&zero);
  assert(foc_current_zero_add(&zero, 2020u, 2030u) == FOC_CURRENT_ZERO_FAULT);
  return 0;
}
