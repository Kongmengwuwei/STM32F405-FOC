#include "foc_transforms.h"

#include <assert.h>
#include <math.h>

static void near(float actual, float expected)
{
  assert(fabsf(actual - expected) < 0.00001f);
}

int main(void)
{
  const foc_alpha_beta_t zero = foc_clarke(0.0f, 0.0f);
  near(zero.alpha, 0.0f);
  near(zero.beta, 0.0f);

  /* Balanced phase currents (1, -0.5, -0.5) point along phase A. */
  const foc_alpha_beta_t phase_a = foc_clarke(1.0f, -0.5f);
  near(phase_a.alpha, 1.0f);
  near(phase_a.beta, 0.0f);

  const foc_alpha_beta_t from_board_shunts = foc_clarke_from_bc(-0.5f, -0.5f);
  near(from_board_shunts.alpha, phase_a.alpha);
  near(from_board_shunts.beta, phase_a.beta);

  const foc_dq_t aligned = foc_park(phase_a, 0.0f, 1.0f);
  near(aligned.d, 1.0f);
  near(aligned.q, 0.0f);

  const foc_dq_t quadrature = foc_park(phase_a, 1.0f, 0.0f);
  near(quadrature.d, 0.0f);
  near(quadrature.q, -1.0f);

  const foc_alpha_beta_t restored = foc_inverse_park(quadrature, 1.0f, 0.0f);
  near(restored.alpha, phase_a.alpha);
  near(restored.beta, phase_a.beta);

  const foc_alpha_beta_t arbitrary = { .alpha = -0.7f, .beta = 1.2f };
  const float angle = 0.73f;
  const foc_dq_t rotating = foc_park(arbitrary, sinf(angle), cosf(angle));
  const foc_alpha_beta_t round_trip =
      foc_inverse_park(rotating, sinf(angle), cosf(angle));
  near(round_trip.alpha, arbitrary.alpha);
  near(round_trip.beta, arbitrary.beta);

  return 0;
}
