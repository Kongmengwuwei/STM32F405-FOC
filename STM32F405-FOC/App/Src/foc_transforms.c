#include "foc_transforms.h"

foc_alpha_beta_t foc_clarke(float ia, float ib)
{
  const float one_over_sqrt_three = 0.5773502691896258f;
  foc_alpha_beta_t result = {
    .alpha = ia,
    .beta = (ia + 2.0f * ib) * one_over_sqrt_three,
  };
  return result;
}

foc_alpha_beta_t foc_clarke_from_bc(float ib, float ic)
{
  return foc_clarke(-ib - ic, ib);
}

foc_dq_t foc_park(foc_alpha_beta_t stationary,
                  float angle_sin, float angle_cos)
{
  foc_dq_t result = {
    .d = stationary.alpha * angle_cos + stationary.beta * angle_sin,
    .q = -stationary.alpha * angle_sin + stationary.beta * angle_cos,
  };
  return result;
}

foc_alpha_beta_t foc_inverse_park(foc_dq_t rotating,
                                  float angle_sin, float angle_cos)
{
  foc_alpha_beta_t result = {
    .alpha = rotating.d * angle_cos - rotating.q * angle_sin,
    .beta = rotating.d * angle_sin + rotating.q * angle_cos,
  };
  return result;
}
