#ifndef FOC_TRANSFORMS_H
#define FOC_TRANSFORMS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Amplitude-invariant coordinates: alpha is aligned with phase A. */
typedef struct
{
  float alpha;
  float beta;
} foc_alpha_beta_t;

typedef struct
{
  float d;
  float q;
} foc_dq_t;

/* Assumes ia + ib + ic = 0; only phase A and B currents are needed. */
foc_alpha_beta_t foc_clarke(float ia, float ib);
/* For this board's B/C low-side shunts; reconstructs ia = -ib - ic. */
foc_alpha_beta_t foc_clarke_from_bc(float ib, float ic);

/* angle_sin and angle_cos are for the electrical rotor angle in radians. */
foc_dq_t foc_park(foc_alpha_beta_t stationary,
                  float angle_sin, float angle_cos);
foc_alpha_beta_t foc_inverse_park(foc_dq_t rotating,
                                  float angle_sin, float angle_cos);

#ifdef __cplusplus
}
#endif

#endif /* FOC_TRANSFORMS_H */
