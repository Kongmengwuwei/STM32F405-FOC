#ifndef DUAL_FINITE_MATH_H
#define DUAL_FINITE_MATH_H
#include <math.h>
static inline float dual_min(float x, float y) {
  if (isnan(x))
    return y;
  if (isnan(y))
    return x;
  return x < y ? x : y;
}
static inline float dual_max(float x, float y) {
  if (isnan(x))
    return y;
  if (isnan(y))
    return x;
  return x > y ? x : y;
}
#define fminf dual_min
#define fmaxf dual_max
#endif
