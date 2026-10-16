#ifndef FIVEBAR_CONFIG_H
#define FIVEBAR_CONFIG_H
/* User dimensions, 2026-10-14: M0 right, M1 left, wheel at distal intersection.
 * Only zero offsets are approximate. Calibrate a known rod pose before drive.
 */
/* Generated calibration header is intentionally separate from dimensions. */
#include "fivebar_calibration.h"
#ifndef FIVEBAR_ZERO_M0_CCW
#define FIVEBAR_ZERO_M0_CCW 20.0f
#endif
#ifndef FIVEBAR_ZERO_M1_CCW
#define FIVEBAR_ZERO_M1_CCW 160.0f
#endif
#define FIVEBAR_GEOMETRY_INITIALIZER                                           \
  {.pivot = {{35, 0}, {-35, 0}},                                               \
   .active = {80, 80},                                                         \
   .passive = {145, 145},                                                      \
   .zero_ccw = {FIVEBAR_ZERO_M0_CCW, FIVEBAR_ZERO_M1_CCW},                     \
   .q_sign = {-1, 1},                                                          \
   .elbow_sign = {1, -1},                                                      \
   .assembly_sign = -1,                                                        \
   .q_min = {2, 2},                                                            \
   .q_max = {108, 108},                                                        \
   .min_sine = .12f}
#ifndef FIVEBAR_ZERO_CALIBRATED
#define FIVEBAR_ZERO_CALIBRATED 0
#endif
#define FIVEBAR_FOOT_SPEED_MM_S 50.0f
#define FIVEBAR_FOOT_ACCEL_MM_S2 1800.0f
#endif
