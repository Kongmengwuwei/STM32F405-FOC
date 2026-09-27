#ifndef FOC_POLICY_H
#define FOC_POLICY_H

/* Bench policy is independent of port, motor and encoder selection.
 * WARN is the default requested for externally powered bench experiments.
 * Thresholds remain useful diagnostics; they do not reject targets or trip.
 * Timer dead time, representable PWM and valid control inputs are required
 * for the algorithm itself and are not bypassed by this policy. */
#ifndef FOC_PROTECTION_TRIP
#define FOC_PROTECTION_TRIP 0
#endif
#if FOC_PROTECTION_TRIP != 0 && FOC_PROTECTION_TRIP != 1
#error FOC_PROTECTION_TRIP must be 0 (WARN) or 1 (TRIP)
#endif
#define FOC_POLICY_NAME (FOC_PROTECTION_TRIP ? "TRIP" : "WARN")
/* 0 follows target steps directly; positive values request an A/s ramp. */
#ifndef FOC_CURRENT_SLEW_A_PER_S
#define FOC_CURRENT_SLEW_A_PER_S (FOC_PROTECTION_TRIP ? 1.0f : 0.0f)
#endif

#endif
