#ifndef FOC_MOTOR_PROFILE_H
#define FOC_MOTOR_PROFILE_H

/* Motor model data and cautious bring-up limits. Verify unit-specific winding
 * values and control gains before increasing limits or attaching a load. */
#ifdef FOC_MOTOR_ZH3620_1
#define FOC_MOTOR_ID 2u
#define FOC_MOTOR_NAME "ZH3620-1"
#define FOC_POLE_PAIRS 7u /* Image: 14 magnetic poles, not 14 pole pairs. */
#define FOC_MOTOR_SLOTS 12u
#define FOC_MOTOR_KV_RPM_PER_V 500.0f
#define FOC_MOTOR_RATED_TORQUE_NM 0.12f
#define FOC_MOTOR_MASS_G 77.4f
#define FOC_MOTOR_STALL_CURRENT_A 12.0f /* Data-sheet value, NOT a limit. */
#define FOC_MOTOR_NOLOAD_CURRENT_A 0.4f
#define FOC_MOTOR_ABSOLUTE_VOLTAGE_MAX 20.0f
#define FOC_MOTOR_AMBIENT_MIN_C 5.0f
#define FOC_MOTOR_AMBIENT_MAX_C 40.0f
#define FOC_MOTOR_BUS_MIN 7.4f
#define FOC_MOTOR_BUS_MAX 12.4f /* 20 V absolute maximum is not a test voltage. */
#define FOC_MOTOR_CURRENT_MAX 0.30f /* Short, unloaded command limit pending hardware checks. */
#define FOC_MOTOR_PHASE_TRIP 1.20f /* Controlled calibration was observed below this limit. */
#define FOC_MOTOR_SPEED_MAX 100.0f /* Unvalidated speed/position loops stay capped. */
#define FOC_MOTOR_TORQUE_SPEED_TRIP_RPM 1000.0f /* Allow short current-loop tests below the 12 V no-load speed. */
#define FOC_MOTOR_ALIGNMENT_VOLTS 0.20f /* Ceiling for current-regulated alignment. */
#define FOC_MOTOR_ALIGNMENT_CURRENT_A 0.45f
#define FOC_MOTOR_ALIGNMENT_SWEEP_TICKS_20KHZ 80000u /* Four seconds per electrical turn. */
#define FOC_MOTOR_RESISTANCE_OHM 0.12f /* Provisional; measure phase resistance. */
#define FOC_MOTOR_INDUCTANCE_H 0.0f /* Feedforward disabled until measured. */
#define FOC_MOTOR_FLUX_WB 0.0f
#define FOC_MOTOR_CURRENT_KP 0.20f
#define FOC_MOTOR_CURRENT_KI_PER_S 20.0f
#define FOC_MOTOR_SPEED_KP 0.002f
#define FOC_MOTOR_SPEED_KI 0.004f
#define FOC_MOTOR_POSITION_KP 1.0f
#define FOC_MOTOR_POSITION_SPEED_MAX 30.0f
#else
/* Original reference firmware motor: exact model name has not been supplied.
 * Keep its historical tuning separate; do not identify it as ZH3620-1. */
#define FOC_MOTOR_ID 1u
#define FOC_MOTOR_NAME "REFERENCE_24V_UNCONFIRMED"
#define FOC_POLE_PAIRS 7u
#define FOC_MOTOR_BUS_MIN 8.0f
#define FOC_MOTOR_BUS_MAX 36.0f /* Historical driver limit; check motor rating. */
#define FOC_MOTOR_CURRENT_MAX 5.0f
#define FOC_MOTOR_PHASE_TRIP 10.0f
#define FOC_MOTOR_SPEED_MAX 9400.0f
#define FOC_MOTOR_TORQUE_SPEED_TRIP_RPM FOC_MOTOR_SPEED_MAX
#define FOC_MOTOR_ALIGNMENT_VOLTS 0.6f
#define FOC_MOTOR_ALIGNMENT_CURRENT_A 0.0f
#define FOC_MOTOR_ALIGNMENT_SWEEP_TICKS_20KHZ 40000u
#define FOC_MOTOR_RESISTANCE_OHM 0.12f
#define FOC_MOTOR_INDUCTANCE_H 50e-6f
#define FOC_MOTOR_FLUX_WB 0.0021f
#define FOC_MOTOR_CURRENT_KP 0.1884955592f
#define FOC_MOTOR_CURRENT_KI_PER_S 452.389342f
#define FOC_MOTOR_SPEED_KP 0.005f
#define FOC_MOTOR_SPEED_KI 0.01f
#define FOC_MOTOR_POSITION_KP 4.0f
#define FOC_MOTOR_POSITION_SPEED_MAX 100.0f
#endif

#endif
