#ifndef FOC_MOTOR_PROFILE_H
#define FOC_MOTOR_PROFILE_H

/* Motor model and per-load tuning. For ZH3620-1, PLASTIC_ARM is the default;
 * FREE_SHAFT selects the unloaded shaft profile. Both were
 * short-tested at about 12 V in September 2026. */
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
#define FOC_MOTOR_CURRENT_MAX 3.00f /* Diagnostic threshold; short loaded tests reached 2.5 A target. */
#define FOC_MOTOR_PHASE_TRIP 4.00f /* Diagnostic threshold in the default WARN policy. */
#define FOC_MOTOR_SPEED_MAX 100.0f /* Diagnostic threshold; loaded tests covered +/-60 rpm. */
#define FOC_MOTOR_TORQUE_SPEED_TRIP_RPM 1000.0f /* Allow short current-loop tests below the 12 V no-load speed. */
#define FOC_MOTOR_ALIGNMENT_VOLTS 0.20f /* Ceiling for current-regulated alignment. */
#define FOC_MOTOR_ALIGNMENT_CURRENT_A 0.45f
#define FOC_MOTOR_ALIGNMENT_SWEEP_TICKS_20KHZ 80000u /* Four seconds per electrical turn. */
#define FOC_MOTOR_RESISTANCE_OHM 0.12f /* Provisional; measure phase resistance. */
#define FOC_MOTOR_INDUCTANCE_H 0.0f /* Feedforward disabled until measured. */
#define FOC_MOTOR_FLUX_WB 0.0f
#define FOC_MOTOR_CURRENT_KP 0.20f
#define FOC_MOTOR_CURRENT_KI_PER_S 192.0f /* Current-loop A/B/A; shared by both load profiles. */
#define FOC_MOTOR_CURRENT_AW_PER_S 1200.0f /* Independent back-calculation gain, 1/s. */
#if defined(FOC_LOAD_PLASTIC_ARM) || !defined(FOC_LOAD_FREE_SHAFT)
#define FOC_LOAD_NAME "PLASTIC_ARM"
#define FOC_MOTOR_SPEED_KP 0.18f
#define FOC_MOTOR_SPEED_KI 0.48f
#define FOC_MOTOR_SPEED_SLEW_RPM_PER_S 4500.0f
#define FOC_MOTOR_POSITION_ACCEL_RPM_PER_S 2000.0f
#define FOC_MOTOR_POSITION_BRAKE_RPM_PER_S 1600.0f
#define FOC_MOTOR_POSITION_KP 12.0f
#else /* Explicit FREE_SHAFT selection. */
#define FOC_LOAD_NAME "FREE_SHAFT"
#define FOC_MOTOR_SPEED_KP 0.04f
#define FOC_MOTOR_SPEED_KI 1.20f
#define FOC_MOTOR_POSITION_SPEED_KP 0.03f /* Less position-hold dither on the free shaft. */
#define FOC_MOTOR_SPEED_SLEW_RPM_PER_S 1500.0f
#define FOC_MOTOR_POSITION_ACCEL_RPM_PER_S 1200.0f
#define FOC_MOTOR_POSITION_BRAKE_RPM_PER_S 960.0f
#define FOC_MOTOR_POSITION_KP 6.0f
#endif
#define FOC_MOTOR_POSITION_SPEED_MAX 90.0f
#else
/* Original reference firmware motor: exact model name has not been supplied.
 * Keep its historical tuning separate; do not identify it as ZH3620-1. */
#define FOC_MOTOR_ID 1u
#define FOC_MOTOR_NAME "REFERENCE_24V_UNCONFIRMED"
#define FOC_LOAD_NAME "REFERENCE"
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
#define FOC_MOTOR_CURRENT_AW_PER_S 2400.0f /* Preserve reference 0.12/cycle at 20 kHz. */
#define FOC_MOTOR_SPEED_KP 0.005f
#define FOC_MOTOR_SPEED_KI 0.01f
#define FOC_MOTOR_SPEED_SLEW_RPM_PER_S 1000.0f
#define FOC_MOTOR_POSITION_ACCEL_RPM_PER_S 1000.0f
#define FOC_MOTOR_POSITION_BRAKE_RPM_PER_S 1000.0f
#define FOC_MOTOR_POSITION_KP 4.0f
#define FOC_MOTOR_POSITION_SPEED_MAX 100.0f
#endif

#endif
