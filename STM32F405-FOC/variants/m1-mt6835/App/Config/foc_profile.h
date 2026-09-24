#ifndef FOC_PROFILE_H
#define FOC_PROFILE_H

/* The application, control loops and communication protocol are common to both
 * builds. Only this profile and the encoder/PWM/ADC board adapters differ.
 * A new motor must have its own measured values before its gates are enabled. */
#ifdef FOC_BOARD_M0
#define FOC_BOARD_ID 2u
#define FOC_MOTOR_ID 1u /* Change when motor or encoder installation changes. */
#define FOC_BOARD_NAME "M0/TLE5012B"
#define FOC_PWM_TIMER TIM1
#define FOC_PWM_ARR 8400u
#define FOC_SAMPLE_HZ 10000u
#define FOC_SAMPLE_CYCLES_MIN 16000u
#define FOC_SAMPLE_CYCLES_MAX 17600u
#define FOC_TRIGGER_TICKS 8200u
#define FOC_POLE_PAIRS 7u /* Provisional: ZH3620-1 pole count needs verification. */
#define FOC_BUS_MIN 8.0f
#define FOC_BUS_MAX 14.0f
#define FOC_SPEED_MAX 100.0f
#define FOC_CURRENT_MAX 0.30f
#define FOC_PHASE_TRIP 0.80f
#define FOC_ALIGNMENT_VOLTS 0.08f
#define FOC_ENCODER_HARMONIC_DEG 0.0f
#define FOC_RPM_FILTER_ALPHA 0.02f
#define FOC_STATIONARY_STEP_DEG 0.05f /* TLE5012B quantises to 0.011 deg. */
#define FOC_MAX_STEP_DEG 0.50f
#define FOC_CURRENT_A_PER_V 50.0f
#define FOC_ADC_VDDA 3.13f
#define FOC_MOTOR_RESISTANCE_OHM 0.12f /* Anti-windup estimate; still provisional. */
#define FOC_MOTOR_INDUCTANCE_H 0.0f /* Disable unmeasured feedforward. */
#define FOC_MOTOR_FLUX_WB 0.0f
#define FOC_CURRENT_KP 0.20f /* From the short M0 0.2 A bench trial. */
#define FOC_CURRENT_KI_STEP 0.002f
#define FOC_VOLTAGE_FRACTION 0.10f
#define FOC_SPEED_KP 0.002f
#define FOC_SPEED_KI 0.004f
#define FOC_POSITION_KP 1.0f
#define FOC_POSITION_SPEED_MAX 30.0f
#define FOC_AUTOCALIBRATE 0 /* Explicit `cal` avoids uncommanded M0 motion. */
#else
#define FOC_BOARD_ID 1u
#define FOC_MOTOR_ID 1u
#define FOC_BOARD_NAME "M1/MT6835"
#define FOC_PWM_TIMER TIM8
#define FOC_PWM_ARR 4200u
#define FOC_SAMPLE_HZ 20000u
#define FOC_SAMPLE_CYCLES_MIN 8000u
#define FOC_SAMPLE_CYCLES_MAX 8800u
#define FOC_TRIGGER_TICKS 4100u
#define FOC_POLE_PAIRS 7u
#define FOC_BUS_MIN 8.0f
#define FOC_BUS_MAX 36.0f
#define FOC_SPEED_MAX 9400.0f
#define FOC_CURRENT_MAX 5.0f
#define FOC_PHASE_TRIP 10.0f
#define FOC_ALIGNMENT_VOLTS 0.6f
#define FOC_ENCODER_HARMONIC_DEG 0.52f
#define FOC_RPM_FILTER_ALPHA 0.01f
#define FOC_STATIONARY_STEP_DEG (5.0f * 6.0f / (float)FOC_SAMPLE_HZ)
#define FOC_MAX_STEP_DEG 10.0f
#define FOC_CURRENT_A_PER_V 50.0f
#define FOC_ADC_VDDA 3.30f
#define FOC_MOTOR_RESISTANCE_OHM 0.12f
#define FOC_MOTOR_INDUCTANCE_H 50e-6f
#define FOC_MOTOR_FLUX_WB 0.0021f
#define FOC_CURRENT_KP 0.1884955592f
#define FOC_CURRENT_KI_STEP 0.0226194671f
#define FOC_VOLTAGE_FRACTION 0.5773502692f
#define FOC_SPEED_KP 0.005f
#define FOC_SPEED_KI 0.01f
#define FOC_POSITION_KP 4.0f
#define FOC_POSITION_SPEED_MAX 100.0f
#define FOC_AUTOCALIBRATE 1
#endif

#define FOC_PWM_PERIOD_TICKS (2u * FOC_PWM_ARR)
#define FOC_CALIBRATION_ID ((FOC_BOARD_ID << 8) | FOC_MOTOR_ID)
#define FOC_PRECHARGE_SAMPLES (FOC_SAMPLE_HZ / 500u)
#define FOC_OFFSET_WAIT_SAMPLES (FOC_SAMPLE_HZ / 5u)
#define FOC_CAL_TICKS(at_20khz) ((at_20khz) * FOC_SAMPLE_HZ / 20000u)

#if FOC_PWM_ARR <= FOC_TRIGGER_TICKS || FOC_SAMPLE_HZ < 10000u
#error Invalid PWM/ADC timing profile
#endif

#endif
