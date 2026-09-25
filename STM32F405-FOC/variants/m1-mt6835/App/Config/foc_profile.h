#ifndef FOC_PROFILE_H
#define FOC_PROFILE_H

/* Orthogonal build selections: power port, angle sensor and motor model.
 * The installation number changes when the sensor magnet or phase wiring is
 * moved. An unknown selection is an error, never an implicit M1 fallback. */
#if (defined(FOC_PORT_M0) + defined(FOC_PORT_M1)) != 1
#error Select exactly one FOC_PORT_M0 or FOC_PORT_M1
#endif
#if (defined(FOC_ENCODER_TLE5012B) + defined(FOC_ENCODER_MT6835)) != 1
#error Select exactly one supported FOC_ENCODER
#endif
#if (defined(FOC_MOTOR_ZH3620_1) + defined(FOC_MOTOR_REFERENCE_24V)) != 1
#error Select exactly one supported FOC_MOTOR
#endif

#include "foc_port.h"
#include "foc_motor.h"
#include "foc_encoder_profile.h"

#ifndef FOC_INSTALLATION_ID
#error FOC_INSTALLATION_ID must be set for the physical motor/encoder mounting
#endif
#if FOC_INSTALLATION_ID < 1 || FOC_INSTALLATION_ID > 15
#error FOC_INSTALLATION_ID must be in 1..15
#endif

/* Both the motor and the output stage must allow a command. A data-sheet
 * maximum or stall current is never treated as a first-run command limit. */
#define FOC_BUS_MIN ((FOC_PORT_BUS_MIN > FOC_MOTOR_BUS_MIN) ? FOC_PORT_BUS_MIN : FOC_MOTOR_BUS_MIN)
#define FOC_BUS_MAX ((FOC_PORT_BUS_MAX < FOC_MOTOR_BUS_MAX) ? FOC_PORT_BUS_MAX : FOC_MOTOR_BUS_MAX)
#define FOC_CURRENT_MAX ((FOC_PORT_CURRENT_MAX < FOC_MOTOR_CURRENT_MAX) ? FOC_PORT_CURRENT_MAX : FOC_MOTOR_CURRENT_MAX)
#define FOC_PHASE_TRIP ((FOC_PORT_PHASE_TRIP < FOC_MOTOR_PHASE_TRIP) ? FOC_PORT_PHASE_TRIP : FOC_MOTOR_PHASE_TRIP)
#define FOC_SPEED_MAX FOC_MOTOR_SPEED_MAX
#define FOC_TORQUE_SPEED_TRIP_RPM FOC_MOTOR_TORQUE_SPEED_TRIP_RPM
#define FOC_ALIGNMENT_VOLTS FOC_MOTOR_ALIGNMENT_VOLTS
#define FOC_ALIGNMENT_CURRENT_A FOC_MOTOR_ALIGNMENT_CURRENT_A
#define FOC_ALIGNMENT_SWEEP_TICKS_20KHZ FOC_MOTOR_ALIGNMENT_SWEEP_TICKS_20KHZ
#define FOC_CURRENT_KP FOC_MOTOR_CURRENT_KP
#define FOC_CURRENT_KI_STEP (FOC_MOTOR_CURRENT_KI_PER_S / (float)FOC_SAMPLE_HZ)
#define FOC_SPEED_KP FOC_MOTOR_SPEED_KP
#define FOC_SPEED_KI FOC_MOTOR_SPEED_KI
#define FOC_POSITION_KP FOC_MOTOR_POSITION_KP
#define FOC_POSITION_SPEED_MAX FOC_MOTOR_POSITION_SPEED_MAX
#define FOC_AUTOCALIBRATE 0 /* First physical test always requires explicit cal. */

/* v3 Flash identity: port, motor model, encoder model, installation (4 bits
 * each). The old v1/v2 records cannot prove all four and are not reused. */
#define FOC_CALIBRATION_ID ((FOC_PORT_ID << 12) | (FOC_MOTOR_ID << 8) | \
                            (FOC_ENCODER_ID << 4) | FOC_INSTALLATION_ID)
#define FOC_PWM_PERIOD_TICKS (2u * FOC_PWM_ARR)
#define FOC_PRECHARGE_SAMPLES (FOC_SAMPLE_HZ / 500u)
#define FOC_PWM_ZERO_SAMPLES FOC_PORT_PWM_ZERO_SAMPLES
#define FOC_PWM_ZERO_SETTLE_SAMPLES (FOC_SAMPLE_HZ / 1000u)
#define FOC_OFFSET_WAIT_SAMPLES (FOC_SAMPLE_HZ / 5u)
#define FOC_CAL_TICKS(at_20khz) ((at_20khz) * FOC_SAMPLE_HZ / 20000u)

#if FOC_PWM_ARR <= FOC_TRIGGER_TICKS || FOC_SAMPLE_HZ < 10000u
#error Invalid PWM/ADC timing profile
#endif
_Static_assert(FOC_BUS_MIN < FOC_BUS_MAX, "Motor and port voltage ranges must overlap");
_Static_assert(FOC_CURRENT_MAX > 0.0f && FOC_CURRENT_MAX < FOC_PHASE_TRIP,
               "Command current must remain below the phase trip threshold");
_Static_assert(FOC_TORQUE_SPEED_TRIP_RPM >= FOC_SPEED_MAX,
               "Current-mode speed trip must not be below the speed-mode trip");
_Static_assert(FOC_ALIGNMENT_CURRENT_A >= 0.0f && FOC_ALIGNMENT_CURRENT_A < FOC_PHASE_TRIP,
               "Alignment current must remain below the phase trip threshold");
_Static_assert(FOC_ALIGNMENT_SWEEP_TICKS_20KHZ >= 40000u && FOC_ALIGNMENT_SWEEP_TICKS_20KHZ <= 80000u,
               "Alignment scan timing is outside the validated range");
_Static_assert(FOC_POLE_PAIRS > 0u && FOC_POLE_PAIRS < 65536u,
               "Pole pairs must fit the calibration record");
#ifdef FOC_MOTOR_ZH3620_1
_Static_assert(FOC_MOTOR_BUS_MAX < FOC_MOTOR_ABSOLUTE_VOLTAGE_MAX,
               "ZH3620-1 test voltage must stay below its absolute maximum");
#endif
#endif
