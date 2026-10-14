/* Reliable physical stop; explicit operator home required after each reboot.
 * Source: docs/data/leg-boundaries-20261014.json, control_reference.home_match_nominal_encoder_deg. */
#ifndef LEG_REFERENCE_CONFIG_H
#define LEG_REFERENCE_CONFIG_H
#ifndef LEG_REFERENCE_RAW_M0
#define LEG_REFERENCE_RAW_M0 343.657836914f
#define LEG_REFERENCE_RAW_M1 303.692620963f
#endif
#define LEG_DIRECTION_M0 1
#define LEG_DIRECTION_M1 (-1)
#define LEG_JOINT_MIN 0.0f
#define LEG_JOINT_MAX 110.0f
#endif
