#ifndef FOC_ENCODER_PROFILE_H
#define FOC_ENCODER_PROFILE_H

/* A sensor model is independent of the three-phase power port. */
#ifdef FOC_ENCODER_TLE5012B
#define FOC_ENCODER_ID 1u
#define FOC_ENCODER_NAME "TLE5012B"
#define FOC_RPM_FILTER_ALPHA 0.02f
#define FOC_STATIONARY_STEP_DEG 0.05f
#define FOC_ENCODER_STEP_FLOOR_DEG 0.50f
#else
#define FOC_ENCODER_ID 2u
#define FOC_ENCODER_NAME "MT6835"
#define FOC_RPM_FILTER_ALPHA 0.01f
#define FOC_STATIONARY_STEP_DEG (5.0f * 6.0f / (float)FOC_SAMPLE_HZ)
#define FOC_ENCODER_STEP_FLOOR_DEG 0.50f
#endif

/* Mechanical angle change per sample at the highest permitted mode speed,
 * with headroom for acceleration. A faster torque-mode trip must also raise
 * the encoder plausibility threshold, or it would trip FOC_SENSOR first. */
#define FOC_MAX_MODE_SPEED_RPM ((FOC_MOTOR_TORQUE_SPEED_TRIP_RPM > FOC_MOTOR_SPEED_MAX) ? \
                                 FOC_MOTOR_TORQUE_SPEED_TRIP_RPM : FOC_MOTOR_SPEED_MAX)
#define FOC_SPEED_STEP_DEG (FOC_MAX_MODE_SPEED_RPM * 6.0f / (float)FOC_SAMPLE_HZ)
#define FOC_MAX_STEP_DEG ((FOC_SPEED_STEP_DEG * 1.5f > FOC_ENCODER_STEP_FLOOR_DEG) ? \
                          FOC_SPEED_STEP_DEG * 1.5f : FOC_ENCODER_STEP_FLOOR_DEG)

/* This correction belongs to one measured sensor/magnet installation, not
 * to all MT6835 units. Other mountings start without a harmonic correction. */
#if defined(FOC_PORT_M1) && defined(FOC_ENCODER_MT6835) && \
    defined(FOC_MOTOR_REFERENCE_24V) && FOC_INSTALLATION_ID == 1
#define FOC_ENCODER_HARMONIC_DEG 0.52f
#else
#define FOC_ENCODER_HARMONIC_DEG 0.0f
#endif

#endif
