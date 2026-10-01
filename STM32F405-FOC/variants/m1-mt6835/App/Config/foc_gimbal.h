#ifndef FOC_GIMBAL_H
#define FOC_GIMBAL_H

/* Limited-travel gimbal; independent of the bench WARN/TRIP selection. */
#ifndef FOC_GIMBAL
#define FOC_GIMBAL 0
#endif
#if FOC_GIMBAL
#ifndef FOC_DUAL
#error FOC_GIMBAL requires the dual-axis firmware
#endif
#ifndef FOC_GIMBAL_AXIS
#define FOC_GIMBAL_AXIS 0
#endif
#if FOC_GIMBAL_AXIS == 0
#define FOC_GIMBAL_TRAVEL_LIMITED 0
#define FOC_GIMBAL_TARGET_MIN_DEG (-1000000.0f)
#define FOC_GIMBAL_TARGET_MAX_DEG 1000000.0f
#define FOC_GIMBAL_TRAVEL_MIN_DEG (-1000000.0f)
#define FOC_GIMBAL_TRAVEL_MAX_DEG 1000000.0f
#define FOC_GIMBAL_ALIGNMENT_CURRENT_A 0.60f
#define FOC_GIMBAL_ALIGNMENT_VOLTS 0.60f
#define FOC_GIMBAL_ALIGNMENT_SPEED_TRIP_RPM 60.0f
#define FOC_GIMBAL_POSITION_KP 1.00f
#define FOC_GIMBAL_SPEED_KP 0.08f
#define FOC_GIMBAL_SPEED_KI 0.60f
#define FOC_GIMBAL_VELOCITY_FILTER_S 0.001f
#define FOC_GIMBAL_CURRENT_A 0.60f
#define FOC_GIMBAL_PHASE_TRIP_A 1.50f
#define FOC_GIMBAL_VOLTAGE_FRACTION 0.10f
#define FOC_GIMBAL_SPEED_RPM 4.0f /* 24 degrees/second; leaves braking margin under cable load. */
#define FOC_GIMBAL_OVERSPEED_BRAKE_A_PER_RPM 0.20f
#else
#define FOC_GIMBAL_TRAVEL_LIMITED 1
#define FOC_GIMBAL_TARGET_MIN_DEG (-85.0f)
#define FOC_GIMBAL_TARGET_MAX_DEG 85.0f
#define FOC_GIMBAL_TRAVEL_MIN_DEG (-90.0f)
#define FOC_GIMBAL_TRAVEL_MAX_DEG 90.0f
#define FOC_GIMBAL_ALIGNMENT_CURRENT_A 0.60f
#define FOC_GIMBAL_ALIGNMENT_VOLTS 0.30f
#define FOC_GIMBAL_ALIGNMENT_SPEED_TRIP_RPM 30.0f
#define FOC_GIMBAL_POSITION_KP 1.50f
#define FOC_GIMBAL_SPEED_KP 0.16f
#define FOC_GIMBAL_SPEED_KI 0.40f
#define FOC_GIMBAL_VELOCITY_FILTER_S 0.008f
#define FOC_GIMBAL_CURRENT_A 2.00f
#define FOC_GIMBAL_PHASE_TRIP_A 3.00f
#define FOC_GIMBAL_VOLTAGE_FRACTION 0.10f
#define FOC_GIMBAL_SPEED_RPM 5.0f /* 30 mechanical degrees/second. */
#define FOC_GIMBAL_OVERSPEED_BRAKE_A_PER_RPM 0.0f
#endif
#define FOC_GIMBAL_SPEED_FOLDBACK_RPM 1.0f
#define FOC_GIMBAL_SPEED_TRIP_RPM 10.0f /* Independent of the planned trajectory speed. */
_Static_assert(FOC_GIMBAL_SPEED_RPM + FOC_GIMBAL_SPEED_FOLDBACK_RPM < FOC_GIMBAL_SPEED_TRIP_RPM,
               "Leave braking headroom below the hard speed trip");
/* A released cable/load can accelerate faster than the speed PI sheds its
   load-holding integral. Remove accelerating torque over limit..limit+1 RPM.
   M0 also needs braking above limit+1 RPM even when the PI still asks for zero
   or accelerating torque. Keep stronger PI braking and the existing current
   ceiling; the independent 10 RPM hard trip is unchanged. */
static inline float foc_gimbal_limit_speed_current(float amps, float rpm, int direction)
{
    float excess = (rpm < 0.0f ? -rpm : rpm) - FOC_GIMBAL_SPEED_RPM;
    if (excess <= 0.0f) return amps;
    if (amps * rpm * (float)direction > 0.0f) {
        amps *= excess >= FOC_GIMBAL_SPEED_FOLDBACK_RPM ? 0.0f :
                1.0f - excess / FOC_GIMBAL_SPEED_FOLDBACK_RPM;
    }
    float brake = FOC_GIMBAL_OVERSPEED_BRAKE_A_PER_RPM *
                  (excess - FOC_GIMBAL_SPEED_FOLDBACK_RPM);
    if (brake <= 0.0f) return amps;
    if (brake > FOC_GIMBAL_CURRENT_A) brake = FOC_GIMBAL_CURRENT_A;
    if (rpm * (float)direction > 0.0f) {
        if (amps > -brake) amps = -brake;
    } else if (amps < brake) amps = brake;
    return amps;
}
#define FOC_GIMBAL_ACCEL_RPM_PER_S 10.0f
#define FOC_GIMBAL_CURRENT_TEST_A 0.30f
#define FOC_GIMBAL_SPEED_TEST_RPM 2.0f
#define FOC_GIMBAL_TEST_TRAVEL_DEG 10.0f
#if FOC_GIMBAL_TRAVEL_LIMITED
_Static_assert(FOC_GIMBAL_TRAVEL_MAX_DEG - FOC_GIMBAL_TRAVEL_MIN_DEG < 360.0f,
               "Gimbal travel must stay below one mechanical revolution");
_Static_assert(FOC_GIMBAL_TARGET_MIN_DEG > FOC_GIMBAL_TRAVEL_MIN_DEG &&
               FOC_GIMBAL_TARGET_MAX_DEG < FOC_GIMBAL_TRAVEL_MAX_DEG,
               "Leave braking space beyond the command limits");
#endif
#endif
#endif
