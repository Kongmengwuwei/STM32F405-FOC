#include "foc_profile.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    assert(FOC_POLE_PAIRS == 7u);
    assert(FOC_CALIBRATION_ID == ((FOC_PORT_ID << 12) |
        (FOC_MOTOR_ID << 8) | (FOC_ENCODER_ID << 4) | FOC_INSTALLATION_ID));
    assert(FOC_BUS_MIN >= FOC_PORT_BUS_MIN && FOC_BUS_MIN >= FOC_MOTOR_BUS_MIN);
    assert(FOC_BUS_MAX <= FOC_PORT_BUS_MAX && FOC_BUS_MAX <= FOC_MOTOR_BUS_MAX);
    assert(FOC_BUS_MIN < FOC_BUS_MAX);
    assert(FOC_CURRENT_MAX <= FOC_PORT_CURRENT_MAX &&
           FOC_CURRENT_MAX <= FOC_MOTOR_CURRENT_MAX);
    assert(FOC_PHASE_TRIP <= FOC_PORT_PHASE_TRIP &&
           FOC_PHASE_TRIP <= FOC_MOTOR_PHASE_TRIP);
    assert(FOC_MAX_STEP_DEG > FOC_SPEED_STEP_DEG);
    assert(FOC_AUTOCALIBRATE == 0);
#if defined(FOC_PORT_M1) && defined(FOC_ENCODER_TLE5012B)
    assert(FOC_SAMPLE_HZ == 10000u && FOC_PWM_ARR == 8400u && FOC_TRIGGER_TICKS == 8200u);
#elif defined(FOC_PORT_M1)
    assert(FOC_SAMPLE_HZ == 20000u && FOC_PWM_ARR == 4200u && FOC_TRIGGER_TICKS == 4100u);
#endif
#ifdef FOC_MOTOR_ZH3620_1
    assert(FOC_MOTOR_STALL_CURRENT_A == 12.0f && FOC_CURRENT_MAX < 12.0f);
    assert(FOC_BUS_MAX <= 12.4f && FOC_MOTOR_ABSOLUTE_VOLTAGE_MAX == 20.0f);
#endif
#ifdef FOC_MOTOR_L4225_6S420_A
    assert(FOC_MOTOR_ID == 3u && FOC_MOTOR_SLOTS == 12u);
    assert(FOC_MOTOR_KV_RPM_PER_V == 420.0f);
    assert(FOC_BUS_MIN == 11.1f && FOC_BUS_MAX <= 24.0f);
    assert(FOC_CURRENT_MAX == 1.0f && FOC_PHASE_TRIP == 4.0f);
    assert(FOC_CURRENT_MAX < FOC_MOTOR_DATASHEET_MAX_CURRENT_A);
    assert(FOC_MOTOR_INDUCTANCE_H == 0.0f && FOC_MOTOR_FLUX_WB == 0.0f);
    assert(FOC_ENCODER_HARMONIC_DEG == 0.0f);
#endif
    printf("PASS: %s/%s/%s installation %u\n", FOC_PORT_NAME,
           FOC_ENCODER_NAME, FOC_MOTOR_NAME, FOC_INSTALLATION_ID);
    return 0;
}
