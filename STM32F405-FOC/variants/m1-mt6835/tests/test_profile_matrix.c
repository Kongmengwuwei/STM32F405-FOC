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
#ifdef FOC_MOTOR_ZH3620_1
    assert(FOC_MOTOR_STALL_CURRENT_A == 12.0f && FOC_CURRENT_MAX < 12.0f);
    assert(FOC_BUS_MAX <= 12.4f && FOC_MOTOR_ABSOLUTE_VOLTAGE_MAX == 20.0f);
#endif
    printf("PASS: %s/%s/%s installation %u\n", FOC_PORT_NAME,
           FOC_ENCODER_NAME, FOC_MOTOR_NAME, FOC_INSTALLATION_ID);
    return 0;
}
