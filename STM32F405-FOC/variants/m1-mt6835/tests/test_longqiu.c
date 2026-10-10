/* Real calibration/current controller, simulated sensor and ADC samples.
 * Run for WARN and TRIP; this is not an electrical motor simulation. */
#include "foc.h"
#include "bsp_motor_record.h"
#include "bsp_uart.h"
#include "control.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%d: %s\n", __LINE__, #x); exit(1); } } while (0)

volatile uint32_t motor_sample_us;
volatile float motor_duty[3];
uint32_t bsp_uart_millis(void) { return motor_sample_us / 1000u; }

static void sample(float degrees)
{
    motor_sample_us = (motor_sample_us + 1000000u / FOC_SAMPLE_HZ) & 0xffffffu;
    foc_step(degrees, 12.0f, 1.65f, 1.65f, 0.0f);
}

static void zero_current(void)
{
    for (unsigned n = 0; n < FOC_OFFSET_WAIT_SAMPLES + 2048u; ++n) sample(0.0f);
    CHECK(foc.state == FOC_IDLE && foc.zero_ready);
}

static void calibrate(int direction)
{
    foc_init(NULL);
    zero_current();
    CHECK(!foc.calibrated && !foc_current(0.1f));
    CHECK(foc_calibrate());
    while (foc.state == FOC_PRECHARGE) sample(0.0f);
    CHECK(foc.state == FOC_CALIBRATE);
    const unsigned hold = FOC_CAL_TICKS(30000u);
    const unsigned sweep = FOC_CAL_TICKS(FOC_ALIGNMENT_SWEEP_TICKS_20KHZ);
    const unsigned end = FOC_CAL_TICKS(40000u) + 2u * sweep;
    /* Precharge transition has already executed calibration tick 1. */
    for (unsigned tick = 2u; tick <= end; ++tick) {
        float turns = 0.0f;
        if (tick > hold && tick <= hold + sweep) turns = (float)(tick - hold) / sweep;
        if (tick > hold + sweep && tick <= hold + 2u * sweep)
            turns = 1.0f - (float)(tick - hold - sweep) / sweep;
        float angle = direction * turns * (360.0f / FOC_POLE_PAIRS);
        sample(angle < 0.0f ? angle + 360.0f : angle);
        CHECK(hypotf(foc.ud, foc.uq) <= FOC_ALIGNMENT_VOLTS + 1e-6f);
        if (tick < end) CHECK(foc.state == FOC_CALIBRATE && foc_window(foc.duty));
    }
    CHECK(foc.state == FOC_SAVE && foc.calibration.direction == direction);
    CHECK(foc.calibration.zero >= 0.0f && foc.calibration.zero < 6.2831854f);
    /* The simulated shaft returns to 0 degrees at the shifted stator field.
       Removing the configured field angle must recover the true zero. */
    CHECK(fabsf(foc_wrap(foc.calibration.zero +
        FOC_MOTOR_ALIGNMENT_START_DEG * 0.01745329252f)) < 1e-5f);
}

int main(void)
{
    CHECK(FOC_SAMPLE_HZ == 10000u && FOC_PWM_ARR == 8400u);
    CHECK(FOC_CALIBRATION_ID == (0x1310u | FOC_INSTALLATION_ID)); /* M1/Longqiu/TLE/current installation. */
    record_t record = {.version = 3u, .magic = 0x464f4331u,
        .poles = (FOC_CALIBRATION_ID << 16) | FOC_POLE_PAIRS,
        .cal = {.zero = 0.0f, .direction = 1}};
    record.checksum = checksum(&record);
    CHECK(record_valid(&record));
    record.poles = ((FOC_PORT_ID << 12 | 2u << 8 | FOC_ENCODER_ID << 4 |
                     FOC_INSTALLATION_ID) << 16) | FOC_POLE_PAIRS;
    record.checksum = checksum(&record);
    CHECK(!record_valid(&record)); /* Reject ZH3620 despite same seven pole pairs. */
    calibrate(1);
    calibrate(-1);

    /* A stationary rotor does not follow the scan: never mark it calibrated. */
    foc_init(NULL);
    zero_current();
    CHECK(foc_calibrate());
    for (unsigned n = 0; n < 200000u &&
         (foc.state == FOC_PRECHARGE || foc.state == FOC_CALIBRATE); ++n) sample(0.0f);
    CHECK(!foc.calibrated && !foc_current(0.1f));
    CHECK(foc.warnings & (1u << FOC_ALIGNMENT));
    CHECK(foc.state == (FOC_PROTECTION_TRIP ? FOC_FAULT : FOC_IDLE));

    foc_calibration_t cal = {0.0f, 1, FOC_POLE_PAIRS};
    foc_init(&cal);
    zero_current();
    if (FOC_PROTECTION_TRIP) CHECK(!foc_current(FOC_CURRENT_MAX + 0.1f));
    else {
        CHECK(foc_current(4.0f)); /* WARN retains existing command semantics. */
        foc_stop();
    }
    CHECK(foc_current(0.9f));
    for (unsigned n = 0; n < 30000u; ++n) {
        sample(0.0f);
        CHECK(foc.state == FOC_PRECHARGE || foc.state == FOC_RUN);
        CHECK(hypotf(foc.ud, foc.uq) <= 12.0f * FOC_MOTOR_RUN_VOLTAGE_FRACTION + 1e-5f);
        if (foc.state == FOC_RUN) CHECK(foc_window(foc.duty));
    }
    CHECK(foc.voltage_scale < 1.0f && (foc.warnings & (1u << FOC_VOLTAGE)));
    CHECK(foc_current(-0.9f));
    for (unsigned n = 0; n < 40000u; ++n) {
        sample(0.0f);
        CHECK(hypotf(foc.ud, foc.uq) <= 12.0f * FOC_MOTOR_RUN_VOLTAGE_FRACTION + 1e-5f && foc_window(foc.duty));
    }
    CHECK(foc.uq < 0.0f);
    foc_stop();
    CHECK(foc.state == FOC_IDLE && foc.ud == 0.0f && foc.uq == 0.0f);
    CHECK(control_speed(100.0f));
    foc.state = FOC_RUN;
    foc.voltage_scale = 1.0f;
    float outer_peak = 0.0f;
    for (unsigned n = 0; n < FOC_SAMPLE_HZ; ++n) {
        motor_sample_us = (motor_sample_us + 1000000u / FOC_SAMPLE_HZ) & 0xffffffu;
        control_step(motor_sample_us, 0.0f);
        CHECK(fabsf(control_iq_ref()) <= FOC_MOTOR_OUTER_CURRENT_MAX);
        outer_peak = fmaxf(outer_peak, fabsf(control_iq_ref()));
        if (FOC_PROTECTION_TRIP) CHECK(fabsf(control_iq_ref()) <= FOC_CURRENT_MAX);
    }
    const float expected_peak = FOC_PROTECTION_TRIP ? fminf(FOC_CURRENT_MAX, FOC_MOTOR_OUTER_CURRENT_MAX) : FOC_MOTOR_OUTER_CURRENT_MAX;
    CHECK(outer_peak > expected_peak - 0.01f);
    foc_stop();
    foc_clear_warnings();
    foc_acquisition_resume();
    sample(2.0f); /* Coast during the Flash pause; not a 100 us angle jump. */
    CHECK(foc.state == FOC_IDLE && foc.rpm == 0.0f && !(foc.warnings & (1u << FOC_SENSOR)));
    sample(4.0f); /* The next ordinary jump must still produce a diagnostic. */
    CHECK(foc.warnings & (1u << FOC_SENSOR));
    puts("PASS: Longqiu M1/TLE, old record rejection, both alignment directions, voltage saturation and stop");
    return 0;
}
