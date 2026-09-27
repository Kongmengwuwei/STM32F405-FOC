/* Observation, persistent targets and actuator saturation in shared WARN code. */
#include "control.h"
#include "foc.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%d: %s\n", __LINE__, #x); exit(1); } } while (0)
static uint32_t tick = 0xffff00u;
static float angle = 100.0f;
uint32_t bsp_uart_millis(void) { return 5000u; }

static void run(unsigned ms, float rpm)
{
    for (unsigned n = 0; n < ms * (FOC_SAMPLE_HZ / 1000u); ++n) {
        tick = (tick + 1000000u / FOC_SAMPLE_HZ) & 0xffffffu;
        angle += rpm * 6.0f / FOC_SAMPLE_HZ;
        control_step(tick, angle);
    }
}

int main(void)
{
    foc = (foc_t){.state = FOC_IDLE, .zero_ready = true, .calibrated = true,
                  .calibration.direction = -1, .voltage_scale = 1.0f};
    control_stop(); CHECK(control_zero());
    control_step(tick, angle);
    run(100, 10.0f);
    CHECK(fabsf(control_position_deg() - 6.0f) < .03f);
    CHECK(fabsf(control_speed_rpm() - 10.0f) < .1f);
    CHECK(control_iq_ref() == 0.0f && foc.state == FOC_IDLE);
    run(20, 0.0f);
    CHECK(control_hold_position());
    CHECK(fabsf(control_position_target() - 6.0f) < .03f);
    run(20, 0.0f);
    CHECK(control_iq_ref() == 0.0f); /* PRECHARGE never integrates torque. */
    foc.state = FOC_PWM_ZERO;
    CHECK(control_speed(20.0f)); /* Targets can change during PWM zero sampling. */
    run(30, 0.0f); CHECK(control_iq_ref() == 0.0f);
    foc.state = FOC_RUN;
    run(500, 0.0f);
    CHECK(control_iq_ref() < -.1f && control_scheduled());
    CHECK(control_fault() == 0 && !(foc.warnings & (1u << FOC_UART)));
    float before = control_iq_ref();
    CHECK(control_speed(20.0f)); run(1, 0.0f);
    CHECK(fabsf(control_iq_ref() - before) < .02f); /* Resend preserves integral. */
    foc.voltage_scale = .5f; foc.iq = 0.0f;
    before = control_iq_ref(); run(500, 0.0f);
    CHECK(fabsf(control_iq_ref() - before) < .01f); /* No voltage windup. */
    CHECK(control_torque(.4f)); CHECK(control_iq_ref() == 0.0f);
    CHECK(control_speed(-20.0f)); CHECK(control_iq_ref() == 0.0f);
    control_stop(); foc.state = FOC_IDLE;
    CHECK(control_zero()); control_step(tick, angle);
    run(50, -20.0f);
    CHECK(fabsf(control_position_deg() + 6.0f) < .03f);
    tick = (tick + 100000u) & 0xffffffu; angle += 500.0f;
    control_step(tick, angle); run(10, 0.0f);
    CHECK(fabsf(control_speed_rpm()) < .1f);
    puts("PASS: idle tracking, timer wrap, holding, startup targets, resend, voltage anti-windup, mode reset, gap recovery");
    return 0;
}
