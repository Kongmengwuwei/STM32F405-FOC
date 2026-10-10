/* Real outer loop: continuous reversal across gain ranges and timestamp wrap.
 * No mechanical plant simulation; hardware response is measured separately. */
#include "control.h"
#include "foc.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%d: %s\n", __LINE__, #x); exit(1); } } while (0)
static uint32_t tick = 0xfff000u;
uint32_t bsp_uart_millis(void) { return 1000u; }
static void run(unsigned ms)
{
    for (unsigned n = 0; n < ms * (FOC_SAMPLE_HZ / 1000u); ++n) {
        tick = (tick + 1000000u / FOC_SAMPLE_HZ) & 0xffffffu;
        control_step(tick, 0.0f);
        CHECK(isfinite(control_speed_reference()) && isfinite(control_iq_ref()));
        CHECK(fabsf(control_iq_ref()) <= FOC_MOTOR_OUTER_CURRENT_MAX);
    }
}
int main(void)
{
    for (int direction = -1; direction <= 1; direction += 2) {
        foc = (foc_t){.state = FOC_IDLE, .zero_ready = true, .calibrated = true,
                      .calibration.direction = direction, .voltage_scale = 1.0f};
        control_stop(); CHECK(control_zero()); control_step(tick, 0.0f);
        run(10); CHECK(control_speed(300.0f)); foc.state = FOC_RUN;
        run(25); CHECK(fabsf(control_speed_reference() - 300.0f) < .01f);
        CHECK(direction * control_iq_ref() > 0.0f);
        float before = control_speed_reference();
        CHECK(control_speed(-300.0f));
        CHECK(control_speed_reference() == before); /* Preserve continuous run. */
        run(20); CHECK(fabsf(control_speed_reference()) < 1.0f);
        run(25); CHECK(fabsf(control_speed_reference() + 300.0f) < .01f);
        CHECK(direction * control_iq_ref() < 0.0f);
        CHECK(control_speed(-300.0f)); run(1);
        CHECK(fabsf(control_speed_reference() + 300.0f) < .01f);
        CHECK(control_position(60.0f));
        CHECK(control_iq_ref() == 0.0f); /* New mode drops the old output. */
        run(100); CHECK(fabsf(control_speed_reference()) <= FOC_POSITION_SPEED_MAX);
        control_stop(); CHECK(control_iq_ref() == 0.0f && !control_scheduled());
    }
    puts("PASS: continuous fast reversal, timer wrap, calibrated direction, mode reset and bounded output");
    return 0;
}
