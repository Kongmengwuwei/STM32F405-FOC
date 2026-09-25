#include "bsp_uart.h"
#include "foc.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

/* A stationary motor and equal PWM duties have no line-to-line command.
   Verify that the current zero follows a repeatable PWM-on amplifier shift. */
volatile uint32_t motor_sample_us;
uint32_t bsp_uart_millis(void) { return motor_sample_us / 1000u; }

static void sample(float b, float c)
{
    motor_sample_us += 100u;
    foc_step(0.0f, 11.5f, b, c, 3e-6f);
}

int main(void)
{
    _Static_assert(FOC_PWM_ZERO_SAMPLES == 500u, "M0 must measure 50 ms PWM zero");
    foc_init(NULL);
    for (unsigned i = 0; i < FOC_OFFSET_WAIT_SAMPLES + 2050u; ++i)
        sample(1.55f, 1.56f);
    assert(foc.state == FOC_IDLE && foc.zero_ready);
    assert(foc_calibrate());
    for (unsigned i = 0; i < FOC_PRECHARGE_SAMPLES; ++i)
        sample(1.55f, 1.56f);
    assert(foc.state == FOC_PWM_ZERO);
    assert(foc.duty[0] == 0.5f && foc.duty[1] == 0.5f && foc.duty[2] == 0.5f);
    for (unsigned i = 0; i < FOC_PWM_ZERO_SETTLE_SAMPLES + FOC_PWM_ZERO_SAMPLES; ++i)
        sample(1.548f, 1.558f);
    assert(foc.state == FOC_CALIBRATE && foc.fault == FOC_OK);
    assert(fabsf(foc.b_offset - 1.548f) < 1e-4f);
    assert(fabsf(foc.c_offset - 1.558f) < 1e-4f);
    sample(1.548f, 1.558f);
    assert(fabsf(foc.id) < 0.01f && fabsf(foc.iq) < 0.01f);
    puts("PASS: M0 zero-vector PWM offset before alignment");
    return 0;
}
