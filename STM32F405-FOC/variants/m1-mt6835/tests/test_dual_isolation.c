#include "dual.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

volatile uint32_t motor0_sample_us, motor1_sample_us;
uint32_t bsp_uart_millis(void) { return 1000u; }

int main(void)
{
    const foc_calibration_t c0 = {.zero = 0.1f, .direction = 1};
    const foc_calibration_t c1 = {.zero = 0.2f, .direction = -1};
    foc0_init(&c0);
    foc1_init(&c1);
    for (unsigned i = 0u; i < FOC_OFFSET_WAIT_SAMPLES + 2048u; ++i)
        foc0_step(10.0f, 12.0f, 1.65f, 1.65f, 0.0f);
    assert(foc0.state == FOC_IDLE && foc0.zero_ready);
    assert(foc1.state == FOC_OFFSET && !foc1.zero_ready);
    for (unsigned i = 0u; i < FOC_OFFSET_WAIT_SAMPLES + 2048u; ++i)
        foc1_step(20.0f, 12.0f, 1.65f, 1.65f, 0.0f);
    assert(foc1.state == FOC_IDLE && foc1.zero_ready);
    assert(control0_speed(10.0f));
    assert(foc0.state == FOC_PRECHARGE && foc1.state == FOC_IDLE);
    assert(control1_position(30.0f));
    assert(foc1.state == FOC_PRECHARGE);
    foc0_trip(FOC_SENSOR);
    assert(foc0.state == FOC_FAULT && foc1.state == FOC_PRECHARGE);
    assert(foc1.calibration.direction == -1 && foc0.calibration.direction == 1);
    puts("dual isolation ok");
    return 0;
}
