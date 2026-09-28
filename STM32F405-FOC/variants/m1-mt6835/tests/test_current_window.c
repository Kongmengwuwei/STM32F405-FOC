/* Regression for port-specific apertures, physical modulation saturation
 * and independent current-PI back calculation. No motor hardware needed. */
#include "foc.h"
#include "bsp_uart.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%d: %s\n", __LINE__, #x); exit(1); } } while (0)
volatile uint32_t motor_sample_us;
uint32_t bsp_uart_millis(void) { return motor_sample_us / 1000u; }

int main(void)
{
#ifdef FOC_PORT_M0
#if FOC_M0_DUAL_ADC
#if FOC_M0_CURRENT_SAMPLE_CYCLES == 56u
    _Static_assert(FOC_APERTURE_TICKS == 448u, "56-cycle simultaneous B/C hold");
#else
    _Static_assert(FOC_APERTURE_TICKS == 224u, "simultaneous B/C hold");
#endif
#else
#if FOC_M0_CURRENT_SAMPLE_CYCLES == 56u
    _Static_assert(FOC_APERTURE_TICKS == 992u, "56-cycle sequential C hold");
#else
    _Static_assert(FOC_APERTURE_TICKS == 544u, "cover sequential C hold");
#endif
#endif
    _Static_assert(FOC_SETTLE_TICKS == 631u, "include 127-tick dead time");
    float unsafe_edge[3] = {(float)(FOC_EDGE_LIMIT + 2u) / FOC_PWM_ARR, 0.5f, 0.5f};
    CHECK(!foc_window(unsafe_edge));
#else
    _Static_assert(FOC_APERTURE_TICKS == 224u && FOC_SETTLE_TICKS == 588u, "M1 unchanged");
#endif
    for (unsigned angle = 0; angle < 360u; ++angle) {
        float theta = angle * 0.01745329252f, duty[3];
        float scale = foc_modulate(30.0f*cosf(theta), 30.0f*sinf(theta), 12.0f, duty);
        CHECK(scale > 0.0f && scale < 1.0f && foc_window(duty));
    }
    foc_calibration_t cal = {0.0f, 1};
    foc_init(&cal);
    foc.b_offset = foc.c_offset = 1.565f;
    foc.zero_ready = true; foc.state = FOC_RUN;
    CHECK(foc_current(50.0f)); /* WARN request deliberately exceeds bridge capability. */
    foc_step(0.0f, 12.0f, 1.565f, 1.565f, 0.0f);
    float d, q;
    foc_integrators(&d, &q);
    float request = FOC_CURRENT_KP * 50.0f;
    CHECK(foc.voltage_scale > 0.0f && foc.voltage_scale < 1.0f);
    CHECK(fabsf(q - (FOC_CURRENT_KI_STEP*50.0f + FOC_CURRENT_AW_STEP*(foc.uq-request))) < 1e-5f);
    CHECK(foc.warnings & (1u << FOC_VOLTAGE));
    foc_stop();
    CHECK(foc.voltage_scale == 0.0f);
    puts("PASS: both ADC apertures, all-angle saturated PWM window, explicit PI anti-windup, stop");
    return 0;
}
