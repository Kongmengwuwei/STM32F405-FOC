#ifndef FOC_PORT_H
#define FOC_PORT_H

/* A port is the board's power stage, current-sense path and timer. */
#ifdef FOC_PORT_M0
#ifndef FOC_M0_DUAL_ADC
#define FOC_M0_DUAL_ADC 0
#endif
#if FOC_M0_DUAL_ADC != 0 && FOC_M0_DUAL_ADC != 1
#error FOC_M0_DUAL_ADC must be 0 or 1
#endif
#define FOC_PORT_ID 2u
#define FOC_PORT_NAME "M0"
#define FOC_PWM_TIMER TIM1
#define FOC_PWM_ARR 8400u
#define FOC_SAMPLE_HZ 10000u
#define FOC_PORT_PWM_ZERO_SAMPLES (FOC_SAMPLE_HZ / 20u) /* 50 ms gate-on zero-vector offset. */
#define FOC_SAMPLE_CYCLES_MIN 16000u
#define FOC_SAMPLE_CYCLES_MAX 17600u
#ifndef FOC_M0_TRIGGER_TICKS
#ifdef FOC_DUAL
/* SPI1 needs 2.625 MHz on the current wiring. Acquire earlier so both
   controllers still meet the unchanged 600-tick PWM preload deadline.
   foc_window()/foc_modulate() retain the quiet aperture and settling guard. */
#define FOC_M0_TRIGGER_TICKS 4000u
#else
#define FOC_M0_TRIGGER_TICKS 8200u
#endif
#endif
#define FOC_TRIGGER_TICKS FOC_M0_TRIGGER_TICKS
#define FOC_DEADTIME_TICKS 127u /* TIM1 BDTR: 127 / 168 MHz. */
#ifndef FOC_M0_CURRENT_SAMPLE_CYCLES
#define FOC_M0_CURRENT_SAMPLE_CYCLES 28u
#endif
#if FOC_M0_CURRENT_SAMPLE_CYCLES != 28u && FOC_M0_CURRENT_SAMPLE_CYCLES != 56u
#error M0 current sample length must be 28 or 56 ADC cycles
#endif
#ifdef FOC_DUAL
/* Two injected ranks on each ADC, with both ADCs sampling simultaneously. */
#if FOC_M0_CURRENT_SAMPLE_CYCLES != 28u
#error Dual acquisition uses 28 ADC sample cycles per rank
#endif
#define FOC_PORT_ADC_APERTURE_TICKS 640u
#elif FOC_M0_DUAL_ADC
#define FOC_PORT_ADC_APERTURE_TICKS (FOC_M0_CURRENT_SAMPLE_CYCLES * 8u)
#else
/* ADC1 sequential B then C: (sample + 12 convert + sample)*8.
 * Cover through C's hold end, not just B's first 28-cycle aperture. */
#define FOC_PORT_ADC_APERTURE_TICKS ((2u * FOC_M0_CURRENT_SAMPLE_CYCLES + 12u) * 8u)
#endif
#define FOC_PORT_BUS_MIN 8.0f
#define FOC_PORT_BUS_MAX 14.0f
#define FOC_PORT_CURRENT_MAX 5.00f /* M0 short-bench diagnostic threshold; default policy is WARN. */
#define FOC_PORT_PHASE_TRIP 6.00f /* M0 short-bench diagnostic threshold; default policy is WARN. */
#define FOC_CURRENT_A_PER_V 50.0f
#define FOC_ADC_VDDA 3.13f
#define FOC_VOLTAGE_FRACTION 0.10f
#else
#define FOC_PORT_ID 1u
#define FOC_PORT_NAME "M1"
#define FOC_PWM_TIMER TIM8
#define FOC_PWM_ARR 4200u
#define FOC_SAMPLE_HZ 20000u
#define FOC_PORT_PWM_ZERO_SAMPLES 0u /* Reference M1 timing remains unchanged. */
#define FOC_SAMPLE_CYCLES_MIN 8000u
#define FOC_SAMPLE_CYCLES_MAX 8800u
#define FOC_TRIGGER_TICKS 4100u
#define FOC_DEADTIME_TICKS 84u /* TIM8 BDTR: 500 ns. */
#define FOC_PORT_ADC_APERTURE_TICKS 224u /* Dual simultaneous 28-cycle hold. */
#define FOC_PORT_BUS_MIN 8.0f
#define FOC_PORT_BUS_MAX 36.0f
#define FOC_PORT_CURRENT_MAX 5.0f
#define FOC_PORT_PHASE_TRIP 10.0f
#define FOC_CURRENT_A_PER_V 50.0f
#define FOC_ADC_VDDA 3.30f
#define FOC_VOLTAGE_FRACTION 0.5773502692f
#endif

#endif
