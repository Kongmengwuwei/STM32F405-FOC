#ifndef FOC_PORT_H
#define FOC_PORT_H

/* A port is the board's power stage, current-sense path and timer. */
#ifdef FOC_PORT_M0
#define FOC_PORT_ID 2u
#define FOC_PORT_NAME "M0"
#define FOC_PWM_TIMER TIM1
#define FOC_PWM_ARR 8400u
#define FOC_SAMPLE_HZ 10000u
#define FOC_SAMPLE_CYCLES_MIN 16000u
#define FOC_SAMPLE_CYCLES_MAX 17600u
#define FOC_TRIGGER_TICKS 8200u
#define FOC_PORT_BUS_MIN 8.0f
#define FOC_PORT_BUS_MAX 14.0f
#define FOC_PORT_CURRENT_MAX 0.30f
#define FOC_PORT_PHASE_TRIP 0.80f
#define FOC_CURRENT_A_PER_V 50.0f
#define FOC_ADC_VDDA 3.13f
#define FOC_VOLTAGE_FRACTION 0.10f
#else
#define FOC_PORT_ID 1u
#define FOC_PORT_NAME "M1"
#define FOC_PWM_TIMER TIM8
#define FOC_PWM_ARR 4200u
#define FOC_SAMPLE_HZ 20000u
#define FOC_SAMPLE_CYCLES_MIN 8000u
#define FOC_SAMPLE_CYCLES_MAX 8800u
#define FOC_TRIGGER_TICKS 4100u
#define FOC_PORT_BUS_MIN 8.0f
#define FOC_PORT_BUS_MAX 36.0f
#define FOC_PORT_CURRENT_MAX 5.0f
#define FOC_PORT_PHASE_TRIP 10.0f
#define FOC_CURRENT_A_PER_V 50.0f
#define FOC_ADC_VDDA 3.30f
#define FOC_VOLTAGE_FRACTION 0.5773502692f
#endif

#endif
