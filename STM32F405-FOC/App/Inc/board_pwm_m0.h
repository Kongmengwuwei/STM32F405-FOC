#ifndef BOARD_PWM_M0_H
#define BOARD_PWM_M0_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_modulation.h"

/* TIM1 runs at 10 kHz center aligned for ADC timing. All six driver inputs
 * remain GPIO outputs driven low; timer CH1..CH3 and MOE stay disabled. */
bool board_pwm_m0_start_sampling_clock(void);
uint16_t board_pwm_m0_period_ticks(void);
bool board_pwm_m0_outputs_disabled(void);
/* Prepare complementary CH1..3 with 1 us dead time while MOE stays off. */
bool board_pwm_m0_prepare_outputs(void);
bool board_pwm_m0_set_duty(const foc_pwm_duty_t *duty);
/* Call only after application interlocks and current polarity/angle alignment. */
bool board_pwm_m0_enable_outputs(void);
void board_pwm_m0_emergency_stop(void);
void board_pwm_m0_disarm(void);

#endif
