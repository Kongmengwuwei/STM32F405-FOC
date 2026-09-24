#ifndef BOARD_PWM_SAFE_H
#define BOARD_PWM_SAFE_H

#ifdef __cplusplus
extern "C" {
#endif

/* EG2134 HIN/LIN are active high. Call before configuring TIM1/TIM8 PWM. */
void board_pwm_inputs_force_low(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_PWM_SAFE_H */
