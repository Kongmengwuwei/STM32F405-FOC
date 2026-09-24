#ifndef BOARD_ADC_H
#define BOARD_ADC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint16_t bus_voltage;
  uint16_t m0_phase_b;
  uint16_t m0_phase_c;
} board_adc_sample_t;

/* Polling, read-only bring-up path. No PWM timing guarantee. */
bool board_adc_init(void);
bool board_adc_read_m0(board_adc_sample_t *sample);
bool board_adc_read_bus(uint16_t *bus_raw);
typedef void (*board_adc_injected_callback_t)(uint16_t phase_b_raw,
                                               uint16_t phase_c_raw);
/* Switch ADC1 from startup polling to TIM1-triggered B/C injected sampling. */
bool board_adc_start_injected(board_adc_injected_callback_t callback);
void board_adc_irq_handler(void);
uint32_t board_adc_last_hal_error(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_ADC_H */
