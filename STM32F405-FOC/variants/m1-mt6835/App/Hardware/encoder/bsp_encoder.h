#ifndef APP_BSP_ENCODER_H
#define APP_BSP_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

/* Both adapters publish mechanical degrees in [0,360), or NAN on failure.
 * begin() runs at the ADC first-rank interrupt. MT6835 completes via SPI DMA;
 * TLE5012B completes synchronously within that interrupt at the slower M0 rate. */
extern volatile float encoder_angle_deg, encoder_raw_deg, encoder_sample_delay;
extern volatile uint32_t encoder_errors;
bool bsp_encoder_init(void);
void bsp_encoder_begin(void);
void bsp_encoder_finish(void);
void bsp_encoder_stop(void);

#endif
