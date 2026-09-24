#include "bsp_encoder.h"
#include "mt6835_port_stm32.h"
#include "stm32f4xx_hal.h"

volatile float encoder_angle_deg, encoder_raw_deg, encoder_sample_delay;
volatile uint32_t encoder_errors;

bool bsp_encoder_init(void)
{
    GPIOA->BSRR = GPIO_PIN_1; /* Keep the other shared-SPI sensor deselected. */
    GPIOA->MODER = (GPIOA->MODER & ~(3u << 2)) | (1u << 2);
    return mt6835_init();
}
void bsp_encoder_begin(void) { mt6835_start(); }
void bsp_encoder_finish(void)
{
    mt6835_finish();
    encoder_angle_deg = mt6835_angle_deg;
    encoder_raw_deg = mt6835_raw_deg;
    encoder_sample_delay = mt6835_sample_delay;
    encoder_errors = mt6835_errors;
}
void bsp_encoder_stop(void) { mt6835_stop(); }
