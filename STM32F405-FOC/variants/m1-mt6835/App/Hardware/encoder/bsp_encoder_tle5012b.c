#include "bsp_encoder.h"
#include "tle5012b_protocol.h"
#include "stm32f4xx_hal.h"
#include <math.h>

volatile float encoder_angle_deg = NAN, encoder_raw_deg = NAN, encoder_sample_delay;
volatile uint32_t encoder_errors;

/* Bounded register-level SPI: HAL timeouts use SysTick, which cannot preempt
 * the acquisition ISR. Three separate 16-bit words preserve the sensor's
 * mandatory >=130 ns command-to-reply idle interval. */
static bool transfer_word(uint16_t tx, uint16_t *rx)
{
    uint32_t start = DWT->CYCCNT;
    while (!(SPI3->SR & SPI_SR_TXE))
        if ((uint32_t)(DWT->CYCCNT - start) > 3000u) return false;
    SPI3->DR = tx;
    start = DWT->CYCCNT;
    while (!(SPI3->SR & SPI_SR_RXNE))
        if ((uint32_t)(DWT->CYCCNT - start) > 3000u) return false;
    *rx = (uint16_t)SPI3->DR;
    start = DWT->CYCCNT;
    while (SPI3->SR & SPI_SR_BSY)
        if ((uint32_t)(DWT->CYCCNT - start) > 3000u) return false;
    return true;
}

static bool read_frame(uint16_t command, uint16_t *data, uint16_t *safety)
{
    uint16_t ignored;
    GPIOA->BSRR = GPIO_PIN_0 << 16;
    bool ok = transfer_word(command, &ignored);
    uint32_t gap = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - gap) < 32u) {}
    if (ok) ok = transfer_word(0xffffu, data);
    if (ok) ok = transfer_word(0xffffu, safety);
    GPIOA->BSRR = GPIO_PIN_0;
    if (!ok) {
        SPI3->CR1 &= ~SPI_CR1_SPE;
        (void)SPI3->DR; (void)SPI3->SR;
        SPI3->CR1 |= SPI_CR1_SPE;
    }
    return ok;
}

bool bsp_encoder_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    GPIOA->BSRR = GPIO_PIN_0;
    GPIOA->BSRR = GPIO_PIN_1;
    GPIOA->MODER = (GPIOA->MODER & ~(3u << 2)) | (1u << 2);
    SPI3->CR1 |= SPI_CR1_SPE;
    uint16_t status, safety;
    bool ok = read_frame(0x8001u, &status, &safety) &&
              tle5012b_safety_crc_ok(0x8001u, status, safety);
    if (!ok) ++encoder_errors;
    return ok;
}

void bsp_encoder_begin(void)
{
    uint16_t data = 0u, safety = 0u;
    bool ok = read_frame(0x8021u, &data, &safety) &&
              tle5012b_angle_sample_valid(0x8021u, data, safety);
    if (!ok) {
        encoder_angle_deg = encoder_raw_deg = NAN;
        ++encoder_errors;
        return;
    }
    encoder_raw_deg = encoder_angle_deg =
        (float)tle5012b_angle15(data) * (360.0f / 32768.0f);
    /* Internal TLE5012B angle age is not calibrated; no prediction yet. */
    encoder_sample_delay = 0.0f;
}

void bsp_encoder_finish(void) {}
void bsp_encoder_stop(void) { GPIOA->BSRR = GPIO_PIN_0; }
