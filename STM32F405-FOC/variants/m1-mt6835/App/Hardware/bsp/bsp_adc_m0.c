#include "bsp_adc.h"
#include "bsp_motor.h"
#include "stm32f4xx_hal.h"
#include <math.h>

volatile bsp_adc_sample_t adc_sample;
volatile uint16_t adc_raw_b, adc_raw_c, adc_raw_bus;
volatile uint32_t adc_errors;
#ifdef FOC_CAPTURE
volatile uint16_t adc_debug[4];
#endif

void bsp_adc_start(void)
{
    bsp_motor_init();
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_ADC2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIOA->MODER |= 3u << 12; /* PA6: bus divider. */
    GPIOC->MODER |= 15u;      /* PC0/PC1: M0 B/C shunt amplifiers. */
    GPIOA->PUPDR &= ~(3u << 12);
    GPIOC->PUPDR &= ~15u;

    TIM1->CCER = TIM_CCER_CC4E;
    TIM1->CR2 = TIM_TRGO_OC4REF;
    DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM1_STOP;

    ADC1->CR2 = ADC2->CR2 = 0u;
    ADC->CCR = ADC_CLOCK_SYNC_PCLK_DIV4;
    ADC1->CR1 = ADC_CR1_SCAN | ADC_CR1_JEOCIE | ADC_CR1_OVRIE;
    ADC1->JSQR = ADC_JSQR_JL_0 | (10u << 10) | (11u << 15);
    ADC1->SMPR1 = (ADC_SAMPLETIME_28CYCLES << 0) |
                  (ADC_SAMPLETIME_28CYCLES << 3);
    ADC1->SR = 0u;
    ADC2->CR1 = 0u;
    ADC2->SQR1 = 0u;
    ADC2->SQR3 = 6u;
    ADC2->SMPR2 = ADC_SAMPLETIME_15CYCLES << 18;
    ADC2->SR = 0u;
    HAL_NVIC_SetPriority(ADC_IRQn, 1u, 0u);
    HAL_NVIC_ClearPendingIRQ(ADC_IRQn);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
    ADC2->CR2 = ADC_CR2_ADON;
    ADC1->CR2 = ADC_EXTERNALTRIGINJECCONV_T1_TRGO | ADC_CR2_JEXTEN_0 | ADC_CR2_ADON;
    HAL_Delay(1u);
    TIM1->BDTR |= TIM_BDTR_MOE; /* CH4 trigger only; M0 gate CCER bits remain off. */
    TIM1->CR1 |= TIM_CR1_CEN;
}

bool bsp_adc_read(void)
{
#ifdef FOC_CAPTURE
    adc_debug[3] = (uint16_t)TIM1->CNT;
#endif
    uint32_t sr1 = ADC1->SR, sr2 = ADC2->SR;
    if ((sr1 & ADC_SR_OVR) || (sr2 & ADC_SR_OVR) || !(sr2 & ADC_SR_EOC)) {
        bsp_motor_off();
        TIM1->DIER = 0u;
        TIM1->CR1 &= ~TIM_CR1_CEN;
        ADC1->CR1 = ADC2->CR1 = 0u;
        ADC1->CR2 = ADC2->CR2 = 0u;
        adc_sample.b_voltage = adc_sample.c_voltage = adc_sample.bus_voltage = NAN;
        adc_raw_b = adc_raw_c = adc_raw_bus = 0xffffu;
        ++adc_errors;
        return false;
    }
    adc_raw_b = (uint16_t)ADC1->JDR1;
    adc_raw_c = (uint16_t)ADC1->JDR2;
    adc_raw_bus = (uint16_t)ADC2->DR;
#ifdef FOC_CAPTURE
    adc_debug[0] = adc_raw_b;
    adc_debug[1] = adc_raw_c;
    adc_debug[2] = adc_raw_bus;
#endif
    adc_sample.b_voltage = (float)adc_raw_b * (FOC_ADC_VDDA / 4095.0f);
    adc_sample.c_voltage = (float)adc_raw_c * (FOC_ADC_VDDA / 4095.0f);
    adc_sample.bus_voltage = (float)adc_raw_bus * ((FOC_ADC_VDDA / 4095.0f) * (41.2f / 2.2f));
    return true;
}

void bsp_adc_stop(void)
{
    bsp_motor_off();
    TIM1->DIER = 0u;
    TIM1->CR1 &= ~TIM_CR1_CEN;
    ADC1->CR1 = ADC2->CR1 = 0u;
    ADC1->CR2 = ADC2->CR2 = 0u;
    HAL_NVIC_ClearPendingIRQ(ADC_IRQn);
    HAL_NVIC_ClearPendingIRQ(TIM1_UP_TIM10_IRQn);
}
