#include "board_pwm_m0.h"

#include "stm32f4xx_hal.h"
#include "board_pwm_safe.h"

#include <math.h>

enum { M0_PWM_PERIOD_TICKS = 8400, M0_ADC_TRIGGER_TICKS = 8200 };
/* TIM1 at 168 MHz: DTG 0x7f gives 127 t_DTS, about 0.76 us. */
enum { M0_DEADTIME_DTG = 0x7F };
static bool outputs_prepared;
volatile uint32_t g_pwm_enable_stage;
volatile uint32_t g_pwm_enable_ccer_before;
volatile uint32_t g_pwm_enable_bdtr_before;
volatile uint32_t g_pwm_enable_ccer_after;
volatile uint32_t g_pwm_enable_bdtr_after;

static const uint32_t m0_channel_enable_bits =
    TIM_CCER_CC1E | TIM_CCER_CC1NE |
    TIM_CCER_CC2E | TIM_CCER_CC2NE |
    TIM_CCER_CC3E | TIM_CCER_CC3NE;

bool board_pwm_m0_start_sampling_clock(void)
{
  outputs_prepared = false;
  if (HAL_RCC_GetHCLKFreq() != 168000000u ||
      HAL_RCC_GetPCLK2Freq() != 84000000u)
  {
    return false;
  }
  __HAL_RCC_TIM1_CLK_ENABLE();
  TIM1->CR1 = 0u;
  TIM1->CCER = 0u;
  TIM1->BDTR = 0u;
  TIM1->DIER = 0u;
  TIM1->PSC = 0u;
  TIM1->ARR = M0_PWM_PERIOD_TICKS;
  TIM1->RCR = 0u;
  TIM1->CCR1 = 0u;
  TIM1->CCR2 = 0u;
  TIM1->CCR3 = 0u;
  TIM1->CCR4 = M0_ADC_TRIGGER_TICKS;
  TIM1->CCMR1 = 0u;
  TIM1->CCMR2 = TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 |
                 TIM_CCMR2_OC4PE;
  TIM1->CR2 = TIM_CR2_MMS_2 | TIM_CR2_MMS_1 | TIM_CR2_MMS_0;
  TIM1->CR1 = TIM_CR1_CMS | TIM_CR1_ARPE;
  TIM1->EGR = TIM_EGR_UG;
  TIM1->SR = 0u;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  TIM1->CR1 |= TIM_CR1_CEN;
  return board_pwm_m0_outputs_disabled();
}

uint16_t board_pwm_m0_period_ticks(void)
{
  return (uint16_t)TIM1->ARR;
}

bool board_pwm_m0_outputs_disabled(void)
{
  return (TIM1->CCER & m0_channel_enable_bits) == 0u &&
         (TIM1->BDTR & TIM_BDTR_MOE) == 0u;
}

bool board_pwm_m0_prepare_outputs(void)
{
  if (!board_pwm_m0_outputs_disabled() ||
      (TIM1->CR1 & TIM_CR1_CEN) == 0u ||
      TIM1->ARR != M0_PWM_PERIOD_TICKS)
  {
    return false;
  }
  TIM1->CCR1 = M0_PWM_PERIOD_TICKS / 2u;
  TIM1->CCR2 = M0_PWM_PERIOD_TICKS / 2u;
  TIM1->CCR3 = M0_PWM_PERIOD_TICKS / 2u;
  TIM1->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 |
                 TIM_CCMR1_OC1PE | TIM_CCMR1_OC2M_2 |
                 TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;
  TIM1->CCMR2 = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 |
                 TIM_CCMR2_OC3PE | TIM_CCMR2_OC4M_2 |
                 TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE;
  TIM1->BDTR = TIM_BDTR_OSSI | TIM_BDTR_OSSR | M0_DEADTIME_DTG;
  TIM1->EGR = TIM_EGR_UG;
  TIM1->SR = 0u;
  outputs_prepared = true;
  return board_pwm_m0_outputs_disabled();
}

bool board_pwm_m0_set_duty(const foc_pwm_duty_t *duty)
{
  if (!outputs_prepared || duty == 0 ||
      !isfinite(duty->a) || !isfinite(duty->b) || !isfinite(duty->c) ||
      duty->a < 0.2f || duty->a > 0.8f ||
      duty->b < 0.2f || duty->b > 0.8f ||
      duty->c < 0.2f || duty->c > 0.8f)
  {
    return false;
  }
  TIM1->CCR1 = (uint32_t)(duty->a * (float)M0_PWM_PERIOD_TICKS + 0.5f);
  TIM1->CCR2 = (uint32_t)(duty->b * (float)M0_PWM_PERIOD_TICKS + 0.5f);
  TIM1->CCR3 = (uint32_t)(duty->c * (float)M0_PWM_PERIOD_TICKS + 0.5f);
  return true;
}

bool board_pwm_m0_enable_outputs(void)
{
  g_pwm_enable_stage = 1u;
  g_pwm_enable_ccer_before = TIM1->CCER;
  g_pwm_enable_bdtr_before = TIM1->BDTR;
  if (!outputs_prepared || !board_pwm_m0_outputs_disabled())
  {
    g_pwm_enable_stage = outputs_prepared ? 2u : 3u;
    return false;
  }
  /* Enable alternate functions only with MOE and all six CCxE bits off. */
  GPIO_InitTypeDef gpio = {0};
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLDOWN;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF1_TIM1;
  gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOB, &gpio);
  g_pwm_enable_stage = 4u;
  TIM1->CCER |= m0_channel_enable_bits;
  g_pwm_enable_ccer_after = TIM1->CCER;
  TIM1->BDTR |= TIM_BDTR_MOE;
  g_pwm_enable_bdtr_after = TIM1->BDTR;
  g_pwm_enable_stage = 5u;
  return (TIM1->BDTR & TIM_BDTR_MOE) != 0u;
}

void board_pwm_m0_emergency_stop(void)
{
  TIM1->BDTR &= ~TIM_BDTR_MOE;
}

void board_pwm_m0_disarm(void)
{
  board_pwm_m0_emergency_stop();
  TIM1->CCER &= ~m0_channel_enable_bits;
  board_pwm_inputs_force_low();
}
