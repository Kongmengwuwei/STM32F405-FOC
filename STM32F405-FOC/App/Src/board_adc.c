#include "board_adc.h"

#include "stm32f4xx_hal.h"

static ADC_HandleTypeDef adc1;
static ADC_HandleTypeDef adc2;
static bool adc_ready;
static bool injected_active;
static board_adc_injected_callback_t injected_callback;
static uint32_t last_hal_error;

static bool read_channel(ADC_HandleTypeDef *adc, uint32_t channel,
                         uint16_t *value)
{
  ADC_ChannelConfTypeDef config = {0};
  config.Channel = channel;
  config.Rank = 1u;
  config.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(adc, &config) != HAL_OK)
  {
    last_hal_error = HAL_ADC_GetError(adc);
    return false;
  }
  if (HAL_ADC_Start(adc) != HAL_OK)
  {
    last_hal_error = HAL_ADC_GetError(adc);
    return false;
  }
  const HAL_StatusTypeDef status = HAL_ADC_PollForConversion(adc, 2u);
  if (status == HAL_OK)
  {
    *value = (uint16_t)HAL_ADC_GetValue(adc);
  }
  const HAL_StatusTypeDef stop_status = HAL_ADC_Stop(adc);
  last_hal_error = HAL_ADC_GetError(adc);
  return status == HAL_OK && stop_status == HAL_OK;
}

bool board_adc_init(void)
{
  adc_ready = false;
  injected_active = false;
  injected_callback = 0;
  last_hal_error = HAL_ADC_ERROR_NONE;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_ADC2_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {0};
  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  gpio.Pin = GPIO_PIN_6;
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  HAL_GPIO_Init(GPIOC, &gpio);

  adc1.Instance = ADC1;
  adc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  adc1.Init.Resolution = ADC_RESOLUTION_12B;
  adc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  adc1.Init.ScanConvMode = ENABLE;
  adc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  adc1.Init.ContinuousConvMode = DISABLE;
  adc1.Init.NbrOfConversion = 1u;
  adc1.Init.DiscontinuousConvMode = DISABLE;
  adc1.Init.NbrOfDiscConversion = 1u;
  adc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  adc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  adc1.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&adc1) != HAL_OK)
  {
    last_hal_error = HAL_ADC_GetError(&adc1);
    return false;
  }

  adc2.Instance = ADC2;
  adc2.Init = adc1.Init;
  adc2.Init.ScanConvMode = DISABLE;
  if (HAL_ADC_Init(&adc2) != HAL_OK)
  {
    last_hal_error = HAL_ADC_GetError(&adc2);
    return false;
  }

  adc_ready = true;
  return true;
}

bool board_adc_read_m0(board_adc_sample_t *sample)
{
  if (!adc_ready || injected_active || sample == 0)
  {
    return false;
  }

  board_adc_sample_t next = {0};
  if (!read_channel(&adc2, ADC_CHANNEL_6, &next.bus_voltage) ||
      !read_channel(&adc1, ADC_CHANNEL_10, &next.m0_phase_b) ||
      !read_channel(&adc1, ADC_CHANNEL_11, &next.m0_phase_c))
  {
    return false;
  }
  *sample = next;
  return true;
}

bool board_adc_read_bus(uint16_t *bus_raw)
{
  return adc_ready && bus_raw != 0 &&
         read_channel(&adc2, ADC_CHANNEL_6, bus_raw);
}

bool board_adc_start_injected(board_adc_injected_callback_t callback)
{
  if (!adc_ready || injected_active || callback == 0)
  {
    return false;
  }
  ADC_InjectionConfTypeDef config = {0};
  config.InjectedChannel = ADC_CHANNEL_10;
  config.InjectedRank = ADC_INJECTED_RANK_1;
  config.InjectedSamplingTime = ADC_SAMPLETIME_28CYCLES;
  config.InjectedOffset = 0u;
  config.InjectedNbrOfConversion = 2u;
  config.InjectedDiscontinuousConvMode = DISABLE;
  config.AutoInjectedConv = DISABLE;
  config.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
  config.ExternalTrigInjecConvEdge =
      ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  if (HAL_ADCEx_InjectedConfigChannel(&adc1, &config) != HAL_OK)
  {
    last_hal_error = HAL_ADC_GetError(&adc1);
    return false;
  }
  config.InjectedChannel = ADC_CHANNEL_11;
  config.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&adc1, &config) != HAL_OK)
  {
    last_hal_error = HAL_ADC_GetError(&adc1);
    return false;
  }

  injected_callback = callback;
  HAL_NVIC_SetPriority(ADC_IRQn, 1u, 0u);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
  if (HAL_ADCEx_InjectedStart_IT(&adc1) != HAL_OK)
  {
    last_hal_error = HAL_ADC_GetError(&adc1);
    HAL_NVIC_DisableIRQ(ADC_IRQn);
    injected_callback = 0;
    return false;
  }
  injected_active = true;
  return true;
}

void board_adc_irq_handler(void)
{
  HAL_ADC_IRQHandler(&adc1);
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *adc)
{
  if (adc == &adc1 && injected_active && injected_callback != 0)
  {
    injected_callback((uint16_t)adc1.Instance->JDR1,
                      (uint16_t)adc1.Instance->JDR2);
  }
}

uint32_t board_adc_last_hal_error(void)
{
  return last_hal_error;
}
