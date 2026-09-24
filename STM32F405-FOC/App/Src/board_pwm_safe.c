#include "board_pwm_safe.h"

#include "stm32f4xx_hal.h"

void board_pwm_inputs_force_low(void)
{
  const uint16_t port_a_pins =
      GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  const uint16_t port_b_pins =
      GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  const uint16_t port_c_pins = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* Preload ODR low before switching pins from reset/input or timer AF mode. */
  HAL_GPIO_WritePin(GPIOA, port_a_pins, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, port_b_pins, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, port_c_pins, GPIO_PIN_RESET);

  GPIO_InitTypeDef gpio = {0};
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_PULLDOWN;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;

  gpio.Pin = port_a_pins;
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Pin = port_b_pins;
  HAL_GPIO_Init(GPIOB, &gpio);
  gpio.Pin = port_c_pins;
  HAL_GPIO_Init(GPIOC, &gpio);
}
