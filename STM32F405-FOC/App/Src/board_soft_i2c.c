#include "board_soft_i2c.h"

#include "stm32f4xx_hal.h"

#define SCL_PIN GPIO_PIN_4
#define SDA_PIN GPIO_PIN_5
#define CLOCK_TIMEOUT_US 1000u

static uint32_t half_period_cycles;
static uint32_t clock_timeout_cycles;
static bool bus_ready;

static void release_scl(void)
{
  GPIOB->BSRR = SCL_PIN;
}

static void pull_scl_low(void)
{
  GPIOB->BSRR = (uint32_t)SCL_PIN << 16;
}

static void release_sda(void)
{
  GPIOB->BSRR = SDA_PIN;
}

static void pull_sda_low(void)
{
  GPIOB->BSRR = (uint32_t)SDA_PIN << 16;
}

static bool scl_is_high(void)
{
  return (GPIOB->IDR & SCL_PIN) != 0u;
}

static bool sda_is_high(void)
{
  return (GPIOB->IDR & SDA_PIN) != 0u;
}

static void delay_half_period(void)
{
  const uint32_t start = DWT->CYCCNT;
  while ((uint32_t)(DWT->CYCCNT - start) < half_period_cycles)
  {
  }
}

static bool wait_scl_high(void)
{
  const uint32_t start = DWT->CYCCNT;
  while (!scl_is_high())
  {
    if ((uint32_t)(DWT->CYCCNT - start) >= clock_timeout_cycles)
    {
      return false;
    }
  }
  return true;
}

static bool make_start(void)
{
  release_sda();
  delay_half_period();
  release_scl();
  if (!wait_scl_high())
  {
    return false;
  }
  delay_half_period();
  if (!sda_is_high())
  {
    return false;
  }
  pull_sda_low();
  delay_half_period();
  pull_scl_low();
  return true;
}

static bool make_stop(void)
{
  pull_sda_low();
  delay_half_period();
  release_scl();
  if (!wait_scl_high())
  {
    return false;
  }
  delay_half_period();
  release_sda();
  delay_half_period();
  return sda_is_high();
}

static bool write_byte(uint8_t value)
{
  for (unsigned bit = 0u; bit < 8u; ++bit)
  {
    if ((value & 0x80u) != 0u)
    {
      release_sda();
    }
    else
    {
      pull_sda_low();
    }
    delay_half_period();
    release_scl();
    if (!wait_scl_high())
    {
      return false;
    }
    delay_half_period();
    pull_scl_low();
    value <<= 1;
  }

  release_sda();
  delay_half_period();
  release_scl();
  if (!wait_scl_high())
  {
    return false;
  }
  delay_half_period();
  const bool acknowledged = !sda_is_high();
  pull_scl_low();
  return acknowledged;
}

static bool read_byte(uint8_t *value, bool acknowledge)
{
  uint8_t result = 0u;
  release_sda();
  for (unsigned bit = 0u; bit < 8u; ++bit)
  {
    delay_half_period();
    release_scl();
    if (!wait_scl_high())
    {
      return false;
    }
    delay_half_period();
    result = (uint8_t)((result << 1) | (sda_is_high() ? 1u : 0u));
    pull_scl_low();
  }

  if (acknowledge)
  {
    pull_sda_low();
  }
  else
  {
    release_sda();
  }
  delay_half_period();
  release_scl();
  if (!wait_scl_high())
  {
    return false;
  }
  delay_half_period();
  pull_scl_low();
  release_sda();
  *value = result;
  return true;
}

static bool recover_bus(void)
{
  release_sda();
  for (unsigned pulse = 0u; pulse < 9u && !sda_is_high(); ++pulse)
  {
    pull_scl_low();
    delay_half_period();
    release_scl();
    if (!wait_scl_high())
    {
      return false;
    }
    delay_half_period();
  }
  return make_stop();
}

bool board_soft_i2c_init(void)
{
  bus_ready = false;
  if (SystemCoreClock < 1000000u)
  {
    return false;
  }

  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIOB->BSRR = SCL_PIN | SDA_PIN;

  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = SCL_PIN | SDA_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  const uint32_t start = DWT->CYCCNT;
  for (unsigned count = 0u; count < 256u; ++count)
  {
    __NOP();
  }
  if (DWT->CYCCNT == start)
  {
    return false;
  }

  /* Each half-period is at least 5 us, so the bus stays below 100 kHz. */
  half_period_cycles = (SystemCoreClock + 199999u) / 200000u;
  clock_timeout_cycles =
      (SystemCoreClock / 1000000u) * CLOCK_TIMEOUT_US;
  bus_ready = recover_bus();
  return bus_ready;
}

bool board_soft_i2c_read_registers(uint8_t address_7bit, uint8_t reg,
                                   uint8_t *data, size_t length)
{
  if (!bus_ready || address_7bit > 0x7Fu || data == 0 ||
      length == 0u || length > 16u)
  {
    return false;
  }

  bool success = make_start();
  if (success)
  {
    success = write_byte((uint8_t)(address_7bit << 1)) &&
              write_byte(reg) && make_start() &&
              write_byte((uint8_t)((address_7bit << 1) | 1u));
  }
  for (size_t index = 0u; success && index < length; ++index)
  {
    success = read_byte(&data[index], index + 1u < length);
  }

  if (!make_stop())
  {
    success = false;
  }
  if (!success && !recover_bus())
  {
    bus_ready = false;
  }
  return success;
}
