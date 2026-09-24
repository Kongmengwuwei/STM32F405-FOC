#include "board_spi_encoder.h"

#include "stm32f4xx_hal.h"

#define M0_CS_PIN GPIO_PIN_0
#define SPI_TRANSFER_TIMEOUT_MS 2u

static SPI_HandleTypeDef spi3;
static bool spi_ready;
static uint32_t last_hal_error;

bool board_spi_encoder_init(void)
{
  spi_ready = false;
  last_hal_error = HAL_SPI_ERROR_NONE;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_SPI3_CLK_ENABLE();

  /* Keep both encoder CS lines deasserted. CS1 may share the SPI bus. */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_SET);
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &gpio);

  spi3.Instance = SPI3;
  spi3.Init.Mode = SPI_MODE_MASTER;
  spi3.Init.Direction = SPI_DIRECTION_2LINES;
  spi3.Init.DataSize = SPI_DATASIZE_16BIT;
  spi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  spi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  spi3.Init.NSS = SPI_NSS_SOFT;
  spi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  spi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  spi3.Init.TIMode = SPI_TIMODE_DISABLE;
  spi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  spi3.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&spi3) != HAL_OK)
  {
    last_hal_error = HAL_SPI_GetError(&spi3);
    return false;
  }

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  spi_ready = true;
  return true;
}

bool board_spi_encoder_transfer(uint16_t command,
                                uint16_t *command_reply,
                                uint16_t *data_reply,
                                uint16_t *safety_reply)
{
  if (!spi_ready || command_reply == NULL || data_reply == NULL ||
      safety_reply == NULL)
  {
    return false;
  }

  uint16_t command_tx = command;
  uint16_t dummy_tx = 0xFFFFu;
  uint16_t command_rx = 0u;
  uint16_t data_rx = 0u;
  uint16_t safety_rx = 0u;

  HAL_GPIO_WritePin(GPIOA, M0_CS_PIN, GPIO_PIN_RESET);
  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
      &spi3, (uint8_t *)&command_tx, (uint8_t *)&command_rx, 1u,
      SPI_TRANSFER_TIMEOUT_MS);
  if (status == HAL_OK)
  {
    /* TLE5012B requires at least 130 ns between command and reply. */
    status = HAL_SPI_TransmitReceive(
        &spi3, (uint8_t *)&dummy_tx, (uint8_t *)&data_rx, 1u,
        SPI_TRANSFER_TIMEOUT_MS);
  }
  if (status == HAL_OK)
  {
    status = HAL_SPI_TransmitReceive(
        &spi3, (uint8_t *)&dummy_tx, (uint8_t *)&safety_rx, 1u,
        SPI_TRANSFER_TIMEOUT_MS);
  }
  HAL_GPIO_WritePin(GPIOA, M0_CS_PIN, GPIO_PIN_SET);
  last_hal_error = HAL_SPI_GetError(&spi3);
  if (status != HAL_OK)
  {
    return false;
  }

  *command_reply = command_rx;
  *data_reply = data_rx;
  *safety_reply = safety_rx;
  return true;
}

uint32_t board_spi_encoder_last_hal_error(void)
{
  return last_hal_error;
}
