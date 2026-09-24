#ifndef BOARD_SPI_ENCODER_H
#define BOARD_SPI_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* M0 encoder SPI: SPI3 PC10/PC11/PC12 and software CS0 on PA0. */
bool board_spi_encoder_init(void);
bool board_spi_encoder_transfer(uint16_t command,
                                uint16_t *command_reply,
                                uint16_t *data_reply,
                                uint16_t *safety_reply);
uint32_t board_spi_encoder_last_hal_error(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_SPI_ENCODER_H */
