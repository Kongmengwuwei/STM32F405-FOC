#ifndef BOARD_SOFT_I2C_H
#define BOARD_SOFT_I2C_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* M0 encoder A=PB4/SCL and B=PB5/SDA, with external 3.3 kOhm pull-ups. */
bool board_soft_i2c_init(void);
bool board_soft_i2c_read_registers(uint8_t address_7bit, uint8_t reg,
                                   uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_SOFT_I2C_H */
