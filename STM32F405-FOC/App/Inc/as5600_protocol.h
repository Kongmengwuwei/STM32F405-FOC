#ifndef AS5600_PROTOCOL_H
#define AS5600_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum
{
  AS5600_I2C_ADDRESS_7BIT = 0x36,
  AS5600_REG_STATUS = 0x0B,
  AS5600_REG_RAW_ANGLE_H = 0x0C,
  AS5600_REG_RAW_ANGLE_L = 0x0D,
  AS5600_REG_AGC = 0x1A,
  AS5600_REG_MAGNITUDE_H = 0x1B,
  AS5600_ANGLE_COUNTS_PER_TURN = 4096
};

/* STATUS: MD=magnet detected, ML=weak field, MH=strong field. */
bool as5600_magnet_status_ok(uint8_t status);

/* RAW_ANGLE is a 12-bit count. The upper nibble of the first byte is unused. */
bool as5600_decode_raw_angle(uint8_t high, uint8_t low,
                              uint16_t *angle_counts);
float as5600_angle_radians(uint16_t angle_counts);

#ifdef __cplusplus
}
#endif

#endif /* AS5600_PROTOCOL_H */
