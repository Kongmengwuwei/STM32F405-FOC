#ifndef AS5047P_PROTOCOL_H
#define AS5047P_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum
{
  AS5047P_REG_NOP = 0x0000,
  AS5047P_REG_DIAAGC = 0x3FFC,
  AS5047P_REG_ANGLECOM = 0x3FFF,
  AS5047P_ANGLE_COUNTS_PER_TURN = 16384
};

/* A read result arrives in the frame after its command (one CSn cycle later). */
uint16_t as5047p_make_read_command(uint16_t address);

/* Returns false on parity or sensor-reported command error. */
bool as5047p_decode_response(uint16_t frame, uint16_t *data);

/* Valid 14-bit angle counts map to [0, 2*pi) mechanical radians. */
float as5047p_angle_radians(uint16_t angle_counts);

#ifdef __cplusplus
}
#endif

#endif /* AS5047P_PROTOCOL_H */
