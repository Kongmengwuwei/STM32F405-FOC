#ifndef MENC15A_PROTOCOL_H
#define MENC15A_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum
{
  MENC15A_READ_ANGLE_COMMAND = 0x8021,
  MENC15A_COUNTS_PER_TURN = 32768
};

/* The TLE5012B angle data word carries 15 angle bits. Validation is done
 * separately against the safety word before the device layer publishes it. */
uint16_t menc15a_candidate_angle(uint16_t response_word);

/* Shortest signed step across the 15-bit angle wrap. Diagnostic use only. */
int16_t menc15a_shortest_step(uint16_t current, uint16_t previous);

#ifdef __cplusplus
}
#endif

#endif /* MENC15A_PROTOCOL_H */
