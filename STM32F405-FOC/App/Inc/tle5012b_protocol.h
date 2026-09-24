#ifndef TLE5012B_PROTOCOL_H
#define TLE5012B_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SSC read command with address 0x02 (angle) and one data word. */
enum { TLE5012B_READ_ANGLE_WITH_SAFETY = 0x8021 };

uint8_t tle5012b_crc8(uint16_t command, uint16_t data);
bool tle5012b_safety_crc_ok(uint16_t command, uint16_t data,
                            uint16_t safety);
bool tle5012b_safety_angle_valid(uint16_t safety);
bool tle5012b_safety_system_ok(uint16_t safety);
bool tle5012b_safety_interface_ok(uint16_t safety);
bool tle5012b_safety_reset_clear(uint16_t safety);
bool tle5012b_safety_sensor0_response(uint16_t safety);
bool tle5012b_angle_sample_valid(uint16_t command, uint16_t data,
                                 uint16_t safety);
uint16_t tle5012b_angle15(uint16_t data);

#ifdef __cplusplus
}
#endif

#endif /* TLE5012B_PROTOCOL_H */
