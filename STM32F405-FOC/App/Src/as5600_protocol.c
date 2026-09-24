#include "as5600_protocol.h"

bool as5600_magnet_status_ok(uint8_t status)
{
  const uint8_t magnet_detected = 1u << 5;
  const uint8_t magnet_too_weak = 1u << 4;
  const uint8_t magnet_too_strong = 1u << 3;
  return (status & magnet_detected) != 0u &&
         (status & (magnet_too_weak | magnet_too_strong)) == 0u;
}

bool as5600_decode_raw_angle(uint8_t high, uint8_t low,
                              uint16_t *angle_counts)
{
  if (angle_counts == 0 || (high & 0xF0u) != 0u)
  {
    return false;
  }
  *angle_counts = (uint16_t)(((uint16_t)high << 8) | low);
  return true;
}

float as5600_angle_radians(uint16_t angle_counts)
{
  const float radians_per_count = 0.0015339807878856412f;
  return (float)(angle_counts & 0x0FFFu) * radians_per_count;
}
