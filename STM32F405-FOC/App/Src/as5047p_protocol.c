#include "as5047p_protocol.h"

static bool has_odd_parity(uint16_t value)
{
  value ^= value >> 8;
  value ^= value >> 4;
  value ^= value >> 2;
  value ^= value >> 1;
  return (value & 1u) != 0u;
}

uint16_t as5047p_make_read_command(uint16_t address)
{
  uint16_t command = (uint16_t)((address & 0x3FFFu) | 0x4000u);
  if (has_odd_parity(command))
  {
    command |= 0x8000u;
  }
  return command;
}

bool as5047p_decode_response(uint16_t frame, uint16_t *data)
{
  if (data == 0 || has_odd_parity(frame) || (frame & 0x4000u) != 0u)
  {
    return false;
  }
  *data = frame & 0x3FFFu;
  return true;
}

float as5047p_angle_radians(uint16_t angle_counts)
{
  const float radians_per_count = 0.0003834951969714103f;
  return (float)(angle_counts & 0x3FFFu) * radians_per_count;
}
