#include "tle5012b_protocol.h"

uint8_t tle5012b_crc8(uint16_t command, uint16_t data)
{
  const uint8_t bytes[4] = {
      (uint8_t)(command >> 8), (uint8_t)command,
      (uint8_t)(data >> 8), (uint8_t)data};
  uint8_t crc = 0xFFu;
  for (unsigned int byte_index = 0u; byte_index < 4u; ++byte_index)
  {
    crc ^= bytes[byte_index];
    for (unsigned int bit_index = 0u; bit_index < 8u; ++bit_index)
    {
      crc = (uint8_t)((crc & 0x80u) != 0u
          ? (uint8_t)((crc << 1) ^ 0x1Du)
          : (uint8_t)(crc << 1));
    }
  }
  return (uint8_t)(crc ^ 0xFFu);
}

bool tle5012b_safety_crc_ok(uint16_t command, uint16_t data,
                            uint16_t safety)
{
  return (uint8_t)safety == tle5012b_crc8(command, data);
}

bool tle5012b_safety_angle_valid(uint16_t safety)
{
  return (safety & 0x1000u) != 0u;
}

bool tle5012b_safety_system_ok(uint16_t safety)
{
  return (safety & 0x4000u) != 0u;
}

bool tle5012b_safety_interface_ok(uint16_t safety)
{
  return (safety & 0x2000u) != 0u;
}

bool tle5012b_safety_reset_clear(uint16_t safety)
{
  return (safety & 0x8000u) != 0u;
}

bool tle5012b_safety_sensor0_response(uint16_t safety)
{
  return (safety & 0x0F00u) == 0x0E00u;
}

bool tle5012b_angle_sample_valid(uint16_t command, uint16_t data,
                                 uint16_t safety)
{
  return tle5012b_safety_crc_ok(command, data, safety) &&
         tle5012b_safety_angle_valid(safety) &&
         tle5012b_safety_system_ok(safety) &&
         tle5012b_safety_interface_ok(safety) &&
         tle5012b_safety_reset_clear(safety) &&
         tle5012b_safety_sensor0_response(safety);
}

uint16_t tle5012b_angle15(uint16_t data)
{
  return data & 0x7FFFu;
}
