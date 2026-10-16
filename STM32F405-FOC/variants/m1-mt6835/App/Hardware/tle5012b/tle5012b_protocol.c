#include "tle5012b_protocol.h"
#ifdef FOC_DUAL_LEG
#include "realtime.h"
#else
#define DUAL_REALTIME
#endif

/* Polynomial 0x1d, init/xor-out 0xff. Two nibble steps per byte avoid
 * 32 data-dependent branches in each 10 kHz encoder sample. */
uint8_t DUAL_REALTIME tle5012b_crc8(uint16_t command, uint16_t data)
{
  static const uint8_t table[16]={0x00,0x1d,0x3a,0x27,0x74,0x69,0x4e,0x53,
                                 0xe8,0xf5,0xd2,0xcf,0x9c,0x81,0xa6,0xbb};
  const uint8_t bytes[4]={(uint8_t)(command>>8),(uint8_t)command,
                         (uint8_t)(data>>8),(uint8_t)data};
  uint8_t crc=0xff;
  for(unsigned n=0;n<4;n++) {
    crc^=bytes[n];
    crc=(uint8_t)((crc<<4)^table[crc>>4]);
    crc=(uint8_t)((crc<<4)^table[crc>>4]);
  }
  return crc^0xff;
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

bool tle5012b_safety_sensor_response(uint16_t safety, unsigned sensor_number)
{
  return sensor_number < 4u &&
         (safety & 0x0F00u) == (uint16_t)((0x0Fu ^ (1u << sensor_number)) << 8);
}

bool tle5012b_safety_sensor0_response(uint16_t safety)
{
  return tle5012b_safety_sensor_response(safety, 0u);
}

unsigned tle5012b_safety_sensor_number(uint16_t safety)
{
  for (unsigned number = 0u; number < 4u; ++number)
    if (tle5012b_safety_sensor_response(safety, number)) return number;
  return 4u;
}

bool DUAL_REALTIME tle5012b_angle_sample_valid_sensor(uint16_t command, uint16_t data,
                                        uint16_t safety, unsigned sensor_number)
{
  return tle5012b_safety_crc_ok(command, data, safety) &&
         tle5012b_safety_angle_valid(safety) &&
         tle5012b_safety_system_ok(safety) &&
         tle5012b_safety_interface_ok(safety) &&
         tle5012b_safety_reset_clear(safety) &&
         tle5012b_safety_sensor_response(safety, sensor_number);
}

bool tle5012b_angle_sample_valid(uint16_t command, uint16_t data,
                                 uint16_t safety)
{
  return tle5012b_angle_sample_valid_sensor(command, data, safety, 0u);
}

uint16_t tle5012b_angle15(uint16_t data)
{
  return data & 0x7FFFu;
}
