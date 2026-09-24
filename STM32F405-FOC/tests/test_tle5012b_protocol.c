#include "tle5012b_protocol.h"

#include <assert.h>

int main(void)
{
  assert(tle5012b_crc8(0x8021u, 0x995Au) == 0xB4u);
  assert(tle5012b_safety_crc_ok(0x8021u, 0x995Au, 0x3EB4u));
  assert(!tle5012b_safety_crc_ok(0x8021u, 0x995Bu, 0x3EB4u));
  assert(tle5012b_safety_angle_valid(0x3EB4u));
  assert(tle5012b_safety_interface_ok(0x3EB4u));
  assert(!tle5012b_safety_system_ok(0x3EB4u));
  assert(!tle5012b_safety_reset_clear(0x3EB4u));
  assert(tle5012b_safety_sensor0_response(0x3EB4u));
  assert(tle5012b_angle_sample_valid(
      0x8021u, 0x995Au,
      (uint16_t)(0xFE00u | tle5012b_crc8(0x8021u, 0x995Au))));
  assert(!tle5012b_angle_sample_valid(0x8021u, 0x995Au, 0x3EB4u));
  assert(!tle5012b_safety_sensor0_response(0xFFFFu));
  assert(tle5012b_angle15(0x995Au) == 0x195Au);
  return 0;
}
