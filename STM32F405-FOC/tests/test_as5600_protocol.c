#include "as5600_protocol.h"

#include <assert.h>
#include <math.h>

int main(void)
{
  assert(as5600_magnet_status_ok(0x20u));
  assert(!as5600_magnet_status_ok(0x00u));
  assert(!as5600_magnet_status_ok(0x30u));
  assert(!as5600_magnet_status_ok(0x28u));

  uint16_t counts = 0u;
  assert(as5600_decode_raw_angle(0x0Au, 0xBCu, &counts));
  assert(counts == 0x0ABCu);
  assert(!as5600_decode_raw_angle(0xFAu, 0xBCu, &counts));
  assert(!as5600_decode_raw_angle(0x00u, 0x00u, 0));

  assert(fabsf(as5600_angle_radians(0u)) < 0.000001f);
  assert(fabsf(as5600_angle_radians(1024u) - 1.57079632679f) < 0.000001f);
  assert(as5600_angle_radians(4095u) < 6.28318530718f);
  return 0;
}
