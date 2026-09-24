#include "as5047p_protocol.h"

#include <assert.h>
#include <math.h>

int main(void)
{
  assert(as5047p_make_read_command(AS5047P_REG_ANGLECOM) == 0xFFFFu);
  assert(as5047p_make_read_command(AS5047P_REG_NOP) == 0xC000u);

  uint16_t data = 0xAAAAu;
  assert(as5047p_decode_response(0x0000u, &data));
  assert(data == 0u);
  assert(as5047p_decode_response(0x8001u, &data));
  assert(data == 1u);
  assert(!as5047p_decode_response(0x8000u, &data));
  assert(!as5047p_decode_response(0xC000u, &data));
  assert(!as5047p_decode_response(0x0000u, 0));

  assert(fabsf(as5047p_angle_radians(0u)) < 0.000001f);
  assert(fabsf(as5047p_angle_radians(4096u) - 1.57079632679f) < 0.000001f);
  assert(as5047p_angle_radians(16383u) < 6.28318530718f);
  return 0;
}
