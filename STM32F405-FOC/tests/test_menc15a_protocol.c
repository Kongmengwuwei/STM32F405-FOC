#include "menc15a_protocol.h"

#include <assert.h>

int main(void)
{
  assert(menc15a_candidate_angle(0xFFFFu) == 32767u);
  assert(menc15a_candidate_angle(0x8021u) == 33u);
  assert(menc15a_shortest_step(100u, 90u) == 10);
  assert(menc15a_shortest_step(0u, 32767u) == 1);
  assert(menc15a_shortest_step(32767u, 0u) == -1);
  assert(menc15a_shortest_step(32760u, 8u) == -16);
  return 0;
}
