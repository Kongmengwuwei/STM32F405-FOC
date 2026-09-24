#include "menc15a_protocol.h"

uint16_t menc15a_candidate_angle(uint16_t response_word)
{
  return response_word & 0x7FFFu;
}

int16_t menc15a_shortest_step(uint16_t current, uint16_t previous)
{
  int32_t step = (int32_t)(current & 0x7FFFu) -
                 (int32_t)(previous & 0x7FFFu);
  if (step > (MENC15A_COUNTS_PER_TURN / 2))
  {
    step -= MENC15A_COUNTS_PER_TURN;
  }
  else if (step < -(MENC15A_COUNTS_PER_TURN / 2))
  {
    step += MENC15A_COUNTS_PER_TURN;
  }
  return (int16_t)step;
}
