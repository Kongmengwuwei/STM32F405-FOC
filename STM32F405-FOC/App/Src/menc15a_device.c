#include "menc15a_device.h"

#include "board_spi_encoder.h"
#include "menc15a_protocol.h"
#include "tle5012b_protocol.h"

volatile uint16_t menc15a_absolute_data[2] = {0u, 0u};
volatile int16_t menc15a_absolute_offset_data[2] = {0, 0};
volatile bool menc15a_last_read_ok = false;
volatile bool menc15a_last_transport_ok = false;
volatile uint16_t menc15a_last_command_reply = 0u;
volatile uint16_t menc15a_last_angle_reply = 0u;
volatile uint16_t menc15a_last_safety_reply = 0u;

static bool has_previous_m0;

uint8_t menc15a_init(void)
{
  has_previous_m0 = false;
  menc15a_last_read_ok = false;
  menc15a_last_transport_ok = false;
  menc15a_absolute_data[0] = 0u;
  menc15a_absolute_offset_data[0] = 0;
  return board_spi_encoder_init() ? 0u : 1u;
}

uint16_t menc15a_get_absolute_data(menc15a_module_enum module)
{
  if (module != menc15a_1_module)
  {
    menc15a_last_read_ok = false;
    menc15a_last_transport_ok = false;
    menc15a_absolute_offset_data[0] = 0;
    has_previous_m0 = false;
    return 0u;
  }

  uint16_t command_reply = 0u;
  uint16_t angle_reply = 0u;
  uint16_t safety_reply = 0u;
  if (!board_spi_encoder_transfer(MENC15A_READ_ANGLE_COMMAND,
                                  &command_reply, &angle_reply,
                                  &safety_reply))
  {
    menc15a_last_read_ok = false;
    menc15a_last_transport_ok = false;
    menc15a_absolute_offset_data[0] = 0;
    has_previous_m0 = false;
    return 0u;
  }

  const uint16_t angle = menc15a_candidate_angle(angle_reply);
  menc15a_last_command_reply = command_reply;
  menc15a_last_angle_reply = angle_reply;
  menc15a_last_safety_reply = safety_reply;
  menc15a_last_transport_ok = true;
  menc15a_last_read_ok = tle5012b_angle_sample_valid(
      MENC15A_READ_ANGLE_COMMAND, angle_reply, safety_reply);
  if (!menc15a_last_read_ok)
  {
    menc15a_absolute_offset_data[0] = 0;
    has_previous_m0 = false;
    return 0u;
  }
  menc15a_absolute_offset_data[0] = has_previous_m0
      ? menc15a_shortest_step(angle, menc15a_absolute_data[0]) : 0;
  menc15a_absolute_data[0] = angle;
  has_previous_m0 = true;
  return angle;
}

bool menc15a_read_status(uint16_t *status_data, uint16_t *safety_word)
{
  if (status_data == 0 || safety_word == 0)
  {
    return false;
  }
  uint16_t command_reply = 0u;
  return board_spi_encoder_transfer(0x8001u, &command_reply,
                                    status_data, safety_word);
}
