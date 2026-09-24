#include "board_spi_encoder.h"
#include "menc15a_device.h"
#include "tle5012b_protocol.h"

#include <assert.h>

static unsigned int transfer_count;
static uint16_t last_command;

bool board_spi_encoder_init(void)
{
  return true;
}

bool board_spi_encoder_transfer(uint16_t command,
                                uint16_t *command_reply,
                                uint16_t *data_reply,
                                uint16_t *safety_reply)
{
  last_command = command;
  ++transfer_count;
  if (transfer_count == 3u)
  {
    return false;
  }
  const uint16_t replies[] = {0u, 32760u, 4u, 0u, 8u, 20u, 30u, 40u};
  *command_reply = command;
  *data_reply = replies[transfer_count];
  *safety_reply = (uint16_t)(0xFE00u |
      tle5012b_crc8(command, *data_reply));
  if (transfer_count == 6u)
  {
    *safety_reply ^= 1u;
  }
  return true;
}

uint32_t board_spi_encoder_last_hal_error(void)
{
  return 0u;
}

int main(void)
{
  assert(menc15a_init() == 0u);
  assert(menc15a_get_absolute_data(menc15a_1_module) == 32760u);
  assert(menc15a_last_read_ok);
  assert(menc15a_last_transport_ok);
  assert(menc15a_absolute_offset_data[0] == 0);
  assert(last_command == 0x8021u);

  assert(menc15a_get_absolute_data(menc15a_1_module) == 4u);
  assert(menc15a_absolute_offset_data[0] == 12);

  assert(menc15a_get_absolute_data(menc15a_1_module) == 0u);
  assert(!menc15a_last_read_ok);
  assert(!menc15a_last_transport_ok);
  assert(menc15a_absolute_offset_data[0] == 0);

  assert(menc15a_get_absolute_data(menc15a_1_module) == 8u);
  assert(menc15a_last_read_ok);
  assert(menc15a_absolute_offset_data[0] == 0);

  assert(menc15a_get_absolute_data(menc15a_1_module) == 20u);
  assert(menc15a_absolute_offset_data[0] == 12);
  assert(menc15a_get_absolute_data(menc15a_1_module) == 0u);
  assert(menc15a_last_transport_ok);
  assert(!menc15a_last_read_ok);
  assert(menc15a_absolute_offset_data[0] == 0);
  assert(menc15a_get_absolute_data(menc15a_1_module) == 40u);
  assert(menc15a_last_read_ok);
  assert(menc15a_absolute_offset_data[0] == 0);
  assert(menc15a_get_absolute_data(menc15a_2_module) == 0u);
  assert(!menc15a_last_read_ok);
  assert(!menc15a_last_transport_ok);
  return 0;
}
