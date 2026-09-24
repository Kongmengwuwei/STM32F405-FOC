#include "as5600_device.h"
#include "as5600_protocol.h"

#include <assert.h>

typedef struct
{
  uint8_t status;
  uint8_t high;
  uint8_t low;
  uint8_t agc;
  uint8_t magnitude_high;
  uint8_t magnitude_low;
  bool fail;
  unsigned reads;
} fake_bus_t;

static bool fake_read(void *context, uint8_t reg, uint8_t *data, size_t length)
{
  fake_bus_t *bus = context;
  ++bus->reads;
  if (bus->fail)
  {
    return false;
  }
  if (reg == AS5600_REG_STATUS && length == 1u)
  {
    data[0] = bus->status;
    return true;
  }
  if (reg == AS5600_REG_RAW_ANGLE_H && length == 2u)
  {
    data[0] = bus->high;
    data[1] = bus->low;
    return true;
  }
  if (reg == AS5600_REG_AGC && length == 1u)
  {
    data[0] = bus->agc;
    return true;
  }
  if (reg == AS5600_REG_MAGNITUDE_H && length == 2u)
  {
    data[0] = bus->magnitude_high;
    data[1] = bus->magnitude_low;
    return true;
  }
  return false;
}

int main(void)
{
  fake_bus_t bus = { .status = 0x20u, .high = 0x01u, .low = 0x23u,
                     .agc = 0x80u, .magnitude_high = 0x04u,
                     .magnitude_low = 0x56u };
  const as5600_device_t device = { .read_registers = fake_read,
                                   .context = &bus };
  as5600_measurement_t result = {0};

  assert(as5600_device_read(&device, &result) == AS5600_READ_OK);
  assert(result.angle_counts == 0x0123u);
  assert(bus.reads == 2u);

  bus.status = 0x10u;
  bus.reads = 0u;
  assert(as5600_device_read(&device, &result) == AS5600_READ_MAGNET_ERROR);
  assert(bus.reads == 1u);

  bus.status = 0x20u;
  bus.fail = true;
  assert(as5600_device_read(&device, &result) == AS5600_READ_BUS_ERROR);

  bus.fail = false;
  bus.high = 0xF1u;
  assert(as5600_device_read(&device, &result) == AS5600_READ_DATA_ERROR);

  bus.reads = 0u;
  as5600_diagnostics_t diagnostics = {0};
  assert(as5600_device_read_diagnostics(&device, &diagnostics));
  assert(bus.reads == 3u);
  assert(diagnostics.raw_angle[0] == 0xF1u);
  assert(diagnostics.raw_angle[1] == 0x23u);
  assert(diagnostics.agc == 0x80u);
  assert(diagnostics.magnitude[0] == 0x04u);
  assert(diagnostics.magnitude[1] == 0x56u);
  return 0;
}
