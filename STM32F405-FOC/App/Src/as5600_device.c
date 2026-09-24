#include "as5600_device.h"

#include "as5600_protocol.h"

as5600_read_status_t as5600_device_read(const as5600_device_t *device,
                                         as5600_measurement_t *measurement)
{
  if (device == 0 || device->read_registers == 0 || measurement == 0)
  {
    return AS5600_READ_DATA_ERROR;
  }

  uint8_t status = 0u;
  if (!device->read_registers(device->context, AS5600_REG_STATUS,
                              &status, 1u))
  {
    return AS5600_READ_BUS_ERROR;
  }
  measurement->magnet_status = status;
  if (!as5600_magnet_status_ok(status))
  {
    return AS5600_READ_MAGNET_ERROR;
  }

  uint8_t angle_bytes[2] = {0u, 0u};
  if (!device->read_registers(device->context, AS5600_REG_RAW_ANGLE_H,
                              angle_bytes, 2u))
  {
    return AS5600_READ_BUS_ERROR;
  }
  if (!as5600_decode_raw_angle(angle_bytes[0], angle_bytes[1],
                               &measurement->angle_counts))
  {
    return AS5600_READ_DATA_ERROR;
  }
  return AS5600_READ_OK;
}

bool as5600_device_read_diagnostics(const as5600_device_t *device,
                                    as5600_diagnostics_t *diagnostics)
{
  if (device == 0 || device->read_registers == 0 || diagnostics == 0)
  {
    return false;
  }

  as5600_diagnostics_t sample = {0};
  if (!device->read_registers(device->context, AS5600_REG_RAW_ANGLE_H,
                              sample.raw_angle, 2u) ||
      !device->read_registers(device->context, AS5600_REG_AGC,
                              &sample.agc, 1u) ||
      !device->read_registers(device->context, AS5600_REG_MAGNITUDE_H,
                              sample.magnitude, 2u))
  {
    return false;
  }
  *diagnostics = sample;
  return true;
}
