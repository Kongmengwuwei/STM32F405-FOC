#ifndef AS5600_DEVICE_H
#define AS5600_DEVICE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef bool (*as5600_read_registers_fn)(void *context, uint8_t reg,
                                          uint8_t *data, size_t length);

typedef struct
{
  as5600_read_registers_fn read_registers;
  void *context;
} as5600_device_t;

typedef enum
{
  AS5600_READ_OK,
  AS5600_READ_BUS_ERROR,
  AS5600_READ_MAGNET_ERROR,
  AS5600_READ_DATA_ERROR
} as5600_read_status_t;

typedef struct
{
  uint16_t angle_counts;
  uint8_t magnet_status;
} as5600_measurement_t;

typedef struct
{
  uint8_t raw_angle[2];
  uint8_t agc;
  uint8_t magnitude[2];
} as5600_diagnostics_t;

/* Reads STATUS first, then RAW_ANGLE. No motor output is controlled here. */
as5600_read_status_t as5600_device_read(const as5600_device_t *device,
                                         as5600_measurement_t *measurement);

/* Raw register values for bring-up; never use this angle for motor control. */
bool as5600_device_read_diagnostics(const as5600_device_t *device,
                                    as5600_diagnostics_t *diagnostics);

#ifdef __cplusplus
}
#endif

#endif /* AS5600_DEVICE_H */
