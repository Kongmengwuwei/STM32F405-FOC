#ifndef MENC15A_DEVICE_H
#define MENC15A_DEVICE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  menc15a_1_module = 0,
  menc15a_2_module = 1
} menc15a_module_enum;

extern volatile uint16_t menc15a_absolute_data[2];
extern volatile int16_t menc15a_absolute_offset_data[2];
extern volatile bool menc15a_last_read_ok;
extern volatile bool menc15a_last_transport_ok;
extern volatile uint16_t menc15a_last_command_reply;
extern volatile uint16_t menc15a_last_angle_reply;
extern volatile uint16_t menc15a_last_safety_reply;

/* Matches the reference driver's return convention: 0 is success. */
uint8_t menc15a_init(void);

/* The reference API is retained for M0. The first validated read has zero
 * offset. Always check menc15a_last_read_ok before using the return value.
 * A failed CRC or sensor status also clears the previous angle and offset.
 * M1 remains unsupported by the current board layer. */
uint16_t menc15a_get_absolute_data(menc15a_module_enum module);

/* Diagnostic read of TLE5012B STAT (address 0x00). Reading this register
 * acknowledges latched status flags; the caller must record both words. */
bool menc15a_read_status(uint16_t *status_data, uint16_t *safety_word);

#ifdef __cplusplus
}
#endif

#endif /* MENC15A_DEVICE_H */
