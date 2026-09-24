#ifndef FOC_DRIVE_STATE_H
#define FOC_DRIVE_STATE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  FOC_DRIVE_DISARMED,
  FOC_DRIVE_ARMED,
  FOC_DRIVE_RUNNING,
  FOC_DRIVE_FAULT
} foc_drive_mode_t;

enum
{
  FOC_FAULT_SENSOR = 1u << 0,
  FOC_FAULT_CURRENT = 1u << 1,
  FOC_FAULT_BUS = 1u << 2,
  FOC_FAULT_TIMING = 1u << 3,
  FOC_FAULT_DRIVER = 1u << 4,
  FOC_FAULT_CONFIGURATION = 1u << 5,
  FOC_FAULT_STALL = 1u << 6
};

typedef struct
{
  bool bus_valid;
  bool currents_calibrated;
  bool angle_valid;
  bool pwm_safe;
  bool user_enable;
} foc_drive_interlocks_t;

typedef struct
{
  foc_drive_mode_t mode;
  uint32_t faults;
} foc_drive_state_t;

void foc_drive_state_init(foc_drive_state_t *drive);
bool foc_drive_arm(foc_drive_state_t *drive,
                   const foc_drive_interlocks_t *interlocks);
bool foc_drive_start(foc_drive_state_t *drive);
void foc_drive_stop(foc_drive_state_t *drive);
void foc_drive_raise_fault(foc_drive_state_t *drive, uint32_t fault);

/* Must only be called after explicit user acknowledgement and outputs-off check. */
bool foc_drive_clear_fault(foc_drive_state_t *drive,
                            bool user_acknowledged, bool outputs_off);
bool foc_drive_pwm_allowed(const foc_drive_state_t *drive);

#ifdef __cplusplus
}
#endif

#endif /* FOC_DRIVE_STATE_H */
