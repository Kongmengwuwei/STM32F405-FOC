#include "foc_drive_state.h"

#include <assert.h>

int main(void)
{
  foc_drive_state_t drive;
  foc_drive_state_init(&drive);
  assert(!foc_drive_pwm_allowed(&drive));
  assert(!foc_drive_start(&drive));

  foc_drive_interlocks_t ready = {
    .bus_valid = true,
    .currents_calibrated = true,
    .angle_valid = true,
    .pwm_safe = true,
    .user_enable = false,
  };
  assert(!foc_drive_arm(&drive, &ready));
  ready.user_enable = true;
  assert(foc_drive_arm(&drive, &ready));
  assert(!foc_drive_pwm_allowed(&drive));
  assert(foc_drive_start(&drive));
  assert(foc_drive_pwm_allowed(&drive));

  foc_drive_raise_fault(&drive, FOC_FAULT_SENSOR);
  foc_drive_raise_fault(&drive, FOC_FAULT_TIMING);
  assert(drive.faults == (FOC_FAULT_SENSOR | FOC_FAULT_TIMING));
  assert(!foc_drive_pwm_allowed(&drive));
  foc_drive_stop(&drive);
  assert(!foc_drive_arm(&drive, &ready));
  assert(!foc_drive_clear_fault(&drive, false, true));
  assert(!foc_drive_clear_fault(&drive, true, false));
  assert(foc_drive_clear_fault(&drive, true, true));
  assert(drive.mode == FOC_DRIVE_DISARMED);
  assert(!foc_drive_pwm_allowed(&drive));
  return 0;
}
