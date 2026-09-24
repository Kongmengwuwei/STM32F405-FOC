#include "foc_drive_state.h"

void foc_drive_state_init(foc_drive_state_t *drive)
{
  if (drive != 0)
  {
    drive->mode = FOC_DRIVE_DISARMED;
    drive->faults = 0u;
  }
}

bool foc_drive_arm(foc_drive_state_t *drive,
                   const foc_drive_interlocks_t *interlocks)
{
  if (drive == 0 || interlocks == 0 || drive->mode != FOC_DRIVE_DISARMED ||
      drive->faults != 0u || !interlocks->bus_valid ||
      !interlocks->currents_calibrated || !interlocks->angle_valid ||
      !interlocks->pwm_safe || !interlocks->user_enable)
  {
    return false;
  }
  drive->mode = FOC_DRIVE_ARMED;
  return true;
}

bool foc_drive_start(foc_drive_state_t *drive)
{
  if (drive == 0 || drive->mode != FOC_DRIVE_ARMED || drive->faults != 0u)
  {
    return false;
  }
  drive->mode = FOC_DRIVE_RUNNING;
  return true;
}

void foc_drive_stop(foc_drive_state_t *drive)
{
  if (drive != 0 && drive->mode != FOC_DRIVE_FAULT)
  {
    drive->mode = FOC_DRIVE_DISARMED;
  }
}

void foc_drive_raise_fault(foc_drive_state_t *drive, uint32_t fault)
{
  if (drive != 0 && fault != 0u)
  {
    drive->faults |= fault;
    drive->mode = FOC_DRIVE_FAULT;
  }
}

bool foc_drive_clear_fault(foc_drive_state_t *drive,
                            bool user_acknowledged, bool outputs_off)
{
  if (drive == 0 || drive->mode != FOC_DRIVE_FAULT ||
      !user_acknowledged || !outputs_off)
  {
    return false;
  }
  drive->faults = 0u;
  drive->mode = FOC_DRIVE_DISARMED;
  return true;
}

bool foc_drive_pwm_allowed(const foc_drive_state_t *drive)
{
  return drive != 0 && drive->mode == FOC_DRIVE_RUNNING &&
         drive->faults == 0u;
}
