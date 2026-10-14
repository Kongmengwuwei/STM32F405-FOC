#ifndef LEG_GUARD_H
#define LEG_GUARD_H
#include "leg_reference_config.h"
#include <stdbool.h>
#include <stdint.h>
#define LEG_GEAR_RATIO 7.0f
#ifndef LEG_TARGET_DEG
#define LEG_TARGET_DEG 2.5f
#endif
#ifndef LEG_TRAVEL_DEG
#define LEG_TRAVEL_DEG 3.8f
#endif
#define LEG_CURRENT_MAX 10.0f
#define LEG_MEASURED_CURRENT_MAX 11.0f
#define LEG_HOME_FLAG 0x400000u
#define LEG_SPEED_TRIP_MOTOR_RPM 180.0f
/* Operator-confirmed stop only: gear slack is not a powered travel allowance. */
#define LEG_HOME_RAW_TOLERANCE_DEG (2.0f * LEG_GEAR_RATIO)
/* Gates off only: up to 7000 motor rpm worth of samples; CRC still mandatory.
 */
#define LEG_PASSIVE_STEP_DEG 4.2f
void leg_set_driven(bool);
bool leg_homed(void);
bool leg_home(void);
void leg_unhome(void);
void leg_clear_tracking_fault(void);
int leg_direction(unsigned);
float leg_joint(unsigned);
float leg_joint_target(unsigned, float motor_target, float motor_position);
float leg_motor_target(unsigned, float joint, float motor_position);
uint32_t leg_motion_fault(bool driven);
float leg_limit_effort(unsigned, float iq, int electrical_direction);
/* Teach inhibits drive; explicit home at the known stop may leave teach. */
void leg_teach_begin(void);
void leg_teach_invalidate(void);
unsigned leg_teach_state(void);
float leg_reference(unsigned);
bool leg_ready(unsigned);
float leg_travel(unsigned);
uint32_t leg_observe(unsigned, float, uint32_t);
bool leg_target_valid(unsigned, float, float);
float leg_current_limit(void);
float leg_motion_rate(void);
bool leg_set_motion_rate(float);
bool leg_set_current_limit(float);
#endif
