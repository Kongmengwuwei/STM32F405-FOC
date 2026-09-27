#ifndef APP_CONTROL_CONTROL_H
#define APP_CONTROL_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/* Shared 1 kHz outer loops above the selected port's current loop. The ISR
   owns observation/controller state; commands publish targets under IRQ lock. */
enum { CONTROL_TORQUE, CONTROL_SPEED, CONTROL_POSITION };

bool control_torque(float amps);      /* Target Iq, amps. */
bool control_speed(float rpm);        /* Target speed, RPM, signed. */
bool control_position(float deg);     /* Target position, mechanical degrees, multi-turn. */
bool control_hold_position(void);     /* Re-target the last position, still scheduled. */
bool control_zero(void);              /* Redefine the current position as 0 deg. */
void control_stop(void);              /* stop/trip: torque mode, cleared integrators. */

/* Called every valid current-loop cycle, including idle, with the unwrapped
   mechanical position. Observes always; produces torque only in RUN at 1 kHz. */
void control_step(uint32_t sample_us, float mechanical_deg);

/* Latched reason to trip the FOC state machine; read every current cycle. */
uint32_t control_fault(void);
bool control_scheduled(void);         /* Target was received recently enough to keep running. */
uint32_t control_mode(void);          /* enum above; 0 means the current loop holds iq_ref. */
float control_iq_ref(void);           /* Speed/position output, amps. */
float control_speed_rpm(void);        /* Measured, from position difference over 1 ms. */
float control_speed_target(void);
float control_position_deg(void);     /* Multi-turn, relative to the last control_zero(). */
float control_position_target(void);

#endif
