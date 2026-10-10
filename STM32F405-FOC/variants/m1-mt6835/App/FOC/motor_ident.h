#ifndef MOTOR_IDENT_H
#define MOTOR_IDENT_H
#include <stdbool.h>
#include <stdint.h>

/* SI units, parallel PI: u = Kp*error + integral(Ki*error dt). */
typedef struct {
    float resistance, ld, lq, kp_d, kp_q, ki, bandwidth_hz;
} motor_parameters_t;
enum { IDENT_IDLE, IDENT_R_LOW, IDENT_R_HIGH, IDENT_LD_LOW, IDENT_LD_HIGH,
       IDENT_LQ_LOW, IDENT_LQ_HIGH, IDENT_READY, IDENT_FAILED, IDENT_CANCELLED };
enum { IDENT_OK, IDENT_BUS, IDENT_CURRENT, IDENT_MOVING, IDENT_EXCITATION,
       IDENT_FIT, IDENT_INPUT, IDENT_TIMING };
typedef struct {
    float sample_hz, test_current, voltage_limit, bus_min, bus_max, bandwidth_hz;
} motor_ident_config_t;
typedef struct {
    volatile uint32_t phase, error;
    volatile bool valid;
    motor_parameters_t candidate;
} motor_ident_status_t;
extern motor_ident_status_t motor_ident;
bool motor_parameters_valid(const motor_parameters_t *p, float sample_hz);
bool motor_ident_start(const motor_ident_config_t *config, float angle, float bus);
bool motor_ident_active(void);
void motor_ident_cancel(void);
void motor_ident_abort(uint32_t error);
/* Applied ud/uq are reconstructed from ACTIVE CCRs and measured Vbus, not
 * the new PWM command. Identifies effective phase/dq R,L including the leads.
 * Returns false on completion/failure; caller must close the power output. */
bool motor_ident_step(float id, float iq, float applied_ud, float applied_uq,
                      float angle, float rpm, float bus, float *ud, float *uq);
#endif
