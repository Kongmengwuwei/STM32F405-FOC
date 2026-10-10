#ifndef APP_SENSOR_CAL_H
#define APP_SENSOR_CAL_H
#include "foc.h"
#define SENSOR_CAL_HOLD_MS 3500u
#define SENSOR_CAL_RAMP_MS 2500u
#define SENSOR_CAL_TOTAL_MS 28000u
enum { SENSOR_CAL_IDLE, SENSOR_CAL_HOLD, SENSOR_CAL_FORWARD, SENSOR_CAL_REVERSE,
       SENSOR_CAL_SETTLE, SENSOR_CAL_SAMPLE, SENSOR_CAL_READY, SENSOR_CAL_FAILED, SENSOR_CAL_CANCELLED };
enum { SENSOR_CAL_OK, SENSOR_CAL_INPUT, SENSOR_CAL_NO_MOTION, SENSOR_CAL_INTEGER,
       SENSOR_CAL_DIRECTION, SENSOR_CAL_FOLLOWING, SENSOR_CAL_RETURN, SENSOR_CAL_ABORT };
typedef struct {
    volatile uint32_t phase, error;
    volatile bool valid;
    foc_calibration_t candidate;
    float forward_pairs, reverse_pairs, residual_deg, return_deg;
} sensor_cal_status_t;
extern sensor_cal_status_t sensor_cal;
bool sensor_cal_start(uint32_t sample_hz);
bool sensor_cal_active(void);
void sensor_cal_cancel(void);
void sensor_cal_abort(void);
/* Unwrapped mechanical degrees; outputs unwrapped stator field radians.
 * 3.5 s hold with 2.5 s current ramp, 3 electrical turns forward/back
 * at .25 turn/s, .5 s settle/sample. */
bool sensor_cal_step(float mechanical_deg, float *field);
#endif
