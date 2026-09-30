#ifndef DUAL_FOC_H
#define DUAL_FOC_H

#include "foc.h"
#include "control.h"

extern foc_t foc0, foc1;
extern volatile uint32_t motor0_sample_us, motor1_sample_us;

#define DECLARE_FOC(n) \
void foc##n##_init(const foc_calibration_t *); \
bool foc##n##_calibrate(void); \
bool foc##n##_test(void); \
void foc##n##_stop(void); \
void foc##n##_trip(uint32_t); \
void foc##n##_warn(uint32_t); \
void foc##n##_clear_warnings(void); \
void foc##n##_step(float, float, float, float, float); \
void foc##n##_outer_step(void); \
bool foc##n##_window(const float *); \
bool control##n##_torque(float); \
bool control##n##_speed(float); \
bool control##n##_position(float); \
bool control##n##_hold_position(void); \
bool control##n##_zero(void); \
uint32_t control##n##_mode(void); \
float control##n##_speed_rpm(void); \
float control##n##_speed_target(void); \
float control##n##_position_deg(void); \
float control##n##_position_target(void)
DECLARE_FOC(0);
DECLARE_FOC(1);
#undef DECLARE_FOC

typedef struct { float b, c, bus; } dual_adc_t;
extern volatile uint32_t dual_sequence, dual_fault;
void dual_hw_safe_pins(void);
bool dual_hw_init(void);
bool dual_hw_read_adc(dual_adc_t *m0, dual_adc_t *m1);
bool dual_hw_read_bus(float *voltage);
void dual_hw_encoder_pair(float angle[2]);
extern volatile uint32_t dual_encoder_cycles_max;
void dual_hw_start(void);
void dual_hw_halt(void);
void dual_hw_off(unsigned motor);
void dual_hw_off_all(void);
void dual_hw_arm(unsigned motor);
bool dual_hw_write(unsigned motor, const float duty[3], unsigned mode);
bool dual_hw_update(unsigned motor);
bool dual_record_load(unsigned motor, foc_calibration_t *out);
bool dual_record_save(unsigned motor, const foc_calibration_t *cal);
void dual_app_sample(void);

#endif
