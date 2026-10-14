#ifndef LEG_DUAL_H
#define LEG_DUAL_H
#include "control.h"
#include "foc.h"
extern foc_t foc_0, foc_1;
extern volatile uint32_t motor_sample_us_0, motor_sample_us_1;
extern volatile float motor_duty_0[3], motor_duty_1[3];
#define DECLARE_AXIS(n)                                                        \
  void foc_init_##n(const foc_calibration_t *);                                \
  void foc_stop_##n(void);                                                     \
  void foc_trip_##n(uint32_t);                                                 \
  void foc_clear_warnings_##n(void);                                           \
  void foc_step_##n(float, float, float, float, float);                        \
  void foc_outer_step_##n(void);                                               \
  bool foc_window_##n(const float *);                                          \
  bool foc_leg_calibrate_##n(float);                                           \
  bool foc_apply_parameters_##n(const motor_parameters_t *);                   \
  void foc_acquisition_resume_##n(void);                                       \
  bool control_position_##n(float);                                            \
  bool control_hold_position_##n(void);                                        \
  bool control_zero_##n(void);                                                 \
  uint32_t control_mode_##n(void);                                             \
  float control_speed_rpm_##n(void);                                           \
  float control_speed_target_##n(void);                                        \
  float control_position_deg_##n(void);                                        \
  float control_position_target_##n(void)
DECLARE_AXIS(0);
DECLARE_AXIS(1);
#undef DECLARE_AXIS
typedef struct {
  float b, c, bus;
} dual_adc_t;
extern volatile uint32_t dual_sequence, dual_fault, dual_encoder_cycles_max;
void dual_hw_safe_pins(void);
bool dual_hw_init(void);
bool dual_hw_read_adc(dual_adc_t *, dual_adc_t *);
bool dual_hw_read_bus(float *);
void dual_hw_encoder_pair(float[2]);
void dual_hw_start(void);
void dual_hw_halt(void);
void dual_hw_off(unsigned);
void dual_hw_off_all(void);
void dual_hw_arm(unsigned);
bool dual_hw_write(unsigned, const float[3], unsigned);
bool dual_hw_update(unsigned);
bool dual_record_load(unsigned, foc_calibration_t *);
bool dual_record_save(unsigned, const foc_calibration_t *);
unsigned dual_hw_mode(unsigned);
extern volatile uint32_t dual_write_counter_min[2];
extern volatile uint32_t dual_irq_cycles_max;
void dual_app_sample(void);
#endif
