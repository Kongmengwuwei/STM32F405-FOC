#include "app.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "bsp_usb.h"
#include "dual.h"
#include "leg_guard.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static volatile bool pending;
static volatile unsigned group;
static unsigned action, axis;
static float value0, value1;
static volatile int calibrating = -1;
static volatile uint32_t rejected;
static uint32_t session;
static float raw_angle[2];
static uint32_t stall_ticks[2], last_pair_ms;
static float stall_anchor[2], joint_goal[2];
static bool acquisition_halted;
static bool have_parameters[2];
static bool idle(foc_t *f) {
  return f->state == FOC_IDLE || f->state == FOC_FAULT;
}
static bool can_position(foc_t *f) {
  float q = leg_joint(f == &foc_1 ? 1u : 0u);
  return leg_homed() && q >= -1 && q <= 111 &&
         (f->state != FOC_IDLE ||
          fabsf(f == &foc_1 ? control_speed_rpm_1() : control_speed_rpm_0()) <
              30) &&
         f->calibrated && f->zero_ready &&
         (f->state == FOC_IDLE || f->state == FOC_RUN ||
          f->state == FOC_PRECHARGE || f->state == FOC_PWM_ZERO);
}

static float position(unsigned i) {
  return i ? control_position_deg_1() : control_position_deg_0();
}
static float speed(unsigned i) {
  return i ? control_speed_rpm_1() : control_speed_rpm_0();
}
static void stop_all(void) {
  pending = false;
  dual_hw_off_all();
  foc_stop_0();
  foc_stop_1();
  calibrating = -1;
}
bool app_init(void) {
  if (!bsp_uart_init() || !bsp_can_init() || !dual_hw_init())
    return false;
  foc_calibration_t c;
  foc_init_0(dual_record_load(0, &c) ? &c : NULL);
  foc_init_1(dual_record_load(1, &c) ? &c : NULL);
  dual_hw_start();
  return true;
}
void app_abort(uint32_t fault) {
  pending = false;
  leg_teach_invalidate();
  dual_hw_off_all();
  if (fault == FOC_SENSOR || fault == FOC_ADC || fault == FOC_TIMING ||
      fault == FOC_WINDOW || fault == FOC_NUMERIC) {
    leg_unhome();
    acquisition_halted = true;
    dual_hw_halt();
  }
  if (dual_fault)
    return;
  dual_fault = fault;
  foc_trip_0(fault);
  foc_trip_1(fault);
  calibrating = -1;
}
void app_fault(uint32_t fault) { app_abort(fault); }
void app_sample(void) { dual_app_sample(); }
static void current_profile(unsigned i, foc_t *f) {
  if (have_parameters[i] || f->state != FOC_IDLE || !f->calibrated ||
      !f->zero_ready)
    return;
  /* Initial gains from the previously measured Longqiu unit, not a new
   * identification. */
  motor_parameters_t p = {
      .123902604f, .0000267886062f, .0000442840028f, 0, 0, 0, 200};
  float w = 6.2831853072f * p.bandwidth_hz;
  p.kp_d = w * p.ld;
  p.kp_q = w * p.lq;
  p.ki = w * p.resistance;
  have_parameters[i] =
      i ? foc_apply_parameters_1(&p) : foc_apply_parameters_0(&p);
}
static bool publish(unsigned i, foc_t *f, const dual_adc_t *a, float angle) {
  if (i)
    foc_step_1(angle, a->bus, a->b, a->c, 0);
  else
    foc_step_0(angle, a->bus, a->b, a->c, 0);
  if (f->state == FOC_FAULT) {
    app_abort(f->fault);
    return false;
  }
  unsigned mode = f->state == FOC_PRECHARGE ? 1u
                  : (f->state == FOC_RUN || f->state == FOC_CALIBRATE ||
                     f->state == FOC_PWM_ZERO)
                      ? 2u
                      : 0u;
  if (mode == 0)
    dual_hw_off(i);
  else if (mode == 2 && !(i ? foc_window_1(f->duty) : foc_window_0(f->duty))) {
    app_abort(FOC_WINDOW);
    return false;
  } else if (!dual_hw_write(i, f->duty, mode)) {
    app_abort(FOC_TIMING);
    return false;
  }
  return true;
}
static bool post_protect(unsigned i, foc_t *f) {
  unsigned mode = (f->state == FOC_RUN || f->state == FOC_CALIBRATE ||
                   f->state == FOC_PWM_ZERO)
                      ? 2u
                      : 0u;
  if ((mode == 2 || dual_hw_mode(1u - i) == 2) &&
      fabsf(f->rpm) > LEG_SPEED_TRIP_MOTOR_RPM) {
    app_abort(FOC_SPEED);
    return false;
  }
  if (mode == 2 && f->id * f->id + f->iq * f->iq >=
                       LEG_MEASURED_CURRENT_MAX * LEG_MEASURED_CURRENT_MAX) {
    app_abort(FOC_CURRENT);
    return false;
  }
  float q = leg_joint(i);
  if (f->state == FOC_RUN && fabsf(f->iq_ref) >= .9f * leg_current_limit() &&
      fabsf(joint_goal[i] - q) > 2) {
    if (!stall_ticks[i])
      stall_anchor[i] = q;
    if (fabsf(q - stall_anchor[i]) >= .2f) {
      stall_ticks[i] = 0;
      stall_anchor[i] = q;
    } else if (++stall_ticks[i] >= 2u * FOC_SAMPLE_HZ) {
      app_abort(FOC_CURRENT);
      return false;
    }
  } else
    stall_ticks[i] = 0;
  return true;
}
static void apply_command(void) {
  if (leg_teach_state()) {
    pending = false;
    dual_hw_off_all();
    return;
  }
  if (!pending)
    return;
  bool ok = false;
  float t0 = leg_motor_target(0, value0, position(0));
  float t1 = leg_motor_target(1, value1, position(1));
  if (action == 0 && calibrating < 0) {
    float t = leg_motor_target(axis, value0, position(axis));
    ok = leg_target_valid(axis, t, position(axis)) &&
         (axis ? control_position_1(t) : control_position_0(t));
    if (ok) {
      joint_goal[axis] = value0;
      dual_hw_arm(axis);
    }
  } else if (action == 1 && calibrating < 0 && can_position(&foc_0) &&
             can_position(&foc_1) && leg_target_valid(0, t0, position(0)) &&
             leg_target_valid(1, t1, position(1))) {
    ok = control_position_0(t0) && control_position_1(t1);
    if (ok) {
      joint_goal[0] = value0;
      joint_goal[1] = value1;
      dual_hw_arm(0);
      dual_hw_arm(1);
    } else
      stop_all();
  } else if (action == 2 && idle(&foc_0) && idle(&foc_1) && calibrating < 0 &&
             leg_homed() && leg_joint(axis) >= 5 && leg_joint(axis) <= 105) {
    ok = axis ? foc_leg_calibrate_1(value0) : foc_leg_calibrate_0(value0);
    if (ok) {
      calibrating = (int)axis;
      dual_hw_arm(axis);
    }
  }
  if (!ok)
    ++rejected;
  pending = false;
}
static float last_bus;
static void telemetry(float bus) {
  float f[25] = {leg_direction(0) * control_speed_target_0() / LEG_GEAR_RATIO,
                 leg_direction(0) * speed(0) / LEG_GEAR_RATIO,
                 joint_goal[0],
                 leg_joint(0),
                 foc_0.iq_ref,
                 foc_0.iq,
                 foc_0.id,
                 bus,
                 (float)control_mode_0(),
                 (float)foc_0.state,
                 (float)foc_0.fault,
                 (float)(foc_0.warnings | (leg_homed() ? LEG_HOME_FLAG : 0u)),
                 leg_direction(1) * control_speed_target_1() / LEG_GEAR_RATIO,
                 leg_direction(1) * speed(1) / LEG_GEAR_RATIO,
                 joint_goal[1],
                 leg_joint(1),
                 foc_1.iq_ref,
                 foc_1.iq,
                 foc_1.id,
                 bus,
                 (float)control_mode_1(),
                 (float)foc_1.state,
                 (float)foc_1.fault,
                 (float)(foc_1.warnings | (leg_homed() ? LEG_HOME_FLAG : 0u)),
                 INFINITY};
  if (group) {
    float d[25] = {leg_travel(0),
                   leg_travel(1),
                   leg_current_limit(),
                   bus,
                   isfinite(raw_angle[0]) ? raw_angle[0] : 0,
                   isfinite(raw_angle[1]) ? raw_angle[1] : 0,
                   (float)motor_sample_us_0,
                   (float)(dual_sequence & 0xffffffu),
                   (float)rejected,
                   (float)dual_hw_mode(0),
                   (float)dual_hw_mode(1),
                   (float)foc_0.calibrated,
                   (float)foc_1.calibrated,
                   foc_0.b_offset,
                   foc_0.c_offset,
                   foc_1.b_offset,
                   foc_1.c_offset,
                   (float)dual_encoder_cycles_max,
                   (float)dual_write_counter_min[0],
                   (float)dual_write_counter_min[1],
                   (float)dual_fault,
                   (float)calibrating,
                   (float)(leg_teach_state() ? leg_teach_state()
                                             : (leg_homed() ? 3u : 0u)),
                   1,
                   INFINITY};
    if (group == 2) {
      d[13] = leg_reference(0);
      d[14] = leg_reference(1);
      d[15] = LEG_GEAR_RATIO;
      d[16] = 1; /* Mechanical teach protocol version. */
      d[23] = 2;
    }
    memcpy(f, d, sizeof f);
  }
  if (bsp_usb_ready())
    (void)bsp_usb_write(f, sizeof f);
  bsp_uart_tick();
}
void dual_app_sample(void) {
  if (acquisition_halted)
    return;
  dual_adc_t a0, a1;
  float angles[2], bus;
  if (!dual_hw_read_adc(&a0, &a1)) {
    app_abort(FOC_ADC);
    return;
  }
  dual_hw_encoder_pair(angles);
  if (!dual_hw_read_bus(&bus)) {
    app_abort(FOC_ADC);
    return;
  }
  a0.bus = a1.bus = bus;
  last_bus = bus;
  raw_angle[0] = angles[1];
  raw_angle[1] = angles[0];
  bool driven = foc_0.state == FOC_RUN || foc_1.state == FOC_RUN ||
                foc_0.state == FOC_PRECHARGE || foc_1.state == FOC_PRECHARGE ||
                calibrating >= 0;
  leg_set_driven(driven);
  /* Both CRC-valid encoder samples precede either bridge calculation. */
  if (!isfinite(angles[0]) || !isfinite(angles[1])) {
    app_abort(FOC_SENSOR);
    return;
  }
  /* The time-critical pair goes first. Statistics and outer loops must not
   * consume M1's PWM write margin after M0's current-loop calculation. */
  if (!leg_teach_state() && !dual_fault &&
      (!publish(0, &foc_0, &a0, angles[1]) ||
       !publish(1, &foc_1, &a1, angles[0])))
    return;
  uint32_t f0 = leg_observe(0, angles[1], motor_sample_us_0);
  uint32_t f1 = leg_observe(1, angles[0], motor_sample_us_0);
  if (f0 || f1) {
    app_abort(f0 ? f0 : f1);
    return;
  }
  last_pair_ms = bsp_uart_millis();
  if (leg_teach_state() || dual_fault) {
    dual_hw_off_all();
    ++dual_sequence;
    if (dual_sequence % 10u == 0)
      telemetry(bus);
    return;
  }
  uint32_t motion_fault = leg_motion_fault(driven);
  if (motion_fault) {
    app_abort(motion_fault);
    return;
  }
  if (!post_protect(0, &foc_0) || !post_protect(1, &foc_1))
    return;
  foc_outer_step_0();
  foc_outer_step_1();
  current_profile(0, &foc_0);
  current_profile(1, &foc_1);
  apply_command();
  ++dual_sequence;
  if (dual_sequence % 10u == 0)
    telemetry(bus);
}
static bool number(const char *s, float *v) {
  if (!s || !*s)
    return false;
  const char *p = s;
  if (*p == '+' || *p == '-')
    ++p;
  if (*p < '0' || *p > '9')
    return false;
  while (*p >= '0' && *p <= '9')
    ++p;
  if (*p == '.') {
    unsigned n = 0;
    while (*++p >= '0' && *p <= '9')
      ++n;
    if (!n || n > 2)
      return false;
  }
  if (*p)
    return false;
  *v = strtof(s, NULL);
  return isfinite(*v);
}
bool app_command(const char *s) {
  if (!strcmp(s, "m0 stop") || !strcmp(s, "m1 stop")) {
    unsigned i = s[1] == '1';
    uint32_t k = __get_PRIMASK();
    __disable_irq();
    pending = false;
    dual_hw_off(i);
    if (i)
      foc_stop_1();
    else
      foc_stop_0();
    if (calibrating == (int)i)
      calibrating = -1;
    __set_PRIMASK(k);
    return true;
  }

  if (!strcmp(s, "stop") || !strcmp(s, "all stop")) {
    uint32_t k = __get_PRIMASK();
    __disable_irq();
    stop_all();
    __set_PRIMASK(k);
    return true;
  }
  if (!strcmp(s, "send 0") || !strcmp(s, "send 6")) {
    group = leg_teach_state() ? 1 : 0;
    return true;
  }
  if (!strcmp(s, "send 1")) {
    group = 1;
    return true;
  }
  if (!strcmp(s, "send 2")) {
    group = 2;
    return true;
  }
  if (!strcmp(s, "leg home")) {
    if (dual_fault || acquisition_halted || pending || calibrating >= 0 ||
        (uint32_t)(bsp_uart_millis() - last_pair_ms) > 50u ||
        (!leg_teach_state() && (!idle(&foc_0) || !idle(&foc_1))) ||
        fabsf(speed(0)) > 5 || fabsf(speed(1)) > 5)
      return false;
    uint32_t k = __get_PRIMASK();
    __disable_irq();
    stop_all();
    bool ok = leg_home();
    if (ok) {
      joint_goal[0] = joint_goal[1] = 0;
      foc_acquisition_resume_0();
      foc_acquisition_resume_1();
      group = 0;
    }
    __set_PRIMASK(k);
    return ok;
  }
  if (!strcmp(s, "leg teach")) {
    /* Deliberate gate-off transition. A previous fault's cached angles are
     * discarded; the first fresh paired sample establishes the A anchors. */
    if (leg_teach_state() || !idle(&foc_0) || !idle(&foc_1))
      return false;
    stop_all();
    dual_hw_halt();
    foc_calibration_t c0 = foc_0.calibration, c1 = foc_1.calibration;
    bool h0 = foc_0.calibrated, h1 = foc_1.calibrated;
    if (!dual_hw_init()) {
      app_abort(FOC_SENSOR);
      return false;
    }
    foc_init_0(h0 ? &c0 : NULL);
    foc_init_1(h1 ? &c1 : NULL);
    have_parameters[0] = have_parameters[1] = false;
    leg_teach_begin();
    acquisition_halted = false;
    dual_fault = 0;
    group = 1;
    dual_hw_start();
    return true;
  }
  if (leg_teach_state() && strcmp(s, "clear"))
    return false; /* Includes clear/zero/hold/cal and all drive commands. */
  if (!strncmp(s, "leg rate ", 9)) {
    float rate;
    if (dual_fault || acquisition_halted || calibrating >= 0 ||
        !number(s + 9, &rate))
      return false;
    /* This changes the trajectory ceiling, never the reference, integrator,
     * run state or command watchdog. Both axes see the same atomic float. */
    return leg_set_motion_rate(rate);
  }
  if (!strcmp(s, "hello")) {
    char text[180];
    int n = snprintf(
        text, sizeof text,
        "#FOC dual leg M0=SPI1/PA1 M1=SPI3/PA0 range=0..110 home-required "
        "joint-deg gear=7:1 "
        "limit=%.2f A fault=%lu rejected=%lu\r\n",
        (double)leg_current_limit(), (unsigned long)dual_fault,
        (unsigned long)rejected);
    if (n > 0 && (size_t)n < sizeof text)
      bsp_uart_write(text, (size_t)n);
    bsp_uart_tick();
    return true;
  }
  if (!strcmp(s, "clear")) {
    if (pending || !idle(&foc_0) || !idle(&foc_1) || calibrating >= 0)
      return false;
    if (!dual_fault) {
      foc_clear_warnings_0();
      foc_clear_warnings_1();
      return true;
    }
    foc_calibration_t c0 = foc_0.calibration, c1 = foc_1.calibration;
    bool h0 = foc_0.calibrated, h1 = foc_1.calibrated;
    if (acquisition_halted) {
      if (!dual_hw_init())
        return false;
      leg_clear_tracking_fault();
    }
    uint32_t k = __get_PRIMASK();
    __disable_irq();
    dual_hw_off_all();
    foc_init_0(h0 ? &c0 : NULL);
    foc_init_1(h1 ? &c1 : NULL);
    have_parameters[0] = have_parameters[1] = false;
    dual_fault = 0;
    if (acquisition_halted) {
      acquisition_halted = false;
      dual_hw_start();
    }
    __set_PRIMASK(k);
    return true;
  }
  if (!strncmp(s, "limit ", 6)) {
    float v;
    return idle(&foc_0) && idle(&foc_1) && calibrating < 0 && !pending &&
           number(s + 6, &v) && leg_set_current_limit(v);
  }
  if (pending || dual_fault || calibrating >= 0)
    return false;
  unsigned i;
  if (!strncmp(s, "m0 ", 3)) {
    i = 0;
    s += 3;
  } else if (!strncmp(s, "m1 ", 3)) {
    i = 1;
    s += 3;
  } else if (!strcmp(s, "leg hold")) {
    if (!can_position(&foc_0) || !can_position(&foc_1))
      return false;
    value0 = leg_joint(0);
    value1 = leg_joint(1);
    action = 1;
    __DMB();
    pending = true;
    return true;
  } else if (!strncmp(s, "leg pos ", 8)) {
    char first[24];
    const char *split = strchr(s + 8, ' ');
    if (!split || split - (s + 8) >= 23)
      return false;
    memcpy(first, s + 8, (size_t)(split - (s + 8)));
    first[split - (s + 8)] = 0;
    if (!number(first, &value0) || !number(split + 1, &value1))
      return false;
    if (value0 < 0 || value0 > 110 || value1 < 0 || value1 > 110)
      return false;
    if (!can_position(&foc_0) || !can_position(&foc_1) ||
        !leg_target_valid(0, leg_motor_target(0, value0, position(0)),
                          position(0)) ||
        !leg_target_valid(1, leg_motor_target(1, value1, position(1)),
                          position(1)))
      return false;
    action = 1;
    __DMB();
    pending = true;
    return true;
  } else
    return false;
  if (!strcmp(s, "stop")) {
    uint32_t k = __get_PRIMASK();
    __disable_irq();
    pending = false;
    dual_hw_off(i);
    if (i)
      foc_stop_1();
    else
      foc_stop_0();
    __set_PRIMASK(k);
    return true;
  }
  if (!strcmp(s, "zero"))
    return false; /* Only paired known-stop home sets mechanical zero. */
  if (!strcmp(s, "hold")) {
    value0 = leg_joint(i);
    action = 0;
  } else if (!strncmp(s, "pos ", 4) && number(s + 4, &value0)) {
    if (value0 < 0 || value0 > 110)
      return false;
    action = 0;
  } else if (!strncmp(s, "cal ", 4) && number(s + 4, &value0)) {
    if (!idle(&foc_0) || !idle(&foc_1) || value0 < .2f ||
        value0 > LEG_CURRENT_MAX || !leg_homed() || leg_joint(i) < 5 ||
        leg_joint(i) > 105)
      return false;
    action = 2;
  } else
    return false;
  if (action == 0 &&
      (!can_position(i ? &foc_1 : &foc_0) ||
       !leg_target_valid(i, leg_motor_target(i, value0, position(i)),
                         position(i))))
    return false;
  axis = i;
  __DMB();
  pending = true;
  return true;
}
typedef struct {
  char s[64];
  unsigned n;
  bool bad;
} line_t;
static void receive(line_t *r, uint8_t c) {
  if (c == '\r' || c == '\n') {
    r->s[r->n] = 0;
    if (r->bad || (r->n && !app_command(r->s)))
      ++rejected;
    r->n = 0;
    r->bad = false;
  } else if (c >= 32 && c < 127 && r->n < sizeof r->s - 1)
    r->s[r->n++] = (char)c;
  else
    r->bad = true;
}
void app_poll(void) {
  static uint32_t fault_sent;
  if (dual_fault && acquisition_halted &&
      (uint32_t)(bsp_uart_millis() - fault_sent) >= 10u) {
    fault_sent = bsp_uart_millis();
    telemetry(last_bus);
  }
  static line_t usb, uart;
  static uint32_t rx_errors;
  bsp_usb_poll();
  uint32_t errors = g_uart_stats.rx_errors + g_uart_stats.rx_lost;
  if (errors != rx_errors) {
    rx_errors = errors;
    uart.bad = true;
    if (foc_0.state == FOC_RUN || foc_1.state == FOC_RUN)
      app_abort(FOC_UART);
  }
  if (session != g_usb_stats.sessions) {
    session = g_usb_stats.sessions;
    usb.n = 0;
    usb.bad = false;
  }
  uint8_t c;
  while (bsp_uart_read(&c, 1))
    receive(&uart, c);
  uint8_t p[64];
  size_t n = bsp_usb_read(p, sizeof p);
  for (size_t k = 0; k < n; ++k)
    receive(&usb, p[k]);
  if (calibrating >= 0) {
    unsigned i = (unsigned)calibrating;
    foc_t *f = i ? &foc_1 : &foc_0;
    if (f->state == FOC_SAVE) {
      leg_unhome(); /* Flash pause loses a provable multi-turn coordinate. */
      dual_hw_halt();
      bool ok = dual_record_save(i, &f->calibration);
      if (!ok) {
        app_abort(FOC_FLASH);
        return;
      }
      f->calibrated = true;
      f->state = FOC_IDLE;
      calibrating = -1;
      if (!dual_hw_init()) {
        app_abort(FOC_SENSOR);
        return;
      }
      foc_acquisition_resume_0();
      foc_acquisition_resume_1();
      dual_hw_start();
    } else if (f->state == FOC_FAULT || f->state == FOC_IDLE)
      calibrating = -1;
  }
}
