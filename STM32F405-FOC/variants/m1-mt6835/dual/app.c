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
static uint32_t stall_ticks[2];
static bool have_parameters[2];
static bool idle(foc_t *f) {
  return f->state == FOC_IDLE || f->state == FOC_FAULT;
}
static bool can_position(foc_t *f) {
  return f->calibrated && f->zero_ready &&
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
  dual_fault = fault;
  dual_hw_halt();
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
  uint32_t fault = leg_observe(i, angle, motor_sample_us_0);
  if (fault) {
    app_abort(fault);
    return false;
  }
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
  if ((mode == 2 || dual_hw_mode(1u - i) == 2) && fabsf(f->rpm) > 60) {
    app_abort(FOC_SPEED);
    return false;
  }
  if (mode == 2 && f->id * f->id + f->iq * f->iq >=
                       LEG_MEASURED_CURRENT_MAX * LEG_MEASURED_CURRENT_MAX) {
    app_abort(FOC_CURRENT);
    return false;
  }
  if (f->state == FOC_RUN && fabsf(f->iq_ref) > 3 && fabsf(speed(i)) < 1 &&
      fabsf((i ? control_position_target_1() : control_position_target_0()) -
            position(i)) > 5) {
    if (++stall_ticks[i] > FOC_SAMPLE_HZ / 2) {
      app_abort(FOC_CURRENT);
      return false;
    }
  } else
    stall_ticks[i] = 0;
  return true;
}
static void apply_command(void) {
  if (!pending)
    return;
  bool ok = false;
  if (action == 0 && calibrating < 0) {
    ok = leg_target_valid(axis, value0, position(axis)) &&
         (axis ? control_position_1(value0) : control_position_0(value0));
    if (ok)
      dual_hw_arm(axis);
  } else if (action == 1 && calibrating < 0 && can_position(&foc_0) &&
             can_position(&foc_1) && leg_target_valid(0, value0, position(0)) &&
             leg_target_valid(1, value1, position(1))) {
    ok = control_position_0(value0) && control_position_1(value1);
    if (ok) {
      dual_hw_arm(0);
      dual_hw_arm(1);
    } else
      stop_all();
  } else if (action == 2 && idle(&foc_0) && idle(&foc_1) && calibrating < 0) {
    ok = axis ? foc_leg_calibrate_1(value0) : foc_leg_calibrate_0(value0);
    if (ok) {
      calibrating = (int)axis;
      dual_hw_arm(axis);
    }
  } else if (action == 3 && (axis ? foc_1.state : foc_0.state) == FOC_IDLE) {
    ok = axis ? control_zero_1() : control_zero_0();
  }
  if (!ok)
    ++rejected;
  pending = false;
}
static float last_bus;
static void telemetry(float bus) {
  float f[25] = {control_speed_target_0() / LEG_GEAR_RATIO,
                 speed(0) / LEG_GEAR_RATIO,
                 control_position_target_0() / LEG_GEAR_RATIO,
                 position(0) / LEG_GEAR_RATIO,
                 foc_0.iq_ref,
                 foc_0.iq,
                 foc_0.id,
                 bus,
                 (float)control_mode_0(),
                 (float)foc_0.state,
                 (float)foc_0.fault,
                 (float)foc_0.warnings,
                 control_speed_target_1() / LEG_GEAR_RATIO,
                 speed(1) / LEG_GEAR_RATIO,
                 control_position_target_1() / LEG_GEAR_RATIO,
                 position(1) / LEG_GEAR_RATIO,
                 foc_1.iq_ref,
                 foc_1.iq,
                 foc_1.id,
                 bus,
                 (float)control_mode_1(),
                 (float)foc_1.state,
                 (float)foc_1.fault,
                 (float)foc_1.warnings,
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
                   0,
                   1,
                   INFINITY};
    memcpy(f, d, sizeof f);
  }
  if (bsp_usb_ready())
    (void)bsp_usb_write(f, sizeof f);
  bsp_uart_tick();
}
void dual_app_sample(void) {
  if (dual_fault)
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
  if (!publish(0, &foc_0, &a0, angles[1]) ||
      !publish(1, &foc_1, &a1, angles[0]))
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
    group = 0;
    return true;
  }
  if (!strcmp(s, "send 1")) {
    group = 1;
    return true;
  }
  if (!strcmp(s, "hello")) {
    char text[180];
    int n = snprintf(text, sizeof text,
                     "#FOC dual leg M0=SPI1/PA1 M1=SPI3/PA0 range=+-2.5/3.8 "
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
    dual_hw_halt();
    foc_calibration_t c0 = foc_0.calibration, c1 = foc_1.calibration;
    bool h0 = foc_0.calibrated, h1 = foc_1.calibrated;
    if (!dual_hw_init())
      return false;
    foc_init_0(h0 ? &c0 : NULL);
    foc_init_1(h1 ? &c1 : NULL);
    have_parameters[0] = have_parameters[1] = false;
    dual_fault = 0;
    dual_hw_start();
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
  } else if (!strncmp(s, "leg pos ", 8)) {
    char first[24];
    const char *split = strchr(s + 8, ' ');
    if (!split || split - (s + 8) >= 23)
      return false;
    memcpy(first, s + 8, (size_t)(split - (s + 8)));
    first[split - (s + 8)] = 0;
    if (!number(first, &value0) || !number(split + 1, &value1))
      return false;
    value0 *= LEG_GEAR_RATIO;
    value1 *= LEG_GEAR_RATIO;
    if (!can_position(&foc_0) || !can_position(&foc_1) ||
        !leg_target_valid(0, value0, position(0)) ||
        !leg_target_valid(1, value1, position(1)))
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
  if (!strcmp(s, "zero")) {
    axis = i;
    action = 3;
    __DMB();
    pending = true;
    return true;
  }
  if (!strcmp(s, "hold")) {
    value0 = position(i);
    action = 0;
  } else if (!strncmp(s, "pos ", 4) && number(s + 4, &value0)) {
    value0 *= LEG_GEAR_RATIO;
    action = 0;
  } else if (!strncmp(s, "cal ", 4) && number(s + 4, &value0)) {
    if (!idle(&foc_0) || !idle(&foc_1) || value0 < .2f ||
        value0 > LEG_CURRENT_MAX)
      return false;
    action = 2;
  } else
    return false;
  if (action == 0 && !leg_target_valid(i, value0, position(i)))
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
  if (dual_fault && (uint32_t)(bsp_uart_millis() - fault_sent) >= 10u) {
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
