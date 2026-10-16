#include "app.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "bsp_usb.h"
#include "dual.h"
#include "fivebar_config.h"
#include "fivebar_motion.h"
#include "fivebar_point.h"
#include "foot_record.h"
#include "leg_guard.h"
#include "realtime.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static volatile bool pending;
static volatile unsigned group;
static unsigned action, axis;
static float value0, value1, velocity0, velocity1;
static volatile int calibrating = -1;
static volatile uint32_t rejected;
static uint32_t session;
static float raw_angle[2];
static uint32_t stall_ticks[2], last_pair_ms;
static float stall_anchor[2], joint_goal[2];
static bool acquisition_halted;
static bool initialized;
static float last_bus, fault_frame[24];
static bool have_parameters[2];
/* Foreground owns geometry/math. IRQ consumes paired targets and this timer. */
static fb_geometry_t foot_geometry = FIVEBAR_GEOMETRY_INITIALIZER;
/* Build/copy with ADC IRQ enabled; publish only a word-sized bank index. */
static fb_point_motion_t foot_motion_bank[2];
static unsigned foot_motion_index;
#define foot_motion foot_motion_bank[foot_motion_index]
static bool foot_calibrated = FIVEBAR_ZERO_CALIBRATED != 0;
static volatile bool foot_owned;
static bool joint_planned;
static float joint_plan_goal[2];
static volatile uint32_t foot_host_ms, foot_epoch;
static uint32_t foot_start_ms, foot_poll_ms, foot_reference_ms;
static float foot_speed = FIVEBAR_FOOT_SPEED_MM_S;
static fb_status_t foot_status = FB_CONFIG;
/* Foreground fills the inactive bank; IRQ snapshots one published index.
 * Cortex-M4 aligned word stores are atomic. No long telemetry copy masks IRQs.
 */
static float foot_frame[2][24], geometry_frame[2][24], ui_frame[2][24];
static volatile unsigned frame_bank, geometry_bank;
static volatile bool foot_saved, foot_save_pending;
static volatile uint32_t ui_command_sequence;
static volatile unsigned ui_result;
static bool foot_cal_bad_pose;
static unsigned home_failure_result;
static unsigned foot_commit_failure;
volatile uint32_t foot_plan_ms_last, foot_plan_ms_max;
volatile uint32_t dual_sample_stage_cycles[7], dual_sample_stage_peak[7];
volatile uint32_t dual_critical_cycles_max, dual_critical_site;
#if defined(__arm__) || defined(__thumb__)
#define APP_LOCK()                                                             \
  __disable_irq();                                                             \
  uint32_t lock_started = DWT->CYCCNT;                                         \
  bool lock_running = foc_0.state == FOC_RUN || foc_1.state == FOC_RUN
#define APP_RELOCK()                                                           \
  __disable_irq();                                                             \
  lock_started = DWT->CYCCNT;                                                  \
  lock_running = foc_0.state == FOC_RUN || foc_1.state == FOC_RUN
#define APP_UNLOCK(key)                                                        \
  do {                                                                         \
    uint32_t cost = DWT->CYCCNT - lock_started;                                \
    if (lock_running && cost > dual_critical_cycles_max) {                     \
      dual_critical_cycles_max = cost;                                         \
      dual_critical_site = __LINE__;                                           \
    }                                                                          \
    __set_PRIMASK(key);                                                        \
  } while (0)
#else
#define APP_LOCK() __disable_irq()
#define APP_RELOCK() __disable_irq()
#define APP_UNLOCK(key) __set_PRIMASK(key)
#endif
#if defined(__arm__) || defined(__thumb__)
#define SAMPLE_BEGIN uint32_t sample_started = DWT->CYCCNT
#define SAMPLE_MARK(n)                                                         \
  (dual_sample_stage_cycles[n] = DWT->CYCCNT - sample_started)
#else
#define SAMPLE_BEGIN
#define SAMPLE_MARK(n) ((void)0)
#endif
/* Widget feedback codes: accepted, fault, home, geometry, offset, stop,
 * range/model, format, moving, electrical calibration, wrong home, saving. */
static fb_geometry_t foot_drive_geometry(void) {
  fb_geometry_t g = foot_geometry;
  g.q_min[0] = g.q_min[1] = 0;
  g.q_max[0] = g.q_max[1] = 110;
  return g;
}
static void foot_cancel(void) {
  joint_planned = false;
  foot_owned = false;
  ++foot_epoch;
}
static void geometry_telemetry(void) {
  const fb_geometry_t *g = &foot_geometry;
  float f[24] = {g->pivot[0][0],
                 g->pivot[0][1],
                 g->pivot[1][0],
                 g->pivot[1][1],
                 g->active[0],
                 g->passive[0],
                 g->active[1],
                 g->passive[1],
                 g->zero_ccw[0],
                 g->zero_ccw[1],
                 (float)g->q_sign[0],
                 (float)g->q_sign[1],
                 (float)g->elbow_sign[0],
                 (float)g->elbow_sign[1],
                 (float)g->assembly_sign,
                 g->q_min[0],
                 g->q_max[0],
                 g->q_min[1],
                 g->q_max[1],
                 g->min_sine,
                 foot_calibrated ? 1.0f : 0.0f,
                 fb_valid(g) ? 1.0f : 0.0f,
                 0,
                 4};
  unsigned next = geometry_bank ^ 1u;
  memcpy(geometry_frame[next], f, sizeof f);
  __DMB();
  geometry_bank = next;
}
static bool idle(foc_t *f) {
  return f->state == FOC_IDLE || f->state == FOC_FAULT;
}
static bool DUAL_REALTIME can_position(foc_t *f) {
  float q = leg_joint(f == &foc_1 ? 1u : 0u);
  return leg_homed() && q >= -1 && q <= 111 &&
         (f->state != FOC_IDLE ||
          fabsf(f == &foc_1 ? control_speed_rpm_1() : control_speed_rpm_0()) <
              30) &&
         f->calibrated && f->zero_ready &&
         (f->state == FOC_IDLE || f->state == FOC_RUN ||
          f->state == FOC_PRECHARGE || f->state == FOC_PWM_ZERO);
}

static float DUAL_REALTIME position(unsigned i) {
  return i ? control_position_deg_1() : control_position_deg_0();
}
static float DUAL_REALTIME speed(unsigned i) {
  return i ? control_speed_rpm_1() : control_speed_rpm_0();
}
static void stop_all(void) {
  foot_cancel();
  pending = false;
  dual_hw_off_all();
  foc_stop_0();
  foc_stop_1();
  calibrating = -1;
}
bool app_init(void) {
  initialized = false;
  if (!bsp_uart_init() || !bsp_can_init() || !dual_hw_init())
    return false;
  foc_calibration_t c;
  foc_init_0(dual_record_load(0, &c) ? &c : NULL);
  foc_init_1(dual_record_load(1, &c) ? &c : NULL);
  if (foot_record_load(&foot_geometry)) {
    foot_calibrated = true;
    foot_saved = true;
  }
  geometry_telemetry();
  acquisition_halted = false;
  initialized = true;
  dual_hw_start();
  return true;
}
void app_abort(uint32_t fault) {
  dual_hw_off_all();
  if (!dual_fault) {
    /* Fault-only cost: latch before controller trip clears references. */
    fault_frame[0] = (float)fault;
    fault_frame[1] = (float)bsp_uart_millis();
    fault_frame[2] = leg_homed() ? leg_joint(0) : NAN;
    fault_frame[3] = leg_homed() ? leg_joint(1) : NAN;
    fault_frame[4] = foc_0.rpm;
    fault_frame[5] = foc_1.rpm;
    fault_frame[6] = speed(0);
    fault_frame[7] = speed(1);
    fault_frame[8] = foc_0.iq;
    fault_frame[9] = foc_1.iq;
    fault_frame[10] = foc_0.iq_ref;
    fault_frame[11] = foc_1.iq_ref;
    fault_frame[12] = joint_goal[0];
    fault_frame[13] = joint_goal[1];
    fault_frame[14] = foot_owned ? foot_motion.goal[0] : NAN;
    fault_frame[15] = foot_owned ? foot_motion.goal[1] : NAN;
    fault_frame[16] = control_position_target_0();
    fault_frame[17] = control_position_target_1();
    fault_frame[18] = leg_speed_rms(0);
    fault_frame[19] = leg_speed_rms(1);
    fault_frame[20] = last_bus;
    fault_frame[21] =
        (float)((leg_homed() ? 1u : 0u) | (foot_owned ? 2u : 0u) |
                (joint_planned ? 4u : 0u) | (initialized ? 8u : 0u));
    fault_frame[22] = (float)(dual_sequence & 0xffffffu);
    fault_frame[23] = 5007;
  }
  foot_cancel();
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
static bool DUAL_REALTIME publish(unsigned i, foc_t *f, const dual_adc_t *a,
                                  float angle) {
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
static bool DUAL_REALTIME post_protect(unsigned i, foc_t *f) {
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
static void DUAL_REALTIME apply_command(void) {
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
  } else if ((action == 1 || action == 3) && calibrating < 0 &&
             can_position(&foc_0) && can_position(&foc_1) &&
             leg_target_valid(0, t0, position(0)) &&
             leg_target_valid(1, t1, position(1))) {
    ok = action == 3 ? (control_position_tracking_0(t0, velocity0) &&
                        control_position_tracking_1(t1, velocity1))
                     : (control_position_0(t0) && control_position_1(t1));
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
  if (!ok) {
    ++rejected;
    if (foot_owned)
      app_abort(FOC_POSITION);
  }
  pending = false;
}
static uint32_t telemetry_sequence, telemetry_fault;
static bool telemetry_sent;
static void telemetry_write(const void *frame, size_t size) {
  __DMB();
  if (dual_sequence != telemetry_sequence || dual_fault != telemetry_fault)
    return;
  if (bsp_usb_ready())
    telemetry_sent = bsp_usb_write(frame, size);
}
static void telemetry_poll(void);
static void foot_reference_poll(void);
static bool foot_planning_service(void *context) {
  uint32_t epoch = *(const uint32_t *)context;
  if (epoch != foot_epoch || dual_fault || acquisition_halted)
    return false;
  /* No command parser recursion: keep the accepted reference and state stream
   * alive while a candidate is checked. Never extend the host watchdog. */
  foot_reference_poll();
  telemetry_poll();
  return epoch == foot_epoch && !dual_fault;
}
static void telemetry(float bus) {
  telemetry_sent = false;
  telemetry_sequence = dual_sequence;
  telemetry_fault = dual_fault;
  if (group == 7) {
    float frame[25];
    memcpy(frame, fault_frame, sizeof fault_frame);
    frame[23] = 5007;
    frame[24] = INFINITY;
    telemetry_write(frame, sizeof frame);
    bsp_uart_tick();
    return;
  }
  if (group == 5) {
    float frame[25];
    memcpy(frame, ui_frame[frame_bank], 24 * sizeof(float));
    frame[0] = leg_joint(0);
    frame[1] = leg_joint(1);
    frame[2] = joint_planned ? joint_plan_goal[0] : joint_goal[0];
    frame[3] = joint_planned ? joint_plan_goal[1] : joint_goal[1];
    frame[8] = foc_0.iq;
    frame[9] = foc_1.iq;
    frame[10] = bus;
    frame[11] = (float)foc_0.state;
    frame[12] = (float)foc_1.state;
    frame[13] = (float)dual_fault;
    frame[15] = (float)ui_result;
    frame[16] = (float)(ui_command_sequence & 0xffffffu);
    frame[17] = leg_motion_rate();
    frame[18] = leg_current_limit();
    frame[19] = (float)foot_status;
    frame[20] = (float)(dual_sequence & 0xffffffu);

    frame[23] = 5005;
    frame[24] = INFINITY;
    telemetry_write(frame, sizeof frame);
    bsp_uart_tick();
    return;
  }
  if (group == 3 || group == 4) {
    float frame[25];
    memcpy(frame,
           group == 3 ? foot_frame[frame_bank] : geometry_frame[geometry_bank],
           24 * sizeof(float));
    frame[22] = leg_homed() ? 1.0f : 0.0f;
    frame[23] = (float)group;
    frame[24] = INFINITY;
    if (group == 3) {
      frame[11] = (float)((unsigned)frame[11] & ~4u) +
                  (foot_owned && !joint_planned ? 4.0f : 0.0f);
      frame[12] = leg_current_limit();
      frame[13] = leg_motion_rate();
      frame[14] = bus;
      frame[15] = foc_0.iq;
      frame[16] = foc_1.iq;
      frame[17] = (float)foc_0.state;
      frame[18] = (float)foc_1.state;
      frame[19] = (float)foc_0.fault;
      frame[20] = (float)foc_1.fault;
    }
    telemetry_write(frame, sizeof frame);
    bsp_uart_tick();
    return;
  }
  float f[25] = {leg_direction(0) * control_speed_target_0() / LEG_GEAR_RATIO,
                 leg_direction(0) * speed(0) / LEG_GEAR_RATIO,
                 joint_planned ? joint_plan_goal[0] : joint_goal[0],
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
                 joint_planned ? joint_plan_goal[1] : joint_goal[1],
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
  telemetry_write(f, sizeof f);
  bsp_uart_tick();
}
void DUAL_REALTIME dual_app_sample(void) {
  if (acquisition_halted)
    return;
  SAMPLE_BEGIN;
  dual_adc_t a0, a1;
  float angles[2], bus;
  if (!dual_hw_read_adc(&a0, &a1)) {
    app_abort(FOC_ADC);
    return;
  }
  SAMPLE_MARK(0);
  dual_hw_encoder_pair(angles);
  SAMPLE_MARK(1);
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
  if (!leg_teach_state() && !dual_fault) {
    if (!publish(0, &foc_0, &a0, angles[1]))
      return;
    SAMPLE_MARK(2);
    if (!publish(1, &foc_1, &a1, angles[0]))
      return;
  }
  SAMPLE_MARK(3);
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

    return;
  }
  if (foot_owned && (uint32_t)(last_pair_ms - foot_host_ms) >= 500u) {
    app_abort(FOC_UART);
    return;
  }
  uint32_t motion_fault = leg_motion_fault(driven);
  if (motion_fault) {
    app_abort(motion_fault);
    return;
  }
  if (!post_protect(0, &foc_0) || !post_protect(1, &foc_1))
    return;
  SAMPLE_MARK(4);
  foc_outer_step_0();
  foc_outer_step_1();
  SAMPLE_MARK(5);
  current_profile(0, &foc_0);
  current_profile(1, &foc_1);
  apply_command();
  SAMPLE_MARK(6);
  ++dual_sequence;
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
/* Publish a coherent command; never run trig while interrupts are off. */
static bool queue_command(unsigned a, unsigned i, float x, float y,
                          bool direct) {
  uint32_t key = __get_PRIMASK();
  APP_LOCK();
  bool ok = !pending && !dual_fault && calibrating < 0;
  if (ok) {
    if (direct)
      foot_cancel();
    action = a;
    axis = i;
    value0 = x;
    value1 = y;
    __DMB();
    pending = true;
  }
  APP_UNLOCK(key);
  return ok;
}
static bool queue_foot_reference(const fb_reference_t *q, uint32_t epoch) {
  float rpm[2];
  for (unsigned i = 0; i < 2; i++) {
    float q_velocity = q->velocity[i];
    rpm[i] = leg_direction(i) * LEG_GEAR_RATIO * q_velocity / 6;
    if (!isfinite(rpm[i]) || fabsf(rpm[i]) > leg_motion_rate())
      return false;
  }
  uint32_t key = __get_PRIMASK();
  APP_LOCK();
  bool ok = epoch == foot_epoch && foot_owned && !pending && !dual_fault &&
            calibrating < 0;
  if (ok) {
    action = 3;
    axis = 0;
    value0 = q->xy[0];
    value1 = q->xy[1];
    velocity0 = rpm[0];
    velocity1 = rpm[1];
    __DMB();
    pending = true;
  }
  APP_UNLOCK(key);
  return ok;
}
static bool pair_numbers(const char *s, float *a, float *b) {
  char first[24];
  const char *split = strchr(s, ' ');
  if (!split || split == s || split - s >= 23)
    return false;
  memcpy(first, s, (size_t)(split - s));
  first[split - s] = 0;
  return number(first, a) && number(split + 1, b);
}
static void foot_snapshot(float q[2], uint32_t *epoch, bool *ready) {
  uint32_t key = __get_PRIMASK();
  APP_LOCK();
  q[0] = leg_joint(0);
  q[1] = leg_joint(1);
  *epoch = foot_epoch;
  APP_UNLOCK(key);
  /* Three motor-encoder counts / 7 (~0.0047 joint deg): numerical model
   * inputs only. Real observations, drive limits and hard trips are unchanged.
   */
  for (unsigned i = 0; i < 2; i++) {
    if (q[i] < 0 && q[i] >= -.005f)
      q[i] = 0;
    if (q[i] > 110 && q[i] <= 110.005f)
      q[i] = 110;
  }
  *ready = !dual_fault && !acquisition_halted && calibrating < 0 &&
           !leg_teach_state() &&
           (uint32_t)(bsp_uart_millis() - last_pair_ms) <= 50u &&
           can_position(&foc_0) && can_position(&foc_1);
}
static bool plan_position(float goal[2], bool hold,
                          const float *joint_endpoint);
static bool joint_plan_rest(void) {
  return foot_owned && joint_planned &&
         (float)(uint32_t)(bsp_uart_millis() - foot_start_ms) * .001f >=
             foot_motion.duration &&
         fabsf(speed(0)) < 5 && fabsf(speed(1)) < 5;
}
static bool foot_command(const char *s) {
  if (!strcmp(s, "foot calibrate")) {
    if (foot_save_pending)
      return false;
    /* One GUI action/ack: never save a stale calibration if the new pose fails.
     */
    return foot_command("foot cal -90 -90") && foot_command("foot save");
  }
  if (!strcmp(s, "foot save")) {
    if (!foot_calibrated || foot_save_pending || pending || dual_fault ||
        acquisition_halted || calibrating >= 0 || foc_0.state != FOC_IDLE ||
        foc_1.state != FOC_IDLE || dual_hw_mode(0) || dual_hw_mode(1))
      return false;
    foot_save_pending = true;
    return true;
  }
  if (!strcmp(s, "foot stop")) {
    uint32_t key = __get_PRIMASK();
    APP_LOCK();
    stop_all();
    APP_UNLOCK(key);
    return true;
  }
  if (!strncmp(s, "foot cal ", 9)) {
    float theta[2], q[2];
    uint32_t epoch;
    bool ready;
    if (!pair_numbers(s + 9, &theta[0], &theta[1]) || fabsf(theta[0]) > 360 ||
        fabsf(theta[1]) > 360)
      return false;
    foot_snapshot(q, &epoch, &ready);
    (void)ready;
    uint32_t key = __get_PRIMASK();
    APP_LOCK();
    bool ok = !dual_fault && !acquisition_halted && leg_homed() && !pending &&
              calibrating < 0 && foc_0.state == FOC_IDLE &&
              foc_1.state == FOC_IDLE && !dual_hw_mode(0) && !dual_hw_mode(1) &&
              (uint32_t)(bsp_uart_millis() - last_pair_ms) <= 50u &&
              fabsf(speed(0)) < 5 && fabsf(speed(1)) < 5 && epoch == foot_epoch;
    if (ok)
      foot_cancel();
    APP_UNLOCK(key);
    if (!ok)
      return false;
    fb_geometry_t candidate = foot_geometry;
    for (unsigned i = 0; i < 2; i++)
      candidate.zero_ccw[i] = theta[i] - candidate.q_sign[i] * q[i];
    if (!fb_valid(&candidate))
      return false;
    fb_geometry_t expected = FIVEBAR_GEOMETRY_INITIALIZER;
    for (unsigned i = 0; i < 2; i++) {
      if (fabsf(remainderf(candidate.zero_ccw[i] - expected.zero_ccw[i], 360)) >
          30) {
        foot_cal_bad_pose = true;
        return false;
      }
    }
    /* The operator's known rod pose may lie beyond the software drive margin,
     * but it must still describe this fixed assembly and a nonsingular linkage.
     */
    fb_geometry_t check = candidate;
    for (unsigned i = 0; i < 2; i++) {
      check.q_min[i] = fminf(check.q_min[i], q[i] - .05f);
      check.q_max[i] = fmaxf(check.q_max[i], q[i] + .05f);
    }
    fb_pose_t pose;
    if (fb_forward(&check, q, &pose) != FB_OK)
      return false;
    key = __get_PRIMASK();
    APP_RELOCK();
    ok = foot_epoch == epoch + 1u && !dual_fault && leg_homed();
    if (ok) {
      foot_geometry = candidate;
      foot_calibrated = true;
      foot_saved = false;
      foot_status = FB_OK;
    }
    APP_UNLOCK(key);
    if (ok)
      geometry_telemetry();
    return ok;
  }
  if (!strncmp(s, "foot speed ", 11)) {
    float mm_per_second;
    bool leg_hold = foc_0.state == FOC_RUN && foc_1.state == FOC_RUN &&
                    control_mode_0() == CONTROL_POSITION &&
                    control_mode_1() == CONTROL_POSITION;
    if ((foot_owned && !joint_plan_rest()) || dual_fault ||
        acquisition_halted || calibrating >= 0 ||
        (!leg_hold && (!idle(&foc_0) || !idle(&foc_1))) ||
        !number(s + 11, &mm_per_second) || mm_per_second < 2 ||
        mm_per_second > 400)
      return false;
    /* Future planning input only; safe to configure while ordinary leg
     * position control holds the mechanism. Never alter an owned foot plan. */
    foot_speed = mm_per_second;
    return true;
  }
  bool hold = !strcmp(s, "foot hold");
  float goal[2];
  if (!hold &&
      (strncmp(s, "foot pos ", 9) || !pair_numbers(s + 9, &goal[0], &goal[1])))
    return false;
  return plan_position(goal, hold, NULL);
}
static bool plan_position(float goal[2], bool hold,
                          const float *joint_endpoint) {
  float q[2];
  uint32_t epoch;
  bool ready;
  foot_snapshot(q, &epoch, &ready);
  if (!ready || !foot_calibrated) {
    foot_status = FB_CONFIG;
    return false;
  }
  uint32_t now = bsp_uart_millis();
  if (foot_owned && !hold && fabsf(goal[0] - foot_motion.goal[0]) < .001f &&
      fabsf(goal[1] - foot_motion.goal[1]) < .001f) {
    uint32_t key = __get_PRIMASK();
    APP_LOCK();
    bool ok = foot_owned && epoch == foot_epoch && !dual_fault;
    if (ok) {
      foot_host_ms = now;
      joint_planned = joint_endpoint != NULL;
      if (joint_endpoint)
        memcpy(joint_plan_goal, joint_endpoint, sizeof joint_plan_goal);
    }
    APP_UNLOCK(key);
    return ok;
  }
  fb_reference_t initial = {0};
  fb_pose_t pose;
  fb_geometry_t drive_geometry = foot_drive_geometry();
  foot_status = fb_forward(&drive_geometry, q, &pose);
  if (foot_status != FB_OK)
    return false;
  if (hold) {
    goal[0] = pose.xy[0];
    goal[1] = pose.xy[1];
  }
  if (foot_owned) {
    foot_status = fb_point_evaluate(
        &drive_geometry, &foot_motion,
        (float)(uint32_t)(now - foot_start_ms) * .001f, &initial, &pose);
    if (foot_status != FB_OK)
      return false;
  } else {
    if (fabsf(speed(0)) > 5 || fabsf(speed(1)) > 5) {
      foot_status = FB_RATE;
      return false;
    }
    memcpy(initial.xy, pose.xy, sizeof initial.xy);
  }
  float cart_speed = joint_endpoint
                         ? fmaxf(foot_speed, leg_motion_rate() * 10 / 3)
                         : foot_speed;
  fb_limits_t limits = {cart_speed, FIVEBAR_FOOT_ACCEL_MM_S2,
                        .8f * leg_motion_rate() * 6 / LEG_GEAR_RATIO,
                        .7f * leg_motion_acceleration() * 6 / LEG_GEAR_RATIO};
  fb_point_motion_t candidate;
  /* Endpoints retain the 2..108 degree margin. Allow departure from a known
   * physical 0/110 boundary without a separate joint-mode positioning step. */
  foot_status = fb_inverse(joint_endpoint ? &drive_geometry : &foot_geometry,
                           goal, &pose);
  if (foot_status != FB_OK)
    return false;
  fb_plan_service_t service = {foot_planning_service, &epoch};
  foot_status = fb_point_plan_serviced(&drive_geometry, &initial, goal, &limits,
                                       &candidate, &service);
  if (foot_status != FB_OK)
    return false;
  foot_plan_ms_last = (uint32_t)(bsp_uart_millis() - now);
  if (foot_plan_ms_last > foot_plan_ms_max)
    foot_plan_ms_max = foot_plan_ms_last;
  /* Evaluate readiness without masking acquisition; faults/stops change
   * the epoch and are checked again in the short atomic publication. */
  bool positions_ready = can_position(&foc_0) && can_position(&foc_1);
  bool fresh = (uint32_t)(bsp_uart_millis() - now) < 100u;
  uint32_t host_now = bsp_uart_millis();
  unsigned next_motion = foot_motion_index ^ 1u;
  foot_motion_bank[next_motion] = candidate;
  __DMB();
  /* A fault/stop occurring during planning invalidates its epoch. */
  uint32_t key = __get_PRIMASK();
  APP_LOCK();
  /* A serviced old foot reference may still await this ADC tick. It is safe
   * to supersede only that slot; direct commands cancel ownership/epoch. */
  bool own_pending = pending && foot_owned && action == 3;
  bool ok = epoch == foot_epoch && (!pending || own_pending) && !dual_fault &&
            calibrating < 0 && positions_ready && fresh;
  if (!ok)
    foot_commit_failure = foot_plan_ms_last >= 100u ? 18u : 12u;
  if (ok) {
    if (own_pending)
      pending = false;
    foot_reference_ms = now - 5u;
    foot_motion_index = next_motion;
    foot_start_ms = now;
    foot_host_ms = host_now;
    foot_owned = true;
    joint_planned = joint_endpoint != NULL;
    if (joint_endpoint)
      memcpy(joint_plan_goal, joint_endpoint, sizeof joint_plan_goal);
  }
  APP_UNLOCK(key);
  return ok;
}
static void foot_reference_poll(void) {
  uint32_t now = bsp_uart_millis();
  if (!foot_owned || (uint32_t)(now - foot_reference_ms) < 5u)
    return;
  foot_reference_ms = now;
  float measured[2];
  uint32_t epoch;
  bool ready;
  foot_snapshot(measured, &epoch, &ready);
  fb_reference_t q;
  fb_status_t status =
      fb_point_joint_evaluate(&foot_motion, (now - foot_start_ms) * .001f, &q);
  bool bounded = status == FB_OK;
  for (unsigned i = 0; i < 2 && bounded; i++)
    bounded = q.xy[i] >= -.001f && q.xy[i] <= 110.001f;
  if (!ready || !bounded) {
    uint32_t key = __get_PRIMASK();
    APP_LOCK();
    if (epoch == foot_epoch && foot_owned)
      app_abort(FOC_POSITION);
    APP_UNLOCK(key);
    return;
  }
  (void)queue_foot_reference(&q, epoch);
}
static void foot_poll(void) {
  uint32_t now = bsp_uart_millis();
  if ((uint32_t)(now - foot_poll_ms) < 20u)
    return;
  foot_poll_ms = now;
  float q[2];
  uint32_t epoch;
  bool ready;
  foot_snapshot(q, &epoch, &ready);
  float f[24] = {0};
  fb_pose_t measured, target;
  fb_reference_t ref = {0};
  fb_geometry_t drive_geometry = foot_drive_geometry();
  bool valid =
      leg_homed() && fb_forward(&drive_geometry, q, &measured) == FB_OK;
  f[4] = f[5] = NAN;
  if (valid) {
    f[4] = measured.xy[0];
    f[5] = measured.xy[1];
    f[21] = fminf(measured.parallel_sine,
                  fminf(measured.serial_sine[0], measured.serial_sine[1]));
  }
  if (foot_owned) {
    fb_status_t status = fb_point_evaluate(
        &drive_geometry, &foot_motion,
        (float)(uint32_t)(now - foot_start_ms) * .001f, &ref, &target);
    if (!ready || status != FB_OK) {
      foot_status = status == FB_OK ? FB_CONFIG : status;
      uint32_t key = __get_PRIMASK();
      APP_LOCK();
      if (epoch == foot_epoch && foot_owned)
        app_abort(FOC_POSITION);
      APP_UNLOCK(key);
    } else {
      /* Display/model checks are 50 Hz; the joint reference runs at 200 Hz. */
      f[6] = target.q[0];
      f[7] = target.q[1];
    }
    f[0] = foot_motion.goal[0];
    f[1] = foot_motion.goal[1];
    f[2] = ref.xy[0];
    f[3] = ref.xy[1];
  } else if (valid) {
    f[0] = f[2] = measured.xy[0];
    f[1] = f[3] = measured.xy[1];
  }
  f[8] = q[0];
  f[9] = q[1];
  f[10] = (float)foot_status;
  f[11] = (float)((valid ? 1u : 0u) | (foot_calibrated ? 2u : 0u) |
                  (foot_owned && !joint_planned ? 4u : 0u));
  f[23] = 3;
  float u[24] = {0};
  u[4] = f[4];
  u[5] = f[5];
  u[6] = foot_owned ? foot_motion.goal[0] : f[4];
  u[7] = foot_owned ? foot_motion.goal[1] : f[5];
  u[21] = foot_geometry.zero_ccw[0];
  u[22] = foot_geometry.zero_ccw[1];
  unsigned flags = (leg_homed() ? 1u : 0u) |
                   (foc_0.calibrated && foc_1.calibrated ? 2u : 0u) |
                   (foc_0.zero_ready && foc_1.zero_ready ? 4u : 0u) |
                   (foot_calibrated ? 8u : 0u) | (foot_saved ? 16u : 0u) |
                   (foot_owned && !joint_planned ? 32u : 0u) |
                   (foot_save_pending ? 64u : 0u) | (valid ? 128u : 0u) |
                   (!acquisition_halted ? 256u : 0u) | (ready ? 512u : 0u) |
                   1024u; /* Near-zero restore protocol. */
  u[14] = (float)flags;
  unsigned next = frame_bank ^ 1u;
  memcpy(foot_frame[next], f, sizeof f);
  memcpy(ui_frame[next], u, sizeof u);
  __DMB();
  frame_bank = next;
}
static bool app_command_impl(const char *s) {
  /* Failed initialization may have left peripherals only partly configured.
   * Keep USB diagnostics responsive without accepting any recovery/drive
   * action. */
  if (!initialized) {
    if (!strcmp(s, "send 5")) {
      group = 5;
      return true;
    }
    if (!strcmp(s, "stop") || !strcmp(s, "foot stop")) {
      dual_hw_off_all();
      return true;
    }
    home_failure_result = 22;
    return false;
  }
  if (!strncmp(s, "foot ", 5))
    return foot_command(s);
  if (!strcmp(s, "send 3") || !strcmp(s, "send 4") || !strcmp(s, "send 5")) {
    group = (unsigned)(s[5] - '0');
    return true;
  }
  if (!strcmp(s, "m0 stop") || !strcmp(s, "m1 stop")) {
    unsigned i = s[1] == '1';
    uint32_t k = __get_PRIMASK();
    APP_LOCK();
    if (foot_owned)
      stop_all();
    pending = false;
    dual_hw_off(i);
    if (i)
      foc_stop_1();
    else
      foc_stop_0();
    if (calibrating == (int)i)
      calibrating = -1;
    APP_UNLOCK(k);
    return true;
  }

  if (!strcmp(s, "stop") || !strcmp(s, "all stop")) {
    uint32_t k = __get_PRIMASK();
    APP_LOCK();
    stop_all();
    APP_UNLOCK(k);
    return true;
  }
  if (!strcmp(s, "send 0") || !strcmp(s, "send 6")) {
    group = leg_teach_state() ? 1 : 0;
    return true;
  }
  if (!strcmp(s, "send 7")) {
    group = 7;
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
  if (!strcmp(s, "leg restore")) {
    if (dual_fault)
      return false;
    if (acquisition_halted ||
        (uint32_t)(bsp_uart_millis() - last_pair_ms) > 50u) {
      home_failure_result = 14;
      return false;
    }
    if (pending || calibrating >= 0 || foot_save_pending) {
      home_failure_result = 12;
      return false;
    }
    if (foc_0.state != FOC_IDLE || foc_1.state != FOC_IDLE || dual_hw_mode(0) ||
        dual_hw_mode(1)) {
      home_failure_result = 5;
      return false;
    }
    if (fabsf(speed(0)) >= 5 || fabsf(speed(1)) >= 5) {
      home_failure_result = 8;
      return false;
    }
    if (leg_homed())
      return true; /* Repeated click preserves valid full-travel tracking. */
    leg_restore_t plan;
    unsigned check = leg_restore_prepare(&plan);
    if (check) {
      home_failure_result = (check & LEG_HOME_TRACKING) ? 14
                            : check == LEG_HOME_M0      ? 19
                            : check == LEG_HOME_M1      ? 20
                                                        : 21;
      return false;
    }
    uint32_t epoch = foot_epoch;
    uint32_t key = __get_PRIMASK();
    APP_LOCK();
    bool ok = epoch == foot_epoch && !dual_fault && !acquisition_halted &&
              !pending && foc_0.state == FOC_IDLE && foc_1.state == FOC_IDLE &&
              leg_restore_apply(&plan);
    if (ok) {
      foot_cancel();
      joint_goal[0] = leg_joint(0);
      joint_goal[1] = leg_joint(1);
      if (group != 5)
        group = 0;
    }
    APP_UNLOCK(key);
    if (!ok)
      home_failure_result = 12;
    return ok; /* No enable, observer reset, flash write or forced zero. */
  }
  if (!strcmp(s, "leg home") || !strcmp(s, "leg home confirmed")) {
    if (dual_fault)
      return false;
    if (acquisition_halted ||
        (uint32_t)(bsp_uart_millis() - last_pair_ms) > 50u) {
      home_failure_result = 14;
      return false;
    }
    if (pending || calibrating >= 0) {
      home_failure_result = 12;
      return false;
    }
    if (!leg_teach_state() && (!idle(&foc_0) || !idle(&foc_1))) {
      home_failure_result = 5;
      return false;
    }
    if (fabsf(speed(0)) > 5 || fabsf(speed(1)) > 5) {
      home_failure_result = 8;
      return false;
    }
    uint32_t k = __get_PRIMASK();
    APP_LOCK();
    stop_all();
    unsigned check = leg_home_check();
    home_failure_result = (check & LEG_HOME_TRACKING) ? 14
                          : check == LEG_HOME_M0      ? 15
                          : check == LEG_HOME_M1      ? 16
                          : check                     ? 17
                                                      : 0;
    bool ok = !strcmp(s, "leg home confirmed") ? leg_home_confirmed_stop()
                                               : leg_home();
    if (ok)
      home_failure_result = 0;
    if (ok) {
      joint_goal[0] = joint_goal[1] = 0;
      foc_acquisition_resume_0();
      foc_acquisition_resume_1();
      if (group != 5)
        group = 0;
    }
    APP_UNLOCK(k);
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
    if ((foot_owned && !joint_plan_rest()) || dual_fault ||
        acquisition_halted || calibrating >= 0 || !number(s + 9, &rate))
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
    APP_LOCK();
    dual_hw_off_all();
    foc_init_0(h0 ? &c0 : NULL);
    foc_init_1(h1 ? &c1 : NULL);
    have_parameters[0] = have_parameters[1] = false;
    dual_fault = 0;
    if (acquisition_halted) {
      acquisition_halted = false;
      dual_hw_start();
    }
    APP_UNLOCK(k);
    return true;
  }
  if (!strncmp(s, "limit ", 6)) {
    float v;
    return idle(&foc_0) && idle(&foc_1) && calibrating < 0 && !pending &&
           number(s + 6, &v) && leg_set_current_limit(v);
  }
  if (pending || dual_fault || calibrating >= 0 || foot_save_pending)
    return false;
  unsigned i, next_action;
  float next0, next1;
  if (!strncmp(s, "m0 ", 3)) {
    i = 0;
    s += 3;
  } else if (!strncmp(s, "m1 ", 3)) {
    i = 1;
    s += 3;
  } else if (!strcmp(s, "leg hold")) {
    if (!can_position(&foc_0) || !can_position(&foc_1))
      return false;
    return queue_command(1, 0, leg_joint(0), leg_joint(1), true);
  } else if (!strncmp(s, "leg pos ", 8)) {
    if (!pair_numbers(s + 8, &next0, &next1) || next0 < 0 || next0 > 110 ||
        next1 < 0 || next1 > 110)
      return false;
    if (!can_position(&foc_0) || !can_position(&foc_1) ||
        !leg_target_valid(0, leg_motor_target(0, next0, position(0)),
                          position(0)) ||
        !leg_target_valid(1, leg_motor_target(1, next1, position(1)),
                          position(1)))
      return false;
    if (foot_calibrated) {
      fb_geometry_t g = foot_drive_geometry();
      fb_pose_t pose;
      float endpoint[2] = {next0, next1};
      foot_status = fb_forward(&g, endpoint, &pose);
      if (foot_status != FB_OK)
        return false;
      return plan_position(pose.xy, false, endpoint);
    }
    /* Fast paired motion requires the known geometry and common preflight.
     * Uncalibrated commissioning retains the slow legacy joint interface. */
    if (leg_motion_rate() > 45)
      return false;
    return queue_command(1, 0, next0, next1, true);
  } else
    return false;
  if (!strcmp(s, "stop")) {
    uint32_t k = __get_PRIMASK();
    APP_LOCK();
    if (foot_owned)
      stop_all();
    pending = false;
    dual_hw_off(i);
    if (i)
      foc_stop_1();
    else
      foc_stop_0();
    APP_UNLOCK(k);
    return true;
  }
  if (!strcmp(s, "zero"))
    return false; /* Only paired known-stop home sets mechanical zero. */
  if (!strcmp(s, "hold")) {
    next0 = leg_joint(i);
    next_action = 0;
  } else if (!strncmp(s, "pos ", 4) && number(s + 4, &next0)) {
    if (next0 < 0 || next0 > 110)
      return false;
    next_action = 0;
  } else if (!strncmp(s, "cal ", 4) && number(s + 4, &next0)) {
    if (!idle(&foc_0) || !idle(&foc_1) || next0 < .2f ||
        next0 > LEG_CURRENT_MAX || !leg_homed() || leg_joint(i) < 5 ||
        leg_joint(i) > 105)
      return false;
    next_action = 2;
  } else
    return false;
  if (next_action == 0 &&
      (!can_position(i ? &foc_1 : &foc_0) ||
       !leg_target_valid(i, leg_motor_target(i, next0, position(i)),
                         position(i))))
    return false;
  return queue_command(next_action, i, next0, 0, true);
}

bool app_command(const char *s) {
  foot_cal_bad_pose = false;
  home_failure_result = 0;
  foot_commit_failure = 0;
  bool ok = app_command_impl(s);
  if (!strncmp(s, "send ", 5) && ok)
    return true;
  unsigned result = 0;
  if (!ok) {
    bool foot = !strncmp(s, "foot ", 5),
         home = !strcmp(s, "leg home") || !strcmp(s, "leg home confirmed") ||
                !strcmp(s, "leg restore");
    if (!initialized)
      result = 22;
    else if (dual_fault)
      result = 1;
    else if (foot_cal_bad_pose)
      result = 13;
    else if (foot_save_pending)
      result = 11;
    else if (!leg_homed() &&
             (foot || !strncmp(s, "leg pos ", 8) || !strcmp(s, "leg hold")))
      result = 2;
    else if (foot &&
             (!strcmp(s, "foot calibrate") || !strcmp(s, "foot save") ||
              !strncmp(s, "foot cal ", 9)) &&
             (foc_0.state != FOC_IDLE || foc_1.state != FOC_IDLE))
      result = 5;
    else if (foot &&
             (!strcmp(s, "foot calibrate") || !strncmp(s, "foot cal ", 9)) &&
             (fabsf(speed(0)) >= 5 || fabsf(speed(1)) >= 5))
      result = 8;
    else if ((!strncmp(s, "leg pos ", 8) && !foot_calibrated &&
              leg_motion_rate() > 45))
      result = 3;
    else if (foot && !foot_calibrated && strncmp(s, "foot cal ", 9) &&
             strcmp(s, "foot calibrate"))
      result = 3;
    else if (home)
      result = home_failure_result ? home_failure_result : 10;
    else if (foot_commit_failure)
      result = foot_commit_failure;
    else if (pending || calibrating >= 0 || leg_teach_state())
      result = 12;
    else if ((!strncmp(s, "limit ", 6) || !strncmp(s, "foot speed ", 11)) &&
             (!idle(&foc_0) || !idle(&foc_1)))
      result = 5;
    else if (!foc_0.calibrated || !foc_1.calibrated)
      result = 9;
    else if (!foc_0.zero_ready || !foc_1.zero_ready)
      result = 4;
    else if (foot && (!strcmp(s, "foot save") || !strncmp(s, "foot cal ", 9)))
      result = 5;
    else if ((foot || !strncmp(s, "leg pos ", 8)) && foot_status != FB_OK)
      result = 6;
    else
      result = 7;
  }
  uint32_t key = __get_PRIMASK();
  APP_LOCK();
  ui_result = result;
  ++ui_command_sequence;
  __DMB();
  APP_UNLOCK(key);
  return ok;
}
static void save_foot_geometry(void) {
  if (!foot_save_pending)
    return;
  foot_save_pending = false;
  stop_all();
  leg_unhome();
  dual_hw_halt();
  acquisition_halted = true;
  /* Sector erase suspends trustworthy multi-turn sampling. Calibration
   * survives power-off; homing never does. Keep both bridges off throughout. */
  bool ok = foot_record_save(&foot_geometry);
  foot_saved = ok;
  if (!ok) {
    ui_result = 1;
    app_abort(FOC_FLASH);
    return;
  }
  if (!dual_hw_init()) {
    app_abort(FOC_SENSOR);
    return;
  }
  leg_clear_tracking_fault();
  foc_acquisition_resume_0();
  foc_acquisition_resume_1();
  acquisition_halted = false;
  geometry_telemetry();
  dual_hw_start();
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
static void startup_telemetry(void) {
  static uint32_t sent_ms;
  uint32_t now = bsp_uart_millis();
  if ((uint32_t)(now - sent_ms) < 20u || !bsp_usb_ready())
    return;
  /* No ADC, bus reading, geometry solve, flash write, retry or motor action.
   * NaN means unavailable, never pretend that failed encoders read zero. */
  float frame[25] = {0};
  for (unsigned i = 0; i < 11; ++i)
    frame[i] = NAN;
  frame[11] = frame[12] = FOC_FAULT;
  frame[13] = (float)dual_fault;
  unsigned flags = 1024u | 2048u;
  if (dual_init_attempts) {
    if (!(dual_init_crc_valid & 2u))
      flags |= 4096u; /* SPI lane 1 = M0. */
    if (!(dual_init_crc_valid & 1u))
      flags |= 8192u; /* SPI lane 0 = M1. */
  }
  frame[14] = (float)flags;
  frame[15] = (float)ui_result;
  frame[16] = (float)(ui_command_sequence & 0xffffffu);
  frame[17] = leg_motion_rate();
  frame[18] = leg_current_limit();
  frame[19] = FB_CONFIG;
  frame[21] = frame[22] = NAN;
  frame[23] = 5005;
  frame[24] = INFINITY;
  if (bsp_usb_write(frame, sizeof frame))
    sent_ms = now;
}
void app_poll(void) {
  static uint32_t fault_sent;
  if (initialized && dual_fault && acquisition_halted &&
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
  if (!initialized) {
    startup_telemetry();
    return;
  }
  save_foot_geometry();
  foot_reference_poll();
  foot_poll();
  telemetry_poll();
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

static void telemetry_poll(void) {
  static uint32_t last_telemetry_ms;
  uint32_t telemetry_ms = bsp_uart_millis();
  /* 50 Hz telemetry: enough for VOFA state/hold plots, one twentieth of the
   * prior USB/foreground load. Current/outer/reference rates are unchanged. */
  if (!acquisition_halted &&
      (uint32_t)(telemetry_ms - last_telemetry_ms) >= 20u) {
    if (!bsp_usb_ready())
      last_telemetry_ms = telemetry_ms;
    else {
      telemetry(last_bus);
      /* A sampling IRQ may invalidate this snapshot. Retry on the next main
       * pass; waiting 20 ms can repeat the same unlucky interrupt phase. */
      if (telemetry_sent)
        last_telemetry_ms = telemetry_ms;
    }
  }
}
