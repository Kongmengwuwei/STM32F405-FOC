#include "app.h"
#include "bsp_uart.h"
#include "bsp_usb.h"
#include "dual.h"
#include "leg_guard.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
volatile uint32_t motor_sample_us_0, motor_sample_us_1, dual_sequence,
    dual_fault, dual_encoder_cycles_max;
volatile uint32_t dual_write_counter_min[2];
volatile float motor_duty_0[3], motor_duty_1[3];
volatile bsp_uart_stats_t g_uart_stats;
volatile bsp_usb_stats_t g_usb_stats;
static uint32_t tick;
static unsigned modes[2], saved;
static float current_a[2], current_b[2], degrees[2] = {18, 350}, rpm[2];
static const float start[2] = {18, 350}, phase_zero[2] = {1.1f, 5.93411946f};
static const int direction[2] = {1, -1};
static bool fixed;
static float friction, inertia = .0002f;
uint32_t bsp_uart_millis(void) { return tick / 1000; }
bool bsp_uart_init(void) { return true; }
bool bsp_can_init(void) { return true; }
void bsp_uart_tick(void) {}
bool bsp_uart_write(const void *p, size_t n) {
  (void)p;
  return n != 0;
}
size_t bsp_uart_read(void *p, size_t n) {
  (void)p;
  (void)n;
  return 0;
}
void bsp_usb_poll(void) {}
bool bsp_usb_ready(void) { return true; }
bool bsp_usb_write(const void *p, size_t n) {
  (void)p;
  assert(n == 100);
  return true;
}
size_t bsp_usb_read(void *p, size_t n) {
  (void)p;
  (void)n;
  return 0;
}
bool dual_record_load(unsigned i, foc_calibration_t *c) {
  (void)i;
  (void)c;
  return false;
}
bool dual_record_save(unsigned i, const foc_calibration_t *c) {
  (void)i;
  assert(c->pole_pairs == 7);
  ++saved;
  return true;
}
bool dual_hw_init(void) {
  tick += 10000;
  return true;
}
void dual_hw_start(void) {}
void dual_hw_halt(void) { dual_hw_off_all(); }
void dual_hw_off(unsigned i) { modes[i] = 0; }
void dual_hw_off_all(void) { modes[0] = modes[1] = 0; }
void dual_hw_arm(unsigned i) { (void)i; }
unsigned dual_hw_mode(unsigned i) { return modes[i]; }
bool dual_hw_update(unsigned i) {
  (void)i;
  return true;
}
bool dual_hw_write(unsigned i, const float d[3], unsigned mode) {
  modes[i] = mode;
  for (unsigned n = 0; n < 3; ++n)
    (i ? motor_duty_1 : motor_duty_0)[n] = d[n];
  return true;
}
static void plant(unsigned i) {
  const float dt = .0001f, resistance = .124f, inductance = .00004f;
  float theta = direction[i] * 7 * degrees[i] * .01745329252f - phase_zero[i];
  volatile float *d = i ? motor_duty_1 : motor_duty_0;
  float va = modes[i] == 2 ? 12 * (2 * d[0] - d[1] - d[2]) / 3 : 0;
  float vb = modes[i] == 2 ? 12 * (d[1] - d[2]) * .5773502692f : 0;
  float omega = rpm[i] * .1047197551f;
  float bemf = direction[i] * .02f * omega;
  float decay = expf(-resistance * dt / inductance);
  current_a[i] = decay * current_a[i] +
                 (1 - decay) * (va + bemf * sinf(theta)) / resistance;
  current_b[i] = decay * current_b[i] +
                 (1 - decay) * (vb - bemf * cosf(theta)) / resistance;
  float iq = -current_a[i] * sinf(theta) + current_b[i] * cosf(theta);
  if (!fixed) {
    float torque = direction[i] * .03f * iq - .002f * omega;
    float next = 0;
    if (fabsf(omega) > 1e-5f || fabsf(torque) > friction) {
      float sign = fabsf(omega) > 1e-5f ? omega : torque;
      next = omega + dt * (torque - copysignf(friction, sign)) / inertia;
      if (next * omega < 0 && fabsf(torque) < friction)
        next = 0;
    }
    omega = next;
    rpm[i] = omega / .1047197551f;
    degrees[i] += omega * dt * 57.2957795f;
  }
}
bool dual_hw_read_adc(dual_adc_t *a, dual_adc_t *b) {
  for (unsigned i = 0; i < 2; ++i)
    plant(i);
  motor_sample_us_0 = motor_sample_us_1 = tick & 0xffffffu;
  *a = (dual_adc_t){
      1.65f + (-.5f * current_a[0] + .8660254038f * current_b[0]) / 50,
      1.65f + (-.5f * current_a[0] - .8660254038f * current_b[0]) / 50, 12};
  *b = (dual_adc_t){
      1.65f + (-.5f * current_a[1] + .8660254038f * current_b[1]) / 50,
      1.65f + (-.5f * current_a[1] - .8660254038f * current_b[1]) / 50, 12};
  return true;
}
bool dual_hw_read_bus(float *v) {
  *v = 12;
  return true;
}
void dual_hw_encoder_pair(float a[2]) {
  for (unsigned i = 0; i < 2; ++i) {
    float d = degrees[i] - 360 * floorf(degrees[i] / 360);
    a[i ? 0 : 1] =
        ((unsigned)floorf(d * 32768 / 360 + .5f) & 32767u) * (360.0f / 32768);
  }
}
static void run(unsigned n, const char *command) {
  for (unsigned k = 0; k < n * 10; ++k) {
    tick += 100;
    if (command && k % 500 == 0)
      assert(app_command(command));
    dual_app_sample();
    if (k % 10 == 0)
      app_poll();
  }
}
int main(void) {
  assert(app_init());
  run(600, NULL);
  assert(foc_0.state == FOC_IDLE && foc_1.state == FOC_IDLE);
  assert(!app_command("m0 Iq 1") && !app_command("m1 rpm 1") &&
         !app_command("m0 cal"));
  assert(app_command("m0 cal 1.00"));
  run(13000, NULL);
  printf("cal0 fault=%u state=%u saved=%u travel=%.3f dir=%d\n", dual_fault,
         foc_0.state, saved, leg_travel(0), foc_0.calibration.direction);
  assert(!dual_fault && foc_0.calibrated && foc_0.calibration.direction == 1 &&
         saved == 1);
  assert(app_command("m1 cal 1.00"));
  run(13000, NULL);
  printf("cal1 fault=%u state=%u saved=%u travel=%.3f dir=%d\n", dual_fault,
         foc_1.state, saved, leg_travel(1), foc_1.calibration.direction);
  assert(!dual_fault && foc_1.calibrated && foc_1.calibration.direction == -1 &&
         saved == 2);
  run(3500, "leg pos 0 0");
  assert(!dual_fault);
  assert(fabsf(control_position_deg_0()) < .2f &&
         fabsf(control_position_deg_1()) < .2f);
  run(2500, "leg pos 2 -2");
  printf("pos0=%.3f pos1=%.3f fault=%u\n", control_position_deg_0(),
         control_position_deg_1(), dual_fault);
  assert(!dual_fault && fabsf(control_position_deg_0() - 14) < .2f &&
         fabsf(control_position_deg_1() + 14) < .2f);
  assert(!app_command("leg pos 1 40"));
  assert(control_position_target_0() == 14 &&
         control_position_target_1() == -14);
  assert(app_command("stop"));
  assert(modes[0] == 0 && modes[1] == 0);
  assert(app_command("m0 zero"));
  run(2, NULL);
  assert(!app_command("m0 pos 30"));
  assert(app_command("limit 9.00") && leg_current_limit() == 9);
  assert(!app_command("limit 9.01"));
  assert(app_command("limit 2.00"));
  run(1000, "leg pos 0 0");
  run(250, NULL);
  assert(dual_fault == FOC_UART && modes[0] == 0 && modes[1] == 0);
  assert(app_command("stop"));
  assert(app_command("clear"));
  run(600, NULL);
  assert(!dual_fault);
  assert(app_command("m0 cal 1.00"));
  run(10, NULL);
  assert(app_command("m0 stop"));
  assert(modes[0] == 0);
  friction = .008f;
  inertia = .002f;
  assert(app_command("m0 cal 7.50"));
  run(13000, NULL);
  printf("loaded cal0 fault=%u saved=%u travel=%.3f rpm=%.3f i=%.3f\n",
         dual_fault, saved, leg_travel(0), foc_0.rpm,
         hypotf(foc_0.id, foc_0.iq));
  assert(!dual_fault && saved == 3 && foc_0.calibrated);
  float err = remainderf(foc_0.calibration.zero - phase_zero[0], 6.2831853072f);
  printf("loaded cal0 zero error=%.3f electrical deg\n", err * 57.2957795f);
  assert(fabsf(err) < 7 * .01745329252f);
  assert(app_command("m1 cal 7.50"));
  run(13000, NULL);
  assert(!dual_fault && saved == 4 && foc_1.calibrated);
  friction = 0;
  run(10, "leg pos 0 0");
  assert(modes[0] == 2 && modes[1] == 2);
  /* Vector magnitude exceeds 9.5 A while each physical phase is below it. */
  current_a[1] = 0;
  current_b[1] = 14;
  run(1, NULL);
  assert(dual_fault == FOC_CURRENT && modes[0] == 0 && modes[1] == 0);
  assert(app_command("clear"));
  run(600, NULL);
  assert(!dual_fault);
  assert(!app_command("m0 cal 9.01"));
  assert(app_command("m0 cal 9.00"));
  run(1, NULL);
  assert(app_command("stop"));
  run(20, NULL);
  fixed = true;
  assert(app_command("m0 cal 1.00"));
  run(13000, NULL);
  assert(dual_fault == FOC_ALIGNMENT && saved == 4);
  assert(app_command("stop"));
  assert(app_command("clear"));
  run(600, NULL);
  /* Passive back-driving must stop the actively controlled other bridge. */
  for (unsigned n = 0; n < 10000 && !dual_fault; ++n) {
    degrees[1] -= .004f;
    run(1, "m0 hold");
  }
  assert(dual_fault == FOC_POSITION && modes[0] == 0 && modes[1] == 0);
  assert(app_command("stop"));
  degrees[0] = start[0] + 29 * LEG_GEAR_RATIO;
  assert(app_command("clear"));
  run(2, NULL);
  assert(dual_fault == FOC_POSITION && modes[0] == 0 && modes[1] == 0);
  puts("PASS: 7:1 joint units,  independent RL/mechanical plants, bounded "
       "calibration both directions, paired positions, unchanged invalid "
       "targets, zero cannot expand travel, current limits, watchdog and "
       "paired fault stop");
}
