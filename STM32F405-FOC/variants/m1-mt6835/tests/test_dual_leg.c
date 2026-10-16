#include "app.h"
#include "bsp_uart.h"
#include "bsp_usb.h"
#include "dual.h"
#include "fivebar.h"
#include "fivebar_config.h"
#include "foot_record.h"
#include "leg_guard.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
volatile uint32_t motor_sample_us_0, motor_sample_us_1, dual_sequence,
    dual_fault, dual_encoder_cycles_max;
volatile uint32_t dual_write_counter_min[2];
volatile uint32_t dual_init_attempts, dual_init_crc_valid;
static bool fail_init;
static const char *usb_input;
volatile float motor_duty_0[3], motor_duty_1[3];
volatile bsp_uart_stats_t g_uart_stats;
volatile bsp_usb_stats_t g_usb_stats;
static uint32_t tick;
static unsigned modes[2], saved;
static float current_a[2], current_b[2], degrees[2] = {18, 350}, rpm[2];
static const float start[2] = {18, 350}, phase_zero[2] = {1.1f, 5.93411946f};
static const int direction[2] = {1, -1};
static float packet[25];
static bool reject_next_frame;
static unsigned frame_attempts, frames_written;
static bool fixed;
static float friction, gravity_current, inertia = .0002f;
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
  ++frame_attempts;
  if (reject_next_frame) {
    reject_next_frame = false;
    return false;
  }
  ++frames_written;
  memcpy(packet, p, n);
  return true;
}
size_t bsp_usb_read(void *p, size_t n) {
  if (!usb_input)
    return 0;
  size_t count = strlen(usb_input);
  if (count > n)
    count = n;
  memcpy(p, usb_input, count);
  usb_input += count;
  return count;
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
static bool geometry_present;
static fb_geometry_t stored_geometry;
bool foot_record_load(fb_geometry_t *g) {
  if (!geometry_present)
    return false;
  *g = stored_geometry;
  return true;
}
bool foot_record_save(const fb_geometry_t *g) {
  stored_geometry = *g;
  geometry_present = true;
  return true;
}
bool dual_hw_init(void) {
  tick += 10000;
  return !fail_init;
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
    float torque = direction[i] * .03f * iq - .002f * omega +
                   (i == 0 ? 1.0f : -1.0f) * .03f * gravity_current;
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
    if (command && k % 500 == 0 && !app_command(command)) {
      printf("command rejected: %s fault=%u q=%.3f/%.3f elapsed=%u ms\n",
             command, dual_fault, leg_joint(0), leg_joint(1), k / 10);
      assert(dual_fault); /* The intentional stall test stops refreshing on
                             trip. */
      return;
    }
    dual_app_sample();
    if (k % 10 == 0)
      app_poll();
  }
}
static void place_at_joint10(void) {
  assert(app_command("stop"));
  bool was_fixed = fixed;
  fixed = true;
  for (unsigned n = 0; n < 2000; ++n) {
    degrees[0] += (start[0] - degrees[0]) * .004f;
    degrees[1] += (start[1] - degrees[1]) * .004f;
    run(1, NULL);
  }
  run(200, NULL);
  assert(app_command("leg home"));
  for (unsigned n = 0; n < 500; ++n) {
    degrees[0] += .14f;
    degrees[1] -= .14f;
    run(1, NULL);
  }
  run(300, NULL);
  fixed = was_fixed;
}
int main(void) {
  /* Cold-boot encoder failure must report through USB, never accept clear,
   * homing, calibration, parameter writes or movement on partial hardware. */
  fail_init = true;
  dual_init_attempts = 5;
  dual_init_crc_valid = 1; /* M1 answered; M0 did not. */
  assert(!app_init());
  app_abort(FOC_SENSOR);
  usb_input = "send 5\r\n";
  tick += 21000;
  app_poll();
  assert(packet[23] == 5005 && packet[13] == FOC_SENSOR);
  assert((unsigned)packet[14] == (1024u | 2048u | 4096u));
  assert(isnan(packet[0]) && isnan(packet[1]) && isnan(packet[10]));
  assert(!leg_homed() && modes[0] == 0 && modes[1] == 0);
  const char *blocked[] = {"clear",     "leg restore",   "leg home confirmed",
                           "leg teach", "leg pos 10 10", "foot pos 0 160",
                           "cal 0",     "limit 10"};
  for (unsigned i = 0; i < sizeof blocked / sizeof blocked[0]; ++i)
    assert(!app_command(blocked[i]));
  assert(app_command("stop"));
  dual_init_crc_valid = 0;
  tick += 21000;
  app_poll();
  assert((unsigned)packet[14] == (1024u | 2048u | 4096u | 8192u));
  dual_init_crc_valid = 2;
  tick += 21000;
  app_poll();
  assert((unsigned)packet[14] == (1024u | 2048u | 8192u));
  assert(modes[0] == 0 && modes[1] == 0 && packet[20] == 0);
  /* Emulate a fresh successful boot for the existing closed-loop cases. */
  fail_init = false;
  dual_fault = 0;
  usb_input = NULL;
  assert(app_init());
  assert(app_command("leg rate 10") && app_command("leg rate 30"));
  assert(!app_command("leg rate 9") && !app_command("leg rate 120.01"));
  assert(!app_command("leg rate NaN"));
  run(600, NULL);
  /* A rejected telemetry attempt must not consume the 20 ms send interval.
   * The next main pass retries; successful streaming remains 50 Hz. */
  unsigned attempts = frame_attempts, written = frames_written;
  reject_next_frame = true;
  for (unsigned ms = 0; ms < 21 && frame_attempts == attempts; ms++)
    run(1, NULL);
  assert(frame_attempts == attempts + 1 && frames_written == written);
  run(1, NULL);
  assert(frame_attempts == attempts + 2 && frames_written == written + 1);
  attempts = frame_attempts;
  run(5, NULL);
  assert(frame_attempts == attempts);
  assert(!app_command("foot pos 0 160"));
  assert(!app_command("foot cal -90 -90"));
  assert(!app_command("leg pos 2 2")); /* Boot never guesses motor turns. */
  /* A stopped axis away from the recorded stop gets axis-specific feedback.
   * A second command remains possible after a rejection; gates stay off. */
  assert(app_command("send 5"));
  fixed = true;
  for (unsigned axis = 0; axis < 2; ++axis) {
    for (unsigned n = 0; n < 50; ++n) {
      degrees[axis] += .4f;
      run(1, NULL);
    }
    run(300, NULL);
    assert(!app_command("leg home"));
    run(25, NULL);
    assert(packet[15] == (axis ? 17 : 15) && !leg_homed() && modes[0] == 0 &&
           modes[1] == 0);
  }
  for (unsigned n = 0; n < 50; ++n) {
    degrees[0] -= .4f;
    run(1, NULL);
  }
  run(300, NULL);
  assert(!app_command("leg home"));
  run(25, NULL);
  assert(packet[15] == 16 && !leg_homed());
  assert(!app_command("leg restore") && !leg_homed());
  run(25, NULL);
  assert(packet[15] == 20 && !modes[0] && !modes[1]);
  /* Only an explicit human-confirmed physical stop may replace the raw
   * plausibility check. It must still be fresh, stationary and gates off. */
  assert(app_command("leg home confirmed") && leg_homed());
  run(25, NULL);
  assert(packet[15] == 0 && fabsf(leg_joint(0)) < .01f &&
         fabsf(leg_joint(1)) < .01f && !modes[0] && !modes[1]);
  leg_unhome();
  for (unsigned n = 0; n < 50; ++n) {
    degrees[1] -= .4f;
    run(1, NULL);
  }
  run(300, NULL);
  tick += 60000; /* Cached angles alone cannot satisfy homing. */
  assert(!app_command("leg home confirmed"));
  assert(!app_command("leg restore"));
  assert(!app_command("leg home"));
  run(25, NULL);
  assert(packet[15] == 14 && !leg_homed());
  assert(app_command("leg home") && leg_homed());
  run(25, NULL);
  assert(packet[15] == 0 && packet[23] == 5005);
  assert(app_command("send 0"));
  /* Recovery preserves a nonzero physical reading, and repeated recovery
   * cannot relabel it. Neither path enables a bridge. */
  for (unsigned n = 0; n < 100; ++n) {
    degrees[0] += .35f;
    degrees[1] -= .35f;
    run(1, NULL);
  }
  run(300, NULL);
  leg_unhome();
  assert(app_command("leg restore") && leg_homed());
  float restored0 = leg_joint(0), restored1 = leg_joint(1);
  assert(fabsf(restored0 - 5) < .01f && fabsf(restored1 - 5) < .01f &&
         !modes[0] && !modes[1]);
  assert(app_command("leg restore") && leg_joint(0) == restored0 &&
         leg_joint(1) == restored1);
  for (unsigned n = 0; n < 100; ++n) {
    degrees[0] -= .35f;
    degrees[1] += .35f;
    run(1, NULL);
  }
  run(300, NULL);
  assert(app_command("leg home"));
  run(25, NULL);
  assert(app_command("send 5"));
  run(25, NULL);
  assert(((unsigned)packet[14] & 1024u) != 0);
  assert(app_command("send 0"));
  fixed = false;
  assert(!app_command("m0 cal 1.00")); /* Never align a field at the stop. */
  for (unsigned n = 0; n < 500; ++n) {
    degrees[0] += .14f;
    degrees[1] -= .14f;
    run(1, NULL);
  }
  run(600, NULL);
  assert(!dual_fault && fabsf(leg_joint(0) - 10) < .01f &&
         fabsf(leg_joint(1) - 10) < .01f);
  assert(app_command("m0 cal 1.00"));
  run(13000, NULL);
  printf("cal0 fault=%u state=%u saved=%u homed=%d q=%.2f/%.2f\n", dual_fault,
         foc_0.state, saved, leg_homed(), leg_joint(0), leg_joint(1));
  assert(!dual_fault && foc_0.calibrated && foc_0.calibration.direction == 1 &&
         saved == 1);
  assert(!leg_homed());
  place_at_joint10();
  assert(app_command("m1 cal 1.00"));
  run(13000, NULL);
  assert(!dual_fault && foc_1.calibrated && foc_1.calibration.direction == -1 &&
         saved == 2);
  place_at_joint10();
  run(4000, "leg pos 10 10");
  assert(app_command("foot speed 150") && !app_command("foot speed 400.01"));
  assert(!dual_fault && fabsf(leg_joint(0) - 10) < .1f &&
         fabsf(leg_joint(1) - 10) < .1f);
  assert(app_command("stop"));
  friction = .008f;
  inertia = .002f;
  assert(app_command("m0 cal 7.50"));
  run(13000, NULL);
  assert(!dual_fault && saved == 3);
  assert(fabsf(remainderf(foc_0.calibration.zero - phase_zero[0],
                          6.2831853072f)) < 7 * .01745329252f);
  place_at_joint10();
  assert(app_command("m1 cal 7.50"));
  run(13000, NULL);
  assert(!dual_fault && saved == 4);
  place_at_joint10();
  friction = 0;
  inertia = .0002f;
  /* Cartesian ownership, physical zero calibration, mirrored asymmetric
   * targets and an independent host watchdog despite internal 200 Hz updates.
   */
  assert(!app_command("foot pos 0 160"));
  assert(app_command("send 4"));
  run(40, NULL);
  assert(packet[23] == 4 && packet[20] == 0 && packet[0] == 35 &&
         packet[2] == -35);
  assert(!app_command("foot cal 90 90")); /* Wrong elbow/assembly pose. */
  assert(app_command("foot cal 10 170"));
  run(40, NULL);
  assert(packet[20] == 1 && packet[22] == 1);
  assert(app_command("send 3"));
  run(15000, "foot pos 5 160");
  printf("foot q=%.3f/%.3f xy=%.3f/%.3f status=%.0f fault=%u\n", leg_joint(0),
         leg_joint(1), packet[4], packet[5], packet[10], dual_fault);
  assert(!dual_fault && packet[23] == 3 && ((unsigned)packet[11] & 7) == 7);
  assert(hypotf(packet[4] - 5, packet[5] - 160) < .5f);
  assert(fabsf(leg_joint(0) - leg_joint(1)) > 1);
  assert(!app_command("foot cal 0 180") && !app_command("leg rate 45") &&
         !app_command("foot speed 80"));
  assert(!app_command("foot pos 0 1000") && !app_command("foot pos NaN 160"));
  run(100, "foot pos 5 160");
  assert(packet[0] == 5 &&
         packet[1] == 160); /* Reject retains the old curve. */
  run(150, "foot pos -5 150");
  run(10000, "foot pos 5 150");
  assert(!dual_fault && hypotf(packet[4] - 5, packet[5] - 150) < .5f);
  run(400, NULL);
  assert(!dual_fault);
  run(150, NULL);
  assert(dual_fault == FOC_UART && modes[0] == 0 && modes[1] == 0 &&
         leg_homed());
  assert(app_command("clear"));
  run(600, NULL);
  assert(!dual_fault && modes[0] == 0 && modes[1] == 0 &&
         !((unsigned)packet[11] & 4));
  run(12000, "foot pos 0 160");
  assert(!dual_fault && app_command("m0 stop"));
  run(40, NULL);
  assert(modes[0] == 0 &&
         modes[1] == 0); /* A coupled foot command stops as a pair. */
  run(10000, "foot pos 0 150");
  run(15000, "leg pos 10 10");
  assert(!dual_fault && !((unsigned)packet[11] & 4));
  assert(fabsf(leg_joint(0) - 10) < .15f && fabsf(leg_joint(1) - 10) < .15f);
  assert(app_command("send 0"));
  assert(app_command("leg rate 60"));
  run(15000, "leg pos 100 100");
  printf("workspace100 q0=%.3f q1=%.3f fault=%u\n", leg_joint(0), leg_joint(1),
         dual_fault);
  assert(!dual_fault && fabsf(leg_joint(0) - 100) < .1f &&
         fabsf(leg_joint(1) - 100) < .1f);
  /* The high-speed joint UI must ACK its final goal immediately, while both
   * axes use the common preflighted reference rather than a raw position step.
   */
  assert(app_command("send 5") && app_command("leg rate 120"));
  assert(app_command("leg pos 20 20"));
  run(40, "leg pos 20 20");
  assert(!dual_fault && packet[2] == 20 && packet[3] == 20 && packet[0] > 90 &&
         (((unsigned)packet[14] & 32u) == 0));
  run(3000, "leg pos 20 20");
  assert(!dual_fault && fabsf(leg_joint(0) - 20) < .5f &&
         fabsf(leg_joint(1) - 20) < .5f);
  run(150, "leg pos 80 80");
  run(3000, "leg pos 30 30");
  assert(!dual_fault && fabsf(leg_joint(0) - 30) < .5f &&
         fabsf(leg_joint(1) - 30) < .5f);
  /* Independent load model: gravity drives both joints inward with a torque
   * equivalent to 2.5 A, including during quick Cartesian reversals. */
  assert(app_command("stop") && app_command("foot speed 400"));
  assert(app_command("send 5"));
  gravity_current = 2.5f;
  /* Establish the load bias before fast reversals, as in the supported
   * slow commissioning approach used on the physical mechanism. */
  run(8000, "foot pos 0 160");
  assert(!dual_fault && hypotf(packet[4], packet[5] - 160) < 2);
  for (unsigned cycle = 0; cycle < 3; ++cycle) {
    run(4000, "foot pos 0 100");
    assert(!dual_fault && fabsf(foc_0.rpm) < LEG_SPEED_TRIP_MOTOR_RPM &&
           fabsf(foc_1.rpm) < LEG_SPEED_TRIP_MOTOR_RPM);
    printf("gravity100 xy=%.3f/%.3f q=%.3f/%.3f rpm=%.2f/%.2f\n",
           packet[4], packet[5], packet[0], packet[1], foc_0.rpm, foc_1.rpm);
    assert(hypotf(packet[4], packet[5] - 100) < 2);
    run(4000, "foot pos 0 200");
    assert(!dual_fault && hypotf(packet[4], packet[5] - 200) < 2);
  }
  gravity_current = 0;
  /* Removal of the external load is another torque step. Keep holding while
   * its learned bias settles before releasing and testing a new idle start. */
  run(5000, "foot pos 0 200");
  assert(!dual_fault && app_command("stop"));
  assert(app_command("send 0") && app_command("leg rate 60"));
  run(15000, "leg pos 2 2");
  assert(!dual_fault && fabsf(leg_joint(0) - 2) < .1f &&
         fabsf(leg_joint(1) - 2) < .1f);
  assert(!app_command("leg pos -1 2") && !app_command("leg pos 2 111"));
  assert(packet[2] == 2 && packet[14] == 2 &&
         ((unsigned)packet[23] & LEG_HOME_FLAG));
  assert(!app_command("m0 zero") && !app_command("leg home") &&
         !app_command("leg home confirmed") && !app_command("leg restore"));
  assert(app_command("limit 10.00") == false); /* Limits change only at idle. */
  assert(app_command("stop"));
  assert(app_command("limit 10.00") && !app_command("limit 10.01"));
  assert(app_command("limit 5.00"));
  /* Passive movement outside the old +/-3.8 deg never freezes acquisition. */
  fixed = true;
  for (unsigned n = 0; n < 500; ++n) {
    degrees[0] += .14f;
    degrees[1] -= .14f;
    run(1, NULL);
  }
  run(40, NULL);
  assert(!dual_fault && modes[0] == 0 && modes[1] == 0 && leg_joint(0) > 11);
  fixed = false;
  run(3000, "leg pos 12 12");
  run(400, NULL);
  assert(!dual_fault && modes[0] == 2 && modes[1] == 2);
  run(160, NULL);
  assert(dual_fault == FOC_UART && modes[0] == 0 && modes[1] == 0);
  assert(app_command("send 7"));
  run(25, NULL);
  assert(packet[23] == 5007 && packet[0] == FOC_UART && isfinite(packet[2]) &&
         isfinite(packet[3]) && ((unsigned)packet[21] & 9u) == 9u);
  assert(app_command("send 0"));
  uint32_t seq = dual_sequence;
  fixed = true;
  degrees[0] += .3f;
  run(100, NULL);
  assert(dual_sequence > seq &&
         leg_homed()); /* Watchdog preserves live turns. */
  assert(app_command("clear"));
  run(600, NULL);
  assert(!dual_fault && leg_homed() && modes[0] == 0 && modes[1] == 0);
  fixed = false;
  run(1000, "leg pos 12 12");
  current_a[1] = 0;
  current_b[1] = 20; /* Remains above 11 A after the simulated RL decay. */
  run(1, NULL);
  assert(dual_fault == FOC_CURRENT && modes[0] == 0 && modes[1] == 0);
  assert(app_command("clear"));
  run(600, NULL);
  fixed = true;
  run(3500, "leg pos 100 100");
  assert(dual_fault == FOC_CURRENT && modes[0] == 0 && modes[1] == 0 &&
         saved == 4);
  assert(app_command("clear"));
  run(600, NULL);
  /* Move the unpowered model back to its known stop, without endpoint guessing.
   */
  for (unsigned n = 0; n < 2000; ++n) {
    degrees[0] += (start[0] - degrees[0]) * .004f;
    degrees[1] += (start[1] - degrees[1]) * .004f;
    run(1, NULL);
  }
  assert(app_command("leg home"));
  assert(leg_limit_effort(0, -1, 1) == 0 && leg_limit_effort(1, -1, -1) == 0);
  assert(app_command(
      "foot cal 20 160")); /* Known stop is outside 2..108 drive margin. */
  /* Encoder quantisation at the reliable stop must not invalidate a legal
   * inward foot target; actual joint measurement remains unclipped. */
  degrees[0] -= .02f;
  run(40, NULL);
  assert(leg_joint(0) < 0 && leg_joint(0) > -.005f);
  assert(app_command("foot pos 0 160"));
  fixed = false;
  run(12000, "foot pos 0 160");
  assert(!dual_fault && app_command("stop"));
  assert(
      !app_command("foot calibrate")); /* Wrong pose must not save old zeros. */
  assert(!geometry_present);
  assert(app_command("foot save"));
  run(40, NULL);
  assert(geometry_present && !leg_homed());
  assert(!app_command("foot pos 0 160"));
  /* Return to the known stop to keep the legacy teaching fixture meaningful. */
  fixed = true;
  for (unsigned n = 0; n < 2000; n++) {
    degrees[0] += (start[0] - degrees[0]) * .004f;
    degrees[1] += (start[1] - degrees[1]) * .004f;
    run(1, NULL);
  }
  run(300, NULL);
  assert(app_command("send 5"));
  assert(app_command("leg home"));
  run(25, NULL);
  assert(packet[23] == 5005 && ((unsigned)packet[14] & 25u) == 25u);
  /* Known vertical pose: one GUI command calibrates and saves, with one ack. */
  for (unsigned n = 0; n < 5000; n++) {
    degrees[0] += .154f;
    degrees[1] -= .154f;
    run(1, NULL);
  }
  run(300, NULL);
  assert(app_command("foot calibrate"));
  run(40, NULL);
  assert(!leg_homed() && packet[23] == 5005 && packet[15] == 0 &&
         ((unsigned)packet[14] & 24u) == 24u);
  for (unsigned n = 0; n < 2000; n++) {
    degrees[0] += (start[0] - degrees[0]) * .004f;
    degrees[1] += (start[1] - degrees[1]) * .004f;
    run(1, NULL);
  }
  run(300, NULL);
  assert(app_command("leg home"));
  assert(app_command("leg teach"));
  run(2, NULL);
  assert(!dual_fault && leg_teach_state() == 1 && !leg_homed());
  float a0 = leg_reference(0), a1 = leg_reference(1);
  assert(!app_command("leg teach") && !app_command("leg pos 0 0"));
  assert(app_command("send 2"));
  for (unsigned n = 0; n < 5000; ++n) {
    degrees[0] += .126f;
    degrees[1] -= .126f;
    run(1, NULL);
    assert(!dual_fault && modes[0] == 0 && modes[1] == 0);
  }
  assert(fabsf(leg_travel(0) - 90) < .05f && fabsf(leg_travel(1) + 90) < .05f);
  assert(packet[23] == 2 && packet[22] == 1 && packet[13] == a0 &&
         packet[14] == a1);
  assert(!app_command(
      "leg home")); /* Single-turn plausibility rejects this pose. */
  assert(app_command("stop"));
  run(1, NULL);
  assert(leg_teach_state() == 1);
  tick += 1000;
  run(1, NULL);
  assert(dual_fault == FOC_SENSOR && !leg_homed() && modes[0] == 0 &&
         modes[1] == 0);
  assert(app_command("clear"));
  run(600, NULL);
  assert(!dual_fault && !leg_homed() && !app_command("leg pos 10 10"));
  puts(
      "PASS: explicit reliable-stop home, mirrored joint coordinates, "
      "100-degree "
      "multi-turn tracking, reverse motion, current caps, 500ms watchdog, live "
      "fault tracking, 2s saturated stall, bounded calibration and teach gap");
}
