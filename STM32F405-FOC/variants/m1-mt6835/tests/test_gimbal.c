/* Real dual command parser, current controller and outer loops; no hardware. */
#include "dual.h"
#include "app.h"
#include "bsp_uart.h"
#include "bsp_usb.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "FAIL line %d: %s; fault=%u/%u rpm=%.3f/%.3f pos=%.3f/%.3f iq=%.3f/%.3f\n", __LINE__, #x, (unsigned)foc0.fault, (unsigned)foc1.fault, (double)foc0.rpm, (double)foc1.rpm, (double)control0_position_deg(), (double)control1_position_deg(), (double)foc0.iq_ref, (double)foc1.iq_ref); exit(1); } } while (0)
volatile uint32_t motor0_sample_us, motor1_sample_us, dual_sequence, dual_fault;
volatile uint32_t dual_encoder_cycles_max;
volatile bsp_uart_stats_t g_uart_stats;
volatile bsp_usb_stats_t g_usb_stats;
static unsigned armed, off;
static unsigned hardware_inits;
static uint32_t milliseconds;
static float shaft[2] = {350.0f, 5.0f};
static bool plant, electrical_plant, locked_current;
static float phase_alpha[2], phase_beta[2];
static float generator_beta_voltage[2];
static float plant_rpm[2];
static float load_current[2]; /* Mechanical disturbance, expressed as equivalent Iq. */
static const foc_calibration_t cal[2] = {{0.1f, 1}, {0.2f, -1}};
bool bsp_uart_init(void) { return true; }
bool bsp_can_init(void) { return true; }
bool dual_hw_init(void) { ++hardware_inits; return true; }
bool dual_record_load(unsigned i, foc_calibration_t *out) { *out = cal[i]; return true; }
bool dual_record_save(unsigned i, const foc_calibration_t *c) { (void)i; (void)c; return true; }
void dual_hw_start(void) {}
void dual_hw_halt(void) {}
void dual_hw_off(unsigned i) { off |= 1u << i; }
void dual_hw_off_all(void) { off |= 3u; }
void dual_hw_arm(unsigned i) { armed |= 1u << i; }
bool dual_hw_read_adc(dual_adc_t *a, dual_adc_t *b)
{
    dual_adc_t *out[2] = {a, b};
    foc_t *motor[2] = {&foc0, &foc1};
    for (unsigned i = 0; i < 2u; ++i) {
        *out[i] = (dual_adc_t){1.65f, 1.65f, 11.5f};
        if (electrical_plant) {
            float da = motor[i]->duty[0], db = motor[i]->duty[1], dc = motor[i]->duty[2];
            float alpha_v = 11.5f * (2.0f * da - db - dc) / 3.0f;
            float beta_v = 11.5f * (db - dc) / 1.73205080757f;
            phase_alpha[i] += 2.0f * (alpha_v - 0.12f * phase_alpha[i]);
            phase_beta[i] += 2.0f * (beta_v - generator_beta_voltage[i] - 0.12f * phase_beta[i]);
            out[i]->b += (-phase_alpha[i] + 1.73205080757f * phase_beta[i]) / 100.0f;
            out[i]->c += (-phase_alpha[i] - 1.73205080757f * phase_beta[i]) / 100.0f;
            continue;
        }
        if ((!plant && !locked_current) || motor[i]->state != FOC_RUN) continue;
        float theta = (float)cal[i].direction * 7.0f * shaft[i] * 0.01745329252f - cal[i].zero;
        float ia = -motor[i]->iq_ref * sinf(theta);
        float beta = motor[i]->iq_ref * cosf(theta);
        out[i]->b += (-ia + 1.73205080757f * beta) / 100.0f;
        out[i]->c += (-ia - 1.73205080757f * beta) / 100.0f;
    }
    return true;
}
bool dual_hw_read_bus(float *v) { *v = 11.5f; return true; }
void dual_hw_encoder_pair(float a[2]) { a[0] = shaft[1]; a[1] = shaft[0]; }
bool dual_hw_write(unsigned i, const float duty[3], unsigned mode) { (void)i; (void)duty; (void)mode; return true; }
bool bsp_usb_ready(void) { return false; }
bool bsp_usb_write(const void *p, size_t n) { (void)p; (void)n; return true; }
bool bsp_uart_write(const void *p, size_t n) { (void)p; (void)n; return true; }
uint32_t bsp_uart_millis(void) { return milliseconds; }
void bsp_uart_tick(void) {}
void bsp_usb_poll(void) {}
size_t bsp_uart_read(void *p, size_t n) { (void)p; (void)n; return 0; }
size_t bsp_usb_read(void *p, size_t n) { (void)p; (void)n; return 0; }

static void sample(void)
{
    if (plant) {
        foc_t *motor[2] = {&foc0, &foc1};
        for (unsigned i = 0; i < 2u; ++i) {
            float omega = plant_rpm[i] * 0.1047197551f;
            float drive = 1230.0f * (motor[i]->iq_ref * (float)cal[i].direction + load_current[i]) - 0.0654f * omega;
            if (fabsf(omega) < 1e-4f && fabsf(drive) <= 192.5f) omega = 0.0f;
            else omega += (drive - 192.5f * (omega > 0 ? 1.0f : omega < 0 ? -1.0f : copysignf(1.0f, drive))) * 0.0001f;
            plant_rpm[i] = omega / 0.1047197551f;
            shaft[i] = fmodf(shaft[i] + plant_rpm[i] * 0.0006f + 360.0f, 360.0f);
        }
    }
    motor0_sample_us = motor1_sample_us = (motor0_sample_us + 100u) & 0xffffffu;
    milliseconds += motor0_sample_us % 1000u == 0u;
    dual_app_sample();
}
static void settle(void) { for (unsigned n = 0; n < 4500u; ++n) sample(); }
static void move(unsigned i, float degrees)
{
    unsigned steps = (unsigned)ceilf(fabsf(degrees) / 0.005f);
    for (unsigned n = 0; n < steps; ++n) {
        shaft[i] += degrees / (float)steps;
        shaft[i] = fmodf(shaft[i] + 360.0f, 360.0f);
        sample();
    }
    for (unsigned n = 0; n < 200u; ++n) sample();
}

static bool command(const char *line)
{
    bool accepted = app_command(line);
    sample(); /* Queued targets apply after the current PWM writes. */
    return accepted;
}

int main(int argc, char **argv)
{
    CHECK(argc == 2 && app_init());
    CHECK(!command("m0 pos 1")); /* Offsets and angle are not ready yet. */
    settle();
    CHECK(foc0.state == FOC_IDLE && foc1.state == FOC_IDLE);
    CHECK(command("m0 zero") && command("m1 zero"));
    if (!strcmp(argv[1], "commands")) {
        CHECK(!command("m0 rpm 1") && !command("m1 Iq 0.1"));
        CHECK(!command("m0 pos 360") && !command("m1 pos -360"));
        CHECK(!command("m0 pos nan") && !command("m1 pos inf"));
        CHECK(app_command("m0 pos 85") && app_command("m1 pos -85"));
        sample();
        CHECK(armed == 3u && control0_mode() == CONTROL_POSITION && control1_mode() == CONTROL_POSITION);
        CHECK(!command("m0 pos 85.01") && !command("m1 pos -85.01"));
        CHECK(control0_position_target() == 85.0f && control1_position_target() == -85.0f);
        CHECK(!command("m0 zero"));
        sample();
        CHECK(foc0.state == FOC_PRECHARGE && foc1.state == FOC_PRECHARGE);
        for (unsigned n = 0; n < 10000u; ++n) sample();
        CHECK(foc0.state == FOC_RUN && foc1.state == FOC_RUN);
        CHECK(fabsf(foc0.iq_ref) <= FOC_GIMBAL_CURRENT_A + .00001f && fabsf(foc1.iq_ref) <= 8.00001f);
        CHECK(fabsf(control0_speed_target()) <= FOC_GIMBAL_SPEED_RPM + .00001f && fabsf(control1_speed_target()) <= 48.00001f);
        CHECK(foc0.iq_ref > 0.0f && foc1.iq_ref > 0.0f); /* Opposite calibrated direction. */
        CHECK(command("m0 stop") && foc0.state == FOC_IDLE && foc1.state == FOC_RUN);
        CHECK(command("stop") && off == 3u && foc1.state == FOC_IDLE);
    } else if (!strcmp(argv[1], "paired_commands")) {
        CHECK(!command("gimbal pos 10 85.01"));
        CHECK(!command("gimbal pos nan 0") && !command("gimbal pos 0 inf"));
        CHECK(!command("gimbal pos 1 2 3") && !command("gimbal pos 1"));
        CHECK(!command("gimbal pos 85.01 0"));
        CHECK(!command("gimbal pos 1.001 2") && !command("gimbal pos  1 2"));
        CHECK(foc0.state == FOC_IDLE && foc1.state == FOC_IDLE && armed == 0u);
        CHECK(app_command("gimbal pos 10.25 -20.50"));
        CHECK(control0_position_target() == 0 && control1_position_target() == 0);
        CHECK(!app_command("m0 pos 1") && !app_command("gimbal pos 0 0"));
        sample();
        CHECK(armed == 3u && control0_position_target() == 10.25f && control1_position_target() == -20.5f);
        CHECK(!command("gimbal pos 30 -86"));
        CHECK(control0_position_target() == 10.25f && control1_position_target() == -20.5f);
        CHECK(command("gimbal pos 85 85"));
        CHECK(app_command("gimbal pos -85 -85") && app_command("stop"));
        sample();
        CHECK(foc0.state == FOC_IDLE && foc1.state == FOC_IDLE);
        CHECK(app_command("gimbal pos 1 2") && app_command("m1 stop"));
        sample();
        CHECK(foc0.state == FOC_IDLE && foc1.state == FOC_IDLE);
        CHECK(app_command("gimbal pos 1 2"));
        foc1_trip(FOC_SENSOR); /* Fault between parse and commit must reject BOTH. */
        sample();
        CHECK(foc0.state == FOC_IDLE && control0_position_target() == 0);
    } else if (!strcmp(argv[1], "response")) {
        plant = true;
        CHECK(command("m0 pos 20") && command("m1 pos -15"));
        for (unsigned n = 0; n < 100000u; ++n) {
            sample();
            CHECK(foc0.fault == FOC_OK && foc1.fault == FOC_OK);
            CHECK(fabsf(foc0.iq_ref) <= FOC_GIMBAL_CURRENT_A + .00001f && fabsf(foc1.iq_ref) <= 8.00001f);
            CHECK(fabsf(control0_speed_target()) <= FOC_GIMBAL_SPEED_RPM + .00001f && fabsf(control1_speed_target()) <= 48.00001f);
        }
        printf("response: M0 pitch %.2f M1 yaw %.2f deg\n", (double)control0_position_deg(), (double)control1_position_deg());
        CHECK(fabsf(control0_position_deg() - 20.0f) < 1.0f);
        CHECK(fabsf(control1_position_deg() + 15.0f) < 1.0f);
        CHECK(command("m0 pos -10") && command("m1 pos 10"));
        for (unsigned n = 0; n < 100000u; ++n) {
            sample();
            CHECK(foc0.fault == FOC_OK && foc1.fault == FOC_OK);
        }
        CHECK(fabsf(control0_position_deg() + 10.0f) < 1.0f);
        CHECK(fabsf(control1_position_deg() - 10.0f) < 1.0f);
        CHECK(command("m0 pos 85") && command("m1 pos -85"));
        for (unsigned n = 0; n < 120000u; ++n) {
            sample();
            CHECK(foc0.fault == FOC_OK && foc1.fault == FOC_OK);
        }
        CHECK(fabsf(control0_position_deg() - 85.0f) < 1.0f);
        CHECK(fabsf(control1_position_deg() + 85.0f) < 1.0f);
    } else if (!strcmp(argv[1], "braking")) {
        CHECK(command("m1 pos 45"));
        for (unsigned n = 0; n < 1000u; ++n) sample();
        /* Simulate a released load carrying the shaft at 8.33 RPM.
           Reaching the target must not leave a
           forward speed request while a second slew ramp catches up. */
        move(1, 45.0f);
        printf("braking: position %.4f reference %.4f RPM\n", (double)control1_position_deg(), (double)control1_speed_reference());
        CHECK(foc1.fault == FOC_OK);
        CHECK(fabsf(control1_position_deg() - 45.0f) < 0.10f);
        CHECK(control1_speed_reference() <= 0.01f && fabsf(control1_speed_reference()) < 0.40f);
    } else if (!strcmp(argv[1], "speed_foldback")) {
        locked_current = true;
        CHECK(command("m1 pos 85"));
        for (unsigned n = 0; n < 7000u; ++n) sample();
        CHECK(foc1.iq_ref < -0.5f);
        /* Assert while moving; the move() helper also waits at rest. */
        const float excess_step = (FOC_GIMBAL_SPEED_RPM + FOC_GIMBAL_SPEED_FOLDBACK_RPM + 3.0f) * .0006f;
        for (unsigned n = 0; n < 500u; ++n) { shaft[1] += excess_step; sample(); }
        CHECK(foc1.fault == FOC_OK);
        CHECK(foc1.iq_ref >= -0.01f); /* No torque accelerating this motion. */
        CHECK(command("m1 pos -85"));
        for (unsigned n = 0; n < 7000u; ++n) sample();
        CHECK(foc1.iq_ref > 0.5f);
        for (unsigned n = 0; n < 500u; ++n) { shaft[1] -= excess_step; sample(); }
        CHECK(foc1.fault == FOC_OK && foc1.iq_ref <= 0.01f);
        CHECK(command("stop"));
        /* M0: a released load outruns the trajectory while its integral is
           still asking for accelerating torque. Cutting torque alone cannot
           oppose that load: require actual bounded braking in both directions. */
        CHECK(command("m0 pos 85"));
        for (unsigned n = 0; n < 7000u; ++n) sample();
        CHECK(foc0.iq_ref > 0.4f);
        for (unsigned n = 0; n < 500u; ++n) { shaft[0] += excess_step; sample(); }
        printf("M0 load-release brake: fault=%lu rpm=%.3f iq_ref=%.3f\n", (unsigned long)foc0.fault, (double)foc0.rpm, (double)foc0.iq_ref);
        CHECK(foc0.fault == FOC_OK && foc0.iq_ref < -0.45f && foc0.iq_ref >= -FOC_GIMBAL_CURRENT_A - .00001f);
        CHECK(command("m0 pos -85"));
        for (unsigned n = 0; n < 7000u; ++n) sample();
        CHECK(foc0.iq_ref < -0.4f);
        for (unsigned n = 0; n < 500u; ++n) { shaft[0] -= excess_step; sample(); }
        CHECK(foc0.fault == FOC_OK && foc0.iq_ref > 0.45f && foc0.iq_ref <= FOC_GIMBAL_CURRENT_A + .00001f);
    } else if (!strcmp(argv[1], "disturbance") || !strcmp(argv[1], "heavy_disturbance")) {
        bool heavy = !strcmp(argv[1], "heavy_disturbance");
        plant = true;
        CHECK(command("gimbal pos 0 0"));
        for (unsigned n = 0; n < 10000u; ++n) sample();
        for (unsigned direction = 0; direction < 2u; ++direction) {
            float load0 = heavy ? 1.5f : .30f;
            float load1 = heavy ? 1.5f : .50f;
            const float wanted_load0 = direction ? -load0 : load0;
            const float wanted_load1 = direction ? load1 : -load1;
            float peak[2] = {0, 0};
            for (unsigned n = 0; n < 30000u; ++n) {
                /* Finger pressure builds over time. A 1.5 A-equivalent
                   instantaneous step drives this low-inertia model through
                   20 RPM in milliseconds and must trip, not be permitted. */
                float ramp = heavy ? fminf((float)n / 1000.0f, 1.0f) : 1.0f;
                load_current[0] = wanted_load0 * ramp;
                load_current[1] = wanted_load1 * ramp;
                sample();
                if (foc0.fault || foc1.fault) {
                    printf("load fault at %.4f s: fault %u/%u, pos %.3f/%.3f, rpm %.3f/%.3f, iq_ref %.3f/%.3f\n",
                        (double)n * .0001, (unsigned)foc0.fault, (unsigned)foc1.fault,
                        (double)control0_position_deg(), (double)control1_position_deg(),
                        (double)foc0.rpm, (double)foc1.rpm, (double)foc0.iq_ref, (double)foc1.iq_ref);
                }
                CHECK(!foc0.fault && !foc1.fault);
                peak[0] = fmaxf(peak[0], fabsf(control0_position_deg()));
                peak[1] = fmaxf(peak[1], fabsf(control1_position_deg()));
                CHECK(fabsf(foc0.iq_ref) <= FOC_GIMBAL_CURRENT_A + .00001f);
                CHECK(fabsf(foc1.iq_ref) <= 8.00001f);
            }
            printf("load step %u: peak %.3f/%.3f residual %.3f/%.3f deg\n", direction,
                (double)peak[0], (double)peak[1], (double)control0_position_deg(), (double)control1_position_deg());
            CHECK(peak[0] < (heavy ? 8.0f : 2.0f) && peak[1] < (heavy ? 8.0f : 2.0f));
            CHECK(fabsf(control0_position_deg()) < .2f && fabsf(control1_position_deg()) < .2f);
            for (unsigned n = 0; n < 30000u; ++n) {
                float release = heavy ? fmaxf(1.0f - (float)n / 1000.0f, 0.0f) : 0.0f;
                load_current[0] = wanted_load0 * release;
                load_current[1] = wanted_load1 * release;
                sample(); CHECK(!foc0.fault && !foc1.fault);
            }
            CHECK(fabsf(control0_position_deg()) < .2f && fabsf(control1_position_deg()) < .2f);
        }
        CHECK(command("stop"));
        CHECK(foc0.iq_ref == 0 && foc1.iq_ref == 0);
    } else if (!strcmp(argv[1], "noisy_hold")) {
        locked_current = true;
        CHECK(command("gimbal pos 0 0"));
        const float start0 = shaft[0], start1 = shaft[1];
        /* A one-count-size oscillation produces >0.5 RPM in the 1 ms
           derivative while remaining inside the position settling window. */
        for (unsigned n = 0; n < 8000u; ++n) {
            float count_noise = ((n / 10u) & 1u) ? .0055f : -.0055f;
            shaft[0] = start0 + count_noise; shaft[1] = start1 - count_noise;
            sample(); CHECK(!foc0.fault && !foc1.fault);
        }
        move(0, .10f); move(1, -.10f);
        CHECK(fabsf(control0_speed_reference() + 8.0f * control0_position_deg()) < .01f);
        CHECK(fabsf(control1_speed_reference() + 6.0f * control1_position_deg()) < .01f);
    } else if (!strcmp(argv[1], "holding")) {
        locked_current = true;
        CHECK(command("gimbal pos 0 0"));
        for (unsigned n = 0; n < 4000u; ++n) sample();
        move(0, .10f); move(1, -.10f);
        CHECK(fabsf(control0_speed_reference() + 8.0f * control0_position_deg()) < .01f);
        CHECK(fabsf(control1_speed_reference() + 6.0f * control1_position_deg()) < .01f);
        /* Restoring current includes the direct position term, with M1's
           reversed electrical direction, rather than only the speed P term. */
        CHECK(foc0.iq_ref < -.080f && foc1.iq_ref < -.085f);
        CHECK(command("gimbal pos 0 0")); /* Identical resend retains fast hold. */
        for (unsigned n = 0; n < 100u; ++n) sample();
        CHECK(fabsf(control0_speed_reference() + 8.0f * control0_position_deg()) < .01f);
        CHECK(command("m0 pos 10")); /* New motion restores bounded acceleration. */
        for (unsigned n = 0; n < 100u; ++n) sample();
        CHECK(control0_speed_reference() >= 0 && control0_speed_reference() < 3.0f);
        CHECK(command("stop"));
        CHECK(foc0.iq_ref == 0 && foc1.iq_ref == 0);
    } else if (!strcmp(argv[1], "run_speed_trip")) {
        locked_current = true;
        CHECK(command("gimbal pos 0 0"));
        for (unsigned n = 0; n < 1000u; ++n) sample();
        CHECK(foc0.state == FOC_RUN && foc1.state == FOC_RUN);
        for (unsigned n = 0; n < 500u && !foc0.fault; ++n) {
            shaft[0] += (FOC_GIMBAL_SPEED_TRIP_RPM + 5) * .0006f;
            sample();
        }
        CHECK(foc0.fault == FOC_SPEED && foc0.state == FOC_FAULT);
        CHECK((off & 1u) && !foc1.fault && foc1.state == FOC_RUN);
        CHECK(command("stop"));
    } else if (!strcmp(argv[1], "zero")) {
        move(0, 40.0f);
        CHECK(fabsf(foc0_travel_deg() - 40.0f) < 0.15f);
        CHECK(!foc0.fault && command("m0 zero"));
        CHECK(!command("m0 pos 50"));
        CHECK(command("m0 pos 40") && command("stop"));
        move(1, 40.0f);
        CHECK(fabsf(foc1_travel_deg() - 40.0f) < 0.15f);
        CHECK(command("m1 zero"));
        CHECK(!command("m1 pos 50"));
        CHECK(command("m1 pos 40") && command("stop"));
        CHECK(command("m1 hold"));
        CHECK(fabsf(control1_position_target()) < 0.01f);
        CHECK(command("stop"));
        foc1_trip(FOC_BUS);
        CHECK(command("clear") && hardware_inits == 1u);
        settle();
        CHECK(fabsf(foc0_travel_deg() - 40.0f) < 0.15f);
        CHECK(!command("m0 pos 50"));
        CHECK(fabsf(foc1_travel_deg() - 40.0f) < 0.15f);
        CHECK(!command("m1 pos 50") && command("m1 pos -40"));
    } else if (!strcmp(argv[1], "travel")) {
        CHECK(command("all test"));
        for (unsigned n = 0; n < 600u; ++n) sample();
        move(1, 91.0f); /* Open-loop test is also subject to the physical limit. */
        CHECK(foc1.state == FOC_FAULT && foc1.fault == FOC_POSITION);
        CHECK(foc0.state != FOC_FAULT);
        CHECK(foc1.duty[0] == 0.0f && (off & 2u));
        CHECK(fabsf(control1_position_deg() - 91.0f) < 0.2f);
        CHECK(!command("m1 pos 0") && !command("m1 zero"));
        CHECK(!command("clear")); /* Other motor is still testing. */
        CHECK(command("stop") && command("clear"));
        sample();
        CHECK(foc1.state == FOC_FAULT); /* Clear does not re-centre the travel. */
        move(1, -91.0f);
        CHECK(command("clear"));
        settle();
        CHECK(foc1.state == FOC_IDLE && command("m1 pos 0"));
    } else if (!strcmp(argv[1], "m0_travel")) {
        CHECK(command("all test"));
        for (unsigned n = 0; n < 600u; ++n) sample();
        move(0, 91.0f);
        CHECK(foc0.state == FOC_FAULT && foc0.fault == FOC_POSITION);
        CHECK(!foc1.fault && foc0.duty[0] == 0.0f && (off & 1u));
        CHECK(!command("m0 pos 0") && !command("m0 zero"));
        CHECK(!command("clear"));
        CHECK(command("stop") && command("clear"));
        sample();
        CHECK(foc0.state == FOC_FAULT); /* Clear must preserve M0's centre. */
        move(0, -91.0f);
        CHECK(command("clear"));
        settle();
        CHECK(foc0.state == FOC_IDLE && command("m0 pos 0"));
    } else if (!strcmp(argv[1], "m0_cal_limit")) {
        CHECK(command("m0 cal"));
        for (unsigned n = 0; n < 600u; ++n) sample();
        move(0, 91.0f);
        CHECK(foc0.state == FOC_FAULT && foc0.fault == FOC_POSITION);
        CHECK(!foc1.fault && foc0.duty[0] == 0.0f);
        app_poll();
        CHECK(command("clear"));
        sample();
        CHECK(foc0.state == FOC_FAULT && foc0.fault == FOC_POSITION);
    } else if (!strcmp(argv[1], "cal_limit")) {
        CHECK(command("m1 cal"));
        for (unsigned n = 0; n < 600u; ++n) sample();
        move(1, 91.0f);
        CHECK(foc1.state == FOC_FAULT && foc1.fault == FOC_POSITION);
        CHECK(!foc0.fault && foc1.duty[0] == 0.0f);
        app_poll();
        CHECK(command("clear"));
        sample();
        CHECK(foc1.state == FOC_FAULT && foc1.fault == FOC_POSITION);
    } else if (!strcmp(argv[1], "negative_limit")) {
        move(0, -91.0f);
        CHECK(foc0.state == FOC_FAULT && foc0.fault == FOC_POSITION);
        move(1, -91.0f);
        CHECK(foc1.state == FOC_FAULT && foc1.fault == FOC_POSITION);
        CHECK(off == 3u);
    } else if (!strcmp(argv[1], "cal_speed")) {
        CHECK(command("m1 cal"));
        for (unsigned n = 0; n < 600u; ++n) sample();
        for (unsigned n = 0; n < 100u && foc1.state != FOC_FAULT; ++n) {
            shaft[1] += 0.025f;
            sample();
        }
        CHECK(foc1.state == FOC_FAULT && foc1.fault == FOC_SPEED);
        CHECK(foc1.duty[0] == 0.0f && foc0.fault == 0u);
        settle();
        app_poll();
        CHECK(command("clear"));
        settle();
        CHECK(command("m1 field 0"));
        for (unsigned n = 0; n < 600u; ++n) sample();
        for (unsigned n = 0; n < 100u && foc1.state != FOC_FAULT; ++n) {
            shaft[1] += 0.012f;
            sample();
        }
        CHECK(foc1.state == FOC_FAULT && foc1.fault == FOC_SPEED);
    } else if (!strcmp(argv[1], "diagnostic")) {
        CHECK(!command("m0 field 1"));
        CHECK(command("m0 field 90"));
        for (unsigned n = 0; n < 3500u; ++n) sample();
        CHECK(foc0.state == FOC_IDLE && !foc0.fault);
        CHECK(app_command("m0 pos 2") && app_command("m1 pos -2"));
        CHECK(app_command("stop"));
        sample();
        CHECK(foc0.state == FOC_IDLE && foc1.state == FOC_IDLE);
        CHECK(!command("m0 itest 0.31") && !command("m1 stest 2.01"));
        CHECK(command("m1 itest 0.2"));
        CHECK(!command("m1 itest 0.2") && !command("m1 pos 3"));
        CHECK(!command("m0 stest 1"));
        for (unsigned n = 0; n < 5000u; ++n) sample();
        CHECK(foc1.state == FOC_IDLE && foc1.iq_ref == 0.0f && !foc1.fault);
        CHECK(command("m0 stest 1"));
        for (unsigned n = 0; n < 16000u; ++n) sample();
        CHECK(foc0.state == FOC_IDLE && foc0.iq_ref == 0.0f);
        CHECK(command("m1 stest 1"));
        for (unsigned n = 0; n < 600u; ++n) sample();
        move(1, 11.0f);
        CHECK(foc1.state == FOC_IDLE && !foc1.fault);
    } else if (!strcmp(argv[1], "cal_settle")) {
        float seed = foc1.electrical_deg * 0.01745329252f;
        CHECK(command("m1 cal"));
        unsigned cal_tick = 0;
        for (unsigned elapsed = 0; elapsed < 120000u && cal_tick < 117000u; ++elapsed) {
            if (foc1.state == FOC_CALIBRATE) {
                ++cal_tick;
                float displacement = 0.0f;
                if (cal_tick > 15000u && cal_tick <= 55000u)
                    displacement = 40.0f * (float)(cal_tick - 15000u) / 40000.0f;
                else if (cal_tick > 55000u && cal_tick <= 65000u)
                    displacement = 40.0f + (360.0f / 7.0f - 40.0f) * (float)(cal_tick - 55000u) / 10000.0f;
                else if (cal_tick > 65000u && cal_tick <= 105000u)
                    displacement = (360.0f / 7.0f) * (1.0f - (float)(cal_tick - 65000u) / 40000.0f);
                shaft[1] = 5.0f + displacement;
            }
            sample();
            CHECK(foc1.fault == FOC_OK && foc0.state == FOC_IDLE);
            CHECK(cal_tick < 117000u || foc1.state == FOC_SAVE);
        }
        CHECK(cal_tick == 117000u && foc1.state == FOC_SAVE && foc1.calibration.direction == 1);
        float expected_zero = fmodf(5.0f * 7.0f * 0.01745329252f - seed + 6.28318530718f, 6.28318530718f);
        CHECK(fabsf(foc1.calibration.zero - expected_zero) < .001f);
    } else if (!strcmp(argv[1], "cal_damping")) {
        electrical_plant = true;
        foc1.calibrated = false; /* No seed record: stator field starts at zero. */
        CHECK(command("m1 cal"));
        for (unsigned n = 0; n < 600u; ++n) sample();
        generator_beta_voltage[1] = 0.03f; /* Rotor EMF in the fixed field. */
        for (unsigned n = 0; n < 2000u; ++n) sample();
        CHECK(foc1.state == FOC_CALIBRATE && foc1.fault == FOC_OK);
        CHECK(phase_alpha[1] > 0.10f);
        CHECK(phase_beta[1] < -0.20f); /* Generator current provides damping. */
    } else if (!strcmp(argv[1], "field_tracking")) {
        electrical_plant = true;
        const char *fields[] = {"m0 field 0", "m0 field 90", "m0 field 180", "m0 field 270"};
        for (unsigned k = 0; k < 4u; ++k) {
            CHECK(command(fields[k]));
            for (unsigned n = 0; n < 2000u; ++n) sample();
            float angle = (float)k * 1.57079632679f;
            CHECK(fabsf(phase_alpha[0] - .2f * cosf(angle)) < .005f);
            CHECK(fabsf(phase_beta[0] - .2f * sinf(angle)) < .005f);
            CHECK(!foc0.fault && foc0.state == FOC_CALIBRATE);
            for (unsigned n = 0; n < 1500u; ++n) sample();
            CHECK(foc0.state == FOC_IDLE && foc0.duty[0] == 0.0f);
        }
    } else if (!strcmp(argv[1], "sensor")) {
        shaft[0] += 90.0f;
        sample();
        CHECK(foc0.state == FOC_FAULT && foc0.fault == FOC_SENSOR);
        CHECK(!foc0_travel_ready());
        CHECK(command("clear"));
        settle();
        CHECK(foc0.state == FOC_FAULT && !command("m0 pos 0"));
    } else if (!strcmp(argv[1], "protection")) {
        CHECK(command("m0 pos 10"));
        for (unsigned n = 0; n < 100u; ++n) sample();
        foc0_step(shaft[0], 15.0f, 1.65f, 1.65f, 0.0f);
        CHECK(foc0.state == FOC_FAULT && foc0.fault == FOC_BUS);
        sample();
        CHECK(foc0.state == FOC_FAULT); /* WARN cannot auto-resume the gimbal. */
        CHECK(command("stop") && command("clear"));
        settle();
        CHECK(command("m0 pos 10"));
        for (unsigned n = 0; n < 100u; ++n) sample();
        foc0_step(shaft[0], 11.5f, 2.0f, 1.65f, 0.0f);
        CHECK(foc0.state == FOC_FAULT && foc0.fault == FOC_CURRENT);
    } else CHECK(false);
    printf("PASS gimbal %s policy=%s\n", argv[1], FOC_POLICY_NAME);
    return 0;
}
