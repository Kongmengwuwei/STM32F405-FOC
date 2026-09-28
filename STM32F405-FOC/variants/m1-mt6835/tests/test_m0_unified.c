/* Host check for the default M0 profile using the real app and control code. */
#include "app.h"
#include "bsp_adc.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "bsp_uart.h"
#include "bsp_usb.h"
#include "control.h"
#include "telemetry.h"
#include "foc.h"
#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef _WIN32
/* A failed console test must report to stderr instead of opening a CRT modal. */
#undef assert
#define assert(condition) do { if (!(condition)) { \
    fprintf(stderr, "Assertion failed: %s:%d: %s\n", __FILE__, __LINE__, #condition); \
    exit(EXIT_FAILURE); } } while (0)
#endif

volatile bsp_uart_stats_t g_uart_stats;
volatile bsp_usb_stats_t g_usb_stats;
volatile bsp_adc_sample_t adc_sample = {1.565f, 1.565f, 12.0f};
volatile uint16_t adc_raw_b = 2048u, adc_raw_c = 2048u, adc_raw_bus = 840u;
volatile float encoder_angle_deg = 20.0f, encoder_raw_deg = 20.0f, encoder_sample_delay;
volatile uint32_t encoder_errors;
volatile float motor_duty[3];
volatile unsigned motor_mode;
volatile uint32_t motor_sample_us;
static const char *input;
static unsigned frames;
static float last_frame[TELEMETRY_OVERVIEW_CHANNELS + 1u];
static size_t last_frame_size;
extern volatile uint32_t app_command_rejected;

uint32_t bsp_motor_lock(void) { return 0u; }
void bsp_motor_unlock(uint32_t key) { (void)key; }
void bsp_motor_arm(void) {}
void bsp_motor_off(void) { motor_mode = MOTOR_OFF; }
bool bsp_motor_load(foc_calibration_t *cal) { (void)cal; return false; }
bool bsp_motor_save(const foc_calibration_t *cal) { (void)cal; return true; }
bool bsp_motor_write(const float duty[3], unsigned mode) { (void)duty; motor_mode = mode; return true; }
void bsp_motor_sample_end(void) {}
void bsp_adc_start(void) {}
void bsp_adc_stop(void) {}
bool bsp_encoder_init(void) { return true; }
void bsp_encoder_stop(void) {}
bool bsp_uart_init(void) { return true; }
bool bsp_can_init(void) { return true; }
uint32_t bsp_uart_millis(void) { return motor_sample_us / 1000u; }
void bsp_uart_tick(void) {}
bool bsp_uart_write(const void *data, size_t size) { (void)data; return size > 0u; }
size_t bsp_uart_read(void *data, size_t size) { (void)data; (void)size; return 0u; }
bool bsp_usb_ready(void) { return true; }
void bsp_usb_poll(void) {}
bool bsp_usb_write(const void *data, size_t size)
{
    assert(size == sizeof last_frame || size == 13u * sizeof(float));
    last_frame_size = size;
    memcpy(last_frame, data, size);
    if (size == sizeof last_frame) {
        assert(isinf(last_frame[TELEMETRY_OVERVIEW_CHANNELS]));
        assert(last_frame[TELEMETRY_IQ_TARGET] == foc.iq_ref);
        assert(last_frame[TELEMETRY_IQ_ACTUAL] == foc.iq);
        assert(last_frame[TELEMETRY_SPEED_ACTUAL] == control_speed_rpm());
        assert(last_frame[TELEMETRY_POSITION_ACTUAL] == control_position_deg());
        assert(last_frame[TELEMETRY_STATE] == foc.state);
        ++frames;
        return true;
    }
    uint32_t index;
    memcpy(&index, &last_frame[1], sizeof index);
    if ((index >> 24) == 4u) {
        assert(last_frame[2] == foc.iq_ref && last_frame[9] == foc.iq);
    }
    ++frames;
    return true;
}
size_t bsp_usb_read(void *data, size_t size)
{
    size_t n = input ? strlen(input) : 0u;
    if (n > size) n = size;
    if (n) { memcpy(data, input, n); input += n; }
    return n;
}
static void usb(const char *command)
{
    input = command;
    do { app_poll(); } while (*input);
}

int main(void)
{
    assert(FOC_SAMPLE_HZ == 10000u && FOC_PWM_ARR == 8400u);
    assert(FOC_CURRENT_MAX == 5.00f && FOC_PHASE_TRIP == 6.00f && FOC_AUTOCALIBRATE == 0);
    assert(app_init() && foc.state == FOC_OFFSET && !foc.calibrated);
    for (unsigned n = 0; n < FOC_OFFSET_WAIT_SAMPLES + 2048u; ++n) {
        foc_step(20.0f, 12.0f, 1.565f, 1.565f, 0.0f);
        motor_sample_us += 100u;
    }
    assert(foc.state == FOC_IDLE && foc.zero_ready && !foc.calibrated);
    usb("Iq 0.20\r");
    assert(app_command_rejected == 1u && foc.state == FOC_IDLE);
    usb("cal\r");
    assert(foc.state == FOC_PRECHARGE);
    usb("stop\r");
    assert(foc.state == FOC_IDLE);
    unsigned before = frames;
    for (unsigned n = 0; n < FOC_USB_DIVIDER * TELEMETRY_OVERVIEW_DECIMATION; ++n) {
        motor_sample_us += 100u; app_sample();
    }
    assert(frames == before + 1u && last_frame_size == sizeof last_frame);
    assert(last_frame[TELEMETRY_ID_TARGET] == 0.0f);
    assert(last_frame[TELEMETRY_POSITION_ERROR] == control_position_target() - control_position_deg());
    assert(app_command("send 0"));
    for (unsigned n = 0; n < FOC_USB_DIVIDER; ++n) { motor_sample_us += 100u; app_sample(); }
    assert(last_frame_size == 13u * sizeof(float));
    assert(app_command("send 6"));
    for (unsigned n = 0; n < FOC_USB_DIVIDER * 2u; ++n) { motor_sample_us += 100u; app_sample(); }
    assert(last_frame_size == sizeof last_frame);
    assert(!app_command("send 7"));
    frames = 0u;
    foc.calibrated = true; /* Stand in for a completed, profile-matched record. */
    foc.calibration.direction = 1;
    foc.calibration.zero = 0.0f;
    usb("send 3\rIq 0.20\r");
    assert(foc.state == FOC_PRECHARGE && control_mode() == CONTROL_TORQUE);
    const unsigned samples = 120u + FOC_PWM_ZERO_SETTLE_SAMPLES + FOC_PWM_ZERO_SAMPLES;
    for (unsigned n = 0; n < samples; ++n) {
        motor_sample_us += 100u;
        app_sample();
    }
    assert(foc.state == FOC_RUN);
    assert(fabsf(foc.iq_ref - (FOC_PROTECTION_TRIP ? 0.010f : 0.20f)) < 0.0002f);
    assert(frames == samples / FOC_USB_DIVIDER && isinf(last_frame[12]));
    usb("send 5\r");
    for (unsigned n = 0; n < FOC_USB_DIVIDER; ++n) {
        motor_sample_us += 100u;
        app_sample();
    }
    uint32_t group_word;
    memcpy(&group_word, &last_frame[1], sizeof group_word);
    assert(group_word >> 24 == 5u && foc.voltage_scale == 1.0f);
    float expected_iq = 0.0f, expected_id = 0.0f;
    for (unsigned n = 0; n < FOC_USB_DIVIDER; ++n) {
        adc_sample.b_voltage = foc.b_offset + (n & 1u ? 0.01f : -0.01f);
        motor_sample_us += 100u; app_sample();
        expected_iq += foc.iq; expected_id += foc.id;
    }
    assert(fabsf(last_frame[9] - expected_iq / FOC_USB_DIVIDER) < 1e-5f);
    assert(fabsf(last_frame[4] - expected_id / FOC_USB_DIVIDER) < 1e-5f);
    if (FOC_USB_DIVIDER == 2u) assert(fabsf(last_frame[9] - foc.iq) > 0.01f);
    adc_sample.b_voltage = foc.b_offset;
    usb("rpm 10\r");
    assert(control_mode() == CONTROL_SPEED);
    usb("pos 90\r");
    assert(control_mode() == CONTROL_POSITION);
    usb("hold\r");
    assert(control_mode() == CONTROL_POSITION);
    assert(control_position_target() == control_position_deg());
    usb("stop\r");
    assert(foc.state == FOC_IDLE && motor_mode == MOTOR_OFF);
#if !FOC_PROTECTION_TRIP
    /* The real parser must accept a formerly rejected target, and one send
       must survive timeout, bus/speed/phase-current diagnostic thresholds. */
    unsigned rejected = app_command_rejected;
    usb("send 4\rIq 5.20\r");
    assert(app_command_rejected == rejected && foc.command == 5.20f);
    for (unsigned n = 0; n < FOC_PRECHARGE_SAMPLES +
         FOC_PWM_ZERO_SETTLE_SAMPLES + FOC_PWM_ZERO_SAMPLES + 1u; ++n) {
        motor_sample_us += 100u;
        app_sample();
    }
    assert(foc.state == FOC_RUN && foc.iq_ref == 5.20f);
    adc_sample.bus_voltage = 20.0f;
    adc_sample.b_voltage += 0.04f; /* 2 A on the nominal B-phase conversion. */
    for (unsigned n = 0; n < 2600u; ++n) {
        encoder_angle_deg = fmodf(encoder_angle_deg + 2.0f, 360.0f);
        motor_sample_us += 100u;
        app_sample();
    }
    assert(foc.state == FOC_RUN && foc.fault == FOC_OK && control_scheduled());
    assert(foc.iq_ref == 5.20f && foc.command == 5.20f);
    assert(!(foc.warnings & (1u << FOC_UART))); /* Persistent target needs no keepalive. */
    assert(foc.warnings & (1u << FOC_CURRENT));
    assert(foc.warnings & (1u << FOC_SPEED));
    assert(foc.warnings & (1u << FOC_BUS));
    assert(foc.warnings & (1u << FOC_SENSOR));
    assert(last_frame[2] == foc.iq_ref); /* Iq may change on an unlogged cycle. */
    assert(last_frame[4] == (float)foc.warnings);
    assert(last_frame[6] == FOC_RUN && last_frame[10] == FOC_OK);
    assert(app_command("Iq -0.65"));
    motor_sample_us += 100u; app_sample();
    assert(fabsf(foc.iq_ref + 0.65f) < 1e-6f && foc.fault == FOC_OK);
    assert(app_command("clear") && foc.warnings == 0u && foc.state == FOC_RUN);
    assert(foc.command == -0.65f); /* clear changes diagnostics, not torque. */
    assert(app_command("stop") && motor_mode == MOTOR_OFF && foc.state == FOC_IDLE);

    /* Missing measurements cannot commutate. Recovery requires a new target,
       but never the old profile's latched clear interlock. */
    encoder_angle_deg = NAN;
    motor_sample_us += 100u; app_sample();
    assert(foc.state == FOC_FAULT && foc.fault == FOC_SENSOR && motor_mode == MOTOR_OFF);
    encoder_angle_deg = 20.0f;
    adc_sample.bus_voltage = 12.0f;
    adc_sample.b_voltage = foc.b_offset;
    motor_sample_us += 100u; app_sample();
    assert(foc.state == FOC_IDLE && foc.fault == FOC_OK && foc.command == 0.0f);
    assert(app_command("Iq 0.50") && foc.state == FOC_PRECHARGE);
    assert(app_command("stop"));
    foc.state = FOC_RUN;
    assert(app_command("Iq 50"));
    motor_sample_us += 100u; app_sample();
    assert(foc.state == FOC_RUN && foc_window(foc.duty));
    assert(hypotf(foc.ud, foc.uq) > adc_sample.bus_voltage * 0.10f);
    assert(foc.warnings & (1u << FOC_VOLTAGE));
    adc_sample.bus_voltage = 0.0f;
    motor_sample_us += 100u; app_sample();
    assert(foc.state == FOC_FAULT && foc.fault == FOC_BUS && motor_mode == MOTOR_OFF);
    adc_sample.bus_voltage = 12.0f;
    motor_sample_us += 100u; app_sample();
    assert(foc.state == FOC_IDLE && foc.fault == FOC_OK);
    adc_sample.b_voltage = NAN;
    motor_sample_us += 100u; app_sample();
    assert(foc.state == FOC_FAULT && foc.fault == FOC_ADC && motor_mode == MOTOR_OFF);
    adc_sample.b_voltage = foc.b_offset;
    motor_sample_us += 100u; app_sample();
    assert(foc.state == FOC_IDLE && foc.fault == FOC_OK);
    foc.state = FOC_RUN;
    assert(app_command("Iq 10000000000000000000000000"));
    motor_sample_us += 100u; app_sample();
    assert(foc.state == FOC_FAULT && foc.fault == FOC_NUMERIC && motor_mode == MOTOR_OFF);
    motor_sample_us += 100u; app_sample();
    assert(foc.state == FOC_IDLE && foc.fault == FOC_OK);
    uint32_t warnings = foc.warnings;
    app_fault(FOC_UART); app_fault(FOC_TIMING);
    assert(foc.state == FOC_IDLE && foc.fault == FOC_OK && foc.warnings != warnings);
    rejected = app_command_rejected;
    usb("Iq nan\rIq 0.123\r");
    assert(app_command_rejected == rejected + 2u && foc.command == 0.0f);
    puts("PASS: WARN over-limit targets, held single commands, no threshold latch, diagnostics, invalid-sensor recovery");
#endif
    puts("PASS: M0 profile, USB parser/telemetry, explicit calibration, torque/speed/position");
    return 0;
}
