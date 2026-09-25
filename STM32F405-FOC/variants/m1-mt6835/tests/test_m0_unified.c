/* Host check for the default M0 profile using the real app and control code. */
#include "app.h"
#include "bsp_adc.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "bsp_uart.h"
#include "bsp_usb.h"
#include "control.h"
#include "foc.h"
#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

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
static float last_frame[13];
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
    assert(size == sizeof last_frame);
    memcpy(last_frame, data, size);
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
    assert(FOC_CURRENT_MAX == 0.30f && FOC_AUTOCALIBRATE == 0);
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
    assert(foc.state == FOC_RUN && fabsf(foc.iq_ref - 0.010f) < 0.0002f);
    assert(frames == samples && isinf(last_frame[12]));
    usb("rpm 10\r");
    assert(control_mode() == CONTROL_SPEED);
    usb("pos 90\r");
    assert(control_mode() == CONTROL_POSITION);
    usb("stop\r");
    assert(foc.state == FOC_IDLE && motor_mode == MOTOR_OFF);
    puts("PASS: M0 profile, USB parser/telemetry, explicit calibration, torque/speed/position");
    return 0;
}
