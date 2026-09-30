#include "dual.h"
#include "app.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "bsp_usb.h"
#include "stm32f4xx_hal.h"
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static volatile int calibration_active = -1;
static volatile uint32_t rejected;
static uint32_t usb_session;

bool app_init(void)
{
    if (!bsp_uart_init() || !bsp_can_init() || !dual_hw_init()) return false;
    foc_calibration_t cal;
    foc0_init(dual_record_load(0u, &cal) ? &cal : NULL);
    foc1_init(dual_record_load(1u, &cal) ? &cal : NULL);
    dual_hw_start();
    return true;
}

void app_abort(uint32_t fault)
{
    dual_fault = fault;
    dual_hw_halt();
    foc0_trip(fault); foc1_trip(fault);
}
void app_fault(uint32_t fault) { app_abort(fault); }
void app_sample(void) { dual_app_sample(); }

static void publish(unsigned i, foc_t *f, const dual_adc_t *a, float angle)
{
    unsigned before = f->state;
    if (!isfinite(angle)) {
        if (i) foc1_trip(FOC_SENSOR); else foc0_trip(FOC_SENSOR);
    } else if (i) foc1_step(angle, a->bus, a->b, a->c, 0.0f);
    else foc0_step(angle, a->bus, a->b, a->c, 0.0f);
    if (before == FOC_OFFSET && f->state == FOC_PRECHARGE) dual_hw_arm(i);
    unsigned mode = f->state == FOC_PRECHARGE ? 1u :
        (f->state == FOC_RUN || f->state == FOC_CALIBRATE || f->state == FOC_PWM_ZERO) ? 2u : 0u;
    if (mode == 0u) dual_hw_off(i);
    else if (mode == 2u && !(i ? foc1_window(f->duty) : foc0_window(f->duty))) {
        app_fault(FOC_WINDOW); return;
    } else if (!dual_hw_write(i, f->duty, mode)) { app_fault(FOC_TIMING); return; }
    if (i) foc1_outer_step(); else foc0_outer_step();
}

void dual_app_sample(void)
{
    if (dual_fault) return;
    dual_adc_t a0, a1;
    if (!dual_hw_read_adc(&a0, &a1)) { app_fault(FOC_ADC); return; }
    float angle[2];
    dual_hw_encoder_pair(angle);
    float bus;
    if (!dual_hw_read_bus(&bus)) { app_fault(FOC_ADC); return; }
    a0.bus = a1.bus = bus;
    /* SPI1/PA1 now belongs to board M0; SPI3/PA0 belongs to board M1. */
    publish(0u, &foc0, &a0, angle[1]);
    if (dual_fault) return;
    publish(1u, &foc1, &a1, angle[0]);
    if (dual_fault) return;
    ++dual_sequence;
    if (dual_sequence % (FOC_USB_DIVIDER * 5u) == 0u) {
        /* Two independent 12-channel snapshots, M0 first, then M1. */
        float frame[25] = {
            control0_speed_target(), control0_speed_rpm(),
            control0_position_target(), control0_position_deg(),
            foc0.iq_ref, foc0.iq, foc0.id, bus,
            (float)control0_mode(), (float)foc0.state, (float)foc0.fault, (float)foc0.warnings,
            control1_speed_target(), control1_speed_rpm(),
            control1_position_target(), control1_position_deg(),
            foc1.iq_ref, foc1.iq, foc1.id, bus,
            (float)control1_mode(), (float)foc1.state, (float)foc1.fault, (float)foc1.warnings,
            INFINITY
        };
        if (bsp_usb_ready()) (void)bsp_usb_write(frame, sizeof frame);
        bsp_uart_tick();
    }
}

static bool number(const char *s, float *out)
{
    const char *p = s;
    if (*p == '-' || *p == '+') ++p;
    if (*p < '0' || *p > '9') return false;
    while (*p >= '0' && *p <= '9') ++p;
    if (*p == '.') {
        unsigned digits = 0u;
        while (*++p >= '0' && *p <= '9') ++digits;
        if (digits < 1u || digits > 2u) return false;
    }
    if (*p) return false;
    char *end;
    float value = strtof(s, &end);
    if (end != p || !isfinite(value) || fabsf(value) > FLT_MAX) return false;
    *out = value;
    return true;
}

static bool command_for(unsigned i, const char *cmd)
{
    foc_t *f = i ? &foc1 : &foc0;
    foc_t *other = i ? &foc0 : &foc1;
    if (!strcmp(cmd, "stop")) {
        dual_hw_off(i);
        if (i) foc1_stop(); else foc0_stop();
        if (calibration_active == (int)i) calibration_active = -1;
        return true;
    }
    if (dual_fault) return false;
    if (calibration_active >= 0 && calibration_active != (int)i) return false;
    if (!strcmp(cmd, "test")) {
        if (f->state != FOC_IDLE) return false;
        bool ok = i ? foc1_test() : foc0_test();
        if (ok) dual_hw_arm(i);
        return ok;
    }
    if (!strcmp(cmd, "cal")) {
        if (calibration_active >= 0 || other->state != FOC_IDLE ||
            f->state != FOC_IDLE) return false;
        bool ok = i ? foc1_calibrate() : foc0_calibrate();
        if (ok) { calibration_active = (int)i; dual_hw_arm(i); }
        return ok;
    }
    if (!strcmp(cmd, "zero")) return i ? control1_zero() : control0_zero();
    if (!strcmp(cmd, "hold")) {
        bool ok = i ? control1_hold_position() : control0_hold_position();
        if (ok) dual_hw_arm(i);
        return ok;
    }
    float value;
    bool ok;
    if (!strncmp(cmd, "Iq ", 3u) && number(cmd + 3, &value))
        ok = i ? control1_torque(value) : control0_torque(value);
    else if (!strncmp(cmd, "rpm ", 4u) && number(cmd + 4, &value))
        ok = i ? control1_speed(value) : control0_speed(value);
    else if (!strncmp(cmd, "pos ", 4u) && number(cmd + 4, &value))
        ok = i ? control1_position(value) : control0_position(value);
    else return false;
    if (ok && f->state == FOC_PRECHARGE) dual_hw_arm(i);
    return ok;
}

bool app_command(const char *line)
{
    if (!strcmp(line, "stop") || !strcmp(line, "all stop")) {
        uint32_t key = __get_PRIMASK(); __disable_irq();
        dual_hw_off_all(); foc0_stop(); foc1_stop(); calibration_active = -1;
        __set_PRIMASK(key);
        return true;
    }
    if (!strcmp(line, "all test")) {
        uint32_t key = __get_PRIMASK(); __disable_irq();
        bool ok = !dual_fault && calibration_active < 0 &&
                  foc0.state == FOC_IDLE && foc1.state == FOC_IDLE &&
                  foc0_test() && foc1_test();
        if (ok) { dual_hw_arm(0u); dual_hw_arm(1u); }
        else { dual_hw_off_all(); foc0_stop(); foc1_stop(); }
        __set_PRIMASK(key);
        return ok;
    }
    if (!strcmp(line, "hello")) {
        char banner[192];
        int n = snprintf(banner, sizeof banner,
            "#FOC dual 10k M0=SPI1/PA1:%lu/%lu M1=SPI3/PA0:%lu/%lu fault=%lu spi_pair_max=%lu cycles; use m0/m1 cal|test|Iq|rpm|pos|hold|zero|stop or all test\r\n",
            (unsigned long)foc0.state, (unsigned long)foc0.fault,
            (unsigned long)foc1.state, (unsigned long)foc1.fault,
            (unsigned long)dual_fault, (unsigned long)dual_encoder_cycles_max);
        if (n > 0 && (size_t)n < sizeof banner) (void)bsp_uart_write(banner, (size_t)n);
        bsp_uart_tick();
        return true;
    }
    if (!strcmp(line, "clear")) {
        if (!dual_fault) {
            uint32_t key = __get_PRIMASK(); __disable_irq();
            foc0_clear_warnings(); foc1_clear_warnings();
            __set_PRIMASK(key);
            return true;
        }
        if (calibration_active >= 0) return false;
        dual_hw_halt();
        if (!dual_hw_init()) return false;
        foc_calibration_t c0 = foc0.calibration, c1 = foc1.calibration;
        bool have0 = foc0.calibrated, have1 = foc1.calibrated;
        foc0_init(have0 ? &c0 : NULL); foc1_init(have1 ? &c1 : NULL);
        dual_fault = 0u; dual_hw_start();
        return true;
    }
    unsigned motor = 0u;
    if (!strncmp(line, "m0 ", 3u)) line += 3;
    else if (!strncmp(line, "m1 ", 3u)) { motor = 1u; line += 3; }
    uint32_t key = __get_PRIMASK(); __disable_irq();
    bool ok = command_for(motor, line);
    __set_PRIMASK(key);
    return ok;
}

typedef struct { char line[40]; unsigned length; bool overflow; } line_rx_t;
static void receive(line_rx_t *rx, uint8_t ch)
{
    if (ch == '\r' || ch == '\n') {
        rx->line[rx->length] = 0;
        if (rx->overflow || (rx->length && !app_command(rx->line))) ++rejected;
        rx->length = 0u; rx->overflow = false;
    } else if (ch >= 32u && ch < 127u && rx->length < sizeof rx->line - 1u)
        rx->line[rx->length++] = (char)ch;
    else rx->overflow = true;
}

void app_poll(void)
{
    static line_rx_t uart, usb;
    static uint32_t rx_errors;
    bsp_usb_poll();
    uint32_t errors = g_uart_stats.rx_errors + g_uart_stats.rx_lost;
    if (errors != rx_errors) {
        rx_errors = errors;
        uart.overflow = true;
        foc0_warn(FOC_UART); foc1_warn(FOC_UART);
        if (FOC_PROTECTION_TRIP && (foc0.state == FOC_RUN || foc1.state == FOC_RUN))
            app_fault(FOC_UART);
    }
    if (usb_session != g_usb_stats.sessions) {
        usb_session = g_usb_stats.sessions; usb.length = 0u; usb.overflow = false;
    }
    uint8_t ch;
    while (bsp_uart_read(&ch, 1u)) receive(&uart, ch);
    uint8_t packet[64];
    size_t n = bsp_usb_read(packet, sizeof packet);
    for (size_t i = 0u; i < n; ++i) receive(&usb, packet[i]);
    if (calibration_active >= 0) {
        unsigned i = (unsigned)calibration_active;
        foc_t *f = i ? &foc1 : &foc0;
        if (f->state == FOC_FAULT || f->state == FOC_IDLE) calibration_active = -1;
        else if (f->state == FOC_SAVE) {
            /* Flash programming stops both timers. The other motor stayed idle. */
            dual_hw_halt();
            bool ok = dual_record_save(i, &f->calibration);
            if (!ok) { app_abort(FOC_FLASH); calibration_active = -1; return; }
            f->calibrated = true; f->state = FOC_IDLE;
            calibration_active = -1;
            if (!dual_hw_init()) { app_abort(FOC_SENSOR); return; }
            dual_hw_start();
        }
    }
}
