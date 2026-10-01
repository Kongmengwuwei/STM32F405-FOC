#include "dual.h"
#include "tle5012b_protocol.h"
#include "stm32f4xx_hal.h"
#include <math.h>

#define GATES (TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE)
#define ADC_SCALE (3.13f / 4095.0f)
enum { OFF, PRECHARGE, PWM };
typedef struct { volatile unsigned mode, pending; volatile bool ready, inhibited; } motor_hw_t;
static motor_hw_t motors[2];
static volatile uint32_t encoder_errors[2];
volatile uint32_t dual_encoder_cycles_max;
volatile uint32_t dual_init_spi_valid, dual_init_crc_valid;
volatile uint32_t dual_init_first_crc_valid, dual_init_attempts;
volatile uint16_t dual_init_status[2], dual_init_safety[2];
/* SPI lane order: M1, M0. Learned only from CRC-valid startup responses,
   confirmed on three consecutive frames, then immutable while sampling. */
volatile uint32_t dual_sensor_number[2];
/* Startup-only diagnostics: index = 2 * SPI lane + CS pin (0=PA0, 1=PA1). */
volatile uint32_t dual_probe_spi_valid, dual_probe_crc_valid;
volatile uint16_t dual_probe_status[4], dual_probe_safety[4];
volatile uint32_t dual_after_probe_crc[8];
#ifdef FOC_ENCODER_DIAGNOSTIC_ONLY
/* Opt-in bench build: no ADC acquisition or motor command processing starts.
 * Raw arrays use [rate][SPI lane][sample][data/safety]; samples alternate
 * STAT (0x8001) and ANGLE (0x8021). SPI3 stays at its normal clock rate. */
volatile uint32_t encoder_diag_complete;
volatile uint32_t encoder_diag_hz[7];
volatile uint32_t encoder_diag_spi_valid[7][2];
volatile uint16_t encoder_diag_raw[7][2][32][2];
volatile uint32_t encoder_diag_long_samples;
volatile uint32_t encoder_diag_long_crc_good[2], encoder_diag_long_angle_good[2];
volatile uint32_t encoder_diag_long_min[2], encoder_diag_long_max[2];
#endif
volatile uint32_t dual_angle_spi_valid, dual_angle_sample_valid;
volatile uint16_t dual_angle_data[2], dual_angle_safety[2];
volatile uint32_t dual_sensor_status_captured;
volatile uint16_t dual_sensor_status[2], dual_sensor_status_safety[2];
volatile uint32_t dual_sensor_error_state[2], dual_sensor_error_mode[2];
volatile uint32_t dual_sensor_error_timer[2], dual_sensor_error_time_us[2];
volatile uint32_t dual_first_active_bad_captured, dual_first_active_bad_spi;
volatile uint16_t dual_first_active_bad_data[2], dual_first_active_bad_safety[2];
volatile uint32_t dual_first_active_bad_state[2], dual_first_active_bad_mode[2];
volatile uint32_t dual_first_bad_pa_odr, dual_first_bad_pa_idr, dual_first_bad_pa_moder;
volatile uint32_t dual_first_bad_pb_idr, dual_first_bad_pb_moder, dual_first_bad_pb_afrl;
volatile uint32_t dual_first_bad_spi1_cr1, dual_first_bad_spi1_sr;
volatile uint32_t motor0_sample_us, motor1_sample_us;
volatile uint32_t dual_sequence, dual_fault;
volatile uint32_t dual_timing_site, dual_timing_counter, dual_timing_direction;
volatile uint32_t dual_timing_reason, dual_timing_ready, dual_timing_mode;
volatile uint32_t dual_timing_pending, dual_timing_foc_state[2], dual_timing_sequence;
/* Motor order M0/M1. Downcount write minima and upcount update maxima expose
   the measured margin to the unchanged 600-tick deadlines while armed. */
volatile uint32_t dual_write_counter_min[2], dual_update_counter_max[2];
static uint32_t last_sample;

static TIM_TypeDef *timer(unsigned i) { return i ? TIM8 : TIM1; }

void dual_hw_safe_pins(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /* CS0/CS1 remain independent GPIO outputs and default deselected. */
    GPIOA->BSRR = GPIO_PIN_0 | GPIO_PIN_1;
    GPIOA->MODER = (GPIOA->MODER & ~15u) | 5u;
    GPIOA->OTYPER &= ~(GPIO_PIN_0 | GPIO_PIN_1);
    GPIOA->PUPDR &= ~15u;
    GPIOA->BSRR = (GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10) << 16;
    GPIOB->BSRR = (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15) << 16;
    GPIOC->BSRR = (GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8) << 16;
    GPIOA->MODER = (GPIOA->MODER & ~((3u << 14) | (63u << 16))) | (1u << 14) | (21u << 16);
    /* PB3/4/5: board M0 encoder on SPI1; board M1 encoder uses SPI3. */
    GPIOB->MODER = (GPIOB->MODER & ~(15u | (63u << 6) | (63u << 26))) |
                   5u | (42u << 6) | (21u << 26);
    GPIOC->MODER = (GPIOC->MODER & ~(63u << 12)) | (21u << 12);
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(15u << 28)) | (3u << 28);
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~0xfffu) | 0x111u;
    GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(0xffu | (0xfffu << 12))) |
                    0x33u | (0x555u << 12);
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~(0xfffu << 20)) | (0x111u << 20);
    GPIOC->AFR[0] = (GPIOC->AFR[0] & ~(0xffu << 24)) | (0x33u << 24);
    GPIOC->AFR[1] = (GPIOC->AFR[1] & ~15u) | 3u;
    GPIOA->OTYPER &= ~(GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10);
    GPIOB->OTYPER &= ~(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4 |
                        GPIO_PIN_5 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
    GPIOB->PUPDR &= ~(63u << 6);
    GPIOB->OSPEEDR |= 63u << 6;
    GPIOC->OTYPER &= ~(GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8);
}

void dual_hw_off(unsigned i)
{
    if (i > 1u) return;
    TIM_TypeDef *t = timer(i);
    motors[i].inhibited = true;
    motors[i].ready = false;
    motors[i].mode = motors[i].pending = OFF;
    t->CCER &= ~GATES;
    if (i == 0u) {
        GPIOA->BSRR = (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10) << 16;
        GPIOB->BSRR = (GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15) << 16;
        GPIOA->MODER = (GPIOA->MODER & ~(63u << 16)) | (21u << 16);
        GPIOB->MODER = (GPIOB->MODER & ~(63u << 26)) | (21u << 26);
    } else {
        GPIOA->BSRR = GPIO_PIN_7 << 16;
        GPIOB->BSRR = (GPIO_PIN_0 | GPIO_PIN_1) << 16;
        GPIOC->BSRR = (GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8) << 16;
        GPIOA->MODER = (GPIOA->MODER & ~(3u << 14)) | (1u << 14);
        GPIOB->MODER = (GPIOB->MODER & ~15u) | 5u;
        GPIOC->MODER = (GPIOC->MODER & ~(63u << 12)) | (21u << 12);
    }
}

void dual_hw_off_all(void) { dual_hw_off(0u); dual_hw_off(1u); }
void dual_hw_arm(unsigned i) { if (i < 2u) motors[i].inhibited = false; }

static void timer_init(TIM_TypeDef *t, bool trigger)
{
    t->CR1 = TIM_CR1_CMS_0 | TIM_CR1_ARPE;
    t->PSC = 0u; t->ARR = FOC_PWM_ARR; t->RCR = 1u;
    t->CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE | (6u << 4) | (6u << 12);
    t->CCMR2 = TIM_CCMR2_OC3PE | (6u << 4) | (4u << 12);
    t->CCR1 = t->CCR2 = t->CCR3 = FOC_PWM_ARR / 2u;
    t->CCR4 = FOC_TRIGGER_TICKS;
    t->BDTR = FOC_DEADTIME_TICKS;
    t->CNT = 0u; t->EGR = TIM_EGR_UG;
    t->CCMR2 = TIM_CCMR2_OC3PE | (6u << 4) | (3u << 12);
    t->CCER = trigger ? TIM_CCER_CC4E : 0u;
    t->CR2 = trigger ? TIM_TRGO_OC4REF : 0u;
    t->SR = 0u; t->DIER = TIM_DIER_UIE;
    t->BDTR |= TIM_BDTR_MOE;
}

static bool spi_wait(SPI_TypeDef *spi, uint32_t flag, bool set)
{
    uint32_t start = DWT->CYCCNT;
    while (((spi->SR & flag) != 0u) != set)
#ifdef FOC_ENCODER_DIAGNOSTIC_ONLY
        if (DWT->CYCCNT - start > 30000u) return false;
#else
        if (DWT->CYCCNT - start > 3000u) return false;
#endif
    return true;
}

static void spi_recover(SPI_TypeDef *spi)
{
    spi->CR1 &= ~SPI_CR1_SPE;
    (void)spi->DR; (void)spi->SR;
    spi->CR1 |= SPI_CR1_SPE;
}

/* Frames select one lane at a time. Avoid dual-lane arrays/loops on every
   word, retaining the same TX/RX/busy waits and timeout recovery. */
static bool transfer_word(SPI_TypeDef *spi, uint16_t tx, uint16_t *rx)
{
    if (!spi_wait(spi, SPI_SR_TXE, true)) return false;
    spi->DR = tx;
    if (!spi_wait(spi, SPI_SR_RXNE, true)) return false;
    *rx = (uint16_t)spi->DR;
    return spi_wait(spi, SPI_SR_BSY, false);
}

/* Keep only one CS low. The board's original encoder wiring can share nets even
 * after the second SPI is wired, and asserting both CS lines corrupted SPI1. */
static unsigned frame_lane(unsigned lane, unsigned cs, uint16_t command,
                           uint16_t *data, uint16_t *safety)
{
    unsigned bit = 1u << lane;
    SPI_TypeDef *spi = lane ? SPI1 : SPI3;
    uint16_t ignored = 0u;
    *data = *safety = 0u;
    GPIOA->BSRR = GPIO_PIN_0 | GPIO_PIN_1;
    uint32_t gap = DWT->CYCCNT;
    while (DWT->CYCCNT - gap < 110u) {} /* SSC CS-off: at least 600 ns. */
    GPIOA->BSRR = (GPIO_PIN_0 << cs) << 16;
    gap = DWT->CYCCNT;
    while (DWT->CYCCNT - gap < 24u) {} /* CS setup: at least 105 ns. */
    bool active = transfer_word(spi, command, &ignored);
    gap = DWT->CYCCNT;
    while (DWT->CYCCNT - gap < 32u) {} /* TLE5012B command-to-read delay. */
    active = active && transfer_word(spi, 0xffffu, data);
    active = active && transfer_word(spi, 0xffffu, safety);
    gap = DWT->CYCCNT;
    while (DWT->CYCCNT - gap < 24u) {} /* CS hold: at least 105 ns. */
    GPIOA->BSRR = GPIO_PIN_0 | GPIO_PIN_1;
    if (!active) spi_recover(spi);
    return active ? bit : 0u;
}

static unsigned frame_pair(uint16_t command, uint16_t data[2], uint16_t safety[2])
{
    unsigned active = frame_lane(1u, 1u, command, &data[1], &safety[1]);
    active |= frame_lane(0u, 0u, command, &data[0], &safety[0]);
    return active;
}

#ifdef FOC_ENCODER_DIAGNOSTIC_ONLY
static void diagnostic_delay(unsigned cycles)
{
    uint32_t start = DWT->CYCCNT;
    while (DWT->CYCCNT - start < cycles) {}
}

static uint16_t diagnostic_gpio_word(uint16_t word)
{
    uint16_t received = 0u;
    for (unsigned bit = 0u; bit < 16u; ++bit) {
        GPIOB->BSRR = word & 0x8000u ? GPIO_PIN_5 : GPIO_PIN_5 << 16;
        diagnostic_delay(1680u);
        GPIOB->BSRR = GPIO_PIN_3;
        diagnostic_delay(1680u);
        GPIOB->BSRR = GPIO_PIN_3 << 16;
        diagnostic_delay(168u);
        received = (uint16_t)((received << 1) | !!(GPIOB->IDR & GPIO_PIN_4));
        word = (uint16_t)(word << 1);
    }
    return received;
}

static void diagnostic_gpio_frame(uint16_t command, uint16_t *data, uint16_t *safety)
{
    GPIOA->BSRR = GPIO_PIN_0 | GPIO_PIN_1;
    diagnostic_delay(1680u);
    GPIOA->BSRR = GPIO_PIN_1 << 16;
    diagnostic_delay(1680u);
    (void)diagnostic_gpio_word(command);
    diagnostic_delay(1680u);
    *data = diagnostic_gpio_word(0xffffu);
    *safety = diagnostic_gpio_word(0xffffu);
    diagnostic_delay(1680u);
    GPIOA->BSRR = GPIO_PIN_0 | GPIO_PIN_1;
}

static void encoder_link_diagnostic(void)
{
    static const unsigned baud_bits[6] = {3u, 4u, 5u, 6u, 7u, 3u};
    dual_hw_off_all();
    for (unsigned rate = 0u; rate < 6u; ++rate) {
        SPI1->CR1 &= ~SPI_CR1_SPE;
        (void)SPI1->DR; (void)SPI1->SR;
        SPI1->CR1 = (SPI1->CR1 & ~SPI_CR1_BR) | (baud_bits[rate] << 3);
        SPI1->CR1 |= SPI_CR1_SPE;
        encoder_diag_hz[rate] = 84000000u / (2u << baud_bits[rate]);
        for (unsigned sample = 0u; sample < 32u; ++sample) {
            uint16_t data[2] = {0u, 0u}, safety[2] = {0u, 0u};
            unsigned valid = frame_pair(sample & 1u ? 0x8021u : 0x8001u,
                                        data, safety);
            for (unsigned lane = 0u; lane < 2u; ++lane) {
                encoder_diag_raw[rate][lane][sample][0] = data[lane];
                encoder_diag_raw[rate][lane][sample][1] = safety[lane];
                if (valid & (1u << lane))
                    encoder_diag_spi_valid[rate][lane] |= 1u << sample;
            }
            HAL_Delay(1u);
        }
    }
    /* CPOL=0/CPHA=1 on the same pins, independent of SPI1's AF input path.
     * Half cycles are at least 10 us; motor gates remain low throughout. */
    SPI1->CR1 &= ~SPI_CR1_SPE;
    GPIOB->BSRR = GPIO_PIN_5 | (GPIO_PIN_3 << 16);
    uint32_t saved_moder = GPIOB->MODER;
    GPIOB->MODER = (saved_moder & ~(63u << 6)) | (1u << 6) | (1u << 10);
    encoder_diag_hz[6] = 50000u; /* Upper bound; loop overhead lowers the clock. */
    for (unsigned sample = 0u; sample < 32u; ++sample) {
        uint16_t command = sample & 1u ? 0x8021u : 0x8001u;
        uint16_t data = 0u, safety = 0u;
        unsigned valid = frame_lane(0u, 0u, command, &data, &safety);
        encoder_diag_raw[6][0][sample][0] = data;
        encoder_diag_raw[6][0][sample][1] = safety;
        if (valid) encoder_diag_spi_valid[6][0] |= 1u << sample;
        diagnostic_gpio_frame(command, &data, &safety);
        encoder_diag_raw[6][1][sample][0] = data;
        encoder_diag_raw[6][1][sample][1] = safety;
        encoder_diag_spi_valid[6][1] |= 1u << sample;
        HAL_Delay(1u);
    }
    GPIOB->MODER = saved_moder;
    SPI1->CR1 |= SPI_CR1_SPE;
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 = (SPI1->CR1 & ~SPI_CR1_BR) | (4u << 3); /* SPI1 2.625 MHz. */
    SPI1->CR1 |= SPI_CR1_SPE;
    encoder_diag_long_min[0] = encoder_diag_long_min[1] = 32768u;
    for (unsigned sample = 0u; sample < 10000u; ++sample) {
        uint16_t data[2] = {0u, 0u}, safety[2] = {0u, 0u};
        unsigned valid = frame_pair(0x8021u, data, safety);
        for (unsigned lane = 0u; lane < 2u; ++lane) {
            if ((valid & (1u << lane)) &&
                tle5012b_safety_crc_ok(0x8021u, data[lane], safety[lane]))
                ++encoder_diag_long_crc_good[lane];
            if ((valid & (1u << lane)) &&
                tle5012b_angle_sample_valid_sensor(0x8021u, data[lane], safety[lane],
                                                   lane ? 3u : 0u)) {
                ++encoder_diag_long_angle_good[lane];
                unsigned angle = tle5012b_angle15(data[lane]);
                if (angle < encoder_diag_long_min[lane]) encoder_diag_long_min[lane] = angle;
                if (angle > encoder_diag_long_max[lane]) encoder_diag_long_max[lane] = angle;
            }
        }
        ++encoder_diag_long_samples;
        HAL_Delay(1u);
    }
    dual_hw_off_all();
    encoder_diag_complete = 0x454e4344u;
}
#endif

static void probe_single(unsigned lane, unsigned cs)
{
    unsigned index = 2u * lane + cs;
    uint16_t data = 0u, safety = 0u;
    unsigned active = frame_lane(lane, cs, 0x8001u, &data, &safety);
    dual_probe_status[index] = data;
    dual_probe_safety[index] = safety;
    if (active) dual_probe_spi_valid |= 1u << index;
    if (active && tle5012b_safety_crc_ok(0x8001u, data, safety) &&
        tle5012b_safety_sensor_response(safety, dual_sensor_number[lane]))
        dual_probe_crc_valid |= 1u << index;
}

void dual_hw_encoder_pair(float angle[2])
{
    uint16_t data[2] = {0u, 0u}, safety[2] = {0u, 0u};
    uint32_t start = DWT->CYCCNT;
    unsigned valid = frame_pair(0x8021u, data, safety);
    uint32_t cycles = DWT->CYCCNT - start;
    if (cycles > dual_encoder_cycles_max) dual_encoder_cycles_max = cycles;
    dual_angle_spi_valid = valid;
    dual_angle_sample_valid = 0u;
    for (unsigned i = 0u; i < 2u; ++i) {
        dual_angle_data[i] = data[i];
        dual_angle_safety[i] = safety[i];
        /* Response numbering is sensor configuration, independent of its CS. */
        bool sample_ok = (valid & (1u << i)) &&
            tle5012b_angle_sample_valid_sensor(0x8021u, data[i], safety[i],
                                               dual_sensor_number[i]);
        if (sample_ok) dual_angle_sample_valid |= 1u << i;
        if (!sample_ok) {
            foc_t *f = i ? &foc0 : &foc1;
            unsigned motor = i ? 0u : 1u;
            if ((f->state == FOC_PRECHARGE || f->state == FOC_PWM_ZERO ||
                 f->state == FOC_CALIBRATE || f->state == FOC_RUN) &&
                !(dual_first_active_bad_captured & (1u << i))) {
                dual_first_active_bad_data[i] = data[i];
                dual_first_active_bad_safety[i] = safety[i];
                dual_first_active_bad_state[i] = f->state;
                dual_first_active_bad_mode[i] = motors[motor].mode;
                if (i == 1u) {
                    dual_first_bad_pa_odr = GPIOA->ODR;
                    dual_first_bad_pa_idr = GPIOA->IDR;
                    dual_first_bad_pa_moder = GPIOA->MODER;
                    dual_first_bad_pb_idr = GPIOB->IDR;
                    dual_first_bad_pb_moder = GPIOB->MODER;
                    dual_first_bad_pb_afrl = GPIOB->AFR[0];
                    dual_first_bad_spi1_cr1 = SPI1->CR1;
                    dual_first_bad_spi1_sr = SPI1->SR;
                }
                dual_first_active_bad_spi |= valid & (1u << i);
                dual_first_active_bad_captured |= 1u << i;
            }
            if ((valid & (1u << i)) &&
                tle5012b_safety_crc_ok(0x8021u, data[i], safety[i]) &&
                !(safety[i] & 0x4000u) &&
                !(dual_sensor_status_captured & (1u << i))) {
                uint16_t status = 0u, status_safety = 0u;
                if (frame_lane(i, i, 0x8001u, &status, &status_safety) &&
                    tle5012b_safety_crc_ok(0x8001u, status, status_safety)) {
                    dual_sensor_status[i] = status;
                    dual_sensor_status_safety[i] = status_safety;
                    dual_sensor_error_state[i] = i ? foc0.state : foc1.state;
                    dual_sensor_error_mode[i] = motors[i ? 0u : 1u].mode;
                    dual_sensor_error_timer[i] = timer(i ? 0u : 1u)->CNT;
                    dual_sensor_error_time_us[i] = TIM5->CNT;
                    dual_sensor_status_captured |= 1u << i;
                }
            }
            angle[i] = NAN;
            ++encoder_errors[i];
        } else angle[i] = (float)tle5012b_angle15(data[i]) * (360.0f / 32768.0f);
    }
}

bool dual_hw_init(void)
{
    dual_hw_safe_pins();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __HAL_RCC_TIM1_CLK_ENABLE(); __HAL_RCC_TIM8_CLK_ENABLE();
    __HAL_RCC_TIM5_CLK_ENABLE();
    DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM1_STOP | DBGMCU_APB2_FZ_DBG_TIM8_STOP;
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM5_STOP;
    TIM5->CR1 = 0u; TIM5->PSC = 83u; TIM5->ARR = 0xffffffffu;
    TIM5->EGR = TIM_EGR_UG; TIM5->CNT = 0u; TIM5->CR1 = TIM_CR1_CEN;
    dual_hw_off_all();
    timer_init(TIM1, true); timer_init(TIM8, false);
    GPIOA->BSRR = GPIO_PIN_0 | GPIO_PIN_1;
    GPIOA->MODER = (GPIOA->MODER & ~15u) | 5u;
    SPI3->CR1 |= SPI_CR1_SPE;
    __HAL_RCC_SPI1_CLK_ENABLE();
    SPI1->CR1 = 0u;
    SPI1->CR2 = 0u;
    /* M0's current wiring fails at APB2/16, but 10000 consecutive samples
       passed at APB2/32 with both encoders connected. SPI3 stays unchanged. */
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI |
                SPI_CR1_DFF | SPI_CR1_CPHA | SPI_CR1_BR_2;
    SPI1->CR1 |= SPI_CR1_SPE;
#ifdef FOC_ENCODER_DIAGNOSTIC_ONLY
    encoder_link_diagnostic();
    /* main's failed-init loop polls USB only and rejects all motor commands. */
    return false;
#endif
    uint16_t status[2] = {0u, 0u}, safety[2] = {0u, 0u};
    dual_init_first_crc_valid = dual_init_attempts = 0u;
    dual_sensor_number[0] = dual_sensor_number[1] = 4u;
    unsigned startup_consecutive = 0u;
    for (unsigned attempt = 0u; attempt < 5u; ++attempt) {
        if (attempt) HAL_Delay(1u);
        unsigned valid = frame_pair(0x8001u, status, safety);
        dual_init_spi_valid = valid;
        dual_init_crc_valid = 0u;
        for (unsigned i = 0u; i < 2u; ++i) {
            dual_init_status[i] = status[i];
            dual_init_safety[i] = safety[i];
            if ((valid & (1u << i)) &&
                tle5012b_safety_crc_ok(0x8001u, status[i], safety[i])) {
                unsigned number = tle5012b_safety_sensor_number(safety[i]);
                if (number < 4u && dual_sensor_number[i] == 4u)
                    dual_sensor_number[i] = number;
                if (number < 4u && number == dual_sensor_number[i])
                    dual_init_crc_valid |= 1u << i;
            }
        }
        if (!attempt) dual_init_first_crc_valid = dual_init_crc_valid;
        dual_init_attempts = attempt + 1u;
        startup_consecutive = dual_init_crc_valid == 3u ? startup_consecutive + 1u : 0u;
        if (startup_consecutive >= 3u) break;
    }
    if (startup_consecutive < 3u) {
        dual_probe_spi_valid = dual_probe_crc_valid = 0u;
        for (unsigned lane = 0u; lane < 2u; ++lane)
            for (unsigned cs = 0u; cs < 2u; ++cs) probe_single(lane, cs);
        if (dual_probe_crc_valid != 0x9u) return false;
        unsigned consecutive = 0u;
        for (unsigned attempt = 0u; attempt < 8u; ++attempt) {
            unsigned valid = frame_pair(0x8001u, status, safety);
            uint32_t crc = 0u;
            for (unsigned i = 0u; i < 2u; ++i) {
                dual_init_status[i] = status[i];
                dual_init_safety[i] = safety[i];
                if ((valid & (1u << i)) &&
                    tle5012b_safety_crc_ok(0x8001u, status[i], safety[i]) &&
                    tle5012b_safety_sensor_response(safety[i], dual_sensor_number[i]))
                    crc |= 1u << i;
            }
            dual_init_spi_valid = valid;
            dual_init_crc_valid = crc;
            dual_after_probe_crc[attempt] = crc;
            consecutive = crc == 3u ? consecutive + 1u : 0u;
        }
        if (consecutive < 3u) return false;
    }
    /* Matching response numbers on separate SPI buses are legitimate, but
       must not hide a shared/crossed CS net. Only each lane's own CS may
       produce a valid response. This check runs with both bridges disabled. */
    dual_probe_spi_valid = dual_probe_crc_valid = 0u;
    for (unsigned lane = 0u; lane < 2u; ++lane)
        for (unsigned cs = 0u; cs < 2u; ++cs) probe_single(lane, cs);
    if (dual_probe_crc_valid != 0x9u) return false;
    __HAL_RCC_ADC1_CLK_ENABLE(); __HAL_RCC_ADC2_CLK_ENABLE();
    GPIOA->MODER |= 3u << 12; /* Bus PA6. */
    GPIOC->MODER |= 0xffu; /* M0 PC0/1; M1 PC2/3. */
    GPIOA->PUPDR &= ~(3u << 12); GPIOC->PUPDR &= ~0xffu;
    ADC1->CR2 = ADC2->CR2 = 0u;
    ADC->CCR = ADC_CLOCK_SYNC_PCLK_DIV4;
    ADC1->CR1 = ADC_CR1_SCAN | ADC_CR1_JEOCIE | ADC_CR1_OVRIE;
    ADC2->CR1 = ADC_CR1_SCAN | ADC_CR1_OVRIE;
    /* Two simultaneous injected ranks: ADC1 B0/B1, ADC2 C0/C1. */
    ADC1->JSQR = ADC_JSQR_JL_0 | (10u << 10) | (13u << 15);
    ADC2->JSQR = ADC_JSQR_JL_0 | (11u << 10) | (12u << 15);
    ADC1->SMPR1 = (ADC_SAMPLETIME_28CYCLES << 0) | (ADC_SAMPLETIME_28CYCLES << 9);
    ADC2->SMPR1 = (ADC_SAMPLETIME_28CYCLES << 3) | (ADC_SAMPLETIME_28CYCLES << 6);
    ADC2->SQR1 = 0u; ADC2->SQR3 = 6u;
    ADC2->SMPR2 = ADC_SAMPLETIME_15CYCLES << 18;
    ADC1->SR = ADC2->SR = 0u;
    ADC->CCR = ADC_CLOCK_SYNC_PCLK_DIV4 | ADC_DUALMODE_INJECSIMULT;
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0u, 0u);
    HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 0u, 0u);
    HAL_NVIC_SetPriority(ADC_IRQn, 1u, 0u);
    HAL_NVIC_ClearPendingIRQ(TIM1_UP_TIM10_IRQn);
    HAL_NVIC_ClearPendingIRQ(TIM8_UP_TIM13_IRQn);
    HAL_NVIC_ClearPendingIRQ(ADC_IRQn);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
    ADC2->CR2 = ADC_CR2_ADON;
    ADC1->CR2 = ADC_EXTERNALTRIGINJECCONV_T1_TRGO | ADC_CR2_JEXTEN_0 | ADC_CR2_ADON;
    HAL_Delay(1u);
    return true;
}

void dual_hw_start(void)
{
    last_sample = 0u;
    dual_write_counter_min[0] = dual_write_counter_min[1] = UINT32_MAX;
    dual_update_counter_max[0] = dual_update_counter_max[1] = 0u;
    TIM8->CR1 |= TIM_CR1_CEN;
    TIM1->CR1 |= TIM_CR1_CEN;
}

void dual_hw_halt(void)
{
    dual_hw_off_all();
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM8->CR1 &= ~TIM_CR1_CEN;
    ADC1->CR2 = ADC2->CR2 = 0u;
    HAL_NVIC_DisableIRQ(ADC_IRQn);
    HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
    HAL_NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
}

bool dual_hw_read_adc(dual_adc_t *m0, dual_adc_t *m1)
{
    if (((ADC1->SR | ADC2->SR) & ADC_SR_OVR) ||
        !(ADC1->SR & ADC_SR_JEOC) || !(ADC2->SR & ADC_SR_JEOC)) return false;
    uint16_t b0 = ADC1->JDR1, b1 = ADC1->JDR2;
    uint16_t c0 = ADC2->JDR1, c1 = ADC2->JDR2;
    ADC1->SR = ~(ADC_SR_JEOC | ADC_SR_JSTRT);
    ADC2->SR = ~(ADC_SR_JEOC | ADC_SR_JSTRT | ADC_SR_EOC | ADC_SR_STRT);
    ADC2->CR2 |= ADC_CR2_SWSTART;
    motor0_sample_us = motor1_sample_us = TIM5->CNT & 0xffffffu;
    uint32_t now = DWT->CYCCNT;
    if (last_sample && (now - last_sample < 16000u || now - last_sample > 17600u)) return false;
    last_sample = now;
    m0->b = (float)b0 * ADC_SCALE; m0->c = (float)c0 * ADC_SCALE;
    m1->b = (float)b1 * ADC_SCALE; m1->c = (float)c1 * ADC_SCALE;
    return true;
}

bool dual_hw_read_bus(float *voltage)
{
    if (!(ADC2->SR & ADC_SR_EOC) || (ADC2->SR & ADC_SR_OVR)) return false;
    *voltage = (float)(uint16_t)ADC2->DR * (ADC_SCALE * (41.2f / 2.2f));
    return true;
}

bool dual_hw_write(unsigned i, const float duty[3], unsigned mode)
{
    TIM_TypeDef *t = timer(i);
    uint32_t counter = t->CNT;
    if (counter < dual_write_counter_min[i]) dual_write_counter_min[i] = counter;
    if (motors[i].inhibited || !(t->CR1 & TIM_CR1_DIR) || t->CNT < 600u) {
        dual_timing_site = i + 1u;
        dual_timing_counter = t->CNT;
        dual_timing_direction = !!(t->CR1 & TIM_CR1_DIR);
        return false;
    }
    t->CCR1 = mode == PWM ? (uint32_t)(duty[0] * (float)FOC_PWM_ARR + 0.5f) : 0u;
    t->CCR2 = mode == PWM ? (uint32_t)(duty[1] * (float)FOC_PWM_ARR + 0.5f) : 0u;
    t->CCR3 = mode == PWM ? (uint32_t)(duty[2] * (float)FOC_PWM_ARR + 0.5f) : 0u;
    motors[i].pending = mode; motors[i].ready = true;
    return true;
}

bool dual_hw_update(unsigned i)
{
    TIM_TypeDef *t = timer(i);
    t->SR = ~TIM_SR_UIF;
    /* An inhibited bridge has no PWM deadline: dual_hw_off() already forced
       every gate low. Foreground stop may briefly mask this idle interrupt.
       Armed/precharge/running bridges still require the checks below. */
    if (motors[i].inhibited) { motors[i].ready = false; return true; }
    uint32_t counter = t->CNT;
    if (counter > dual_update_counter_max[i]) dual_update_counter_max[i] = counter;
    if ((t->CR1 & TIM_CR1_DIR) || t->CNT > 600u ||
        (!motors[i].ready && motors[i].mode != OFF)) {
        dual_timing_site = i + 3u;
        dual_timing_counter = t->CNT;
        dual_timing_direction = !!(t->CR1 & TIM_CR1_DIR);
        dual_timing_reason = (!!(t->CR1 & TIM_CR1_DIR)) |
                             ((t->CNT > 600u) << 1) |
                             ((!motors[i].ready && motors[i].mode != OFF) << 2);
        dual_timing_ready = motors[i].ready;
        dual_timing_mode = motors[i].mode;
        dual_timing_pending = motors[i].pending;
        dual_timing_foc_state[0] = foc0.state;
        dual_timing_foc_state[1] = foc1.state;
        dual_timing_sequence = dual_sequence;
        dual_hw_off_all(); return false;
    }
    if (motors[i].ready && motors[i].pending != motors[i].mode) {
        unsigned mode = motors[i].pending;
        if (mode == OFF) dual_hw_off(i);
        else if (mode == PRECHARGE) {
            if (i) { GPIOA->BSRR = GPIO_PIN_7; GPIOB->BSRR = GPIO_PIN_0 | GPIO_PIN_1; }
            else GPIOB->BSRR = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
        } else {
            if (i) { GPIOA->BSRR = GPIO_PIN_7 << 16; GPIOB->BSRR = (GPIO_PIN_0 | GPIO_PIN_1) << 16; }
            else GPIOB->BSRR = (GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15) << 16;
            uint32_t wait = DWT->CYCCNT;
            while (DWT->CYCCNT - wait < 100u) {}
            t->CCER |= GATES;
            if (i) {
                GPIOA->MODER = (GPIOA->MODER & ~(3u << 14)) | (2u << 14);
                GPIOB->MODER = (GPIOB->MODER & ~15u) | 10u;
                GPIOC->MODER = (GPIOC->MODER & ~(63u << 12)) | (42u << 12);
            } else {
                GPIOA->MODER = (GPIOA->MODER & ~(63u << 16)) | (42u << 16);
                GPIOB->MODER = (GPIOB->MODER & ~(63u << 26)) | (42u << 26);
            }
        }
        motors[i].mode = mode;
    }
    motors[i].ready = false;
    return true;
}
