#include "bsp_motor.h"
#include "bsp_motor_record.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stddef.h>

uint32_t bsp_motor_lock(void) { uint32_t key = __get_PRIMASK(); __disable_irq(); return key; }
void bsp_motor_unlock(uint32_t key) { __set_PRIMASK(key); }

#define GATE_CHANNELS (TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE)
static volatile unsigned pending_mode;
volatile unsigned motor_mode;
static volatile bool ready, inhibited;
static uint32_t sample_start, last_sample;
volatile float motor_duty[3];
volatile uint32_t motor_cycles, motor_period_min = UINT32_MAX, motor_period_max, motor_work_max;
volatile uint32_t motor_sample_us;
volatile uint32_t motor_write_min = FOC_PWM_ARR, motor_timing_fault;

void bsp_motor_safe_pins(void)
{
#ifdef FOC_PORT_M0
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIOA->BSRR = (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10) << 16;
    GPIOB->BSRR = (GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15) << 16;
    GPIOA->OTYPER &= ~(GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10);
    GPIOB->OTYPER &= ~(GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
    GPIOA->OSPEEDR |= 63u << 16;
    GPIOB->OSPEEDR |= 63u << 26;
    GPIOA->PUPDR &= ~(63u << 16);
    GPIOB->PUPDR &= ~(63u << 26);
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~0xfffu) | 0x111u;
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~(0xfffu << 20)) | (0x111u << 20);
    GPIOA->MODER = (GPIOA->MODER & ~(63u << 16)) | (21u << 16);
    GPIOB->MODER = (GPIOB->MODER & ~(63u << 26)) | (21u << 26);
#endif
}

void bsp_motor_arm(void) { inhibited = false; }

void bsp_motor_off(void)
{
    inhibited = true;
    FOC_PWM_TIMER->CCER &= ~GATE_CHANNELS; /* CH4 remains for acquisition. */
#ifdef FOC_PORT_M0
    GPIOA->BSRR = (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10) << 16;
    GPIOB->BSRR = (GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15) << 16;
    GPIOA->MODER = (GPIOA->MODER & ~(63u << 16)) | (21u << 16);
    GPIOB->MODER = (GPIOB->MODER & ~(63u << 26)) | (21u << 26);
#else
    GPIOA->BSRR = GPIO_PIN_7 << 16;
    GPIOB->BSRR = (GPIO_PIN_0 | GPIO_PIN_1) << 16;
    GPIOC->BSRR = (GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8) << 16;
    GPIOA->MODER = (GPIOA->MODER & ~(3u << 14)) | (1u << 14);
    GPIOB->MODER = (GPIOB->MODER & ~15u) | 5u;
    GPIOC->MODER = (GPIOC->MODER & ~(63u << 12)) | (21u << 12);
#endif
    motor_mode = pending_mode = MOTOR_OFF;
    for (unsigned i = 0; i < 3; ++i) motor_duty[i] = 0.0f;
}

void bsp_motor_init(void)
{
#ifdef FOC_PORT_M0
    __HAL_RCC_TIM1_CLK_ENABLE();
    bsp_motor_safe_pins();
#endif
    bsp_motor_off();
    ready = false;
    inhibited = false; /* Initial sampling starts with gates disconnected. */
    last_sample = 0u;
    motor_period_min = UINT32_MAX;
    motor_period_max = motor_work_max = motor_cycles = 0u;
    motor_write_min = FOC_PWM_ARR; motor_timing_fault = 0u;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    /* APB1 timer clock is 84 MHz. TIM5 is dedicated to acquisition timestamps. */
    __HAL_RCC_TIM5_CLK_ENABLE();
    TIM5->CR1 = 0u;
    TIM5->PSC = 83u;
    TIM5->ARR = 0xffffffffu;
    TIM5->EGR = TIM_EGR_UG;
    TIM5->CNT = 0u;
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM5_STOP;
    TIM5->CR1 = TIM_CR1_CEN;
#ifdef FOC_PORT_M0
    DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM1_STOP;
#endif
    /* UG loads RCR=1 at CNT=0: overflow counts down, underflow latches CCRs.
       Force CH4 low before toggle mode so rising trigger is on the up-count. */
    FOC_PWM_TIMER->CR1 = TIM_CR1_CMS_0 | TIM_CR1_ARPE;
    FOC_PWM_TIMER->PSC = 0u; FOC_PWM_TIMER->ARR = FOC_PWM_ARR; FOC_PWM_TIMER->RCR = 1u;
    FOC_PWM_TIMER->CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE | (6u << 4) | (6u << 12);
    FOC_PWM_TIMER->CCMR2 = TIM_CCMR2_OC3PE | (6u << 4) | (4u << 12);
    FOC_PWM_TIMER->CCR1 = FOC_PWM_TIMER->CCR2 = FOC_PWM_TIMER->CCR3 = FOC_PWM_ARR / 2u;
    FOC_PWM_TIMER->CCR4 = FOC_TRIGGER_TICKS;
#ifdef FOC_PORT_M0
    FOC_PWM_TIMER->BDTR = 127u; /* Existing M0 0.76 us dead time. */
#else
    FOC_PWM_TIMER->BDTR = 84u;
#endif
    FOC_PWM_TIMER->CNT = 0u; FOC_PWM_TIMER->EGR = TIM_EGR_UG;
    FOC_PWM_TIMER->CCMR2 = TIM_CCMR2_OC3PE | (6u << 4) | (3u << 12);
    FOC_PWM_TIMER->SR = 0u;
    FOC_PWM_TIMER->DIER = TIM_DIER_UIE;
#ifdef FOC_PORT_M0
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0u, 0u);
    HAL_NVIC_ClearPendingIRQ(TIM1_UP_TIM10_IRQn);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
#else
    HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 0u, 0u);
    HAL_NVIC_ClearPendingIRQ(TIM8_UP_TIM13_IRQn);
    HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
#endif
}

bool bsp_motor_write(const float duty[3], unsigned mode)
{
    /* All three preloads must be written before the next valley, never across it. */
    uint32_t counter = FOC_PWM_TIMER->CNT;
    if (counter < motor_write_min) motor_write_min = counter;
    if (inhibited || !(FOC_PWM_TIMER->CR1 & TIM_CR1_DIR) || counter < 600u) {
        motor_timing_fault = 1u | (counter << 8); return false;
    }
    FOC_PWM_TIMER->CCR1 = mode == MOTOR_PWM ? (uint32_t)(duty[0] * (float)FOC_PWM_ARR + 0.5f) : 0u;
    FOC_PWM_TIMER->CCR2 = mode == MOTOR_PWM ? (uint32_t)(duty[1] * (float)FOC_PWM_ARR + 0.5f) : 0u;
    FOC_PWM_TIMER->CCR3 = mode == MOTOR_PWM ? (uint32_t)(duty[2] * (float)FOC_PWM_ARR + 0.5f) : 0u;
    pending_mode = mode;
    ready = true;
    return !inhibited;
}

bool bsp_motor_update(void)
{
    FOC_PWM_TIMER->SR = ~TIM_SR_UIF;
    if ((FOC_PWM_TIMER->CR1 & TIM_CR1_DIR) || FOC_PWM_TIMER->CNT > 600u ||
        (!ready && motor_mode != MOTOR_OFF)) {
        motor_timing_fault = 2u | (FOC_PWM_TIMER->CNT << 8);
        ready = false;
        if (FOC_PROTECTION_TRIP) bsp_motor_off();
        return false; /* WARN holds a coherent previous period, no gate latch. */
    }
    /* A priority-0 fault may interrupt the priority-1 FOC write. Never let
       its resumed/stale preload re-enable gates after an emergency stop. */
    if (inhibited) { ready = false; return true; }
    if (ready) {
        if (pending_mode != motor_mode) {
            unsigned mode = pending_mode;
            if (mode == MOTOR_OFF) bsp_motor_off();
            else if (mode == MOTOR_PRECHARGE) {
#ifdef FOC_PORT_M0
                GPIOB->BSRR = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
#else
                GPIOA->BSRR = GPIO_PIN_7;
                GPIOB->BSRR = GPIO_PIN_0 | GPIO_PIN_1;
#endif
            } else {
                /* GPIO precharge lows must fall before any high-side AF is exposed. */
#ifdef FOC_PORT_M0
                GPIOB->BSRR = (GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15) << 16;
#else
                GPIOA->BSRR = GPIO_PIN_7 << 16;
                GPIOB->BSRR = (GPIO_PIN_0 | GPIO_PIN_1) << 16;
#endif
                uint32_t deadtime = DWT->CYCCNT;
                while (DWT->CYCCNT - deadtime < 100u) {}
                FOC_PWM_TIMER->CCER |= GATE_CHANNELS;
#ifdef FOC_PORT_M0
                GPIOA->MODER = (GPIOA->MODER & ~(63u << 16)) | (42u << 16);
                GPIOB->MODER = (GPIOB->MODER & ~(63u << 26)) | (42u << 26);
#else
                GPIOA->MODER = (GPIOA->MODER & ~(3u << 14)) | (2u << 14);
                GPIOB->MODER = (GPIOB->MODER & ~15u) | 10u;
                GPIOC->MODER = (GPIOC->MODER & ~(63u << 12)) | (42u << 12);
#endif
            }
            motor_mode = mode;
        }
        /* Report quantized CCR/ARR, not the unrounded floating command. */
        motor_duty[0] = motor_mode == MOTOR_PWM ? (float)FOC_PWM_TIMER->CCR1 / (float)FOC_PWM_ARR : 0.0f;
        motor_duty[1] = motor_mode == MOTOR_PWM ? (float)FOC_PWM_TIMER->CCR2 / (float)FOC_PWM_ARR : 0.0f;
        motor_duty[2] = motor_mode == MOTOR_PWM ? (float)FOC_PWM_TIMER->CCR3 / (float)FOC_PWM_ARR : 0.0f;
    }
    ready = false;
    return true;
}

bool bsp_motor_sample_begin(void)
{
    motor_sample_us = TIM5->CNT & 0xffffffu;
    uint32_t now = DWT->CYCCNT;
    uint32_t period = now - last_sample;
    bool valid = !last_sample ||
        (period >= FOC_SAMPLE_CYCLES_MIN && period <= FOC_SAMPLE_CYCLES_MAX);
    if (!valid) motor_timing_fault = 3u | (period << 8);
    if (last_sample) {
        if (period < motor_period_min) motor_period_min = period;
        if (period > motor_period_max) motor_period_max = period;
    }
    last_sample = sample_start = now;
    ++motor_cycles;
    return valid;
}

void bsp_motor_sample_end(void)
{
    uint32_t elapsed = DWT->CYCCNT - sample_start;
    if (elapsed > motor_work_max) motor_work_max = elapsed;
}

bool bsp_motor_load(foc_calibration_t *calibration)
{
    const record_t *r = (const record_t *)0x080e0000u;
    if (!record_valid(r)) return false;
    *calibration = r->cal;
    return true;
}

bool bsp_motor_save(const foc_calibration_t *calibration)
{
    if (motor_mode != MOTOR_OFF || (FOC_PWM_TIMER->CR1 & TIM_CR1_CEN)) return false;
    record_t r = {.version = 3u, .poles = (FOC_CALIBRATION_ID << 16) | FOC_POLE_PAIRS,
                  .cal = *calibration, .magic = 0x464f4331u};
    r.checksum = checksum(&r);
    FLASH_EraseInitTypeDef erase = {.TypeErase = FLASH_TYPEERASE_SECTORS,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3, .Sector = FLASH_SECTOR_11, .NbSectors = 1u};
    uint32_t failed;
    if (HAL_FLASH_Unlock() != HAL_OK) return false;
    bool ok = HAL_FLASHEx_Erase(&erase, &failed) == HAL_OK;
    for (unsigned i = 0; ok && i < sizeof r; i += 4u) {
        uint32_t word;
        memcpy(&word, (const uint8_t *)&r + i, sizeof word);
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x080e0000u + i, word) == HAL_OK;
    }
    HAL_FLASH_Lock();
    foc_calibration_t readback;
    return ok && bsp_motor_load(&readback) && memcmp(&readback, calibration, sizeof readback) == 0;
}
