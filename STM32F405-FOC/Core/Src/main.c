/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "board_pwm_safe.h"
#include "board_adc.h"
#include "board_spi_encoder.h"
#include "board_pwm_m0.h"
#include "foc_angle_tracker.h"
#include "foc_current_zero.h"
#include "foc_current_loop.h"
#include "foc_speed_loop.h"
#include "foc_measurements.h"
#include "foc_drive_state.h"
#include "foc_modulation.h"
#include <math.h>
#include "menc15a_device.h"
#include "menc15a_protocol.h"
#include "tle5012b_protocol.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile bool g_menc15a_spi_ready = false;
volatile bool g_menc15a_transport_ok = false;
volatile uint16_t g_menc15a_command_reply = 0u;
volatile uint16_t g_menc15a_angle_reply = 0u;
volatile uint16_t g_menc15a_candidate_angle = 0u;
volatile uint32_t g_menc15a_read_count = 0u;
volatile uint32_t g_menc15a_error_count = 0u;
volatile uint32_t g_menc15a_last_hal_error = 0u;
volatile uint32_t g_menc15a_last_read_us = 0u;
volatile uint16_t g_menc15a_safety_reply = 0u;
volatile bool g_tle5012b_crc_ok = false;
volatile bool g_tle5012b_angle_valid = false;
volatile bool g_tle5012b_system_ok = false;
volatile bool g_tle5012b_interface_ok = false;
volatile bool g_tle5012b_reset_clear = false;
volatile bool g_tle5012b_sensor_response_ok = false;
volatile bool g_tle5012b_sample_valid = false;
volatile uint32_t g_tle5012b_crc_error_count = 0u;
volatile uint32_t g_tle5012b_sensor_error_count = 0u;
volatile bool g_tle5012b_initial_stat_read_ok = false;
volatile bool g_tle5012b_initial_stat_crc_ok = false;
volatile uint16_t g_tle5012b_initial_stat_data = 0u;
volatile uint16_t g_tle5012b_initial_stat_safety = 0u;
volatile bool g_tle5012b_followup_stat_read_ok = false;
volatile bool g_tle5012b_followup_stat_crc_ok = false;
volatile uint16_t g_tle5012b_followup_stat_data = 0u;
volatile uint16_t g_tle5012b_followup_stat_safety = 0u;
volatile int16_t g_menc15a_reference_offset = 0;
volatile int16_t g_menc15a_last_step = 0;
volatile uint16_t g_menc15a_max_abs_step = 0u;
volatile uint32_t g_menc15a_large_step_count = 0u;
volatile int64_t g_menc15a_net_counts = 0;
volatile uint16_t g_menc15a_min_angle = 32767u;
volatile uint16_t g_menc15a_max_angle = 0u;
volatile uint16_t g_menc15a_first_jump_before = 0u;
volatile uint16_t g_menc15a_first_jump_after = 0u;
volatile uint16_t g_menc15a_first_jump_command_reply = 0u;
volatile uint16_t g_menc15a_first_jump_angle_reply = 0u;
volatile int16_t g_menc15a_first_jump_step = 0;
volatile uint32_t g_menc15a_first_jump_at_read = 0u;
volatile uint16_t g_menc15a_first_jump_following[8] = {0u};
volatile uint8_t g_menc15a_first_jump_following_count = 0u;
volatile bool g_menc15a_tracker_ready = false;
volatile foc_angle_status_t g_menc15a_angle_status = FOC_ANGLE_PRIMING;
volatile float g_menc15a_mechanical_speed_rad_s = 0.0f;
volatile uint32_t g_menc15a_tracker_fault_count = 0u;
volatile bool g_adc_m0_ready = false;
volatile bool g_adc_m0_sample_ok = false;
volatile uint16_t g_adc_m0_bus_raw = 0u;
volatile uint16_t g_adc_m0_phase_b_raw = 0u;
volatile uint16_t g_adc_m0_phase_c_raw = 0u;
volatile uint16_t g_adc_m0_bus_min = UINT16_MAX;
volatile uint16_t g_adc_m0_bus_max = 0u;
volatile uint16_t g_adc_m0_phase_b_min = UINT16_MAX;
volatile uint16_t g_adc_m0_phase_b_max = 0u;
volatile uint16_t g_adc_m0_phase_c_min = UINT16_MAX;
volatile uint16_t g_adc_m0_phase_c_max = 0u;
volatile uint32_t g_adc_m0_read_count = 0u;
volatile uint32_t g_adc_m0_error_count = 0u;
volatile uint32_t g_adc_m0_last_hal_error = 0u;
volatile foc_current_zero_status_t g_current_zero_status =
    FOC_CURRENT_ZERO_COLLECTING;
volatile uint16_t g_current_zero_count = 0u;
volatile float g_current_zero_offset_b = 0.0f;
volatile float g_current_zero_offset_c = 0.0f;
volatile uint16_t g_current_zero_span_b = 0u;
volatile uint16_t g_current_zero_span_c = 0u;
volatile bool g_sync_adc_ready = false;
volatile bool g_sync_pwm_outputs_off = false;
volatile bool g_pwm_m0_prepared = false;
volatile uint32_t g_sync_sample_count = 0u;
volatile uint16_t g_sync_phase_b_raw = 0u;
volatile uint16_t g_sync_phase_c_raw = 0u;
volatile uint32_t g_sync_last_period_cycles = 0u;
volatile uint32_t g_sync_last_sample_cycles = 0u;
volatile uint32_t g_sync_period_fault_count = 0u;
volatile bool g_sync_zero_ready = false;
volatile bool g_sync_zero_fault = false;
volatile uint16_t g_sync_zero_count = 0u;
volatile uint16_t g_sync_zero_span_b = 0u;
volatile uint16_t g_sync_zero_span_c = 0u;
volatile bool g_shadow_loop_ready = false;
volatile uint32_t g_shadow_loop_count = 0u;
volatile uint32_t g_shadow_loop_fault_count = 0u;
volatile uint32_t g_shadow_max_execution_cycles = 0u;
volatile float g_shadow_id_amps = 0.0f;
volatile float g_shadow_iq_amps = 0.0f;
volatile float g_shadow_duty_a = 0.0f;
volatile float g_shadow_duty_b = 0.0f;
volatile float g_shadow_duty_c = 0.0f;
volatile float g_shadow_electrical_angle_rad = 0.0f;
volatile uint32_t g_shadow_angle_cycles = 0u;
volatile uint32_t g_shadow_bus_cycles = 0u;
static foc_current_loop_t shadow_current_loop;
static foc_measurement_scale_t shadow_measurement_scale;
static foc_current_zero_t sync_current_zero;
typedef enum
{
  M0_ALIGN_IDLE,
  M0_ALIGN_RUNNING,
  M0_ALIGN_COMPLETE,
  M0_ALIGN_FAULT,
  M0_TORQUE_RUNNING,
  M0_TORQUE_COMPLETE
} m0_align_state_t;
enum { M0_ALIGN_COMMAND = 0xA11A0A11u,
       M0_TORQUE_COMMAND = 0x70A9E001u };
volatile uint32_t g_m0_align_request = 0u;
volatile uint32_t g_m0_torque_request = 0u;
volatile m0_align_state_t g_m0_align_state = M0_ALIGN_IDLE;
volatile uint32_t g_m0_align_faults = 0u;
volatile uint16_t g_m0_align_start_angle = 0u;
volatile uint16_t g_m0_align_end_angle = 0u;
volatile uint16_t g_m0_align_mid_angle = 0u;
volatile int16_t g_m0_align_direction_step = 0;
volatile bool g_m0_align_direction_stage = false;
volatile bool g_m0_align_sweep_started = false;
volatile bool g_m0_align_hold_returned = false;
volatile uint16_t g_m0_align_sweep_start_angle = 0u;
volatile uint16_t g_m0_align_sweep_end_angle = 0u;
volatile int16_t g_m0_align_sweep_step = 0;
volatile float g_m0_align_peak_phase_amps = 0.0f;
volatile uint32_t g_m0_align_start_cycles = 0u;
volatile uint32_t g_m0_align_sample_count = 0u;
volatile uint32_t g_m0_align_fault_origin = 0u;
volatile uint32_t g_m0_align_elapsed_cycles = 0u;
volatile uint32_t g_m0_align_angle_age_cycles = 0u;
volatile float g_m0_align_last_ib_amps = 0.0f;
volatile float g_m0_align_last_ic_amps = 0.0f;
volatile bool g_m0_align_outputs_off = true;
volatile uint32_t g_m0_torque_start_cycles = 0u;
volatile bool g_m0_torque_loop_ready = false;
volatile uint32_t g_m0_torque_sample_count = 0u;
volatile uint32_t g_m0_torque_faults = 0u;
volatile uint32_t g_m0_torque_start_rejects = 0u;
volatile float g_m0_torque_peak_phase_amps = 0.0f;
volatile float g_m0_torque_mean_iq_amps = 0.0f;
volatile float g_m0_torque_tail_mean_iq_amps = 0.0f;
volatile uint32_t g_m0_torque_tail_sample_count = 0u;
volatile uint32_t g_m0_torque_voltage_limit_count = 0u;
volatile uint32_t g_m0_torque_max_execution_cycles = 0u;
volatile float g_m0_torque_last_id_amps = 0.0f;
volatile float g_m0_torque_last_iq_amps = 0.0f;
volatile float g_m0_torque_last_vq_volts = 0.0f;
volatile uint16_t g_m0_torque_start_angle = 0u;
volatile uint16_t g_m0_torque_end_angle = 0u;
volatile uint32_t g_m0_auto_enable = 1u;
volatile bool g_m0_auto_align_attempted = false;
volatile bool g_m0_auto_torque_attempted = false;
volatile bool g_m0_auto_speed_fault = false;
volatile bool g_m0_auto_stall_fault = false;
volatile uint32_t g_m0_auto_stall_ms = 0u;
volatile uint32_t g_m0_auto_under_speed_ms = 0u;
volatile uint32_t g_m0_auto_high_current_ms = 0u;
volatile bool g_m0_auto_speed_loop_ready = false;
volatile float g_m0_auto_speed_rpm = 0.0f;
volatile float g_m0_auto_peak_speed_rpm = 0.0f;
volatile float g_m0_auto_iq_target_amps = 0.0f;
static foc_drive_state_t m0_drive_state;
static foc_current_loop_t m0_torque_loop;
static foc_speed_loop_t m0_speed_loop;
static const foc_speed_loop_config_t m0_speed_config = {
    .kp_amps_per_rad_s = 0.05f,
    .ki_amps_per_rad = 0.015f,
    .period_seconds = 0.001f,
    .target_rad_s = 6.28318531f,
    .max_iq_amps = 0.6f,
    .max_forward_rad_s = 9.42477796f,
    .max_reverse_rad_s = 3.14159265f,
    .filter_alpha = 0.05f};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static bool m0_direction_confirmed(void)
{
  return g_m0_align_sweep_step < -500 ||
         g_m0_align_direction_step < -20;
}

static void m0_on_injected_sample(uint16_t phase_b_raw,
                                  uint16_t phase_c_raw)
{
  static uint32_t previous_cycles;
  const uint32_t cycles = DWT->CYCCNT;
  bool period_valid = true;
  if (g_sync_sample_count != 0u)
  {
    const uint32_t period_cycles = cycles - previous_cycles;
    g_sync_last_period_cycles = period_cycles;
    if (period_cycles < 16000u || period_cycles > 17600u)
    {
      ++g_sync_period_fault_count;
      period_valid = false;
    }
  }
  previous_cycles = cycles;
  g_sync_last_sample_cycles = cycles;
  g_sync_phase_b_raw = phase_b_raw;
  g_sync_phase_c_raw = phase_c_raw;
  ++g_sync_sample_count;
  if (g_m0_align_state == M0_TORQUE_RUNNING)
  {
    const float ib = foc_phase_current_from_adc(
        phase_b_raw, g_current_zero_offset_b, 1.0f,
        &shadow_measurement_scale);
    const float ic = foc_phase_current_from_adc(
        phase_c_raw, g_current_zero_offset_c, 1.0f,
        &shadow_measurement_scale);
    const float ia = -ib - ic;
    const float peak = fmaxf(fabsf(ia), fmaxf(fabsf(ib), fabsf(ic)));
    const float bus = foc_bus_voltage_from_adc(
        g_adc_m0_bus_raw, &shadow_measurement_scale);
    const int16_t mechanical_step = menc15a_shortest_step(
        g_menc15a_candidate_angle, g_m0_align_mid_angle);
    float electrical_angle = fmodf(
        -(float)mechanical_step * 0.00134223319f, 6.28318531f);
    if (electrical_angle < 0.0f) electrical_angle += 6.28318531f;
    uint32_t fault = 0u;
    if (!period_valid ||
        (uint32_t)(cycles - g_shadow_angle_cycles) >= 420000u)
      fault |= FOC_FAULT_TIMING;
    if (g_menc15a_angle_status != FOC_ANGLE_VALID ||
        !g_tle5012b_sample_valid)
      fault |= FOC_FAULT_SENSOR;
    if (!isfinite(peak) || peak > 1.5f)
      fault |= FOC_FAULT_CURRENT;
    if (!isfinite(bus) || bus < 8.0f || bus > 14.0f ||
        (uint32_t)(cycles - g_shadow_bus_cycles) >= 3360000u)
      fault |= FOC_FAULT_BUS;
    if (!m0_direction_confirmed() ||
        !g_m0_auto_speed_loop_ready ||
        g_m0_auto_speed_fault)
      fault |= FOC_FAULT_CONFIGURATION;
    if (g_m0_auto_stall_fault)
      fault |= FOC_FAULT_STALL;
    if (g_m0_auto_enable == 0u)
    {
      board_pwm_m0_emergency_stop();
      foc_drive_stop(&m0_drive_state);
      g_m0_torque_end_angle = g_menc15a_candidate_angle;
      g_m0_align_state = M0_TORQUE_COMPLETE;
      return;
    }
    foc_pwm_duty_t duty;
    if (fault == 0u &&
        (!foc_current_loop_step(&m0_torque_loop, ib, ic,
                                electrical_angle, bus,
                                0.0f, g_m0_auto_iq_target_amps,
                                &duty) ||
         !board_pwm_m0_set_duty(&duty)))
      fault |= FOC_FAULT_DRIVER;
    if (g_m0_torque_sample_count != UINT32_MAX)
      ++g_m0_torque_sample_count;
    if (peak > g_m0_torque_peak_phase_amps)
      g_m0_torque_peak_phase_amps = peak;
    if (fault != 0u)
    {
      board_pwm_m0_emergency_stop();
      foc_drive_raise_fault(&m0_drive_state, fault);
      g_m0_torque_faults = m0_drive_state.faults;
      g_m0_torque_end_angle = g_menc15a_candidate_angle;
      g_m0_align_state = M0_ALIGN_FAULT;
    }
    else
    {
      g_m0_torque_last_id_amps = m0_torque_loop.measured_amps.d;
      g_m0_torque_last_iq_amps = m0_torque_loop.measured_amps.q;
      g_m0_torque_last_vq_volts = m0_torque_loop.commanded_volts.q;
      g_m0_torque_mean_iq_amps += 0.001f *
          (m0_torque_loop.measured_amps.q - g_m0_torque_mean_iq_amps);
      g_m0_torque_tail_mean_iq_amps += 0.0001f *
          (m0_torque_loop.measured_amps.q -
           g_m0_torque_tail_mean_iq_amps);
      if (g_m0_torque_tail_sample_count != UINT32_MAX)
        ++g_m0_torque_tail_sample_count;
      if (m0_torque_loop.voltage_limited)
      {
        if (g_m0_torque_voltage_limit_count != UINT32_MAX)
          ++g_m0_torque_voltage_limit_count;
      }
    }
    const uint32_t execution_cycles = DWT->CYCCNT - cycles;
    if (execution_cycles > g_m0_torque_max_execution_cycles)
      g_m0_torque_max_execution_cycles = execution_cycles;
    return;
  }
  if (g_m0_align_state == M0_ALIGN_RUNNING)
  {
    ++g_m0_align_sample_count;
    const float ib = foc_phase_current_from_adc(
        phase_b_raw, g_current_zero_offset_b, 1.0f,
        &shadow_measurement_scale);
    const float ic = foc_phase_current_from_adc(
        phase_c_raw, g_current_zero_offset_c, 1.0f,
        &shadow_measurement_scale);
    const float ia = -ib - ic;
    g_m0_align_last_ib_amps = ib;
    g_m0_align_last_ic_amps = ic;
    float peak = fabsf(ia);
    if (fabsf(ib) > peak) peak = fabsf(ib);
    if (fabsf(ic) > peak) peak = fabsf(ic);
    if (peak > g_m0_align_peak_phase_amps)
    {
      g_m0_align_peak_phase_amps = peak;
    }
    const float bus = foc_bus_voltage_from_adc(
        g_adc_m0_bus_raw, &shadow_measurement_scale);
    const int16_t displacement = menc15a_shortest_step(
        g_menc15a_candidate_angle, g_m0_align_start_angle);
    const uint16_t absolute_displacement = (uint16_t)(
        displacement < 0 ? -(int32_t)displacement : displacement);
    uint32_t fault = 0u;
    if (!period_valid ||
        (uint32_t)(cycles - g_shadow_angle_cycles) >= 420000u)
      fault |= FOC_FAULT_TIMING;
    if (g_menc15a_angle_status != FOC_ANGLE_VALID ||
        !g_tle5012b_sample_valid)
      fault |= FOC_FAULT_SENSOR;
    if (!isfinite(peak) || peak > 1.5f)
      fault |= FOC_FAULT_CURRENT;
    if (!isfinite(bus) || bus < 8.0f || bus > 14.0f ||
        (uint32_t)(cycles - g_shadow_bus_cycles) >= 3360000u)
      fault |= FOC_FAULT_BUS;
    if (absolute_displacement > 12000u)
      fault |= FOC_FAULT_CONFIGURATION;
    const uint32_t elapsed = cycles - g_m0_align_start_cycles;
    if (fault == 0u && elapsed >= 33600000u &&
        elapsed < 705600000u)
    {
      if (!g_m0_align_sweep_started)
      {
        g_m0_align_sweep_start_angle = g_menc15a_candidate_angle;
        g_m0_align_sweep_started = true;
      }
      const float field_angle = (float)(elapsed - 33600000u) *
          (6.28318531f / 672000000.0f);
      foc_pwm_duty_t sweep_duty;
      if (!foc_svpwm_compute(0.2f * cosf(field_angle),
                             0.2f * sinf(field_angle),
                             bus, 0.2f, &sweep_duty) ||
          !board_pwm_m0_set_duty(&sweep_duty))
        fault |= FOC_FAULT_DRIVER;
    }
    else if (fault == 0u && elapsed >= 705600000u &&
             !g_m0_align_hold_returned)
    {
      foc_pwm_duty_t hold_duty;
      if (!foc_svpwm_compute(0.2f, 0.0f, bus, 0.2f,
                             &hold_duty) ||
          !board_pwm_m0_set_duty(&hold_duty))
        fault |= FOC_FAULT_DRIVER;
      else
      {
        g_m0_align_sweep_end_angle = g_menc15a_candidate_angle;
        g_m0_align_sweep_step = menc15a_shortest_step(
            g_m0_align_sweep_end_angle, g_m0_align_sweep_start_angle);
        g_m0_align_hold_returned = true;
      }
    }
    else if (fault == 0u && elapsed >= 789600000u &&
             !g_m0_align_direction_stage)
    {
      foc_pwm_duty_t direction_duty;
      if (!foc_svpwm_compute(0.152968f, 0.128844f, bus, 0.2f,
                             &direction_duty) ||
          !board_pwm_m0_set_duty(&direction_duty))
        fault |= FOC_FAULT_DRIVER;
      else
      {
        g_m0_align_mid_angle = g_menc15a_candidate_angle;
        g_m0_align_direction_stage = true;
      }
    }
    if (fault != 0u)
    {
      g_m0_align_fault_origin = (fault & FOC_FAULT_DRIVER) != 0u ? 6u :
          (!period_valid ? 1u :
          ((uint32_t)(cycles - g_shadow_angle_cycles) >= 420000u ?
           2u : 5u));
      g_m0_align_elapsed_cycles = cycles - g_m0_align_start_cycles;
      g_m0_align_angle_age_cycles = cycles - g_shadow_angle_cycles;
      board_pwm_m0_emergency_stop();
      foc_drive_raise_fault(&m0_drive_state, fault);
      g_m0_align_faults = m0_drive_state.faults;
      g_m0_align_state = M0_ALIGN_FAULT;
      g_m0_align_end_angle = g_menc15a_candidate_angle;
    }
    else if (elapsed >= 873600000u)
    {
      g_m0_align_elapsed_cycles = cycles - g_m0_align_start_cycles;
      board_pwm_m0_emergency_stop();
      foc_drive_stop(&m0_drive_state);
      g_m0_align_end_angle = g_menc15a_candidate_angle;
      g_m0_align_direction_step = menc15a_shortest_step(
          g_m0_align_end_angle, g_m0_align_mid_angle);
      g_m0_align_state = M0_ALIGN_COMPLETE;
    }
    return;
  }
  if (g_sync_adc_ready && !g_sync_zero_ready && !g_sync_zero_fault)
  {
    if (g_sync_sample_count > 100u)
    {
      const foc_current_zero_status_t status = foc_current_zero_add(
          &sync_current_zero, phase_b_raw, phase_c_raw);
      g_sync_zero_count = sync_current_zero.count;
      if (status == FOC_CURRENT_ZERO_READY)
      {
        g_current_zero_offset_b = sync_current_zero.offset_b;
        g_current_zero_offset_c = sync_current_zero.offset_c;
        g_sync_zero_span_b = (uint16_t)(
            sync_current_zero.max_b - sync_current_zero.min_b);
        g_sync_zero_span_c = (uint16_t)(
            sync_current_zero.max_c - sync_current_zero.min_c);
        foc_current_loop_reset(&shadow_current_loop);
        g_sync_zero_ready = true;
      }
      else if (status == FOC_CURRENT_ZERO_FAULT)
      {
        g_sync_zero_fault = true;
      }
    }
    return;
  }
  if (g_sync_adc_ready && g_sync_zero_ready &&
      g_shadow_loop_ready && period_valid &&
      g_current_zero_status == FOC_CURRENT_ZERO_READY &&
      g_menc15a_angle_status == FOC_ANGLE_VALID &&
      (uint32_t)(cycles - g_shadow_angle_cycles) < 420000u &&
      (uint32_t)(cycles - g_shadow_bus_cycles) < 3360000u)
  {
    const float ib = foc_phase_current_from_adc(
        phase_b_raw, g_current_zero_offset_b, 1.0f,
        &shadow_measurement_scale);
    const float ic = foc_phase_current_from_adc(
        phase_c_raw, g_current_zero_offset_c, 1.0f,
        &shadow_measurement_scale);
    const float bus = foc_bus_voltage_from_adc(
        g_adc_m0_bus_raw, &shadow_measurement_scale);
    foc_pwm_duty_t duty;
    if (foc_current_loop_step(&shadow_current_loop, ib, ic,
                              g_shadow_electrical_angle_rad, bus,
                              0.0f, 0.0f, &duty))
    {
      g_shadow_id_amps = shadow_current_loop.measured_amps.d;
      g_shadow_iq_amps = shadow_current_loop.measured_amps.q;
      g_shadow_duty_a = duty.a;
      g_shadow_duty_b = duty.b;
      g_shadow_duty_c = duty.c;
      ++g_shadow_loop_count;
    }
    else
    {
      ++g_shadow_loop_fault_count;
      foc_current_loop_reset(&shadow_current_loop);
    }
    const uint32_t execution_cycles = DWT->CYCCNT - cycles;
    if (execution_cycles > g_shadow_max_execution_cycles)
    {
      g_shadow_max_execution_cycles = execution_cycles;
    }
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  board_pwm_inputs_force_low();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  g_menc15a_spi_ready = (menc15a_init() == 0u);
  if (g_menc15a_spi_ready)
  {
    uint16_t stat_data = 0u;
    uint16_t stat_safety = 0u;
    g_tle5012b_initial_stat_read_ok =
        menc15a_read_status(&stat_data, &stat_safety);
    if (g_tle5012b_initial_stat_read_ok)
    {
      g_tle5012b_initial_stat_data = stat_data;
      g_tle5012b_initial_stat_safety = stat_safety;
      g_tle5012b_initial_stat_crc_ok = tle5012b_safety_crc_ok(
          0x8001u, stat_data, stat_safety);
    }
  }
  g_adc_m0_ready = board_adc_init();
  foc_current_zero_init(&sync_current_zero);
  foc_drive_state_init(&m0_drive_state);
  const foc_current_loop_config_t shadow_config = {
      .kp_volts_per_amp = 1.0f,
      .ki_volts_per_amp_second = 100.0f,
      .period_seconds = 0.0001f,
      .max_phase_current_amps = 1.0f,
      .max_target_amps = 0.5f,
      .max_voltage_fraction = 0.08f,
      .minimum_duty = 0.2f};
  g_shadow_loop_ready = foc_measurement_scale_init(
      &shadow_measurement_scale, 3.13f, 39000.0f, 2200.0f,
      0.001f, 20.0f) &&
      foc_current_loop_init(&shadow_current_loop, &shadow_config);
  const foc_current_loop_config_t torque_config = {
      .kp_volts_per_amp = 0.2f,
      .ki_volts_per_amp_second = 20.0f,
      .period_seconds = 0.0001f,
      .max_phase_current_amps = 1.8f,
      .max_target_amps = 1.1f,
      .max_voltage_fraction = 0.1f,
      .minimum_duty = 0.2f};
  g_m0_torque_loop_ready = foc_current_loop_init(
      &m0_torque_loop, &torque_config);
  g_m0_auto_speed_loop_ready = foc_speed_loop_init(
      &m0_speed_loop, &m0_speed_config);
  foc_current_zero_t current_zero = {0};
  foc_current_zero_init(&current_zero);
  foc_angle_tracker_t angle_tracker = {0};
  g_menc15a_tracker_ready = foc_angle_tracker_init(
      &angle_tracker, 700.0f, 0.05f, 4000u);
  if (!g_menc15a_tracker_ready)
  {
    g_menc15a_angle_status = FOC_ANGLE_FAULT;
  }
  uint32_t last_read_tick = HAL_GetTick();
  uint32_t last_retry_tick = last_read_tick;
  uint32_t last_adc_read_tick = last_read_tick;
  uint32_t last_adc_retry_tick = last_read_tick;
  bool align_pending = false;
  uint32_t align_pending_tick = 0u;
  bool torque_pending = false;
  uint32_t torque_pending_tick = 0u;
  bool followup_stat_attempted = false;
  bool have_previous_angle = false;
  uint16_t previous_angle = 0u;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    const uint32_t now = HAL_GetTick();
    if (g_m0_auto_enable != 0u &&
        !g_m0_auto_align_attempted && now >= 2000u &&
        g_sync_zero_ready && g_adc_m0_sample_ok &&
        g_menc15a_angle_status == FOC_ANGLE_VALID &&
        g_tle5012b_sample_valid && g_pwm_m0_prepared &&
        g_shadow_loop_ready && g_m0_torque_loop_ready &&
        g_m0_auto_speed_loop_ready)
    {
      g_m0_auto_align_attempted = true;
      g_m0_align_request = M0_ALIGN_COMMAND;
    }
    if (g_m0_auto_enable != 0u && g_m0_auto_align_attempted &&
        !g_m0_auto_torque_attempted &&
        g_m0_align_state == M0_ALIGN_COMPLETE &&
        g_m0_align_outputs_off)
    {
      g_m0_auto_torque_attempted = true;
      if (m0_direction_confirmed())
      {
        g_m0_torque_request = M0_TORQUE_COMMAND;
      }
      else
      {
        foc_drive_raise_fault(&m0_drive_state, FOC_FAULT_CONFIGURATION);
        g_m0_align_faults = m0_drive_state.faults;
        g_m0_align_state = M0_ALIGN_FAULT;
      }
    }
    if (!g_menc15a_spi_ready && (uint32_t)(now - last_retry_tick) >= 1000u)
    {
      last_retry_tick = now;
      g_menc15a_spi_ready = (menc15a_init() == 0u);
    }
    if (g_menc15a_spi_ready && !followup_stat_attempted && now >= 1500u)
    {
      followup_stat_attempted = true;
      uint16_t stat_data = 0u;
      uint16_t stat_safety = 0u;
      g_tle5012b_followup_stat_read_ok =
          menc15a_read_status(&stat_data, &stat_safety);
      if (g_tle5012b_followup_stat_read_ok)
      {
        g_tle5012b_followup_stat_data = stat_data;
        g_tle5012b_followup_stat_safety = stat_safety;
        g_tle5012b_followup_stat_crc_ok = tle5012b_safety_crc_ok(
            0x8001u, stat_data, stat_safety);
      }
    }
    if (!g_adc_m0_ready &&
        (uint32_t)(now - last_adc_retry_tick) >= 1000u)
    {
      last_adc_retry_tick = now;
      g_adc_m0_ready = board_adc_init();
      foc_current_zero_init(&sync_current_zero);
    }
    if (g_adc_m0_ready &&
        (uint32_t)(now - last_adc_read_tick) >= 10u)
    {
      last_adc_read_tick = now;
      board_adc_sample_t sample = {0};
      if (g_sync_adc_ready)
      {
        g_adc_m0_sample_ok = board_adc_read_bus(&sample.bus_voltage);
      }
      else
      {
        g_adc_m0_sample_ok = board_adc_read_m0(&sample);
      }
      g_adc_m0_last_hal_error = board_adc_last_hal_error();
      if (g_adc_m0_sample_ok)
      {
        g_adc_m0_bus_raw = sample.bus_voltage;
        g_shadow_bus_cycles = DWT->CYCCNT;
        if (!g_sync_adc_ready)
        {
          g_adc_m0_phase_b_raw = sample.m0_phase_b;
          g_adc_m0_phase_c_raw = sample.m0_phase_c;
        }
        if (sample.bus_voltage < g_adc_m0_bus_min)
          g_adc_m0_bus_min = sample.bus_voltage;
        if (sample.bus_voltage > g_adc_m0_bus_max)
          g_adc_m0_bus_max = sample.bus_voltage;
        if (!g_sync_adc_ready)
        {
          if (sample.m0_phase_b < g_adc_m0_phase_b_min)
            g_adc_m0_phase_b_min = sample.m0_phase_b;
          if (sample.m0_phase_b > g_adc_m0_phase_b_max)
            g_adc_m0_phase_b_max = sample.m0_phase_b;
          if (sample.m0_phase_c < g_adc_m0_phase_c_min)
            g_adc_m0_phase_c_min = sample.m0_phase_c;
          if (sample.m0_phase_c > g_adc_m0_phase_c_max)
            g_adc_m0_phase_c_max = sample.m0_phase_c;
        }
        ++g_adc_m0_read_count;
        if (!g_sync_adc_ready)
        {
          g_current_zero_status = foc_current_zero_add(
              &current_zero, sample.m0_phase_b, sample.m0_phase_c);
          g_current_zero_count = current_zero.count;
          g_current_zero_offset_b = current_zero.offset_b;
          g_current_zero_offset_c = current_zero.offset_c;
          if (current_zero.count != 0u)
          {
            g_current_zero_span_b =
                (uint16_t)(current_zero.max_b - current_zero.min_b);
            g_current_zero_span_c =
                (uint16_t)(current_zero.max_c - current_zero.min_c);
          }
        }
      }
      else
      {
        ++g_adc_m0_error_count;
        foc_current_zero_abort(&current_zero);
        g_current_zero_status = current_zero.status;
      }
    }
    if (g_current_zero_status == FOC_CURRENT_ZERO_READY &&
        !g_sync_adc_ready)
    {
      if (board_adc_start_injected(m0_on_injected_sample) &&
          board_pwm_m0_start_sampling_clock())
      {
        g_sync_adc_ready = true;
        g_pwm_m0_prepared = board_pwm_m0_prepare_outputs();
        g_sync_pwm_outputs_off = board_pwm_m0_outputs_disabled();
      }
      else
      {
        g_current_zero_status = FOC_CURRENT_ZERO_FAULT;
      }
    }
    if (g_m0_align_request == M0_ALIGN_COMMAND)
    {
      g_m0_align_request = 0u;
      if (g_m0_align_state == M0_ALIGN_IDLE && !align_pending)
      {
        align_pending = true;
        align_pending_tick = now;
      }
    }
    if (align_pending && (uint32_t)(now - align_pending_tick) >= 500u)
    {
      align_pending = false;
      const float bus = foc_bus_voltage_from_adc(
          g_adc_m0_bus_raw, &shadow_measurement_scale);
      const foc_drive_interlocks_t interlocks = {
          .bus_valid = isfinite(bus) && bus >= 8.0f && bus <= 14.0f &&
                       (uint32_t)(DWT->CYCCNT - g_shadow_bus_cycles) <
                           3360000u,
          .currents_calibrated = g_sync_zero_ready && !g_sync_zero_fault,
          .angle_valid = g_menc15a_angle_status == FOC_ANGLE_VALID &&
                         g_tle5012b_sample_valid &&
                         (uint32_t)(DWT->CYCCNT - g_shadow_angle_cycles) <
                             420000u,
          .pwm_safe = g_pwm_m0_prepared &&
                      board_pwm_m0_outputs_disabled() &&
                      g_sync_adc_ready &&
                      g_sync_last_period_cycles >= 16000u &&
                      g_sync_last_period_cycles <= 17600u &&
                      g_shadow_loop_ready &&
                      g_shadow_loop_fault_count == 0u,
          .user_enable = true};
      foc_pwm_duty_t align_duty;
      if (g_m0_align_state == M0_ALIGN_IDLE &&
          foc_svpwm_compute(0.2f, 0.0f, bus, 0.2f, &align_duty) &&
          foc_drive_arm(&m0_drive_state, &interlocks) &&
          board_pwm_m0_set_duty(&align_duty) &&
          foc_drive_start(&m0_drive_state))
      {
        const uint32_t previous_primask = __get_PRIMASK();
        __disable_irq();
        g_m0_align_start_angle = g_menc15a_candidate_angle;
        g_m0_align_peak_phase_amps = 0.0f;
        g_m0_align_mid_angle = 0u;
        g_m0_align_direction_step = 0;
        g_m0_align_direction_stage = false;
        g_m0_align_sweep_started = false;
        g_m0_align_hold_returned = false;
        g_m0_align_sweep_start_angle = 0u;
        g_m0_align_sweep_end_angle = 0u;
        g_m0_align_sweep_step = 0;
        g_m0_align_sample_count = 0u;
        g_m0_align_fault_origin = 0u;
        g_m0_align_elapsed_cycles = 0u;
        g_m0_align_angle_age_cycles = 0u;
        g_m0_align_last_ib_amps = 0.0f;
        g_m0_align_last_ic_amps = 0.0f;
        g_m0_align_start_cycles = DWT->CYCCNT;
        g_m0_align_outputs_off = false;
        g_m0_align_state = M0_ALIGN_RUNNING;
        if (!board_pwm_m0_enable_outputs())
        {
          board_pwm_m0_emergency_stop();
          foc_drive_raise_fault(&m0_drive_state, FOC_FAULT_DRIVER);
          g_m0_align_faults = m0_drive_state.faults;
          g_m0_align_state = M0_ALIGN_FAULT;
        }
        if (previous_primask == 0u)
        {
          __enable_irq();
        }
      }
    }
    if (g_m0_torque_request == M0_TORQUE_COMMAND)
    {
      g_m0_torque_request = 0u;
      if (g_m0_align_state == M0_ALIGN_COMPLETE &&
          m0_direction_confirmed() &&
          g_m0_align_outputs_off && !torque_pending)
      {
        torque_pending = true;
        torque_pending_tick = now;
      }
      else
      {
        ++g_m0_torque_start_rejects;
      }
    }
    if (torque_pending && (uint32_t)(now - torque_pending_tick) >= 500u)
    {
      torque_pending = false;
      const uint32_t bus_cycles = g_shadow_bus_cycles;
      const uint32_t angle_cycles = g_shadow_angle_cycles;
      const uint32_t check_cycles = DWT->CYCCNT;
      const float bus = foc_bus_voltage_from_adc(
          g_adc_m0_bus_raw, &shadow_measurement_scale);
      const foc_drive_interlocks_t interlocks = {
          .bus_valid = isfinite(bus) && bus >= 8.0f && bus <= 14.0f &&
                       (uint32_t)(check_cycles - bus_cycles) < 3360000u,
          .currents_calibrated = g_sync_zero_ready && !g_sync_zero_fault,
          .angle_valid = g_menc15a_angle_status == FOC_ANGLE_VALID &&
                         g_tle5012b_sample_valid &&
                         (uint32_t)(check_cycles - angle_cycles) < 420000u,
          .pwm_safe = g_pwm_m0_prepared &&
                      board_pwm_m0_outputs_disabled() &&
                      g_sync_adc_ready && g_m0_torque_loop_ready &&
                      g_m0_auto_speed_loop_ready &&
                      g_sync_last_period_cycles >= 16000u &&
                      g_sync_last_period_cycles <= 17600u &&
                      g_shadow_loop_ready &&
                      g_shadow_loop_fault_count == 0u,
          .user_enable = true};
      foc_pwm_duty_t zero_duty;
      if (g_m0_align_state == M0_ALIGN_COMPLETE &&
          m0_direction_confirmed() &&
          foc_svpwm_compute(0.0f, 0.0f, bus, 0.2f, &zero_duty) &&
          foc_drive_arm(&m0_drive_state, &interlocks) &&
          board_pwm_m0_set_duty(&zero_duty) &&
          foc_drive_start(&m0_drive_state))
      {
        const uint32_t previous_primask = __get_PRIMASK();
        __disable_irq();
        foc_current_loop_reset(&m0_torque_loop);
        g_m0_auto_speed_loop_ready = foc_speed_loop_init(
            &m0_speed_loop, &m0_speed_config);
        g_m0_auto_speed_fault = false;
        g_m0_auto_stall_fault = false;
        g_m0_auto_stall_ms = 0u;
        g_m0_auto_under_speed_ms = 0u;
        g_m0_auto_high_current_ms = 0u;
        g_m0_auto_speed_rpm = 0.0f;
        g_m0_auto_peak_speed_rpm = 0.0f;
        g_m0_auto_iq_target_amps = 0.0f;
        g_m0_torque_sample_count = 0u;
        g_m0_torque_faults = 0u;
        g_m0_torque_peak_phase_amps = 0.0f;
        g_m0_torque_mean_iq_amps = 0.0f;
        g_m0_torque_tail_mean_iq_amps = 0.0f;
        g_m0_torque_tail_sample_count = 0u;
        g_m0_torque_voltage_limit_count = 0u;
        g_m0_torque_max_execution_cycles = 0u;
        g_m0_torque_start_angle = g_menc15a_candidate_angle;
        g_m0_torque_end_angle = 0u;
        g_m0_torque_start_cycles = DWT->CYCCNT;
        g_m0_align_outputs_off = false;
        g_m0_align_state = M0_TORQUE_RUNNING;
        if (!board_pwm_m0_enable_outputs())
        {
          board_pwm_m0_emergency_stop();
          foc_drive_raise_fault(&m0_drive_state, FOC_FAULT_DRIVER);
          g_m0_torque_faults = m0_drive_state.faults;
          g_m0_align_state = M0_ALIGN_FAULT;
        }
        if (previous_primask == 0u) __enable_irq();
      }
      else
      {
        ++g_m0_torque_start_rejects;
      }
    }
    const uint32_t last_sync_sample = g_sync_last_sample_cycles;
    const uint32_t watchdog_cycles = DWT->CYCCNT;
    const bool sync_sample_stale =
        (uint32_t)(watchdog_cycles - last_sync_sample) >= 504000u;
    if ((g_m0_align_state == M0_ALIGN_RUNNING &&
         ((uint32_t)(watchdog_cycles - g_m0_align_start_cycles) >=
              890400000u || sync_sample_stale)) ||
        (g_m0_align_state == M0_TORQUE_RUNNING && sync_sample_stale))
    {
      const uint32_t previous_primask = __get_PRIMASK();
      __disable_irq();
      if (g_m0_align_state == M0_ALIGN_RUNNING ||
          g_m0_align_state == M0_TORQUE_RUNNING)
      {
        const bool torque_was_running =
            g_m0_align_state == M0_TORQUE_RUNNING;
        g_m0_align_fault_origin = sync_sample_stale ? 4u : 3u;
        g_m0_align_elapsed_cycles =
            watchdog_cycles - g_m0_align_start_cycles;
        g_m0_align_angle_age_cycles =
            watchdog_cycles - g_shadow_angle_cycles;
        board_pwm_m0_emergency_stop();
        foc_drive_raise_fault(&m0_drive_state, FOC_FAULT_TIMING);
        g_m0_align_faults = m0_drive_state.faults;
        if (torque_was_running)
        {
          g_m0_torque_faults = m0_drive_state.faults;
          g_m0_torque_end_angle = g_menc15a_candidate_angle;
        }
        g_m0_align_end_angle = g_menc15a_candidate_angle;
        g_m0_align_state = M0_ALIGN_FAULT;
      }
      if (previous_primask == 0u)
      {
        __enable_irq();
      }
    }
    if ((g_m0_align_state == M0_ALIGN_COMPLETE ||
         g_m0_align_state == M0_TORQUE_COMPLETE ||
         g_m0_align_state == M0_ALIGN_FAULT) &&
        !g_m0_align_outputs_off)
    {
      board_pwm_m0_disarm();
      g_m0_align_outputs_off = board_pwm_m0_outputs_disabled();
    }
    if (g_menc15a_spi_ready &&
        (uint32_t)(now - last_read_tick) >=
            (g_m0_align_state == M0_TORQUE_RUNNING ? 1u : 2u))
    {
      last_read_tick = now;
      const uint32_t read_start_cycles = DWT->CYCCNT;
      const uint16_t reference_angle =
          menc15a_get_absolute_data(menc15a_1_module);
      const bool transfer_ok = menc15a_last_transport_ok;
      const uint16_t command_reply = menc15a_last_command_reply;
      const uint16_t angle_reply = menc15a_last_angle_reply;
      const uint16_t safety_reply = menc15a_last_safety_reply;
      const uint32_t read_cycles = DWT->CYCCNT - read_start_cycles;
      g_menc15a_last_read_us = (uint32_t)(
          ((uint64_t)read_cycles * 1000000u) / SystemCoreClock);
      g_menc15a_transport_ok = transfer_ok;
      g_menc15a_last_hal_error = board_spi_encoder_last_hal_error();
      g_tle5012b_sample_valid = false;
      if (transfer_ok)
      {
        g_menc15a_command_reply = command_reply;
        g_menc15a_angle_reply = angle_reply;
        g_menc15a_safety_reply = safety_reply;
        g_menc15a_candidate_angle = menc15a_candidate_angle(angle_reply);
        g_menc15a_reference_offset = menc15a_absolute_offset_data[0];
        g_tle5012b_crc_ok = tle5012b_safety_crc_ok(
            TLE5012B_READ_ANGLE_WITH_SAFETY, angle_reply, safety_reply);
        g_tle5012b_angle_valid =
            tle5012b_safety_angle_valid(safety_reply);
        g_tle5012b_system_ok =
            tle5012b_safety_system_ok(safety_reply);
        g_tle5012b_interface_ok =
            tle5012b_safety_interface_ok(safety_reply);
        g_tle5012b_reset_clear =
            tle5012b_safety_reset_clear(safety_reply);
        g_tle5012b_sensor_response_ok =
            tle5012b_safety_sensor0_response(safety_reply);
        g_tle5012b_sample_valid = menc15a_last_read_ok;
        if (!g_tle5012b_crc_ok)
        {
          ++g_tle5012b_crc_error_count;
        }
        else if (!g_tle5012b_sample_valid)
        {
          ++g_tle5012b_sensor_error_count;
        }
        if (g_tle5012b_sample_valid)
        {
          if (reference_angle < g_menc15a_min_angle)
          {
            g_menc15a_min_angle = reference_angle;
          }
          if (reference_angle > g_menc15a_max_angle)
          {
            g_menc15a_max_angle = reference_angle;
          }
          if (have_previous_angle)
          {
            const int16_t step = menc15a_shortest_step(
                reference_angle, previous_angle);
            const uint16_t absolute_step =
                (uint16_t)(step < 0 ? -(int32_t)step : step);
            g_menc15a_last_step = step;
            g_menc15a_net_counts += step;
            if (absolute_step > g_menc15a_max_abs_step)
            {
              g_menc15a_max_abs_step = absolute_step;
            }
            if (absolute_step > 1024u)
            {
              ++g_menc15a_large_step_count;
              if (g_menc15a_large_step_count == 1u)
              {
                g_menc15a_first_jump_before = previous_angle;
                g_menc15a_first_jump_after = reference_angle;
                g_menc15a_first_jump_command_reply = command_reply;
                g_menc15a_first_jump_angle_reply = angle_reply;
                g_menc15a_first_jump_step = step;
                g_menc15a_first_jump_at_read = g_menc15a_read_count + 1u;
              }
            }
            if (g_menc15a_large_step_count != 0u &&
                g_menc15a_read_count + 1u > g_menc15a_first_jump_at_read &&
                g_menc15a_first_jump_following_count < 8u)
            {
              g_menc15a_first_jump_following[
                  g_menc15a_first_jump_following_count] = reference_angle;
              ++g_menc15a_first_jump_following_count;
            }
          }
          previous_angle = reference_angle;
          have_previous_angle = true;
        }
        else
        {
          have_previous_angle = false;
          g_menc15a_reference_offset = 0;
        }
        ++g_menc15a_read_count;
      }
      else
      {
        ++g_menc15a_error_count;
        g_menc15a_reference_offset = 0;
        have_previous_angle = false;
        g_tle5012b_crc_ok = false;
        g_tle5012b_angle_valid = false;
        g_tle5012b_system_ok = false;
        g_tle5012b_interface_ok = false;
        g_tle5012b_reset_clear = false;
        g_tle5012b_sensor_response_ok = false;
      }
      if (g_menc15a_tracker_ready)
      {
        float speed_rad_s = 0.0f;
        const foc_angle_status_t previous_status = g_menc15a_angle_status;
        const float angle_rad =
            (float)g_menc15a_candidate_angle *
            (6.2831853071795865f / 32768.0f);
        g_menc15a_angle_status = foc_angle_tracker_update(
            &angle_tracker,
            g_tle5012b_sample_valid,
            angle_rad, HAL_GetTick() * 1000u, &speed_rad_s);
        if (g_menc15a_angle_status == FOC_ANGLE_VALID)
        {
          float electrical_angle = angle_rad * 7.0f;
          while (electrical_angle >= 6.2831853071795865f)
          {
            electrical_angle -= 6.2831853071795865f;
          }
          g_shadow_electrical_angle_rad = electrical_angle;
          g_shadow_angle_cycles = DWT->CYCCNT;
        }
        g_menc15a_mechanical_speed_rad_s = speed_rad_s;
        if (g_m0_align_state == M0_TORQUE_RUNNING &&
            g_menc15a_angle_status == FOC_ANGLE_VALID)
        {
          float iq_target = 0.0f;
          if (!foc_speed_loop_step(&m0_speed_loop, -speed_rad_s,
                                   &iq_target))
          {
            g_m0_auto_speed_fault = true;
          }
          else
          {
            g_m0_auto_iq_target_amps = iq_target;
            g_m0_auto_speed_rpm =
                m0_speed_loop.filtered_rad_s * 9.54929659f;
            if (g_m0_auto_speed_rpm > g_m0_auto_peak_speed_rpm)
              g_m0_auto_peak_speed_rpm = g_m0_auto_speed_rpm;
            if (g_m0_auto_speed_rpm < 5.0f)
            {
              if (g_m0_auto_stall_ms < 3001u)
                ++g_m0_auto_stall_ms;
              if (g_m0_auto_stall_ms >= 3000u)
                g_m0_auto_stall_fault = true;
            }
            else
            {
              g_m0_auto_stall_ms = 0u;
            }
            if (g_m0_auto_speed_rpm < 30.0f)
            {
              if (g_m0_auto_under_speed_ms < 5001u)
                ++g_m0_auto_under_speed_ms;
              if (g_m0_auto_under_speed_ms >= 5000u)
                g_m0_auto_stall_fault = true;
            }
            else
            {
              g_m0_auto_under_speed_ms = 0u;
            }
            if (iq_target >= 0.5f)
            {
              if (g_m0_auto_high_current_ms < 10001u)
                ++g_m0_auto_high_current_ms;
              if (g_m0_auto_high_current_ms >= 10000u)
                g_m0_auto_stall_fault = true;
            }
            else
            {
              g_m0_auto_high_current_ms = 0u;
            }
          }
        }
        if (previous_status != FOC_ANGLE_FAULT &&
            g_menc15a_angle_status == FOC_ANGLE_FAULT)
        {
          ++g_menc15a_tracker_fault_count;
        }
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  board_pwm_inputs_force_low();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
