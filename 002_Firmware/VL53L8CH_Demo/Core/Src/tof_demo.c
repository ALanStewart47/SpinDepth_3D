#include "tof_demo.h"

#include <stdarg.h>
#include <stdio.h>

#include "main.h"
#include "usart.h"
#include "vl53lmz_api.h"

#define TOF_DEMO_ROWS 8U
#define TOF_DEMO_ZONES VL53LMZ_RESOLUTION_8X8
#define TOF_DEMO_FREQUENCY_HZ 5U
#define TOF_DEMO_INTEGRATION_MS 40U

/* 8x8 uses four integrations; leave room for overhead within each frame. */
#if (4U * TOF_DEMO_INTEGRATION_MS + 1U) * TOF_DEMO_FREQUENCY_HZ >= 1000U
#error "Integration time exceeds the 8x8 frame budget"
#endif

static VL53LMZ_Configuration tof_device;
static VL53LMZ_ResultsData tof_results;

static void logf_uart(const char *format, ...)
{
  char line[96];
  va_list args;
  int length;

  va_start(args, format);
  length = vsnprintf(line, sizeof(line), format, args);
  va_end(args);

  if (length > 0)
  {
    uint16_t bytes = (length >= (int)sizeof(line)) ?
                     (uint16_t)(sizeof(line) - 1U) : (uint16_t)length;
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)line, bytes, 1000U);
  }
}

static uint8_t report_error(const char *stage, uint8_t status)
{
  logf_uart("ERROR %s status=%u\r\n", stage, status);
  return status;
}

uint8_t tof_demo_init(void)
{
  uint8_t status;
  uint8_t alive = 0U;

  logf_uart("VL53L8CH demo: 8x8, 5 Hz, distance in mm\r\n");
  tof_device.platform.address = VL53LMZ_DEFAULT_I2C_ADDRESS;

  status = vl53lmz_is_alive(&tof_device, &alive);
  if (status != VL53LMZ_STATUS_OK)
  {
    return report_error("alive-check", status);
  }
  if (alive == 0U)
  {
    return report_error("device-not-found", VL53LMZ_STATUS_UNKNOWN_DEVICE);
  }
  logf_uart("I2C device found at 0x52\r\n");

  status = vl53lmz_init(&tof_device);
  if (status != VL53LMZ_STATUS_OK)
  {
    return report_error("init", status);
  }
  if (tof_device.module_type != VL53LMZ_MODULE_TYPE_L8)
  {
    return report_error("module-type", VL53LMZ_STATUS_UNKNOWN_DEVICE);
  }
  logf_uart("Init OK, module type L8\r\n");

  status = vl53lmz_set_resolution(&tof_device, TOF_DEMO_ZONES);
  if (status != VL53LMZ_STATUS_OK)
  {
    return report_error("set-resolution", status);
  }
  status = vl53lmz_set_ranging_frequency_hz(&tof_device, TOF_DEMO_FREQUENCY_HZ);
  if (status != VL53LMZ_STATUS_OK)
  {
    return report_error("set-frequency", status);
  }
  status = vl53lmz_set_ranging_mode(&tof_device, VL53LMZ_RANGING_MODE_AUTONOMOUS);
  if (status != VL53LMZ_STATUS_OK)
  {
    return report_error("set-mode", status);
  }
  status = vl53lmz_set_integration_time_ms(&tof_device, TOF_DEMO_INTEGRATION_MS);
  if (status != VL53LMZ_STATUS_OK)
  {
    return report_error("set-integration", status);
  }
  /* Strongest-first reduces multipath false targets with a >100 ms frame budget. */
  status = vl53lmz_set_target_order(&tof_device, VL53LMZ_TARGET_ORDER_STRONGEST);
  if (status != VL53LMZ_STATUS_OK)
  {
    return report_error("set-target-order", status);
  }
  status = vl53lmz_start_ranging(&tof_device);
  if (status != VL53LMZ_STATUS_OK)
  {
    return report_error("start-ranging", status);
  }

  logf_uart("Ranging started: 8x8 @ %u Hz, autonomous %u ms, strongest target\r\n",
            TOF_DEMO_FREQUENCY_HZ, TOF_DEMO_INTEGRATION_MS);
  return VL53LMZ_STATUS_OK;
}

static void print_distance_row(uint8_t row, const int16_t *values)
{
  uint8_t col;

  logf_uart("D%u:", row);
  for (col = 0U; col < TOF_DEMO_ROWS; ++col)
  {
    uint8_t zone = (uint8_t)(row * TOF_DEMO_ROWS + col);
    logf_uart(" %5d", values[zone]);
  }
  logf_uart("\r\n");
}

static void print_status_row(uint8_t row)
{
  uint8_t col;

  logf_uart("S%u:", row);
  for (col = 0U; col < TOF_DEMO_ROWS; ++col)
  {
    uint8_t zone = (uint8_t)(row * TOF_DEMO_ROWS + col);
    logf_uart(" %2u", tof_results.target_status[zone]);
  }
  logf_uart("\r\n");
}

static void print_count_row(uint8_t row)
{
  uint8_t col;

  logf_uart("N%u:", row);
  for (col = 0U; col < TOF_DEMO_ROWS; ++col)
  {
    uint8_t zone = (uint8_t)(row * TOF_DEMO_ROWS + col);
    logf_uart(" %2u", tof_results.nb_target_detected[zone]);
  }
  logf_uart("\r\n");
}

uint8_t tof_demo_process(void)
{
  uint8_t ready = 0U;
  uint8_t status = vl53lmz_check_data_ready(&tof_device, &ready);
  uint8_t row;
  uint16_t target_sum = 0U;

  if (status != VL53LMZ_STATUS_OK)
  {
    return report_error("data-ready", status);
  }
  if (ready == 0U)
  {
    return VL53LMZ_STATUS_OK;
  }

  status = vl53lmz_get_ranging_data(&tof_device, &tof_results);
  if (status != VL53LMZ_STATUS_OK)
  {
    return report_error("read-frame", status);
  }

  for (row = 0U; row < TOF_DEMO_ZONES; ++row)
  {
    target_sum = (uint16_t)(target_sum + tof_results.nb_target_detected[row]);
  }
  logf_uart("\r\nFRAME %u target_sum=%u\r\n", tof_device.streamcount, target_sum);
  logf_uart("D(mm)\r\n");
  for (row = 0U; row < TOF_DEMO_ROWS; ++row)
  {
    print_distance_row(row, tof_results.distance_mm);
  }
  logf_uart("STATUS (5/9 valid)\r\n");
  for (row = 0U; row < TOF_DEMO_ROWS; ++row)
  {
    print_status_row(row);
  }
  logf_uart("TARGETS\r\n");
  for (row = 0U; row < TOF_DEMO_ROWS; ++row)
  {
    print_count_row(row);
  }

  return VL53LMZ_STATUS_OK;
}
