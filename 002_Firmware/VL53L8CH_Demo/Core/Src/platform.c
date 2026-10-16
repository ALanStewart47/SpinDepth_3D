/*********************************************************
 * STM32G474 HAL transport for the VL53LMZ ULD.
 * Adapted from the module vendor's STM32 platform layer:
 * I2C2, 16-bit register addresses, and HAL status are preserved.
 *********************************************************/

#include "platform.h"
#include "i2c.h"
#include "vl53lmz_api.h"

#define TOF_I2C_TIMEOUT_MS 2000U

static uint8_t hal_status(HAL_StatusTypeDef status)
{
  return (status == HAL_OK) ? VL53LMZ_STATUS_OK : (uint8_t)status;
}

uint8_t RdByte(VL53LMZ_Platform *platform, uint16_t reg, uint8_t *value)
{
  return hal_status(HAL_I2C_Mem_Read(&hi2c2, platform->address, reg,
                                      I2C_MEMADD_SIZE_16BIT, value, 1U,
                                      TOF_I2C_TIMEOUT_MS));
}

uint8_t WrByte(VL53LMZ_Platform *platform, uint16_t reg, uint8_t value)
{
  return hal_status(HAL_I2C_Mem_Write(&hi2c2, platform->address, reg,
                                       I2C_MEMADD_SIZE_16BIT, &value, 1U,
                                       TOF_I2C_TIMEOUT_MS));
}

uint8_t RdMulti(VL53LMZ_Platform *platform, uint16_t reg, uint8_t *values, uint32_t size)
{
  if (size > 0xFFFFU)
  {
    return VL53LMZ_STATUS_INVALID_PARAM;
  }

  return hal_status(HAL_I2C_Mem_Read(&hi2c2, platform->address, reg,
                                      I2C_MEMADD_SIZE_16BIT, values,
                                      (uint16_t)size, TOF_I2C_TIMEOUT_MS));
}

uint8_t WrMulti(VL53LMZ_Platform *platform, uint16_t reg, uint8_t *values, uint32_t size)
{
  if (size > 0xFFFFU)
  {
    return VL53LMZ_STATUS_INVALID_PARAM;
  }

  return hal_status(HAL_I2C_Mem_Write(&hi2c2, platform->address, reg,
                                       I2C_MEMADD_SIZE_16BIT, values,
                                       (uint16_t)size, TOF_I2C_TIMEOUT_MS));
}

void SwapBuffer(uint8_t *buffer, uint16_t size)
{
  uint16_t i;

  for (i = 0; i < size; i = (uint16_t)(i + 4U))
  {
    uint32_t word = ((uint32_t)buffer[i] << 24) |
                    ((uint32_t)buffer[i + 1U] << 16) |
                    ((uint32_t)buffer[i + 2U] << 8) |
                    (uint32_t)buffer[i + 3U];
    memcpy(&buffer[i], &word, sizeof(word));
  }
}

uint8_t WaitMs(VL53LMZ_Platform *platform, uint32_t time_ms)
{
  (void)platform;
  HAL_Delay(time_ms);
  return VL53LMZ_STATUS_OK;
}
