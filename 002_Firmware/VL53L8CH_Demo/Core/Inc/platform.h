/*********************************************************
 * VL53LMZ platform adaptation for STM32G474 / STM32Cube HAL.
 * Based on the module vendor's platform interface; HAL transport added here.
 * Copyright (c) 2012 - 2024 RTrobot Inc. Original interface notice follows
 * the vendor ULD license in VL53LMZ_ULD_API/LICENSE.txt.
 *********************************************************/

#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#include <stdint.h>
#include <string.h>

typedef struct
{
  uint16_t address; /* HAL 8-bit address form, default 0x52 */
} VL53LMZ_Platform;

#define VL53LMZ_NB_TARGET_PER_ZONE 1U

/* Keep only fields used by this demo: per-zone target count, distance, status. */
#define VL53LMZ_DISABLE_AMBIENT_PER_SPAD
#define VL53LMZ_DISABLE_NB_SPADS_ENABLED
#define VL53LMZ_DISABLE_SIGNAL_PER_SPAD
#define VL53LMZ_DISABLE_RANGE_SIGMA_MM
#define VL53LMZ_DISABLE_REFLECTANCE_PERCENT
#define VL53LMZ_DISABLE_MOTION_INDICATOR

uint8_t RdByte(VL53LMZ_Platform *platform, uint16_t reg, uint8_t *value);
uint8_t WrByte(VL53LMZ_Platform *platform, uint16_t reg, uint8_t value);
uint8_t RdMulti(VL53LMZ_Platform *platform, uint16_t reg, uint8_t *values, uint32_t size);
uint8_t WrMulti(VL53LMZ_Platform *platform, uint16_t reg, uint8_t *values, uint32_t size);
void SwapBuffer(uint8_t *buffer, uint16_t size);
uint8_t WaitMs(VL53LMZ_Platform *platform, uint32_t time_ms);

#endif
