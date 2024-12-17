/*
 * Copyright (c) Riverdi Sp. z o.o. sp. k. <riverdi@riverdi.com>
 * Copyright (c) Skalski Embedded Technologies <contact@lukasz-skalski.com>
 */

#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#define FT81X_ENABLE

/* C library inclusions */
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
/* STM32 library inclusions */
#include "stm32f4xx_hal.h"
#include "SEGGER_RTT.h"

/*****************************************************************************/
#include "spi.h"
#include "dma.h"

/* Predefined Riverdi modules */
#include "../riverdi_modules/modules.h"

/* EVE inclusions */
#include "../eve_layer/Gpu_Hal.h"
#include "../eve_layer/Gpu.h"
#include "../eve_layer/CoPro_Cmds.h"
#include "../eve_layer/Hal_Utils.h"

/*****************************************************************************/

typedef enum {
  GPIO_CS   = GPIO_PIN_14,
  GPIO_PD   = GPIO_PIN_15,
  GPIO_INT  = GPIO_PIN_11
} gpio_name;

typedef enum {
  GPIO_HIGH = GPIO_PIN_SET,
  GPIO_LOW  = GPIO_PIN_RESET
} gpio_val;

/*****************************************************************************/

bool platform_init (Gpu_HalInit_t*);
void platform_sleep_ms (uint32_t);

bool platform_spi_init (Gpu_Hal_Context_t*);
void platform_spi_deinit (Gpu_Hal_Context_t*);

uint8_t platform_spi_send_recv_byte (Gpu_Hal_Context_t*, uint8_t, uint32_t);
void platform_spi_send_byte (Gpu_Hal_Context_t*, uint8_t, uint32_t);
uint16_t platform_spi_send_recv_half_word (Gpu_Hal_Context_t*, uint16_t, uint32_t);
uint16_t platform_spi_send_data (Gpu_Hal_Context_t*, unsigned char*, uint16_t, uint32_t);
void platform_spi_recv_data (Gpu_Hal_Context_t*, uint8_t*, uint16_t, uint32_t);

bool platform_gpio_init (Gpu_Hal_Context_t*, gpio_name);
bool platform_gpio_value (Gpu_Hal_Context_t*, gpio_name, gpio_val);

/*****************************************************************************/

#endif /*_PLATFORM_H_*/
