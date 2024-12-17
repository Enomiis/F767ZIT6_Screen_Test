/*
 * Copyright (c) Riverdi Sp. z o.o. sp. k. <riverdi@riverdi.com>
 * Copyright (c) Skalski Embedded Technologies <contact@lukasz-skalski.com>
 */

#include "platform.h"

/*
 * platform_init()
 */
bool platform_init (Gpu_HalInit_t *halinit)
{
  /* System clock configuration is done in STM32 Core folder */
  return true;
}


/*
 * platform_sleep_ms()
 */
void platform_sleep_ms (uint32_t ms)
{
  HAL_Delay(ms);
}


/*
 * platform_spi_init()
 */
bool platform_spi_init (Gpu_Hal_Context_t *host)
{
  /* Initialization of SPI is done in STM32 Core folder */
  if (!LL_SPI_IsEnabled(SPI1)) {
    LL_SPI_Enable(SPI1);
  }
  return true;
}


/*
 * platform_spi_deinit()
 */
void platform_spi_deinit (Gpu_Hal_Context_t *host)
{
  LL_SPI_Disable(SPI1);
}


/*
 * platform_spi_send_recv_byte();
 */
uint8_t platform_spi_send_recv_byte (Gpu_Hal_Context_t *host,
                                     uint8_t data,
                                     uint32_t opt)
{
  uint8_t answer;

  // Wait until TX becomes empty.
  while (!LL_SPI_IsActiveFlag_TXE(SPI1));
  // Send byte over the SPI
  LL_SPI_TransmitData8(SPI1, data);
  // Wait until data is transmitted.
  while (LL_SPI_IsActiveFlag_BSY(SPI1));
  // Wait until RX is not empty.
  while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
  // Read RX data.
  answer = LL_SPI_ReceiveData8(SPI1);

  return answer;
}


/*
 * platform_spi_send_byte();
 */
void platform_spi_send_byte (Gpu_Hal_Context_t *host,
                                uint8_t data,
                                uint32_t opt)
{

  // Wait until TX becomes empty.
  while (!LL_SPI_IsActiveFlag_TXE(SPI1));
  // Send byte over the SPI
  LL_SPI_TransmitData8(SPI1, data);
  // Wait until data is transmitted.
  while (LL_SPI_IsActiveFlag_BSY(SPI1));
}


/*
 * platform_spi_send_recv_half_word();
 */
uint16_t platform_spi_send_recv_half_word (Gpu_Hal_Context_t *host,
                                     uint16_t data,
                                     uint32_t opt)
{
  uint16_t answer;

  // Wait until TX becomes empty.
  while (!LL_SPI_IsActiveFlag_TXE(SPI1));
  // Send 16 bit data over the SPI
  LL_SPI_TransmitData16(SPI1, data);
  // Wait until data is transmitted.
  while (LL_SPI_IsActiveFlag_BSY(SPI1));
  // Wait until RX is not empty.
  while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
  // Read RX data.
  answer = LL_SPI_ReceiveData16(SPI1);

  return answer;
}


/*
 * platform_spi_send_data()
 */
uint16_t platform_spi_send_data (Gpu_Hal_Context_t *host,
                                 uint8_t *data,
                                 uint16_t size,
                                 uint32_t opt)
{
  for (uint16_t i = 0; i < size; i++) {
    // Wait until TX becomes empty.
    while (!LL_SPI_IsActiveFlag_TXE(SPI1));
    // Send byte over the SPI
    LL_SPI_TransmitData8(SPI1, *(data + i));
    // Wait until data is transmitted.
    while (LL_SPI_IsActiveFlag_BSY(SPI1));
  }

  return size;
}


/*
 * platform_spi_recv_data()
 */
void platform_spi_recv_data (Gpu_Hal_Context_t *host,
                             unsigned char *data,
                             uint16_t size,
                             uint32_t opt)
{
  // Wait until RX is not empty.
  while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
  // Read RX data.
  *data = LL_SPI_ReceiveData8(SPI1);
}


/*
 * platform_gpio_init()
 */
bool platform_gpio_init (Gpu_Hal_Context_t *host,
                         gpio_name ngpio)
{
  /* Initialization of GPIO is done in STM32 Core folder */
  (void)host;
  (void)ngpio;
  return true;
}


/*
 * platform_gpio_value()
 */
bool platform_gpio_value (Gpu_Hal_Context_t *host,
                          gpio_name ngpio,
                          gpio_val vgpio)
{
  HAL_GPIO_WritePin (GPIOD, ngpio, vgpio);
  return true;
}
