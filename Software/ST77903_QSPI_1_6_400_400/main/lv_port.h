/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LCD_H_RES           (400)
#define LCD_V_RES           (400)

#define LCD_SPI_HOST        (SPI2_HOST)
#define LCD_SPI_PCLK        (SPI_MASTER_FREQ_40M)

#if CONFIG_IDF_TARGET_ESP32S3
/* ESP32-S3 */
#define LCD_SPI_CS          (GPIO_NUM_9)
#define LCD_SPI_SCK         (GPIO_NUM_10)
#define LCD_SPI_D0          (GPIO_NUM_11)
#define LCD_SPI_D1          (GPIO_NUM_12)
#define LCD_SPI_D2          (GPIO_NUM_13)
#define LCD_SPI_D3          (GPIO_NUM_14)
#define LCD_BACKLED         (GPIO_NUM_5)
#elif CONFIG_IDF_TARGET_ESP32C6
/* ESP32-C6 */
#define LCD_SPI_CS          (GPIO_NUM_5)
#define LCD_SPI_SCK         (GPIO_NUM_1)
#define LCD_SPI_D0          (GPIO_NUM_0)
#define LCD_SPI_D1          (GPIO_NUM_19)
#define LCD_SPI_D2          (GPIO_NUM_22)
#define LCD_SPI_D3          (GPIO_NUM_18)
#define LCD_BACKLED         (GPIO_NUM_14)
#endif
#define LCD_RST             (GPIO_NUM_NC)

void lv_port_init(void);

bool lv_port_lock(uint32_t timeout_ms);

void lv_port_unlock(void);

#ifdef __cplusplus
}
#endif
