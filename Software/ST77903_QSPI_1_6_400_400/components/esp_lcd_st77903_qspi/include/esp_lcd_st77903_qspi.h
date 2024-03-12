/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#pragma once

#include "driver/spi_common.h"
#include "esp_err.h"
#include "esp_lcd_types.h"

#define ST77903_QSPI_VERSION_MAJOR  (0)
#define ST77903_QSPI_VERSION_MINOR  (1)
#define ST77903_QSPI_VERSION_PATCH  (0)

#ifdef __cplusplus
extern "C" {
#endif

#define ST77903_QSPI_REFRESH_TASK_PRIO_DEFAULT      (23)
#define ST77903_QSPI_REFRESH_TASK_SIZE_DEFAULT      (3 * 1024)
#define ST77903_QSPI_REFRESH_TASK_CORE_DEFAULT      (tskNO_AFFINITY)

#define ST77903_QSPI_LOAD_TASK_PRIO_DEFAULT         (ST77903_QSPI_REFRESH_TASK_PRIO_DEFAULT)
#define ST77903_QSPI_LOAD_TASK_SIZE_DEFAULT         (3 * 1024)
#define ST77903_QSPI_LOAD_TASK_CORE_DEFAULT         (tskNO_AFFINITY)

/**
 * Default configuration for the task.
 */
#define ST77903_QSPI_TASK_CONFIG_DEFAULT                    \
    {                                                       \
        .refresh_priority = ST77903_QSPI_REFRESH_TASK_PRIO_DEFAULT, \
        .refresh_size = ST77903_QSPI_REFRESH_TASK_SIZE_DEFAULT,     \
        .refresh_core = ST77903_QSPI_REFRESH_TASK_CORE_DEFAULT,     \
        .load_priority = ST77903_QSPI_LOAD_TASK_PRIO_DEFAULT, \
        .load_size = ST77903_QSPI_LOAD_TASK_SIZE_DEFAULT,     \
        .load_core = ST77903_QSPI_LOAD_TASK_CORE_DEFAULT,     \
    }

/**
 * This structure is the configuration for ST77903 QSPI LCD panel.
 */
typedef struct {
    struct {
        spi_host_device_t host_id;  ///< SPI host ID.
        int pclk_hz;                ///< SPI clock frequency in Hz.
        int cs_io_num;              ///< GPIO pin for LCD CS signal, set to -1 if not used.
        int sclk_io_num;            ///< GPIO pin for LCD SCK(SCL) signal.
        int data0_io_num;           ///< GPIO pin for LCD D0(SDA) signal.
        int data1_io_num;           ///< GPIO pin for LCD D1 signal.
        int data2_io_num;           ///< GPIO pin for LCD D2 signal.
        int data3_io_num;           ///< GPIO pin for LCD D3 signal.
    } qspi;
    int reset_gpio_num;             ///< GPIO pin for LCD RST signal, set to -1 if not used.
    struct {
        uint8_t refresh_priority;   ///< Priority of refresh task.
        uint32_t refresh_size;      ///< Stack size of refresh task.
        int refresh_core;           ///< Pined core of refresh task, set to `tskNO_AFFINITY` if not pin to any core.
        uint8_t load_priority;      ///< Priority of load memory task.
        uint32_t load_size;         ///< Stack size of load memory task.
        int load_core;              ///< Pined core of load memory task, set to `tskNO_AFFINITY` if not pin to any core.
    } task;
    size_t bits_per_pixel;          ///< Frame buffer color depth, in bpp. Only support 16 (RGB565) and 24 (RGB888).
    size_t fb_num;                  /**< Number of screen-sized frame buffers that allocated by the driver.
                                      *  By default (set to either 0 or 1) only one frame buffer will be used.
                                      */
    size_t trans_pool_size;         /**< Size of one transaction pool. Each pool contains mutiple transactions which used to transfer color buffer.
                                      *  Each transaction contains one line of LCD frame buffer.
                                      *  It also decides the size of bounce buffer if `flags.fb_in_psram` is set to true.
                                      */
    size_t trans_pool_num;          ///< Number of transaction pool.
    struct {
        unsigned int print_fps_log : 1;         ///< If this flag is enabled, the refresh task will calculate and print FPS.
        unsigned int fb_in_psram : 1;           ///< If this flag is enabled, the frame buffer will be allocated from PSRAM, preferentially.
        unsigned int reset_active_high: 1;      ///< Setting this if the panel reset is high level active.
        unsigned int spi_bus_manual_init: 1;    ///< If this flag is enabled, the driver won't initialize SPI bus and user should initialize it by himself.
    } flags;
    uint16_t h_res;                 ///< Horizontal resolution of LCD.
    uint16_t v_res;                 ///< Vertical resolution of LCD.
    uint16_t expect_fps;            ///< Expected FPS, the driver will try to approach.
    void  (*frame_done_cb)(void);   ///< The callback funcition which will be invoked by refresh task in the end of every frame.
} esp_lcd_st77903_qspi_config_t;

/**
 * @brief Create ST77903 QSPI LCD panel
 *
 * @param[in]  config LCD panel & QSPI configuration
 * @param[out] handle Returned LCD panel handle
 * @return
 *      - ESP_OK:    Success
 *      - Otherwise: Fail
 */
esp_err_t esp_lcd_new_st77903_qspi_panel(const esp_lcd_st77903_qspi_config_t *config, esp_lcd_panel_handle_t *handle);

/**
 * @brief Get the address of the frame buffer(s) that allocated by the driver
 *
 * @param[in] handle  LCD panel handle, returned from `esp_lcd_new_st77903_qspi_panel()`
 * @param[in] fb_num Number of frame buffer(s) to get. This value must be the same as the number of the following parameters.
 * @param[out] fb0   Returned address of the frame buffer 0
 * @param[out] ...   List of other frame buffer addresses
 * @return
 *      - ESP_OK:              Success
 *      - ESP_ERR_INVALID_ARG: Fail
 */
esp_err_t esp_lcd_st77903_qspi_get_frame_buffer(esp_lcd_panel_handle_t handle, uint32_t fb_num, void **fb0, ...);

/**
 * @brief Display color bar to test the LCD
 *
 * @param[in] handle  LCD panel handle, returned from `esp_lcd_new_st77903_qspi_panel()`
 * @return
 *      - ESP_OK:    Success
 *      - Otherwise: Fail
 */
esp_err_t esp_lcd_st77903_qspi_color_bar_test(esp_lcd_panel_handle_t handle);

/**
 * @brief Stop the refresh task
 *
 * @note  Unlike `esp_lcd_panel_reset()`, this function will not reset the LCD panel.
 *        Use `esp_lcd_panel_init()` to restart the refresh task.
 *
 * @param[in] handle LCD panel handle, returned from `esp_lcd_new_st77903_qspi_panel()`
 * @return
 *      - ESP_OK:    Success
 *      - Otherwise: Fail
 */
esp_err_t esp_lcd_st77903_qspi_stop_refresh(esp_lcd_panel_handle_t handle);

#ifdef __cplusplus
}
#endif
