/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "sys/param.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_private/spi_master_internal.h"
#include "hal/spi_ll.h"
#include "hal/dma_types.h"
#include "hal/cache_hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "esp_psram.h"

#include "esp_lcd_st77903_qspi.h"

#define LCD_INS_DATA                (0xDE)
#define LCD_INS_READ                (0xDD)
#define LCD_INS_CMD                 (0xD8)
#define LCD_CMD_HSYNC               (0x60)
#define LCD_CMD_VSYNC               (0x61)
#define LCD_CMD_BPC                 (0xb5)
#define LCD_CMD_DISCN               (0xb6)
#define LCD_CMD_MOLMOD              (0x3a)

#define LCD_VSYNC_FRONT             (0x08)
#define LCD_VSYNC_BACK              (0x08)

#define LCD_BITS_PER_PIXEL_RGB565   (16)
#define LCD_BITS_PER_PIXEL_RGB888   (24)
#define LCD_DELAY_MIN_US            (42)

#define SPI_CMD_BITS                (8)
#define SPI_PARAM_BITS              (24)
#define SPI_MODE                    (0)

#if CONFIG_IDF_TARGET_ESP32S3
#define SPI_SEG_GAP_CLK_SRC         (80)
#elif CONFIG_IDF_TARGET_ESP32C6
#define SPI_SEG_GAP_CLK_SRC         (40)
#endif

#define SPI_SEG_GAP_BASIC_US        (2)
#define SPI_SEG_GAP_GET_US(h_res, bytes)    MAX(0, (int)(MAX(LCD_DELAY_MIN_US, (h_res + 9) / 10) - bytes / 20 - SPI_SEG_GAP_BASIC_US))
#define SPI_SEG_GAP_GET_CLK_LEN(time_us)    (time_us * SPI_SEG_GAP_CLK_SRC)
#define SPI_SEG_INTERVAL_US(h_res, bytes)   (SPI_SEG_GAP_GET_CLK_LEN(SPI_SEG_GAP_GET_US(h_res, bytes)) / \
                                            SPI_SEG_GAP_CLK_SRC + bytes / 20 + SPI_SEG_GAP_BASIC_US)

#define TASK_CHECK_TIME_MS          (10)
#define TASK_STOP_WAIT_MS           (500)

typedef struct {
    esp_lcd_panel_t base;
    // SPI
    spi_host_device_t spi_host_id;
    spi_device_handle_t spi_dev;
    spi_multi_transaction_t **trans_pool;
    size_t trans_pool_size;
    size_t trans_pool_num;
    // Frame buffer
    void **fbs;
    size_t fb_num;
    size_t bytes_per_pixel;
    size_t bytes_per_fb;
    size_t bytes_per_line;
    uint16_t cur_fb_index;
    uint16_t next_fb_index;
    SemaphoreHandle_t sem_count_free_trans;
    void (*frame_done_cb)(void);
    // Boucne buffer
    uint16_t cur_bb_fb_index;
    void **bbs;
    size_t bb_size;
    QueueHandle_t queue_load_mem_info;
    // Others
    int reset_gpio_num;
    struct {
        unsigned int print_fps_log : 1;
        unsigned int fb_in_psram : 1;
        unsigned int reset_level: 1;
        unsigned int spi_bus_manual_init: 1;
        unsigned int refresh_task_end: 1;
        unsigned int load_task_end: 1;
        unsigned int panel_need_reconfig: 1;
    } flags;
    struct {
        uint8_t refresh_priority;
        uint16_t refresh_size;
        int refresh_core;
        TaskHandle_t refresh_task_handle;
        uint8_t load_priority;
        uint16_t load_size;
        int load_core;
        TaskHandle_t load_task_handle;
    } task;
    uint16_t load_bb_index;
    uint16_t load_line;
    uint16_t write_pool_index;
    uint16_t h_res;
    uint16_t v_res;
    uint16_t refresh_delay_ms;
    int rotate_mask;                // panel rotate_mask mask, Or'ed of `panel_rotate_mask_t`
} st77903_qspi_panel_t;

typedef struct {
    void *from;
    void *to;
    uint16_t bytes;
} queue_load_mem_info_t;

#define PANEL_SWAP_XY  0
#define PANEL_MIRROR_Y 1
#define PANEL_MIRROR_X 2

typedef enum {
    ROTATE_MASK_SWAP_XY = BIT(PANEL_SWAP_XY),
    ROTATE_MASK_MIRROR_Y = BIT(PANEL_MIRROR_Y),
    ROTATE_MASK_MIRROR_X = BIT(PANEL_MIRROR_X),
} panel_rotate_mask_t;

static const char *TAG = "lcd_panel.st77903_qspi";

static void load_trans_pool(st77903_qspi_panel_t *panel, spi_multi_transaction_t *seg_trans, bool in_isr);
static void post_trans_color_cb(spi_transaction_t *trans);
static esp_err_t lcd_cmd_config(st77903_qspi_panel_t *panel);
static esp_err_t lcd_write_color(st77903_qspi_panel_t *panel);
static esp_err_t lcd_write_cmd(st77903_qspi_panel_t *panel, uint8_t cmd, void *param, size_t param_size);
static void refresh_task(void *arg);

static esp_err_t st77903_qspi_panel_init(esp_lcd_panel_t *panel);
static esp_err_t st77903_qspi_panel_reset(esp_lcd_panel_t *handle);
static esp_err_t st77903_qspi_panel_del(esp_lcd_panel_t *handle);
static esp_err_t st77903_qspi_panel_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t st77903_qspi_panel_mirror(esp_lcd_panel_t *handle, bool mirror_x, bool mirror_y);
static esp_err_t st77903_qspi_panel_swap_xy(esp_lcd_panel_t *handle, bool swap_axes);
static void load_memory_task(void *arg);

static esp_err_t st77903_qspi_alloc_frame_buffers(st77903_qspi_panel_t *panel)
{
    // fb_in_psram is only an option, if there's no PSRAM on board, we fallback to alloc from SRAM
    bool fb_in_psram = false;
    if (panel->flags.fb_in_psram) {
#if CONFIG_SPIRAM_USE_MALLOC || SPIRAM_USE_MALLOC
        if (esp_psram_is_initialized()) {
            fb_in_psram = true;
        }
#endif
    }

    // Alloc frame buffer
    panel->fbs = (void **)heap_caps_malloc(panel->fb_num * sizeof(void *), MALLOC_CAP_INTERNAL);
    for (int i = 0; i < panel->fb_num; i++) {
        if (fb_in_psram) {
            panel->fbs[i] = heap_caps_calloc(1, panel->bytes_per_fb, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        } else {
            panel->fbs[i] = heap_caps_calloc(1, panel->bytes_per_fb, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        }
        ESP_RETURN_ON_FALSE(panel->fbs[i], ESP_ERR_NO_MEM, TAG, "No mem for frame buffer(%dKB)", panel->bytes_per_fb / 1024);
    }
    panel->cur_fb_index = 0;
    panel->next_fb_index = 0;
    panel->flags.fb_in_psram = fb_in_psram;
    ESP_LOGI(TAG, "Frame buffer size: %d, total: %dKB", panel->bytes_per_fb, panel->fb_num * panel->bytes_per_fb / 1024);

    // Alloc bounce buffer
    if (panel->bb_size) {
        panel->bbs = (void **)heap_caps_malloc(panel->trans_pool_num * sizeof(void *), MALLOC_CAP_INTERNAL);
        for (int i = 0; i < panel->trans_pool_num; i++) {
            panel->bbs[i] = heap_caps_calloc(1, panel->bb_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
            ESP_RETURN_ON_FALSE(panel->bbs[i], ESP_ERR_NO_MEM, TAG, "No mem for bounce buffer(%d, need %dKB)",
                                panel->trans_pool_num, panel->trans_pool_num * panel->bb_size / 1024);
        }
        ESP_LOGI(TAG, "Bounce buffer size: %d, total: %dKB", panel->bb_size, panel->trans_pool_num * panel->bb_size / 1024);
        panel->cur_bb_fb_index = 0;
    }

    // Alloc trans pool
    panel->trans_pool = (spi_multi_transaction_t **)heap_caps_malloc(panel->trans_pool_num * sizeof(spi_multi_transaction_t *), MALLOC_CAP_INTERNAL);
    uint16_t trans_pool_bytes = panel->trans_pool_size * sizeof(spi_transaction_t);
    for (int i = 0; i < panel->trans_pool_num; i++) {
        panel->trans_pool[i] = (spi_multi_transaction_t *)heap_caps_calloc(panel->trans_pool_size, sizeof(spi_multi_transaction_t), MALLOC_CAP_DMA);
        ESP_RETURN_ON_FALSE(panel->trans_pool[i], ESP_ERR_NO_MEM, TAG, "No mem for trans pool(%d, need %dKB)",
                            panel->trans_pool_size, trans_pool_bytes / 1024);
    }
    ESP_LOGI(TAG, "Trans pool size: %d, total: %dKB", panel->trans_pool_size, panel->trans_pool_num * trans_pool_bytes / 1024);

    // Init the constant data for all transactions
    spi_multi_transaction_t seg_temp = {
        .base = {
            .cmd = LCD_INS_DATA,
            .length = panel->bytes_per_line * 8,
            .addr = ((uint32_t)LCD_CMD_HSYNC) << 8,
            .flags = SPI_TRANS_MODE_QIO,
            .user = (void *)panel,
            .sct_gap_len = SPI_SEG_GAP_GET_CLK_LEN(SPI_SEG_GAP_GET_US(panel->h_res, panel->bytes_per_line)),
        },
    };
    ESP_LOGI(TAG, "segment_gap_clock_len: %d", seg_temp.base.sct_gap_len);
    for (int i = 0; i < panel->trans_pool_num; i++) {
        for (int j = 0; j < panel->trans_pool_size; j++) {
            memcpy(&panel->trans_pool[i][j], &seg_temp, sizeof(spi_multi_transaction_t));
        }
    }

    return ESP_OK;
}

esp_err_t esp_lcd_new_st77903_qspi_panel(const esp_lcd_st77903_qspi_config_t *config, esp_lcd_panel_handle_t *handle)
{
    ESP_RETURN_ON_FALSE(config && handle, ESP_ERR_INVALID_ARG, TAG, "Invalid args");
    ESP_RETURN_ON_FALSE(config->bits_per_pixel == LCD_BITS_PER_PIXEL_RGB565 ||
                        config->bits_per_pixel == LCD_BITS_PER_PIXEL_RGB888 , ESP_ERR_INVALID_ARG, TAG,
                        "The bits_per_pixel only support 16(RGB565) and 24(RGB888)");
    ESP_RETURN_ON_FALSE(config->trans_pool_size > 0, ESP_ERR_INVALID_ARG, TAG, "Trans pool size must > 0");

    esp_err_t ret = ESP_OK;
    size_t bytes_per_pixel = config->bits_per_pixel / 8;
    size_t bytes_per_fb_row = bytes_per_pixel * config->h_res;
    size_t dma_nodes_per_spi_trans =  1 + lldesc_get_required_num(bytes_per_fb_row);

    // Init SPI bus if necessary
    if (!config->flags.spi_bus_manual_init) {
        spi_bus_config_t spi_config = {
            .sclk_io_num = config->qspi.sclk_io_num,
            .data0_io_num = config->qspi.data0_io_num,
            .data1_io_num = config->qspi.data1_io_num,
            .data2_io_num = config->qspi.data2_io_num,
            .data3_io_num = config->qspi.data3_io_num,
            .flags = SPICOMMON_BUSFLAG_QUAD,
            .max_transfer_sz = config->trans_pool_size * config->trans_pool_num * dma_nodes_per_spi_trans * \
                               DMA_DESCRIPTOR_BUFFER_MAX_SIZE * 2,
        };
        ESP_LOGI(TAG, "SPI max transfer size: %dKB", spi_config.max_transfer_sz / 1024);
        ESP_RETURN_ON_ERROR(spi_bus_initialize(config->qspi.host_id, &spi_config, SPI_DMA_CH_AUTO), TAG, "SPI bus init failed");
        ESP_LOGI(TAG, "Init SPI bus[%d]", config->qspi.host_id);
    }

    // Create SPI device with segment mode
    spi_device_handle_t spi_device = NULL;
    spi_device_interface_config_t dev_config = {
        .command_bits = SPI_CMD_BITS,
        .address_bits = SPI_PARAM_BITS,
        .mode = SPI_MODE,
        .clock_speed_hz = config->qspi.pclk_hz,
        .spics_io_num = config->qspi.cs_io_num,
        .queue_size = config->trans_pool_num,
        .flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_RETURN_RESULT,
        .post_cb = post_trans_color_cb,
    };
    st77903_qspi_panel_t *panel = NULL;
    ESP_GOTO_ON_ERROR(spi_bus_add_device((spi_host_device_t)config->qspi.host_id, &dev_config, &spi_device), err, TAG, "Device init failed");
    ESP_GOTO_ON_ERROR(spi_bus_multi_trans_mode_enable(spi_device, true), err, TAG, "Segment mode enable failed");
    ESP_LOGI(TAG, "Add SPI device success");

    if (config->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "Configure GPIO for RST line failed");
    }

    panel = (st77903_qspi_panel_t *)heap_caps_calloc(1, sizeof(st77903_qspi_panel_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    ESP_GOTO_ON_FALSE(panel, ESP_ERR_NO_MEM, err, TAG, "Malloc failed");

    panel->spi_dev = spi_device;
    panel->spi_host_id = config->qspi.host_id;
    panel->fb_num = (config->fb_num == 0) ? 1 : config->fb_num;
    panel->bytes_per_pixel = config->bits_per_pixel / 8;
    panel->trans_pool_size = config->trans_pool_size;
    panel->trans_pool_num = config->trans_pool_num;
    panel->reset_gpio_num = config->reset_gpio_num;
    panel->flags.fb_in_psram = config->flags.fb_in_psram;
    panel->flags.print_fps_log = config->flags.print_fps_log;
    panel->flags.reset_level = config->flags.reset_active_high;
    panel->flags.refresh_task_end = true;
    panel->flags.load_task_end = true;
    panel->flags.panel_need_reconfig = true;
    panel->task.refresh_priority = config->task.refresh_priority;
    panel->task.refresh_core = config->task.refresh_core;
    panel->task.refresh_size = config->task.refresh_size;
    panel->task.load_priority = config->task.load_priority;
    panel->task.load_core = config->task.load_core;
    panel->task.load_size = config->task.load_size;
    panel->h_res = config->h_res;
    panel->v_res = config->v_res;
    panel->load_bb_index = 0;
    panel->load_line = 0;
    panel->write_pool_index = 0;
    panel->bytes_per_line = config->h_res * panel->bytes_per_pixel;
    panel->bytes_per_fb = panel->bytes_per_line * config->v_res;
    if (config->flags.fb_in_psram) {
        panel->bb_size = panel->trans_pool_size * panel->bytes_per_line;
    }
    panel->frame_done_cb = config->frame_done_cb;
    panel->base.init = st77903_qspi_panel_init;
    panel->base.draw_bitmap = st77903_qspi_panel_draw_bitmap;
    panel->base.reset = st77903_qspi_panel_reset;
    panel->base.del = st77903_qspi_panel_del;
    panel->base.mirror = st77903_qspi_panel_mirror;
    panel->base.swap_xy = st77903_qspi_panel_swap_xy;

    ESP_GOTO_ON_ERROR(st77903_qspi_alloc_frame_buffers(panel), err, TAG, "Alloc frame buffers failed");

    // Create semaphore for refresh task
    panel->sem_count_free_trans = xSemaphoreCreateCounting(panel->trans_pool_num, 0);
    ESP_RETURN_ON_FALSE(panel->sem_count_free_trans, ESP_ERR_NO_MEM, TAG, "Malloc free_trans_count failed");
    // Create queue for load task
    if (panel->flags.fb_in_psram) {
        panel->queue_load_mem_info = xQueueCreate(panel->trans_pool_num, sizeof(queue_load_mem_info_t));
        ESP_RETURN_ON_FALSE(panel->queue_load_mem_info, ESP_ERR_NO_MEM, TAG, "No mem for queue load mem info");
    }

    // Calculate the time interval between each two transaction, and calculate the delay time of refresh task to achieve expected FPS
    int spi_seg_interval_us = SPI_SEG_INTERVAL_US(panel->h_res, panel->bytes_per_line);
    panel->refresh_delay_ms = MAX(0, (int)(1000000 / (config->expect_fps + 1) - spi_seg_interval_us * config->v_res -
                                      LCD_DELAY_MIN_US * (LCD_VSYNC_FRONT + LCD_VSYNC_BACK)) / 1000);
    ESP_LOGI(TAG, "segment_interval(us): %d, refresh_delay(ms): %d", spi_seg_interval_us, panel->refresh_delay_ms);

    *handle = &panel->base;
    ESP_LOGD(TAG, "Create panel @%p", panel);
    ESP_LOGI(TAG, "version: %d.%d.%d", ST77903_QSPI_VERSION_MAJOR, ST77903_QSPI_VERSION_MINOR, ST77903_QSPI_VERSION_PATCH);

    return ESP_OK;

err:
    if (spi_device) {
        spi_bus_remove_device(spi_device);
    }
    spi_bus_free(config->qspi.host_id);

    if (panel) {
        free(panel);
    }

    return ret;
}

static esp_err_t st77903_qspi_panel_init(esp_lcd_panel_t *handle)
{
    st77903_qspi_panel_t *panel = __containerof(handle, st77903_qspi_panel_t, base);

    if (panel->flags.fb_in_psram) {
        // Because data in PSRAM can't be transmitted by SPI DMA currently, so we have to copy them from frame buffer (PSRAM) to bounce buffer (SRAM) first.
        // Load memory task is used to copy data from frame buffer to bounce buffer
        panel->flags.load_task_end = false;
        xTaskCreatePinnedToCore(
            load_memory_task, "lcd_load_mem", panel->task.load_size, panel, panel->task.load_priority,
            &panel->task.load_task_handle, panel->task.load_core
        );
    }

    // Configure the LCD using initial commands
    if (panel->flags.panel_need_reconfig) {
        panel->flags.panel_need_reconfig = false;
        ESP_RETURN_ON_ERROR(lcd_cmd_config(panel), TAG, "LCD cmd config failed");
    }

    // Here, we should load transaction pool in advance to meet fast SPI timing
    // Then the rest loading operations will be finished by `post_trans_color_cb()` automatically
    panel->load_bb_index = 0;
    panel->load_line = 0;
    for (int i = 0; i < panel->trans_pool_num; i++) {
        load_trans_pool(panel, panel->trans_pool[i], false);
    }
    // Refresh task is used to transmit the segments through SPI
    panel->write_pool_index = 0;
    panel->flags.refresh_task_end = false;
    xTaskCreatePinnedToCore(
        refresh_task, "lcd_refresh", panel->task.refresh_size, panel, panel->task.refresh_priority,
        &panel->task.refresh_task_handle, panel->task.refresh_core
    );

    return ESP_OK;
}

static esp_err_t st77903_qspi_panel_reset(esp_lcd_panel_t *handle)
{
    st77903_qspi_panel_t *panel = __containerof(handle, st77903_qspi_panel_t, base);

    // Stop LCD refreh related tasks
    ESP_RETURN_ON_ERROR(esp_lcd_st77903_qspi_stop_refresh(handle), TAG, "Stop refresh failed");

    // Perform hardware reset
    if (panel->reset_gpio_num >= 0) {
        gpio_set_level(panel->reset_gpio_num, panel->flags.reset_level);
        vTaskDelay(pdMS_TO_TICKS(20));
        gpio_set_level(panel->reset_gpio_num, !panel->flags.reset_level);
        vTaskDelay(pdMS_TO_TICKS(130));
    } else {
        // Perform software reset
        ESP_RETURN_ON_ERROR(lcd_write_cmd(panel, LCD_CMD_SWRESET, NULL, 0), TAG, "LCD write cmd failed");
        vTaskDelay(pdMS_TO_TICKS(130));
    }
    panel->flags.panel_need_reconfig = true;

    return ESP_OK;
}

static esp_err_t st77903_qspi_panel_del(esp_lcd_panel_t *handle)
{
    st77903_qspi_panel_t *panel = __containerof(handle, st77903_qspi_panel_t, base);

    // Stop LCD refreh related tasks
    ESP_RETURN_ON_ERROR(esp_lcd_st77903_qspi_stop_refresh(handle), TAG, "Stop refresh failed");

    // Reset gpio
    if (panel->reset_gpio_num >= 0) {
        gpio_reset_pin(panel->reset_gpio_num);
    }

    // Reset SPI
    ESP_RETURN_ON_ERROR(spi_bus_remove_device(panel->spi_dev), TAG, "SPI device remove failed");
    if (!panel->flags.spi_bus_manual_init) {
        ESP_RETURN_ON_ERROR(spi_bus_free(panel->spi_host_id), TAG, "SPI bus free failed");
    }

    // Free buffers' memory
    for (int i = 0; i < panel->trans_pool_num; i++) {
        free(panel->trans_pool[i]);
    }
    for (int i = 0; i < panel->fb_num; i++) {
        free(panel->fbs[i]);
    }
    free(panel->fbs);
    vSemaphoreDelete(panel->sem_count_free_trans);
    if (panel->flags.fb_in_psram) {
        vQueueDelete(panel->queue_load_mem_info);
        for (int i = 0; i < panel->trans_pool_num; i++) {
            free(panel->bbs[i]);
        }
        free(panel->bbs);
    }
    free(panel);

    ESP_LOGD(TAG, "Del panel @%p", panel);
    return ESP_OK;
}

__attribute__((always_inline))
static inline void copy_pixel_16bpp(uint8_t *to, const uint8_t *from)
{
    *to++ = *from++;
    *to++ = *from++;
}

__attribute__((always_inline))
static inline void copy_pixel_24bpp(uint8_t *to, const uint8_t *from)
{
    *to++ = *from++;
    *to++ = *from++;
    *to++ = *from++;
}

static esp_err_t st77903_qspi_panel_draw_bitmap(esp_lcd_panel_t *handle, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    st77903_qspi_panel_t *panel = __containerof(handle, st77903_qspi_panel_t, base);

    ESP_RETURN_ON_FALSE((x_start < x_end) && (x_end - x_start <= panel->h_res), ESP_ERR_INVALID_ARG, TAG, "Invalid x(%d, %d)", x_start, x_end);
    ESP_RETURN_ON_FALSE((y_start < y_end) && (y_end - y_start <= panel->v_res), ESP_ERR_INVALID_ARG, TAG, "Invalid y(%d, %d)", y_start, y_end);
    ESP_RETURN_ON_FALSE(color_data, ESP_ERR_INVALID_ARG, TAG, "Invalid color_data");

    void **frame_buf = panel->fbs;

    bool do_copy = true;
    for (int i = 0; i < panel->fb_num; i++) {
        if (frame_buf[i] == color_data) {
            panel->next_fb_index = i;
            do_copy = false;
            break;
        }
    }

    int bytes_per_pixel = panel->bytes_per_pixel;
    int pixels_per_line = panel->h_res;
    uint32_t bytes_per_line = bytes_per_pixel * pixels_per_line;
    uint8_t *fb = frame_buf[panel->cur_fb_index];
    int h_res = panel->h_res;
    int v_res = panel->v_res;

    if (do_copy) {
        // copy the UI draw buffer into internal frame buffer
        const uint8_t *from = (const uint8_t *)color_data;
        uint32_t copy_bytes_per_line = (x_end - x_start) * bytes_per_pixel;
        size_t offset = y_start * copy_bytes_per_line + x_start * bytes_per_pixel;
        uint8_t *to = fb;
        if (2 == bytes_per_pixel) {
            switch (panel->rotate_mask) {
            case 0: {
                uint8_t *to = fb + (y_start * h_res + x_start) * bytes_per_pixel;
                for (int y = y_start; y < y_end; y++) {
                    memcpy(to, from, copy_bytes_per_line);
                    to += bytes_per_line;
                    from += copy_bytes_per_line;
                }
            }
            break;
            case ROTATE_MASK_MIRROR_X:
                for (int y = y_start; y < y_end; y++) {
                    uint32_t index = (y * h_res + (h_res - 1 - x_start)) * bytes_per_pixel;
                    for (size_t x = x_start; x < x_end; x++) {
                        copy_pixel_16bpp(to + index, from);
                        index -= bytes_per_pixel;
                        from += bytes_per_pixel;
                    }
                }
                break;
            case ROTATE_MASK_MIRROR_Y: {
                uint8_t *to = fb + ((v_res - 1 - y_start) * h_res + x_start) * bytes_per_pixel;
                for (int y = y_start; y < y_end; y++) {
                    memcpy(to, from, copy_bytes_per_line);
                    to -= bytes_per_line;
                    from += copy_bytes_per_line;
                }
            }
            break;
            case ROTATE_MASK_MIRROR_X | ROTATE_MASK_MIRROR_Y:
                for (int y = y_start; y < y_end; y++) {
                    uint32_t index = ((v_res - 1 - y) * h_res + (h_res - 1 - x_start)) * bytes_per_pixel;
                    for (size_t x = x_start; x < x_end; x++) {
                        copy_pixel_16bpp(to + index, from);
                        index -= bytes_per_pixel;
                        from += bytes_per_pixel;
                    }
                }
                break;
            case ROTATE_MASK_SWAP_XY:
                for (int y = y_start; y < y_end; y++) {
                    for (int x = x_start; x < x_end; x++) {
                        uint32_t j = y * copy_bytes_per_line + x * bytes_per_pixel - offset;
                        uint32_t i = (x * h_res + y) * bytes_per_pixel;
                        copy_pixel_16bpp(to + i, from + j);
                    }
                }
                break;
            case ROTATE_MASK_SWAP_XY | ROTATE_MASK_MIRROR_X:
                for (int y = y_start; y < y_end; y++) {
                    for (int x = x_start; x < x_end; x++) {
                        uint32_t j = y * copy_bytes_per_line + x * bytes_per_pixel - offset;
                        uint32_t i = (x * h_res + h_res - 1 - y) * bytes_per_pixel;
                        copy_pixel_16bpp(to + i, from + j);
                    }
                }
                break;
            case ROTATE_MASK_SWAP_XY | ROTATE_MASK_MIRROR_Y:
                for (int y = y_start; y < y_end; y++) {
                    for (int x = x_start; x < x_end; x++) {
                        uint32_t j = y * copy_bytes_per_line + x * bytes_per_pixel - offset;
                        uint32_t i = ((v_res - 1 - x) * h_res + y) * bytes_per_pixel;
                        copy_pixel_16bpp(to + i, from + j);
                    }
                }
                break;
            case ROTATE_MASK_SWAP_XY | ROTATE_MASK_MIRROR_X | ROTATE_MASK_MIRROR_Y:
                for (int y = y_start; y < y_end; y++) {
                    for (int x = x_start; x < x_end; x++) {
                        uint32_t j = y * copy_bytes_per_line + x * bytes_per_pixel - offset;
                        uint32_t i = ((v_res - 1 - x) * h_res + h_res - 1 - y) * bytes_per_pixel;
                        copy_pixel_16bpp(to + i, from + j);
                    }
                }
                break;
            default:
                break;
            }
        } else if (3 == bytes_per_pixel) {
            switch (panel->rotate_mask) {
            case 0: {
                uint8_t *to = fb + (y_start * h_res + x_start) * bytes_per_pixel;
                for (int y = y_start; y < y_end; y++) {
                    memcpy(to, from, copy_bytes_per_line);
                    to += bytes_per_line;
                    from += copy_bytes_per_line;
                }
            }
            break;
            case ROTATE_MASK_MIRROR_X:
                for (int y = y_start; y < y_end; y++) {
                    uint32_t index = (y * h_res + (h_res - 1 - x_start)) * bytes_per_pixel;
                    for (size_t x = x_start; x < x_end; x++) {
                        copy_pixel_24bpp(to + index, from);
                        index -= bytes_per_pixel;
                        from += bytes_per_pixel;
                    }
                }
                break;
            case ROTATE_MASK_MIRROR_Y: {
                uint8_t *to = fb + ((v_res - 1 - y_start) * h_res + x_start) * bytes_per_pixel;
                for (int y = y_start; y < y_end; y++) {
                    memcpy(to, from, copy_bytes_per_line);
                    to -= bytes_per_line;
                    from += copy_bytes_per_line;
                }
            }
            break;
            case ROTATE_MASK_MIRROR_X | ROTATE_MASK_MIRROR_Y:
                for (int y = y_start; y < y_end; y++) {
                    uint32_t index = ((v_res - 1 - y) * h_res + (h_res - 1 - x_start)) * bytes_per_pixel;
                    for (size_t x = x_start; x < x_end; x++) {
                        copy_pixel_24bpp(to + index, from);
                        index -= bytes_per_pixel;
                        from += bytes_per_pixel;
                    }
                }
                break;
            case ROTATE_MASK_SWAP_XY:
                for (int y = y_start; y < y_end; y++) {
                    for (int x = x_start; x < x_end; x++) {
                        uint32_t j = y * copy_bytes_per_line + x * bytes_per_pixel - offset;
                        uint32_t i = (x * h_res + y) * bytes_per_pixel;
                        copy_pixel_24bpp(to + i, from + j);
                    }
                }
                break;
            case ROTATE_MASK_SWAP_XY | ROTATE_MASK_MIRROR_X:
                for (int y = y_start; y < y_end; y++) {
                    for (int x = x_start; x < x_end; x++) {
                        uint32_t j = y * copy_bytes_per_line + x * bytes_per_pixel - offset;
                        uint32_t i = (x * h_res + h_res - 1 - y) * bytes_per_pixel;
                        copy_pixel_24bpp(to + i, from + j);
                    }
                }
                break;
            case ROTATE_MASK_SWAP_XY | ROTATE_MASK_MIRROR_Y:
                for (int y = y_start; y < y_end; y++) {
                    for (int x = x_start; x < x_end; x++) {
                        uint32_t j = y * copy_bytes_per_line + x * bytes_per_pixel - offset;
                        uint32_t i = ((v_res - 1 - x) * h_res + y) * bytes_per_pixel;
                        copy_pixel_24bpp(to + i, from + j);
                    }
                }
                break;
            case ROTATE_MASK_SWAP_XY | ROTATE_MASK_MIRROR_X | ROTATE_MASK_MIRROR_Y:
                for (int y = y_start; y < y_end; y++) {
                    for (int x = x_start; x < x_end; x++) {
                        uint32_t j = y * copy_bytes_per_line + x * bytes_per_pixel - offset;
                        uint32_t i = ((v_res - 1 - x) * h_res + h_res - 1 - y) * bytes_per_pixel;
                        copy_pixel_24bpp(to + i, from + j);
                    }
                }
                break;
            default:
                break;
            }
        }
    }

    return ESP_OK;
}

static esp_err_t st77903_qspi_panel_mirror(esp_lcd_panel_t *handle, bool mirror_x, bool mirror_y)
{
    st77903_qspi_panel_t *panel = __containerof(handle, st77903_qspi_panel_t, base);
    panel->rotate_mask &= ~(ROTATE_MASK_MIRROR_X | ROTATE_MASK_MIRROR_Y);
    panel->rotate_mask |= (mirror_x << PANEL_MIRROR_X | mirror_y << PANEL_MIRROR_Y);
    return ESP_OK;
}

static esp_err_t st77903_qspi_panel_swap_xy(esp_lcd_panel_t *handle, bool swap_axes)
{
    st77903_qspi_panel_t *panel = __containerof(handle, st77903_qspi_panel_t, base);
    panel->rotate_mask &= ~(ROTATE_MASK_SWAP_XY);
    panel->rotate_mask |= swap_axes << PANEL_SWAP_XY;
    return ESP_OK;
}

esp_err_t esp_lcd_st77903_qspi_color_bar_test(esp_lcd_panel_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    st77903_qspi_panel_t *panel = __containerof(handle, st77903_qspi_panel_t, base);
    uint16_t row_line = panel->v_res / panel->bytes_per_pixel / 8;
    uint8_t *color = (uint8_t *)malloc(row_line * panel->h_res * panel->bytes_per_pixel);
    ESP_RETURN_ON_FALSE(color, ESP_ERR_NO_MEM, TAG, "Malloc color buffer failed");

    uint16_t bytes_per_pixel = panel->bytes_per_pixel;
    uint16_t bits_per_pixel = bytes_per_pixel * 8;
    uint32_t count = 0;
    for (int i = 0; i < bits_per_pixel; i++) {
        count = 0;
        for (int j = 0; j < row_line * panel->h_res; j++) {
            for (int k = 0; k < bytes_per_pixel; k++) {
                color[count++] = (SPI_SWAP_DATA_TX(BIT(i), bits_per_pixel) >> (k * 8)) & 0xff;
            }
        }
        ESP_RETURN_ON_ERROR(esp_lcd_panel_draw_bitmap(handle, 0, i * row_line, panel->h_res, (i + 1) * row_line, color), TAG, "Draw bitmap failed");
    }

    return ESP_OK;
}

esp_err_t esp_lcd_st77903_qspi_get_frame_buffer(esp_lcd_panel_handle_t handle, uint32_t fb_num, void **fb0, ...)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    st77903_qspi_panel_t *panel = __containerof(handle, st77903_qspi_panel_t, base);
    ESP_RETURN_ON_FALSE(fb_num && fb_num <= panel->fb_num, ESP_ERR_INVALID_ARG, TAG, "Frame buffer num out of range(< %d)", panel->fb_num);

    void **fb_itor = fb0;
    va_list args;
    va_start(args, fb0);
    for (int i = 0; i < fb_num; i++) {
        if (fb_itor) {
            *fb_itor = panel->fbs[i];
            fb_itor = va_arg(args, void **);
        }
    }
    va_end(args);
    return ESP_OK;
}

esp_err_t esp_lcd_st77903_qspi_stop_refresh(esp_lcd_panel_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    st77903_qspi_panel_t *panel = __containerof(handle, st77903_qspi_panel_t, base);
    uint32_t task_wait_ms = 0;

    // Stop refrehs & load mem task
    if (panel->task.refresh_task_handle) {
        panel->flags.refresh_task_end = true;
        while ((eTaskGetState(panel->task.refresh_task_handle) != eDeleted) && (task_wait_ms < TASK_STOP_WAIT_MS)) {
            vTaskDelay(pdTICKS_TO_MS(TASK_CHECK_TIME_MS));
            task_wait_ms += TASK_CHECK_TIME_MS;
        }
        ESP_RETURN_ON_FALSE(task_wait_ms < TASK_STOP_WAIT_MS, ESP_ERR_TIMEOUT, TAG, "Stop refresh task timeout");
        panel->task.refresh_task_handle = NULL;
    }

    if (panel->task.load_task_handle) {
        while ((eTaskGetState(panel->task.load_task_handle) != eDeleted) && (task_wait_ms < TASK_STOP_WAIT_MS)) {
            vTaskDelay(pdTICKS_TO_MS(TASK_CHECK_TIME_MS));
            task_wait_ms += TASK_CHECK_TIME_MS;
        }
        ESP_RETURN_ON_FALSE(task_wait_ms < TASK_STOP_WAIT_MS, ESP_ERR_TIMEOUT, TAG, "Stop load task timeout");
        panel->task.load_task_handle = NULL;
    }

    return ESP_OK;
}

static void load_trans_pool(st77903_qspi_panel_t *panel, spi_multi_transaction_t *seg_trans, bool in_isr)
{
    uint8_t *load_buf;
    uint16_t trans_num = MIN(panel->trans_pool_size, panel->v_res - panel->load_line);
    void **fbs = panel->fbs;
    void **bbs = panel->bbs;
    BaseType_t need_yield = pdFALSE;

    if (panel->flags.fb_in_psram) {
        // If frame buffers are in PSRAM, they can't be transmitted by SPI DMA currently, so we have to use bounce buffers to transmit them
        // Here, we assemble the addresses to queue and use load memory task to copy data from frame buffer to bounce buffer
        load_buf = (uint8_t *)fbs[panel->cur_bb_fb_index] + panel->load_line * panel->bytes_per_line;
        queue_load_mem_info_t load_info = {
            .from = (void *)load_buf,
            .to = (void *)bbs[panel->load_bb_index],
            .bytes = trans_num * panel->bytes_per_line,
        };
        if (in_isr) {
            xQueueSendFromISR(panel->queue_load_mem_info, &load_info, &need_yield);
        } else {
            xQueueSend(panel->queue_load_mem_info, &load_info, portMAX_DELAY);
        }
        load_buf = (uint8_t *)(bbs[panel->load_bb_index]);
        panel->load_bb_index++;
        if (panel->load_bb_index >= panel->trans_pool_num) {
            panel->load_bb_index = 0;
        }
    } else {
        // If frame buffers are in SRAM, they can be transmitted by SPI DMA directly
        // So, we should release semaphore to allow refresh task continue
        load_buf = (uint8_t *)fbs[panel->cur_fb_index] + panel->load_line * panel->bytes_per_line;
        if (in_isr) {
            xSemaphoreGiveFromISR(panel->sem_count_free_trans, &need_yield);
        } else {
            xSemaphoreGive(panel->sem_count_free_trans);
        }
    }

    for (int i = 0; i < trans_num; i++) {
        seg_trans[i].base.tx_buffer = (void *)load_buf;
        load_buf += panel->bytes_per_line;
    }
    panel->load_line += trans_num;
    if (panel->load_line >= panel->v_res) {
        panel->load_line = 0;
    }

    if (need_yield == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void post_trans_color_cb(spi_transaction_t *trans)
{
    st77903_qspi_panel_t *panel = (st77903_qspi_panel_t *)trans->user;

    if (panel) {
        load_trans_pool(panel, (spi_multi_transaction_t *)trans, true);
    }
}

static esp_err_t lcd_write_cmd(st77903_qspi_panel_t *panel, uint8_t cmd, void *param, size_t param_size)
{
    static spi_multi_transaction_t send_seg = {
        .base = {
            .cmd = LCD_INS_CMD,
            .user = NULL,
        },
     };

    if (param_size == 0) {
        param = NULL;
    }

    send_seg.base.addr = ((uint32_t)cmd) << 8;
    send_seg.base.length = param_size * 8;
    send_seg.base.tx_buffer = param;
    ESP_RETURN_ON_ERROR(spi_device_queue_multi_trans(panel->spi_dev, &send_seg, 1, portMAX_DELAY), TAG, "SPI segment trans failed");
    esp_rom_delay_us(LCD_DELAY_MIN_US);

    return ESP_OK;
}

static esp_err_t lcd_write_color(st77903_qspi_panel_t *panel)
{
    int row_count = panel->v_res;
    int pool_size = 0;

    do {
        pool_size = MIN(row_count, panel->trans_pool_size);
        xSemaphoreTake(panel->sem_count_free_trans, portMAX_DELAY);
        ESP_RETURN_ON_ERROR(spi_device_queue_multi_trans(panel->spi_dev, panel->trans_pool[panel->write_pool_index], pool_size,
                                                         portMAX_DELAY), TAG, "SPI segment trans failed");
        panel->write_pool_index++;
        if (panel->write_pool_index >= panel->trans_pool_num) {
            panel->write_pool_index = 0;
        }
        row_count -= pool_size;
    } while(row_count > 0);

    // Wait until all segments were finished
    for (int i = 0; i < panel->trans_pool_num; i++) {
        xSemaphoreTake(panel->sem_count_free_trans, portMAX_DELAY);
    }
    for (int i = 0; i < panel->trans_pool_num; i++) {
        xSemaphoreGive(panel->sem_count_free_trans);
    }

    if (panel->flags.fb_in_psram) {
        panel->cur_bb_fb_index = panel->next_fb_index;
    }
    panel->cur_fb_index = panel->next_fb_index;

    return ESP_OK;
}

static void refresh_task(void *arg)
{
    st77903_qspi_panel_t *panel = (st77903_qspi_panel_t *)arg;
    uint16_t frame_cnt = 0;
    int64_t start_time = 0;
    esp_err_t ret = ESP_OK;

    ESP_LOGI(TAG, "Refresh task start");
    if (panel->flags.print_fps_log) {
        start_time  = esp_timer_get_time();
    }

    while (!panel->flags.refresh_task_end) {
        // Send a vsync command
        ESP_GOTO_ON_ERROR(lcd_write_cmd(panel, LCD_CMD_VSYNC, NULL, 0), end, TAG, "LCD write vsync cmd failed");

        // Send hsync commands
        for (int i = 0; i < LCD_VSYNC_BACK; i++) {
            ESP_GOTO_ON_ERROR(lcd_write_cmd(panel, LCD_CMD_HSYNC, NULL, 0), end, TAG, "LCD write hsync cmd failed");
        }

        // Send the whole frame data
        ESP_GOTO_ON_ERROR(lcd_write_color(panel), end, TAG, "LCD write color failed");

        // Send hsync commands
        for (int i = 0; i < LCD_VSYNC_FRONT; i++) {
            ESP_GOTO_ON_ERROR(lcd_write_cmd(panel, LCD_CMD_HSYNC, NULL, 0), end, TAG, "LCD write hsync cmd failed");
        }

        if (panel->frame_done_cb) {
            panel->frame_done_cb();
        }

        vTaskDelay(pdMS_TO_TICKS(panel->refresh_delay_ms));

        if (panel->flags.print_fps_log) {
            if (++frame_cnt == 100) {
                frame_cnt = 0;
                ESP_LOGI(TAG, "FPS: %d", (int)(100000000ULL / (esp_timer_get_time() - start_time)));
                start_time = esp_timer_get_time();
            }
        }
    }

end:
    panel->flags.load_task_end = true;
    ESP_LOGW(TAG, "Refresh task stop (%s)", esp_err_to_name(ret));
    vTaskDelete(NULL);
}

static void load_memory_task(void *arg)
{
    st77903_qspi_panel_t *panel = (st77903_qspi_panel_t *)arg;
    BaseType_t result = pdFALSE;
    queue_load_mem_info_t load_info;

    ESP_LOGI(TAG, "Load memory task start");
    while (1) {
        result = pdFALSE;
        while (result == pdFALSE) {
            result = xQueueReceive(panel->queue_load_mem_info, &load_info, pdMS_TO_TICKS(TASK_CHECK_TIME_MS));
            if (panel->flags.load_task_end) {
                panel->flags.load_task_end = false;
                goto end;
            }
        }
        memcpy(load_info.to, load_info.from, load_info.bytes);
        xSemaphoreGive(panel->sem_count_free_trans);
    }

end:
    ESP_LOGW(TAG, "Load memory task stop");
    vTaskDelete(NULL);
}

/* Please modify the below commands according to the actual situation */
#define LCD_CMD_DATA_BYTES_MAX      (14)

typedef struct {
    uint8_t cmd;
    uint8_t data[LCD_CMD_DATA_BYTES_MAX];
    uint16_t data_bytes;
} lcd_init_cmd_t;

static lcd_init_cmd_t vendor_specific_init[] = {
    {0xf0, {0xc3}, 1},
    {0xf0, {0x96}, 1},
    {0xf0, {0xa5}, 1},
    {0xe9, {0x20}, 1},
    {0xe7, {0x80, 0x77, 0x1f, 0xcc}, 4},
    {0xc1, {0x77, 0x07, 0xcf, 0x16}, 4},
    {0xc2, {0x77, 0x07, 0xcf, 0x16}, 4},
    {0xc3, {0x22, 0x02, 0x22, 0x04}, 4},
    {0xc4, {0x22, 0x02, 0x22, 0x04}, 4},
    {0xc5, {0xed},1},
    {0xe0, {0x87, 0x09, 0x0c, 0x06, 0x05, 0x03, 0x29, 0x32, 0x49, 0x0f, 0x1b, 0x17, 0x2a, 0x2f}, 14},
    {0xe1, {0x87, 0x09, 0x0c, 0x06, 0x05, 0x03, 0x29, 0x32, 0x49, 0x0f, 0x1b, 0x17, 0x2a, 0x2f}, 14},
    {0xe5, {0xbe, 0xf5, 0xb1, 0x22, 0x22, 0x25, 0x10, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22}, 14},
    {0xe6, {0xbe, 0xf5, 0xb1, 0x22, 0x22, 0x25, 0x10, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22}, 14},
    {0xec, {0x40, 0x03},2},
    {0x36, {0x0c}, 1},
    {0xb2, {0x00},1},
    {0xb3, {0x01},1},
    {0xb4, {0x00},1},
    {0xa5, {0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x2a, 0x8a, 0x02}, 9},
    {0xa6, {0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x2a, 0x8a, 0x02}, 9},
    {0xba, {0x0a, 0x5a, 0x23, 0x10, 0x25, 0x02, 0x00}, 7},
    {0xbb, {0x00, 0x30, 0x00, 0x2c, 0x82, 0x87, 0x18, 0x00}, 8},
    {0xbc, {0x00, 0x30, 0x00, 0x2c, 0x82, 0x87, 0x18, 0x00}, 8},
    {0xbd, {0xa1, 0xb2, 0x2b, 0x1a, 0x56, 0x43, 0x34, 0x65, 0xff, 0xff, 0x0f}, 11},
    {0x35, {0x00}, 1},
    {0x21, {0x00}, 0},

    {0x00, {0x00}, 0xff},
};

static esp_err_t lcd_cmd_config(st77903_qspi_panel_t *panel)
{
    // Initialize LCD
    // Vendor specific initialization, it can be different between manufacturers
    // Should consult the LCD supplier for initialization sequence code
    for (int i = 0; vendor_specific_init[i].data_bytes != 0xff; i++) {
        ESP_RETURN_ON_ERROR(
            lcd_write_cmd(panel, vendor_specific_init[i].cmd, vendor_specific_init[i].data, vendor_specific_init[i].data_bytes),
            TAG, "QSPI write cmd failed"
        );
    }

    uint8_t NL = panel->v_res / 2 - 1;
    uint8_t NC = panel->h_res / 8 - 1;
    lcd_write_cmd(panel, LCD_CMD_DISCN, (uint8_t []){NL, NC}, 2);       // Resolution
    lcd_write_cmd(panel, LCD_CMD_BPC, (uint8_t []){0x00, LCD_VSYNC_FRONT, 0x00, LCD_VSYNC_BACK}, 4);    // VFP & VBP
    if (panel->bytes_per_pixel == 2) {
        lcd_write_cmd(panel, LCD_CMD_MOLMOD, (uint8_t []){ 0x05 }, 1);  // RGB565
    } else if (panel->bytes_per_pixel == 3) {
        lcd_write_cmd(panel, LCD_CMD_MOLMOD, (uint8_t []){ 0x07 }, 1);  // RGB888
    }
    lcd_write_cmd(panel, 0x21, NULL, 0);
    lcd_write_cmd(panel, 0x11, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(120));
    lcd_write_cmd(panel, 0x29, NULL, 0);

    return ESP_OK;
}
/*********************************************************************/
