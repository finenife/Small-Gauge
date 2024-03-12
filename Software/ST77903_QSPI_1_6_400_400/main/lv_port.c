/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_lcd_panel_ops.h"

#include "lvgl.h"
#include "esp_lcd_st77903_qspi.h"
#include "lv_port.h"

#define LVGL_TASK_PRIORITY          (CONFIG_BSP_DISPLAY_LVGL_TASK_PRIORITY)
#define LVGL_TASK_STACK_SIZE        (CONFIG_BSP_DISPLAY_LVGL_TASK_STACK_SIZE * 1024)
#define LVGL_TICK_PERIOD_MS         (CONFIG_BSP_DISPLAY_LVGL_TICK)
#define LVGL_BUFFER_HEIGHT          (CONFIG_BSP_DISPLAY_LVGL_BUF_HEIGHT)
#if CONFIG_BSP_DISPLAY_LVGL_PSRAM
#define LVGL_BUFFER_MALLOC          (MALLOC_CAP_SPIRAM)
#else
#define LVGL_BUFFER_MALLOC          (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
#endif

static const char *TAG = "lv_port";
static esp_lcd_panel_handle_t panel_handle = NULL;
static SemaphoreHandle_t lvgl_mux = NULL;                  // LVGL mutex
static TaskHandle_t lvgl_task_handle = NULL;

static void disp_init(void);
static void tick_init(void);
static void lv_task(void *arg);

void lv_port_init(void)
{
    lv_init();
    disp_init();
    tick_init();

    lvgl_mux = xSemaphoreCreateMutex();
    assert(lvgl_mux);
    xTaskCreate(lv_task, "lvgl", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, &lvgl_task_handle);
}

bool lv_port_lock(uint32_t timeout_ms)
{
    assert(lvgl_mux && "lv_port_start must be called first");

    const TickType_t timeout_ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(lvgl_mux, timeout_ticks) == pdTRUE;
}

void lv_port_unlock(void)
{
    assert(lvgl_mux && "lv_port_start must be called first");
    xSemaphoreGive(lvgl_mux);
}

#if CONFIG_BSP_DISPLAY_LVGL_DIRECT_MODE
// This function is located in ROM (also see esp_rom/${target}/ld/${target}.rom.ld)
extern int Cache_WriteBack_Addr(uint32_t addr, uint32_t size);

static inline void *lv_port_flush_get_next_buf(void *buf)
{
    lv_disp_t *disp = _lv_refr_get_disp_refreshing();
    lv_disp_draw_buf_t *draw_buf = disp->driver->draw_buf;
    return (buf == draw_buf->buf1) ? draw_buf->buf2 : draw_buf->buf1;
}

typedef struct {
    uint16_t inv_p;
    uint8_t inv_area_joined[LV_INV_BUF_SIZE];
    lv_area_t inv_areas[LV_INV_BUF_SIZE];
} lv_port_dirty_area_t;

static lv_port_dirty_area_t dirty_area;

static void lv_port_flush_dirty_save(lv_port_dirty_area_t *dirty_area)
{
    lv_disp_t *disp = _lv_refr_get_disp_refreshing();
    dirty_area->inv_p = disp->inv_p;
    for (int i = 0; i < disp->inv_p; i++) {
        dirty_area->inv_area_joined[i] = disp->inv_area_joined[i];
        dirty_area->inv_areas[i] = disp->inv_areas[i];
    }
}

/**
 * @brief Copy dirty area
 *
 * @note This function is used to avoid tearing effect, and only work with LVGL direct-mode.
 *
 */
static void lv_port_flush_dirty_copy(void *dst, void *src, lv_port_dirty_area_t *dirty_area)
{
    lv_disp_t *disp_refr = _lv_refr_get_disp_refreshing();

    lv_coord_t x_start, x_end, y_start, y_end;
    uint32_t copy_bytes_per_line;
    uint32_t bytes_to_flush;
    uint32_t h_res = disp_refr->driver->hor_res;
    uint32_t bytes_per_line = h_res * 2;
    uint8_t *from, *to;
    uint8_t *flush_ptr;
    for (int i = 0; i < disp_refr->inv_p; i++) {
        /* Refresh the unjoined areas*/
        if (disp_refr->inv_area_joined[i] == 0) {
            x_start = disp_refr->inv_areas[i].x1;
            x_end = disp_refr->inv_areas[i].x2 + 1;
            y_start = disp_refr->inv_areas[i].y1;
            y_end = disp_refr->inv_areas[i].y2 + 1;

            copy_bytes_per_line = (x_end - x_start) * 2;
            from = src + (y_start * h_res + x_start) * 2;
            to = dst + (y_start * h_res + x_start) * 2;
            for (int y = y_start; y < y_end; y++) {
                memcpy(to, from, copy_bytes_per_line);
                from += bytes_per_line;
                to += bytes_per_line;
            }
            bytes_to_flush = (y_end - y_start) * bytes_per_line;
            flush_ptr = dst + y_start * bytes_per_line;

            Cache_WriteBack_Addr((uint32_t)(flush_ptr), bytes_to_flush);
        }
    }
}

typedef enum {
    FLUSH_STATUS_PART,
    FLUSH_STATUS_FULL
} lv_port_flush_status_t;

typedef enum {
    FLUSH_PROBE_PART_COPY,
    FLUSH_PROBE_SKIP_COPY,
    FLUSH_PROBE_FULL_COPY,
} lv_port_flush_probe_t;

/**
 * @brief Probe dirty area to copy
 *
 * @note This function is used to avoid tearing effect, and only work with LVGL direct-mode.
 *
 */
lv_port_flush_probe_t lv_port_flush_copy_probe(void)
{
    static lv_port_flush_status_t prev_status = FLUSH_PROBE_PART_COPY;
    lv_port_flush_status_t cur_status;
    uint8_t probe_result;
    lv_disp_t *disp_refr = _lv_refr_get_disp_refreshing();

    uint32_t flush_ver = 0;
    uint32_t flush_hor = 0;
    for (int i = 0; i < disp_refr->inv_p; i++) {
        if (disp_refr->inv_area_joined[i] == 0) {
            flush_ver = (disp_refr->inv_areas[i].y2 + 1 - disp_refr->inv_areas[i].y1);
            flush_hor = (disp_refr->inv_areas[i].x2 + 1 - disp_refr->inv_areas[i].x1);
            break;
        }
    }
    /* Check if the current full screen refreshes */
    cur_status = ((flush_ver == LCD_V_RES) && (flush_hor == LCD_V_RES)) ? (FLUSH_STATUS_FULL) : (FLUSH_STATUS_PART);

    if (prev_status == FLUSH_STATUS_FULL) {
        if ((cur_status == FLUSH_STATUS_PART)) {
            probe_result = FLUSH_PROBE_FULL_COPY;
        } else {
            probe_result = FLUSH_PROBE_SKIP_COPY;
        }
    } else {
        probe_result = FLUSH_PROBE_PART_COPY;
    }
    prev_status = cur_status;

    return probe_result;
}
#endif

#if CONFIG_BSP_DISPLAY_LVGL_FULL_REFRESH && CONFIG_BSP_LCD_BUFFER_NUMS == 3
static void *lvgl_port_rgb_last_buf = NULL;
static void *lvgl_port_rgb_next_buf = NULL;
static void *lvgl_port_flush_next_buf = NULL;
#endif

static void lvgl_port_flush_callback(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    const int offsetx1 = area->x1;
    const int offsetx2 = area->x2;
    const int offsety1 = area->y1;
    const int offsety2 = area->y2;

#if CONFIG_BSP_DISPLAY_LVGL_FULL_REFRESH
#if CONFIG_BSP_LCD_BUFFER_NUMS == 3
    drv->draw_buf->buf1 = color_map;
    drv->draw_buf->buf2 = lvgl_port_flush_next_buf;
    lvgl_port_flush_next_buf = color_map;
#endif
    /* Due to full-refresh mode, here we just swtich pointer of frame buffer rather than draw bitmap */
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, (void *)color_map));
#if CONFIG_BSP_LCD_BUFFER_NUMS == 3
    lvgl_port_rgb_next_buf = color_map;
#else
    /* Waiting for the current frame buffer to complete transmission */
    ulTaskNotifyValueClear(NULL, ULONG_MAX);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
#endif
#elif CONFIG_BSP_DISPLAY_LVGL_DIRECT_MODE
    lv_port_flush_probe_t probe_result;
    /* Action after last area refresh */
    if (lv_disp_flush_is_last(drv)) {
        if (drv->full_refresh) {
            drv->full_refresh = 0;
            /* Due to direct-mode, here we just swtich driver's pointer of frame buffer rather than draw bitmap */
            ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, (void *)color_map));
            /* Waiting for the current frame buffer to complete transmission */
            ulTaskNotifyValueClear(NULL, ULONG_MAX);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            lv_port_flush_dirty_copy(lv_port_flush_get_next_buf(color_map), color_map, &dirty_area);
            drv->draw_buf->buf_act = (color_map == drv->draw_buf->buf1) ? drv->draw_buf->buf2 : drv->draw_buf->buf1;
        } else {
            /* Probe and copy dirty area from the current LVGL's frame buffer to the next LVGL's frame buffer */
            probe_result = lv_port_flush_copy_probe();
            if (probe_result == FLUSH_PROBE_FULL_COPY) {
                lv_port_flush_dirty_save(&dirty_area);
                /* Set LVGL full-refresh flag and force to refresh whole screen */
                drv->full_refresh = 1;
                lv_disp_flush_ready(drv);
                lv_refr_now(_lv_refr_get_disp_refreshing());
            } else {
                /* Due to direct-mode, here we just swtich driver's pointer of frame buffer rather than draw bitmap */
                ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, (void *)color_map));
                /* Waiting for the current frame buffer to complete transmission */
                ulTaskNotifyValueClear(NULL, ULONG_MAX);
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                if (probe_result == FLUSH_PROBE_PART_COPY) {
                    lv_port_flush_dirty_save(&dirty_area);
                    lv_port_flush_dirty_copy(lv_port_flush_get_next_buf(color_map), color_map, &dirty_area);
                }
            }
        }
    }
#else

    uint8_t *color24 = (uint8_t *)color_map;
    uint8_t *color32 = (uint8_t *)color_map;
    uint32_t size = 0;
    uint8_t temp;
    if (sizeof(lv_color_t) > 2) {
        size = lv_area_get_size(area);
        for(uint32_t i = 0; i < size; i++) {
            temp = color32[0];
            color24[0] = color32[2];
            color24[1] = color32[1];
            color24[2] = temp;
            color24+=3;
            color32+=4;
        }
    }

    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, (void *)color_map));

#endif

    lv_disp_flush_ready(drv);
}

#if CONFIG_BSP_DISPLAY_LVGL_AVOID_TEAR
static void lvgl_port_lcd_trans_done()
{
#if CONFIG_BSP_DISPLAY_LVGL_FULL_REFRESH && CONFIG_BSP_LCD_BUFFER_NUMS == 3
    if (lvgl_port_rgb_next_buf != lvgl_port_rgb_last_buf) {
        lvgl_port_flush_next_buf = lvgl_port_rgb_last_buf;
        lvgl_port_rgb_last_buf = lvgl_port_rgb_next_buf;
    }
#else
    if (lvgl_task_handle) {
        xTaskNotify(lvgl_task_handle, ULONG_MAX, eNoAction);
    }
#endif
}
#endif

static void disp_init(void)
{
    esp_lcd_st77903_qspi_config_t panel_config = {
        .qspi = {
            .host_id = LCD_SPI_HOST,
            .pclk_hz = LCD_SPI_PCLK,
            .cs_io_num = LCD_SPI_CS,
            .sclk_io_num = LCD_SPI_SCK,
            .data0_io_num = LCD_SPI_D0,
            .data1_io_num = LCD_SPI_D1,
            .data2_io_num = LCD_SPI_D2,
            .data3_io_num = LCD_SPI_D3,
        },
        .task = ST77903_QSPI_TASK_CONFIG_DEFAULT,
        .bits_per_pixel = (sizeof(lv_color_t) * 8) > 16 ? 24 : 16,
        .fb_num = CONFIG_BSP_LCD_BUFFER_NUMS,
        .trans_pool_size = 20,
        .trans_pool_num = 2,
        .flags = {
            .print_fps_log = 1,
#if CONFIG_IDF_TARGET_ESP32S3
            .fb_in_psram = 1,
#elif CONFIG_IDF_TARGET_ESP32C6
            .fb_in_psram = 0,
#endif
        },
        .reset_gpio_num = LCD_RST,
        .h_res = LCD_H_RES,
        .v_res = LCD_V_RES,
        .expect_fps = CONFIG_BSP_LCD_EXPECT_FPS,
#if CONFIG_BSP_DISPLAY_LVGL_AVOID_TEAR
        .frame_done_cb = lvgl_port_lcd_trans_done,
#endif
    };
    ESP_ERROR_CHECK(esp_lcd_new_st77903_qspi_panel(&panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    // ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, true));
    // ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    // ESP_ERROR_CHECK(esp_lcd_st77903_qspi_color_bar_test(panel_handle));
    // while (1) {
    //     vTaskDelay(pdMS_TO_TICKS(100));
    // }

    static lv_disp_drv_t disp_drv;
    static lv_disp_draw_buf_t disp_buf;
    lv_disp_drv_init(&disp_drv);

    void *buf1 = NULL;
    void *buf2 = NULL;
    int buffer_size;

#if CONFIG_BSP_DISPLAY_LVGL_AVOID_TEAR
    buffer_size = LCD_H_RES * LCD_V_RES;
#if CONFIG_BSP_DISPLAY_LVGL_FULL_REFRESH && CONFIG_BSP_LCD_BUFFER_NUMS == 3
    ESP_ERROR_CHECK(esp_lcd_st77903_qspi_get_frame_buffer(panel_handle, 3, &lvgl_port_rgb_last_buf, &buf1, &buf2));
    lvgl_port_rgb_next_buf = lvgl_port_rgb_last_buf;
    lvgl_port_flush_next_buf = buf2;
#else
    ESP_ERROR_CHECK(esp_lcd_st77903_qspi_get_frame_buffer(panel_handle, 2, &buf1, &buf2));
#endif
#else
    buffer_size = LCD_H_RES * LVGL_BUFFER_HEIGHT;
    buf1 = heap_caps_malloc(buffer_size * sizeof(lv_color_t), LVGL_BUFFER_MALLOC);
    assert(buf1);
#endif
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, buffer_size);

    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.flush_cb = lvgl_port_flush_callback;
#if CONFIG_BSP_DISPLAY_LVGL_FULL_REFRESH
    disp_drv.full_refresh = 1;
#elif CONFIG_BSP_DISPLAY_LVGL_DIRECT_MODE
    disp_drv.direct_mode = 1;
#endif
    lv_disp_drv_register(&disp_drv);
}

static void tick_increment(void *arg)
{
    /* Tell LVGL how many milliseconds have elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static void tick_init(void)
{
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &tick_increment,
        .name = "LVGL tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));
}

static void lv_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    while (1) {
        lv_port_lock(0);
        uint32_t task_delay_ms = lv_timer_handler();
        lv_port_unlock();
        if (task_delay_ms > 500) {
            task_delay_ms = 500;
        } else if (task_delay_ms < LVGL_TICK_PERIOD_MS) {
            task_delay_ms = LVGL_TICK_PERIOD_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}
