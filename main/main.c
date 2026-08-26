#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "lv_demos.h"
#include "esp_lcd_sh8601.h"
#include "i2c_bsp.h"
#include "lcd_touch_bsp.h"
#include "user_config.h"
#include "lcd_bl_pwm_bsp.h"
#include "user_encoder_bsp.h"

static const char *TAG = "example";
static SemaphoreHandle_t lvgl_mux = NULL;

static lv_obj_t *menu_cont = NULL;
static volatile int32_t encoder_steps = 0;

#if CONFIG_LV_COLOR_DEPTH == 32
#define LCD_BIT_PER_PIXEL (24)
#elif CONFIG_LV_COLOR_DEPTH == 16
#define LCD_BIT_PER_PIXEL (16)
#endif
// d5-d7
static const sh8601_lcd_init_cmd_t lcd_init_cmds[] = {
    {0xF0, (uint8_t[]){0x28}, 1, 0},
    {0xF2, (uint8_t[]){0x28}, 1, 0},
    {0x73, (uint8_t[]){0xF0}, 1, 0},
    {0x7C, (uint8_t[]){0xD1}, 1, 0},
    {0x83, (uint8_t[]){0xE0}, 1, 0},
    {0x84, (uint8_t[]){0x61}, 1, 0},
    {0xF2, (uint8_t[]){0x82}, 1, 0},
    {0xF0, (uint8_t[]){0x00}, 1, 0},
    {0xF0, (uint8_t[]){0x01}, 1, 0},
    {0xF1, (uint8_t[]){0x01}, 1, 0},
    {0xB0, (uint8_t[]){0x56}, 1, 0},
    {0xB1, (uint8_t[]){0x4D}, 1, 0},
    {0xB2, (uint8_t[]){0x24}, 1, 0},
    {0xB4, (uint8_t[]){0x87}, 1, 0},
    {0xB5, (uint8_t[]){0x44}, 1, 0},
    {0xB6, (uint8_t[]){0x8B}, 1, 0},
    {0xB7, (uint8_t[]){0x40}, 1, 0},
    {0xB8, (uint8_t[]){0x86}, 1, 0},
    {0xBA, (uint8_t[]){0x00}, 1, 0},
    {0xBB, (uint8_t[]){0x08}, 1, 0},
    {0xBC, (uint8_t[]){0x08}, 1, 0},
    {0xBD, (uint8_t[]){0x00}, 1, 0},
    {0xC0, (uint8_t[]){0x80}, 1, 0},
    {0xC1, (uint8_t[]){0x10}, 1, 0},
    {0xC2, (uint8_t[]){0x37}, 1, 0},
    {0xC3, (uint8_t[]){0x80}, 1, 0},
    {0xC4, (uint8_t[]){0x10}, 1, 0},
    {0xC5, (uint8_t[]){0x37}, 1, 0},
    {0xC6, (uint8_t[]){0xA9}, 1, 0},
    {0xC7, (uint8_t[]){0x41}, 1, 0},
    {0xC8, (uint8_t[]){0x01}, 1, 0},
    {0xC9, (uint8_t[]){0xA9}, 1, 0},
    {0xCA, (uint8_t[]){0x41}, 1, 0},
    {0xCB, (uint8_t[]){0x01}, 1, 0},
    {0xD0, (uint8_t[]){0x91}, 1, 0},
    {0xD1, (uint8_t[]){0x68}, 1, 0},
    {0xD2, (uint8_t[]){0x68}, 1, 0},
    {0xF5, (uint8_t[]){0x00, 0xA5}, 2, 0},
    {0xDD, (uint8_t[]){0x4F}, 1, 0},
    {0xDE, (uint8_t[]){0x4F}, 1, 0},
    {0xF1, (uint8_t[]){0x10}, 1, 0},
    {0xF0, (uint8_t[]){0x00}, 1, 0},
    {0xF0, (uint8_t[]){0x02}, 1, 0},
    {0xE0, (uint8_t[]){0xF0, 0x0A, 0x10, 0x09, 0x09, 0x36, 0x35, 0x33, 0x4A, 0x29, 0x15, 0x15, 0x2E, 0x34}, 14, 0},
    {0xE1, (uint8_t[]){0xF0, 0x0A, 0x0F, 0x08, 0x08, 0x05, 0x34, 0x33, 0x4A, 0x39, 0x15, 0x15, 0x2D, 0x33}, 14, 0},
    {0xF0, (uint8_t[]){0x10}, 1, 0},
    {0xF3, (uint8_t[]){0x10}, 1, 0},
    {0xE0, (uint8_t[]){0x07}, 1, 0},
    {0xE1, (uint8_t[]){0x00}, 1, 0},
    {0xE2, (uint8_t[]){0x00}, 1, 0},
    {0xE3, (uint8_t[]){0x00}, 1, 0},
    {0xE4, (uint8_t[]){0xE0}, 1, 0},
    {0xE5, (uint8_t[]){0x06}, 1, 0},
    {0xE6, (uint8_t[]){0x21}, 1, 0},
    {0xE7, (uint8_t[]){0x01}, 1, 0},
    {0xE8, (uint8_t[]){0x05}, 1, 0},
    {0xE9, (uint8_t[]){0x02}, 1, 0},
    {0xEA, (uint8_t[]){0xDA}, 1, 0},
    {0xEB, (uint8_t[]){0x00}, 1, 0},
    {0xEC, (uint8_t[]){0x00}, 1, 0},
    {0xED, (uint8_t[]){0x0F}, 1, 0},
    {0xEE, (uint8_t[]){0x00}, 1, 0},
    {0xEF, (uint8_t[]){0x00}, 1, 0},
    {0xF8, (uint8_t[]){0x00}, 1, 0},
    {0xF9, (uint8_t[]){0x00}, 1, 0},
    {0xFA, (uint8_t[]){0x00}, 1, 0},
    {0xFB, (uint8_t[]){0x00}, 1, 0},
    {0xFC, (uint8_t[]){0x00}, 1, 0},
    {0xFD, (uint8_t[]){0x00}, 1, 0},
    {0xFE, (uint8_t[]){0x00}, 1, 0},
    {0xFF, (uint8_t[]){0x00}, 1, 0},
    {0x60, (uint8_t[]){0x40}, 1, 0},
    {0x61, (uint8_t[]){0x04}, 1, 0},
    {0x62, (uint8_t[]){0x00}, 1, 0},
    {0x63, (uint8_t[]){0x42}, 1, 0},
    {0x64, (uint8_t[]){0xD9}, 1, 0},
    {0x65, (uint8_t[]){0x00}, 1, 0},
    {0x66, (uint8_t[]){0x00}, 1, 0},
    {0x67, (uint8_t[]){0x00}, 1, 0},
    {0x68, (uint8_t[]){0x00}, 1, 0},
    {0x69, (uint8_t[]){0x00}, 1, 0},
    {0x6A, (uint8_t[]){0x00}, 1, 0},
    {0x6B, (uint8_t[]){0x00}, 1, 0},
    {0x70, (uint8_t[]){0x40}, 1, 0},
    {0x71, (uint8_t[]){0x03}, 1, 0},
    {0x72, (uint8_t[]){0x00}, 1, 0},
    {0x73, (uint8_t[]){0x42}, 1, 0},
    {0x74, (uint8_t[]){0xD8}, 1, 0},
    {0x75, (uint8_t[]){0x00}, 1, 0},
    {0x76, (uint8_t[]){0x00}, 1, 0},
    {0x77, (uint8_t[]){0x00}, 1, 0},
    {0x78, (uint8_t[]){0x00}, 1, 0},
    {0x79, (uint8_t[]){0x00}, 1, 0},
    {0x7A, (uint8_t[]){0x00}, 1, 0},
    {0x7B, (uint8_t[]){0x00}, 1, 0},
    {0x80, (uint8_t[]){0x48}, 1, 0},
    {0x81, (uint8_t[]){0x00}, 1, 0},
    {0x82, (uint8_t[]){0x06}, 1, 0},
    {0x83, (uint8_t[]){0x02}, 1, 0},
    {0x84, (uint8_t[]){0xD6}, 1, 0},
    {0x85, (uint8_t[]){0x04}, 1, 0},
    {0x86, (uint8_t[]){0x00}, 1, 0},
    {0x87, (uint8_t[]){0x00}, 1, 0},
    {0x88, (uint8_t[]){0x48}, 1, 0},
    {0x89, (uint8_t[]){0x00}, 1, 0},
    {0x8A, (uint8_t[]){0x08}, 1, 0},
    {0x8B, (uint8_t[]){0x02}, 1, 0},
    {0x8C, (uint8_t[]){0xD8}, 1, 0},
    {0x8D, (uint8_t[]){0x04}, 1, 0},
    {0x8E, (uint8_t[]){0x00}, 1, 0},
    {0x8F, (uint8_t[]){0x00}, 1, 0},
    {0x90, (uint8_t[]){0x48}, 1, 0},
    {0x91, (uint8_t[]){0x00}, 1, 0},
    {0x92, (uint8_t[]){0x0A}, 1, 0},
    {0x93, (uint8_t[]){0x02}, 1, 0},
    {0x94, (uint8_t[]){0xDA}, 1, 0},
    {0x95, (uint8_t[]){0x04}, 1, 0},
    {0x96, (uint8_t[]){0x00}, 1, 0},
    {0x97, (uint8_t[]){0x00}, 1, 0},
    {0x98, (uint8_t[]){0x48}, 1, 0},
    {0x99, (uint8_t[]){0x00}, 1, 0},
    {0x9A, (uint8_t[]){0x0C}, 1, 0},
    {0x9B, (uint8_t[]){0x02}, 1, 0},
    {0x9C, (uint8_t[]){0xDC}, 1, 0},
    {0x9D, (uint8_t[]){0x04}, 1, 0},
    {0x9E, (uint8_t[]){0x00}, 1, 0},
    {0x9F, (uint8_t[]){0x00}, 1, 0},
    {0xA0, (uint8_t[]){0x48}, 1, 0},
    {0xA1, (uint8_t[]){0x00}, 1, 0},
    {0xA2, (uint8_t[]){0x05}, 1, 0},
    {0xA3, (uint8_t[]){0x02}, 1, 0},
    {0xA4, (uint8_t[]){0xD5}, 1, 0},
    {0xA5, (uint8_t[]){0x04}, 1, 0},
    {0xA6, (uint8_t[]){0x00}, 1, 0},
    {0xA7, (uint8_t[]){0x00}, 1, 0},
    {0xA8, (uint8_t[]){0x48}, 1, 0},
    {0xA9, (uint8_t[]){0x00}, 1, 0},
    {0xAA, (uint8_t[]){0x07}, 1, 0},
    {0xAB, (uint8_t[]){0x02}, 1, 0},
    {0xAC, (uint8_t[]){0xD7}, 1, 0},
    {0xAD, (uint8_t[]){0x04}, 1, 0},
    {0xAE, (uint8_t[]){0x00}, 1, 0},
    {0xAF, (uint8_t[]){0x00}, 1, 0},
    {0xB0, (uint8_t[]){0x48}, 1, 0},
    {0xB1, (uint8_t[]){0x00}, 1, 0},
    {0xB2, (uint8_t[]){0x09}, 1, 0},
    {0xB3, (uint8_t[]){0x02}, 1, 0},
    {0xB4, (uint8_t[]){0xD9}, 1, 0},
    {0xB5, (uint8_t[]){0x04}, 1, 0},
    {0xB6, (uint8_t[]){0x00}, 1, 0},
    {0xB7, (uint8_t[]){0x00}, 1, 0},
    {0xB8, (uint8_t[]){0x48}, 1, 0},
    {0xB9, (uint8_t[]){0x00}, 1, 0},
    {0xBA, (uint8_t[]){0x0B}, 1, 0},
    {0xBB, (uint8_t[]){0x02}, 1, 0},
    {0xBC, (uint8_t[]){0xDB}, 1, 0},
    {0xBD, (uint8_t[]){0x04}, 1, 0},
    {0xBE, (uint8_t[]){0x00}, 1, 0},
    {0xBF, (uint8_t[]){0x00}, 1, 0},
    {0xC0, (uint8_t[]){0x10}, 1, 0},
    {0xC1, (uint8_t[]){0x47}, 1, 0},
    {0xC2, (uint8_t[]){0x56}, 1, 0},
    {0xC3, (uint8_t[]){0x65}, 1, 0},
    {0xC4, (uint8_t[]){0x74}, 1, 0},
    {0xC5, (uint8_t[]){0x88}, 1, 0},
    {0xC6, (uint8_t[]){0x99}, 1, 0},
    {0xC7, (uint8_t[]){0x01}, 1, 0},
    {0xC8, (uint8_t[]){0xBB}, 1, 0},
    {0xC9, (uint8_t[]){0xAA}, 1, 0},
    {0xD0, (uint8_t[]){0x10}, 1, 0},
    {0xD1, (uint8_t[]){0x47}, 1, 0},
    {0xD2, (uint8_t[]){0x56}, 1, 0},
    {0xD3, (uint8_t[]){0x65}, 1, 0},
    {0xD4, (uint8_t[]){0x74}, 1, 0},
    {0xD5, (uint8_t[]){0x88}, 1, 0},
    {0xD6, (uint8_t[]){0x99}, 1, 0},
    {0xD7, (uint8_t[]){0x01}, 1, 0},
    {0xD8, (uint8_t[]){0xBB}, 1, 0},
    {0xD9, (uint8_t[]){0xAA}, 1, 0},
    {0xF3, (uint8_t[]){0x01}, 1, 0},
    {0xF0, (uint8_t[]){0x00}, 1, 0},
    {0x21, (uint8_t[]){0x00}, 1, 0},
    {0x11, (uint8_t[]){0x00}, 1, 120},
    {0x29, (uint8_t[]){0x00}, 1, 0},
    // Rotate SH8601 display 180 degrees in hardware
    {0x36, (uint8_t[]){0xC0}, 1, 0},
};

static void create_main_menu(void);
static void create_second_menu(void);
static void open_second_menu_cb(lv_event_t *e);
static void back_to_main_menu_cb(lv_event_t *e);

static void open_second_menu_cb(lv_event_t *e)
{
    if (menu_cont != NULL)
    {
        lv_obj_delete(menu_cont);
        menu_cont = NULL;
    }

    create_second_menu();
}
static void create_main_menu(void)
{
    menu_cont = lv_obj_create(lv_screen_active());

    lv_obj_set_size(menu_cont, 360, 360);
    lv_obj_center(menu_cont);

    lv_obj_set_flex_flow(
        menu_cont,
        LV_FLEX_FLOW_COLUMN);

    lv_obj_set_flex_align(
        menu_cont,
        LV_FLEX_ALIGN_START,
        LV_FLEX_ALIGN_CENTER,
        LV_FLEX_ALIGN_CENTER);

    lv_obj_set_scroll_snap_y(
        menu_cont,
        LV_SCROLL_SNAP_CENTER);

    lv_obj_set_style_pad_top(menu_cont, 140, 0);
    lv_obj_set_style_pad_bottom(menu_cont, 140, 0);
    lv_obj_set_style_pad_row(menu_cont, 20, 0);

    lv_obj_t *label1 =
        lv_label_create(menu_cont);

    lv_label_set_text(
        label1,
        "Main Menu");

    lv_obj_t *btn1 =
        lv_button_create(menu_cont);

    lv_obj_set_size(
        btn1,
        200,
        50);

    lv_obj_t *btn_lbl =
        lv_label_create(btn1);

    lv_label_set_text(
        btn_lbl,
        "Open Settings");

    lv_obj_center(btn_lbl);

    /*
     * This is what makes touching the button
     * open the second menu.
     */
    lv_obj_add_event_cb(
        btn1,
        open_second_menu_cb,
        LV_EVENT_CLICKED,
        NULL);
}

static void create_second_menu(void)
{
    menu_cont = lv_obj_create(lv_screen_active());

    lv_obj_set_size(menu_cont, 360, 360);
    lv_obj_center(menu_cont);

    lv_obj_set_flex_flow(
        menu_cont,
        LV_FLEX_FLOW_COLUMN);

    lv_obj_set_flex_align(
        menu_cont,
        LV_FLEX_ALIGN_START,
        LV_FLEX_ALIGN_CENTER,
        LV_FLEX_ALIGN_CENTER);

    lv_obj_set_scroll_snap_y(
        menu_cont,
        LV_SCROLL_SNAP_CENTER);

    lv_obj_set_style_pad_top(menu_cont, 140, 0);
    lv_obj_set_style_pad_bottom(menu_cont, 140, 0);
    lv_obj_set_style_pad_row(menu_cont, 20, 0);

    lv_obj_t *title =
        lv_label_create(menu_cont);

    lv_label_set_text(
        title,
        "Settings");

    lv_obj_t *brightness =
        lv_slider_create(menu_cont);

    lv_obj_set_size(
        brightness,
        200,
        20);

    lv_slider_set_range(
        brightness,
        0,
        100);

    lv_slider_set_value(
        brightness,
        75,
        LV_ANIM_OFF);

    lv_obj_t *volume =
        lv_arc_create(menu_cont);

    lv_obj_set_size(
        volume,
        150,
        150);

    lv_arc_set_range(
        volume,
        0,
        100);

    lv_arc_set_value(
        volume,
        50);

    lv_obj_t *back_btn =
        lv_button_create(menu_cont);

    lv_obj_set_size(
        back_btn,
        200,
        50);

    lv_obj_t *back_label =
        lv_label_create(back_btn);

    lv_label_set_text(
        back_label,
        "Back");

    lv_obj_center(back_label);

    lv_obj_add_event_cb(
        back_btn,
        back_to_main_menu_cb,
        LV_EVENT_CLICKED,
        NULL);
}

static void back_to_main_menu_cb(lv_event_t *e)
{
    if (menu_cont != NULL)
    {
        lv_obj_delete(menu_cont);
        menu_cont = NULL;
    }

    create_main_menu();
}

static bool notify_lvgl_flush_ready(
    esp_lcd_panel_io_handle_t panel_io,
    esp_lcd_panel_io_event_data_t *edata,
    void *user_ctx)
{
    lv_display_t *disp =
        (lv_display_t *)user_ctx;

    if (disp != NULL)
    {
        lv_display_flush_ready(disp);
    }

    return false;
}

static void lvgl_flush_cb(
    lv_display_t *disp,
    const lv_area_t *area,
    uint8_t *px_map)
{
    esp_lcd_panel_handle_t panel_handle =
        (esp_lcd_panel_handle_t)
            lv_display_get_user_data(disp);

    uint32_t pixel_count =
        (area->x2 - area->x1 + 1) *
        (area->y2 - area->y1 + 1);

    uint16_t *pixels = (uint16_t *)px_map;

    // SH8601 expects opposite RGB565 byte order
    for (uint32_t i = 0; i < pixel_count; i++)
    {
        pixels[i] =
            (pixels[i] >> 8) |
            (pixels[i] << 8);
    }

    esp_lcd_panel_draw_bitmap(
        panel_handle,
        area->x1,
        area->y1,
        area->x2 + 1,
        area->y2 + 1,
        px_map);
}

static void lvgl_rounder_event_cb(lv_event_t *e)
{
    lv_area_t *area = lv_event_get_param(e);

    uint16_t x1 = area->x1;
    uint16_t x2 = area->x2;
    uint16_t y1 = area->y1;
    uint16_t y2 = area->y2;

    area->x1 = (x1 >> 1) << 1;
    area->y1 = (y1 >> 1) << 1;

    area->x2 = ((x2 >> 1) << 1) + 1;
    area->y2 = ((y2 >> 1) << 1) + 1;
}

#if USE_TOUCH
static void lvgl_touch_cb(
    lv_indev_t *indev,
    lv_indev_data_t *data)
{
    uint16_t tp_x;
    uint16_t tp_y;

    uint8_t touched =
        tpGetCoordinates(&tp_x, &tp_y);

    if (touched)
    {
        // Match hardware 180-degree LCD rotation
        data->point.x =
            (LCD_H_RES - 1) - tp_x;

        data->point.y =
            (LCD_V_RES - 1) - tp_y;

        if (data->point.x >= LCD_H_RES)
            data->point.x = LCD_H_RES - 1;

        if (data->point.y >= LCD_V_RES)
            data->point.y = LCD_V_RES - 1;

        data->state = LV_INDEV_STATE_PRESSED;
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}
#endif

static void increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static bool lvgl_lock(int timeout_ms)
{
    assert(lvgl_mux && "bsp_display_start must be called first");

    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(lvgl_mux, timeout_ticks) == pdTRUE;
}

static void lvgl_unlock(void)
{
    assert(lvgl_mux && "bsp_display_start must be called first");
    xSemaphoreGive(lvgl_mux);
}

static void lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
    while (1)
    {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        if (lvgl_lock(-1))
        {
            task_delay_ms = lv_timer_handler();
            // Release the mutex
            lvgl_unlock();
        }
        if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS)
        {
            task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
        }
        else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS)
        {
            task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

#ifdef Backlight_Testing
void backlight_test_task(void *arg)
{
    for (;;)
    {
        setUpduty(LCD_PWM_MODE_255);
        vTaskDelay(pdMS_TO_TICKS(1000));
        setUpduty(LCD_PWM_MODE_200);
        vTaskDelay(pdMS_TO_TICKS(1000));
        setUpduty(LCD_PWM_MODE_150);
        vTaskDelay(pdMS_TO_TICKS(1000));
        setUpduty(LCD_PWM_MODE_100);
        vTaskDelay(pdMS_TO_TICKS(1000));
        setUpduty(LCD_PWM_MODE_50);
        vTaskDelay(pdMS_TO_TICKS(1000));
        setUpduty(LCD_PWM_MODE_0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
#endif

static void user_encoder_loop_task(void *arg)
{
    for (;;)
    {
        EventBits_t event = xEventGroupWaitBits(
            knob_even_,
            BIT_EVEN_ALL,
            pdTRUE,
            pdFALSE,
            portMAX_DELAY);

        if (READ_BIT(event, 0))
        {
            encoder_steps--;
        }

        if (READ_BIT(event, 1))
        {
            encoder_steps++;
        }
    }
}

static void encoder_scroll_timer_cb(lv_timer_t *timer)
{
    if (menu_cont == NULL)
    {
        return;
    }

    int32_t steps = encoder_steps;

    if (steps == 0)
    {
        return;
    }

    encoder_steps = 0;

    /*
     * Positive and negative directions may need reversing
     * depending on how the bezel feels physically.
     */
    const int32_t scroll_per_step = 20;

    lv_obj_scroll_by_bounded(
        menu_cont,
        0,
        -steps * scroll_per_step,
        LV_ANIM_OFF);
}
void app_main(void)
{

    ESP_LOGI(TAG, "ENTERED app_main");

    static lv_display_t *disp = NULL; // contains callback functions

    ESP_LOGI(TAG, "STEP 1: backlight");
    lcd_bl_pwm_bsp_init(LCD_PWM_MODE_255);

    ESP_LOGI(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg =
        {
            .data0_io_num = PIN_NUM_LCD_DATA0,
            .data1_io_num = PIN_NUM_LCD_DATA1,
            .sclk_io_num = PIN_NUM_LCD_PCLK,
            .data2_io_num = PIN_NUM_LCD_DATA2,
            .data3_io_num = PIN_NUM_LCD_DATA3,
            .max_transfer_sz = LCD_H_RES * LCD_V_RES * sizeof(uint16_t),
        };

    ESP_LOGI(TAG, "STEP 2: SPI init");
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "STEP 3: panel IO");
    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config = SH8601_PANEL_IO_QSPI_CONFIG(PIN_NUM_LCD_CS,
                                                                                notify_lvgl_flush_ready,
                                                                                NULL);
    sh8601_vendor_config_t vendor_config = {
        .init_cmds = lcd_init_cmds,
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]),
        .flags = {
            .use_qspi_interface = 1,
        },
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config,
    };
    ESP_LOGI(TAG, "STEP 4: install panel driver");
    ESP_LOGI(TAG, "Install SH8601 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_sh8601(io_handle, &panel_config, &panel_handle));

    ESP_LOGI(TAG, "STEP 5: panel reset");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));

    ESP_LOGI(TAG, "STEP 6: panel init");
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    ESP_LOGI(TAG, "STEP 7: I2C");
    i2c_master_Init(); // I2C_Init

#if USE_TOUCH
    lcd_touch_init();
#endif

    ESP_LOGI(TAG, "Initialize LVGL library");
    ESP_LOGI(TAG, "STEP 8: LVGL init");
    lv_init();

    ESP_LOGI(TAG, "Allocate LVGL draw buffers");

    const size_t draw_buf_size =
        LCD_H_RES *
        LVGL_BUF_HEIGHT *
        sizeof(uint16_t);

    uint8_t *buf1 = heap_caps_malloc(
        draw_buf_size,
        MALLOC_CAP_DMA);
    assert(buf1);

    uint8_t *buf2 = heap_caps_malloc(
        draw_buf_size,
        MALLOC_CAP_DMA);
    assert(buf2);

    ESP_LOGI(TAG, "Register display driver to LVGL");

    disp = lv_display_create(
        LCD_H_RES,
        LCD_V_RES);

    assert(disp);

    lv_display_set_color_format(
        disp,
        LV_COLOR_FORMAT_RGB565);

    lv_display_set_buffers(
        disp,
        buf1,
        buf2,
        draw_buf_size,
        LV_DISPLAY_RENDER_MODE_PARTIAL);

    lv_display_set_flush_cb(
        disp,
        lvgl_flush_cb);

    lv_display_set_user_data(
        disp,
        panel_handle);

    lv_display_add_event_cb(
        disp,
        lvgl_rounder_event_cb,
        LV_EVENT_INVALIDATE_AREA,
        NULL);

    const esp_lcd_panel_io_callbacks_t io_callbacks = {
        .on_color_trans_done = notify_lvgl_flush_ready,
    };

    ESP_ERROR_CHECK(
        esp_lcd_panel_io_register_event_callbacks(
            io_handle,
            &io_callbacks,
            disp));

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &increase_lvgl_tick,
        .name = "lvgl_tick"};
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

#if USE_TOUCH
    lv_indev_t *touch_indev =
        lv_indev_create();

    assert(touch_indev);

    lv_indev_set_type(
        touch_indev,
        LV_INDEV_TYPE_POINTER);

    lv_indev_set_display(
        touch_indev,
        disp);

    lv_indev_set_read_cb(
        touch_indev,
        lvgl_touch_cb);
#endif

    // ----------------------------
    // Encoder input
    // ----------------------------

    lvgl_mux = xSemaphoreCreateMutex();
    assert(lvgl_mux);
    xTaskCreate(lvgl_port_task, "LVGL", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);

#ifdef Backlight_Testing
    xTaskCreate(backlight_test_task, "backlight", 3 * 1024, NULL, 2, NULL);
#endif

    user_encoder_init();
    xTaskCreate(
        user_encoder_loop_task,
        "user_encoder_loop_task",
        3000,
        NULL,
        2,
        NULL);

    lv_timer_create(
        encoder_scroll_timer_cb,
        5,
        NULL);
    // draw items here

    ESP_LOGI(TAG, "Display UI");

    if (lvgl_lock(-1))
    {
        lv_obj_set_scrollbar_mode(
            lv_screen_active(),
            LV_SCROLLBAR_MODE_OFF);

        create_main_menu();

        lv_timer_create(
            encoder_scroll_timer_cb,
            5,
            NULL);

        lvgl_unlock();
    }
}
