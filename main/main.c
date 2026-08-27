/* ============================================================================
 * Waveshare ESP32-S3-Knob-Touch-LCD-1.8
 * LVGL 9 application + hardware diagnostics
 *
 * Major sections:
 *   1. Includes and global state
 *   2. LCD initialization data
 *   3. Menu/navigation declarations
 *   4. DRV2605 haptic driver
 *   5. Menu implementation
 *   6. LVGL display/touch bridge
 *   7. FreeRTOS tasks
 *   8. app_main hardware initialization
 * ========================================================================== */

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
#include "esp_check.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "lvgl.h"
#include "lv_demos.h"
#include "esp_lcd_sh8601.h"
#include "i2c_bsp.h"
#include "lcd_touch_bsp.h"
#include "user_config.h"
#include "lcd_bl_pwm_bsp.h"
#include "user_encoder_bsp.h"

/* ============================================================================
 * 1. GLOBAL APPLICATION STATE
 * ========================================================================== */

/*
 * Shared ESP-IDF logging tag used by this source file.
 */
static const char *TAG = "example";

/*
 * LVGL mutex.
 *
 * LVGL is not generally thread-safe. FreeRTOS tasks must take this mutex before
 * manipulating LVGL objects or calling LVGL APIs from outside the LVGL task.
 */
static SemaphoreHandle_t lvgl_mux = NULL;

/*
 * Current scrollable menu container.
 *
 * The encoder scroll handler always acts on this pointer, so the same bezel
 * code works on Main, Settings, Hardware Tests, and every submenu.
 */
static lv_obj_t *menu_cont = NULL;

/*
 * Encoder movement shared between the hardware task and the LVGL timer.
 * The hardware task only accumulates raw steps; it never calls LVGL directly.
 */
static volatile int32_t encoder_steps = 0;

/* Number of vertical pixels applied for one bezel step. */
static int32_t encoder_scroll_per_step = 20;

/*
 * Hardware diagnostic page state.
 *
 * These LVGL object pointers are only valid while the corresponding test pages
 * are visible. show_menu() clears them before deleting the old container.
 */
static lv_obj_t *touch_test_label = NULL;
static lv_obj_t *encoder_test_label = NULL;
static int32_t encoder_test_count = 0;

/* Last PWM duty selected by the backlight test (0..255). */
static uint8_t backlight_test_level = 255;

/* Label used by the generic "driver not integrated" test page. */
static const char *unavailable_hw_name = "Peripheral";

/* ============================================================================
 * 2. DISPLAY CONFIGURATION
 * ========================================================================== */

#if CONFIG_LV_COLOR_DEPTH == 32
#define LCD_BIT_PER_PIXEL (24)
#elif CONFIG_LV_COLOR_DEPTH == 16
#define LCD_BIT_PER_PIXEL (16)
#endif
/*
 * SH8601 panel initialization table.
 * This is board/panel-specific data inherited from the Waveshare example.
 * Avoid changing individual register values unless debugging the panel itself.
 */
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

/* ============================================================================
 * 3. MENU / NAVIGATION DECLARATIONS
 * ========================================================================== */

typedef enum
{
    MENU_MAIN,
    MENU_SETTINGS,
    MENU_DISPLAY,
    MENU_AUDIO,
    MENU_INPUT,
    MENU_ABOUT,

    MENU_HARDWARE_TESTS,
    MENU_HW_DISPLAY,
    MENU_HW_TOUCH,
    MENU_HW_ENCODER,
    MENU_HW_BACKLIGHT,
    MENU_HW_MEMORY,
    MENU_HW_HAPTICS,
    MENU_HW_UNAVAILABLE
} menu_id_t;

/*
 * Menu navigation history.
 *
 * push_menu() stores the current page before moving forward.
 * pop_menu() restores the previous page when Back is pressed.
 */
#define MENU_STACK_SIZE 12

static menu_id_t current_menu = MENU_MAIN;
static menu_id_t menu_stack[MENU_STACK_SIZE];

/* -1 means the navigation history is empty. */
static int menu_stack_top = -1;

static lv_obj_t *create_menu_container(void);
static lv_obj_t *create_menu_button(lv_obj_t *parent, const char *text, lv_event_cb_t callback);

static void show_menu(menu_id_t menu);
static void push_menu(menu_id_t menu);
static void pop_menu(void);

static void create_main_menu(void);
static void create_settings_menu(void);
static void create_display_menu(void);
static void create_audio_menu(void);
static void create_input_menu(void);
static void create_about_menu(void);

static void create_hardware_tests_menu(void);
static void create_hw_display_test_menu(void);
static void create_hw_touch_test_menu(void);
static void create_hw_encoder_test_menu(void);
static void create_hw_backlight_test_menu(void);
static void create_hw_memory_test_menu(void);
static void create_hw_haptics_test_menu(void);
static void create_hw_unavailable_menu(void);

static void open_settings_cb(lv_event_t *e);
static void open_display_cb(lv_event_t *e);
static void open_audio_cb(lv_event_t *e);
static void open_input_cb(lv_event_t *e);
static void open_about_cb(lv_event_t *e);
static void open_hardware_tests_cb(lv_event_t *e);

static void open_hw_display_cb(lv_event_t *e);
static void open_hw_touch_cb(lv_event_t *e);
static void open_hw_encoder_cb(lv_event_t *e);
static void open_hw_backlight_cb(lv_event_t *e);
static void open_hw_memory_cb(lv_event_t *e);
static void open_hw_sd_cb(lv_event_t *e);
static void open_hw_haptics_cb(lv_event_t *e);
static void open_hw_audio_mic_cb(lv_event_t *e);
static void open_hw_battery_cb(lv_event_t *e);
static void open_hw_wireless_cb(lv_event_t *e);

static void back_cb(lv_event_t *e);
static void bezel_sensitivity_changed_cb(lv_event_t *e);
static void backlight_test_changed_cb(lv_event_t *e);

/* Haptic menu callbacks */
static void haptic_strong_click_cb(lv_event_t *e);
static void haptic_double_click_cb(lv_event_t *e);
static void haptic_buzz_cb(lv_event_t *e);
static void haptic_soft_bump_cb(lv_event_t *e);
static void haptic_stop_cb(lv_event_t *e);
static void haptic_rtp_test_cb(lv_event_t *e);
static void button_press_haptic_cb(lv_event_t *e);

/* DRV2605 driver helpers */
static esp_err_t drv2605_read_reg(uint8_t reg, uint8_t *value);
static esp_err_t drv2605_write_reg(uint8_t reg, uint8_t value);
static esp_err_t drv2605_init(void);
static esp_err_t drv2605_play_effect(uint8_t effect);
static esp_err_t drv2605_stop(void);
static esp_err_t drv2605_rtp_start(uint8_t amplitude);
static esp_err_t drv2605_rtp_stop(void);
static void drv2605_rtp_test(void);
static void drv2605_dump_registers(void);

/* ============================================================================
 * 4. DRV2605 HAPTIC DRIVER
 * ============================================================================
 *
 * The DRV2605 shares the board's existing I2C bus with the touch controller.
 * drv2605_dev_handle is created by i2c_bsp.c.
 *
 * Two playback paths are provided:
 *   - Internal waveform library: drv2605_play_effect()
 *   - Real-time playback (RTP): drv2605_rtp_start()/stop()
 *
 * RTP is useful as a hardware diagnostic because it bypasses the waveform
 * sequencer and directly requests a drive amplitude.
 * ========================================================================== */

#define DRV2605_REG_STATUS 0x00
#define DRV2605_REG_MODE 0x01
#define DRV2605_REG_RTPIN 0x02
#define DRV2605_REG_LIBRARY 0x03
#define DRV2605_REG_WAVESEQ1 0x04
#define DRV2605_REG_WAVESEQ2 0x05
#define DRV2605_REG_GO 0x0C
#define DRV2605_REG_OVERDRIVE 0x0D
#define DRV2605_REG_SUSTAINPOS 0x0E
#define DRV2605_REG_SUSTAINNEG 0x0F
#define DRV2605_REG_BREAK 0x10
#define DRV2605_REG_AUDIOMAX 0x13
#define DRV2605_REG_FEEDBACK 0x1A
#define DRV2605_REG_CONTROL3 0x1D

/**
 * @brief Read one DRV2605 register.
 */
static esp_err_t drv2605_read_reg(uint8_t reg, uint8_t *value)
{
    if (drv2605_dev_handle == NULL || value == NULL)
    {
        ESP_LOGE(TAG, "DRV2605 read attempted before device initialization");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = (esp_err_t)i2c_read_buff(
        drv2605_dev_handle,
        reg,
        value,
        1);

    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "DRV2605 read failed: reg=0x%02X err=%s",
            reg,
            esp_err_to_name(err));
    }

    return err;
}

/**
 * @brief Write one DRV2605 register.
 */
static esp_err_t drv2605_write_reg(uint8_t reg, uint8_t value)
{
    if (drv2605_dev_handle == NULL)
    {
        ESP_LOGE(TAG, "DRV2605 write attempted before device initialization");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = (esp_err_t)i2c_write_buff(
        drv2605_dev_handle,
        reg,
        &value,
        1);

    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "DRV2605 write failed: reg=0x%02X value=0x%02X err=%s",
            reg,
            value,
            esp_err_to_name(err));
    }

    return err;
}

/**
 * @brief Dump the most useful DRV2605 diagnostic registers.
 *
 * This is intentionally small: STATUS proves the device can be read,
 * MODE shows the active playback mode, and RTPIN confirms the requested
 * direct-drive amplitude.
 */
static void drv2605_dump_registers(void)
{
    uint8_t status = 0;
    uint8_t mode = 0;
    uint8_t rtp = 0;
    uint8_t feedback = 0;
    uint8_t control3 = 0;

    esp_err_t status_err = drv2605_read_reg(DRV2605_REG_STATUS, &status);
    esp_err_t mode_err = drv2605_read_reg(DRV2605_REG_MODE, &mode);
    esp_err_t rtp_err = drv2605_read_reg(DRV2605_REG_RTPIN, &rtp);
    esp_err_t feedback_err = drv2605_read_reg(DRV2605_REG_FEEDBACK, &feedback);
    esp_err_t control3_err = drv2605_read_reg(DRV2605_REG_CONTROL3, &control3);

    if (status_err == ESP_OK &&
        mode_err == ESP_OK &&
        rtp_err == ESP_OK &&
        feedback_err == ESP_OK &&
        control3_err == ESP_OK)
    {
        ESP_LOGI(
            TAG,
            "DRV2605 regs: STATUS=0x%02X MODE=0x%02X RTP=0x%02X "
            "FEEDBACK=0x%02X CONTROL3=0x%02X",
            status,
            mode,
            rtp,
            feedback,
            control3);
    }
    else
    {
        ESP_LOGE(TAG, "DRV2605 register dump incomplete because an I2C read failed");
    }
}

/**
 * @brief Initialize the DRV2605 for internal-trigger LRA operation.
 *
 * The onboard actuator was verified to vibrate correctly when configured as an
 * LRA. Earlier ERM/open-loop settings allowed I2C communication to work but did
 * not produce physical vibration.
 *
 * This initialization therefore:
 *   1. Places the DRV2605 in internal-trigger mode.
 *   2. Selects waveform library 1.
 *   3. Sets FEEDBACK[7] = 1 (LRA).
 *   4. Clears CONTROL3[5] (disable ERM open-loop).
 *   5. Dumps useful registers for serial-monitor diagnostics.
 */
static esp_err_t drv2605_init(void)
{
    ESP_LOGI(TAG, "Initializing DRV2605");

    uint8_t status = 0;
    esp_err_t err = drv2605_read_reg(DRV2605_REG_STATUS, &status);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "DRV2605 not responding: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "DRV2605 status register = 0x%02X", status);

    /* Internal-trigger mode and out of standby. */
    ESP_RETURN_ON_ERROR(
        drv2605_write_reg(DRV2605_REG_MODE, 0x00),
        TAG,
        "Failed to set DRV2605 internal-trigger mode");

    /* Clear real-time-playback input while using the waveform library. */
    ESP_RETURN_ON_ERROR(
        drv2605_write_reg(DRV2605_REG_RTPIN, 0x00),
        TAG,
        "Failed to clear DRV2605 RTP input");

    /* Select waveform library 1 and terminate sequence after slot 1. */
    ESP_RETURN_ON_ERROR(
        drv2605_write_reg(DRV2605_REG_LIBRARY, 0x01),
        TAG,
        "Failed to select DRV2605 waveform library");

    ESP_RETURN_ON_ERROR(
        drv2605_write_reg(DRV2605_REG_WAVESEQ1, 1),
        TAG,
        "Failed to initialize DRV2605 waveform slot 1");

    ESP_RETURN_ON_ERROR(
        drv2605_write_reg(DRV2605_REG_WAVESEQ2, 0),
        TAG,
        "Failed to terminate DRV2605 waveform sequence");

    /* Clear timing overrides. */
    ESP_RETURN_ON_ERROR(
        drv2605_write_reg(DRV2605_REG_OVERDRIVE, 0),
        TAG,
        "Failed to clear DRV2605 overdrive");

    ESP_RETURN_ON_ERROR(
        drv2605_write_reg(DRV2605_REG_SUSTAINPOS, 0),
        TAG,
        "Failed to clear DRV2605 positive sustain");

    ESP_RETURN_ON_ERROR(
        drv2605_write_reg(DRV2605_REG_SUSTAINNEG, 0),
        TAG,
        "Failed to clear DRV2605 negative sustain");

    ESP_RETURN_ON_ERROR(
        drv2605_write_reg(DRV2605_REG_BREAK, 0),
        TAG,
        "Failed to clear DRV2605 brake time");

    ESP_RETURN_ON_ERROR(
        drv2605_write_reg(DRV2605_REG_AUDIOMAX, 0x64),
        TAG,
        "Failed to configure DRV2605 audio max");

    /*
     * Select the actuator type.
     *
     * FEEDBACK register bit 7:
     *   0 = ERM (eccentric rotating mass motor)
     *   1 = LRA (linear resonant actuator)
     *
     * The board's actuator is being operated as an LRA because that is the
     * configuration that produced real vibration during hardware testing.
     */
    uint8_t feedback = 0;
    ESP_RETURN_ON_ERROR(
        drv2605_read_reg(DRV2605_REG_FEEDBACK, &feedback),
        TAG,
        "Failed to read DRV2605 feedback register");
    /* Set FEEDBACK[7] to select LRA operation. */
    feedback |= 0x80;

    ESP_RETURN_ON_ERROR(
        drv2605_write_reg(DRV2605_REG_FEEDBACK, feedback),
        TAG,
        "Failed to configure DRV2605 for LRA");

    /*
     * CONTROL3[5] is ERM_OPEN_LOOP.
     *
     * Because this project uses LRA mode, this bit must remain cleared.
     */
    uint8_t control3 = 0;
    ESP_RETURN_ON_ERROR(
        drv2605_read_reg(DRV2605_REG_CONTROL3, &control3),
        TAG,
        "Failed to read DRV2605 CONTROL3");

    /* Clear ERM_OPEN_LOOP while operating the actuator as an LRA. */
    control3 &= (uint8_t)~0x20;

    ESP_RETURN_ON_ERROR(
        drv2605_write_reg(DRV2605_REG_CONTROL3, control3),
        TAG,
        "Failed to disable DRV2605 ERM open-loop mode");

    ESP_LOGI(TAG, "DRV2605 initialized successfully");
    drv2605_dump_registers();

    return ESP_OK;
}

/**
 * @brief Play a built-in DRV2605 waveform effect.
 */
static esp_err_t drv2605_play_effect(uint8_t effect)
{
    ESP_RETURN_ON_ERROR(
        drv2605_write_reg(DRV2605_REG_MODE, 0x00),
        TAG,
        "Failed to enter internal-trigger mode");

    ESP_RETURN_ON_ERROR(
        drv2605_write_reg(DRV2605_REG_WAVESEQ1, effect),
        TAG,
        "Failed to set haptic effect");

    ESP_RETURN_ON_ERROR(
        drv2605_write_reg(DRV2605_REG_WAVESEQ2, 0),
        TAG,
        "Failed to terminate haptic sequence");

    ESP_RETURN_ON_ERROR(
        drv2605_write_reg(DRV2605_REG_GO, 1),
        TAG,
        "Failed to start haptic effect");

    ESP_LOGI(TAG, "DRV2605 effect %u started", effect);
    return ESP_OK;
}

/**
 * @brief Stop internal-waveform playback.
 */
static esp_err_t drv2605_stop(void)
{
    return drv2605_write_reg(DRV2605_REG_GO, 0x00);
}

/**
 * @brief Enter real-time playback mode at the requested amplitude.
 *
 * 0x7F is used as the strongest positive signed RTP value for diagnostics.
 */
static esp_err_t drv2605_rtp_start(uint8_t amplitude)
{
    ESP_LOGI(TAG, "DRV2605 RTP start, amplitude=%u", amplitude);

    ESP_RETURN_ON_ERROR(
        drv2605_write_reg(DRV2605_REG_MODE, 0x05),
        TAG,
        "Failed to enter DRV2605 RTP mode");

    ESP_RETURN_ON_ERROR(
        drv2605_write_reg(DRV2605_REG_RTPIN, amplitude),
        TAG,
        "Failed to set DRV2605 RTP amplitude");

    return ESP_OK;
}

/**
 * @brief Stop RTP drive and return to internal-trigger mode.
 */
static esp_err_t drv2605_rtp_stop(void)
{
    ESP_RETURN_ON_ERROR(
        drv2605_write_reg(DRV2605_REG_RTPIN, 0x00),
        TAG,
        "Failed to clear DRV2605 RTP amplitude");

    ESP_RETURN_ON_ERROR(
        drv2605_write_reg(DRV2605_REG_MODE, 0x00),
        TAG,
        "Failed to leave DRV2605 RTP mode");

    ESP_LOGI(TAG, "DRV2605 RTP stopped");
    return ESP_OK;
}

/**
 * @brief Run a short direct-drive haptic diagnostic.
 *
 * This currently blocks the LVGL event callback for 500 ms. That is acceptable
 * for a deliberate hardware test page, but should not be used for normal UI
 * haptic feedback.
 */
static void drv2605_rtp_test(void)
{
    ESP_LOGI(TAG, "Starting direct haptic RTP test");

    drv2605_dump_registers();

    esp_err_t err = drv2605_rtp_start(0x7F);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Unable to start RTP test: %s", esp_err_to_name(err));
        return;
    }

    drv2605_dump_registers();

    vTaskDelay(pdMS_TO_TICKS(500));

    err = drv2605_rtp_stop();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Unable to stop RTP test cleanly: %s", esp_err_to_name(err));
    }

    drv2605_dump_registers();
}

/**
 * @brief LVGL callback for the direct RTP diagnostic button.
 */
static void haptic_rtp_test_cb(lv_event_t *e)
{
    (void)e;
    drv2605_rtp_test();
}

/* ============================================================================
 * 5. MENU / UI IMPLEMENTATION
 * ========================================================================== */

/**
 * @brief Create the common scrollable container used by every menu page.
 *
 * Centralizing the layout here keeps all pages consistent:
 *   - Full logical display size
 *   - Vertical flex layout
 *   - Centered menu entries
 *   - Vertical scroll snapping
 *   - Hidden scrollbar
 *   - Extra top/bottom padding so first/last items can reach the center of the
 *     circular display
 */
static lv_obj_t *create_menu_container(void)
{
    lv_obj_t *cont = lv_obj_create(lv_screen_active());

    lv_obj_set_size(cont, LCD_H_RES, LCD_V_RES);
    lv_obj_center(cont);

    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(
        cont,
        LV_FLEX_ALIGN_START,
        LV_FLEX_ALIGN_CENTER,
        LV_FLEX_ALIGN_CENTER);

    lv_obj_set_scroll_snap_y(cont, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_OFF);

    /*
     * Allow the first and last items to reach the center of the
     * 360 x 360 circular display.
     */
    lv_obj_set_style_pad_top(cont, 140, 0);
    lv_obj_set_style_pad_bottom(cont, 140, 0);
    lv_obj_set_style_pad_row(cont, 20, 0);

    return cont;
}

/**
 * @brief Create one standard menu button.
 *
 * Each normal menu button intentionally has two event callbacks:
 *
 *   LV_EVENT_PRESSED
 *       Runs immediately when the finger touches the button. This is used only
 *       for tactile feedback so the interface feels responsive.
 *
 *   LV_EVENT_CLICKED
 *       Runs after LVGL recognizes a complete press-and-release click. This is
 *       used for the button's actual action (open a menu, go Back, etc.).
 *
 * Keeping these two jobs separate means haptic feedback happens immediately,
 * while navigation still uses normal LVGL click semantics.
 */
static lv_obj_t *create_menu_button(
    lv_obj_t *parent,
    const char *text,
    lv_event_cb_t callback)
{
    /* Create the button as a child of the supplied menu container. */
    lv_obj_t *button = lv_button_create(parent);
    lv_obj_set_size(button, 200, 50);

    /* Create and center the text label inside the button. */
    lv_obj_t *label = lv_label_create(button);
    lv_label_set_text(label, text);
    lv_obj_center(label);

    /*
     * Immediate tactile feedback on finger-down.
     *
     * This callback is intentionally separate from the actual button action.
     */
    lv_obj_add_event_cb(
        button,
        button_press_haptic_cb,
        LV_EVENT_PRESSED,
        NULL);

    /*
     * Attach the real menu/action callback only when one was supplied.
     * Keeping NULL legal makes this helper reusable for display-only buttons.
     */
    if (callback != NULL)
    {
        lv_obj_add_event_cb(
            button,
            callback,
            LV_EVENT_PRESSED,
            NULL);
    }

    return button;
}


/**
 * @brief Provide immediate generic haptic feedback for normal UI buttons.
 *
 * This callback is attached to LV_EVENT_PRESSED, so the vibration begins when
 * the finger first touches the button rather than when it is released.
 *
 * The dedicated Haptic Motor Test page is excluded. Buttons on that page are
 * supposed to demonstrate specific DRV2605 effects, and a generic click before
 * every test would make those effects harder to judge.
 */
static void button_press_haptic_cb(lv_event_t *e)
{
    (void)e;

    if (current_menu == MENU_HW_HAPTICS)
    {
        return;
    }

    /*
     * Effect 1 is currently used as the normal UI click.
     *
     * ESP_ERROR_CHECK_WITHOUT_ABORT is deliberate: haptic feedback is optional
     * UI polish. A haptic failure should be logged, but it should not reboot the
     * entire display/menu application.
     */
    ESP_ERROR_CHECK_WITHOUT_ABORT(
        drv2605_play_effect(1));
}

/**
 * @brief Destroy the current menu and construct the requested menu.
 *
 * This function is the central router for the entire menu system.
 */
static void show_menu(menu_id_t menu)
{
    /*
     * Clear test-widget pointers before deleting menu_cont. Deleting the parent
     * also deletes all child LVGL objects.
     */
    touch_test_label = NULL;
    encoder_test_label = NULL;

    if (menu_cont != NULL)
    {
        lv_obj_delete(menu_cont);
        menu_cont = NULL;
    }

    current_menu = menu;
    encoder_steps = 0;

    switch (menu)
    {
    case MENU_MAIN:
        create_main_menu();
        break;
    case MENU_SETTINGS:
        create_settings_menu();
        break;
    case MENU_DISPLAY:
        create_display_menu();
        break;
    case MENU_AUDIO:
        create_audio_menu();
        break;
    case MENU_INPUT:
        create_input_menu();
        break;
    case MENU_ABOUT:
        create_about_menu();
        break;
    case MENU_HARDWARE_TESTS:
        create_hardware_tests_menu();
        break;
    case MENU_HW_DISPLAY:
        create_hw_display_test_menu();
        break;
    case MENU_HW_TOUCH:
        create_hw_touch_test_menu();
        break;
    case MENU_HW_ENCODER:
        create_hw_encoder_test_menu();
        break;
    case MENU_HW_BACKLIGHT:
        create_hw_backlight_test_menu();
        break;
    case MENU_HW_MEMORY:
        create_hw_memory_test_menu();
        break;
    case MENU_HW_HAPTICS:
        create_hw_haptics_test_menu();
        break;
    case MENU_HW_UNAVAILABLE:
        create_hw_unavailable_menu();
        break;
    default:
        current_menu = MENU_MAIN;
        create_main_menu();
        break;
    }
}

/**
 * @brief Navigate forward while preserving the current page for Back.
 */
static void push_menu(menu_id_t menu)
{
    if (menu_stack_top < MENU_STACK_SIZE - 1)
    {
        menu_stack[++menu_stack_top] = current_menu;
    }

    show_menu(menu);
}

/**
 * @brief Return to the most recently saved parent page.
 */
static void pop_menu(void)
{
    if (menu_stack_top >= 0)
    {
        menu_id_t previous = menu_stack[menu_stack_top--];
        show_menu(previous);
    }
    else if (current_menu != MENU_MAIN)
    {
        show_menu(MENU_MAIN);
    }
}

static void open_settings_cb(lv_event_t *e)
{
    (void)e;
    push_menu(MENU_SETTINGS);
}

static void open_display_cb(lv_event_t *e)
{
    (void)e;
    push_menu(MENU_DISPLAY);
}

static void open_audio_cb(lv_event_t *e)
{
    (void)e;
    push_menu(MENU_AUDIO);
}

static void open_input_cb(lv_event_t *e)
{
    (void)e;
    push_menu(MENU_INPUT);
}

static void open_about_cb(lv_event_t *e)
{
    (void)e;
    push_menu(MENU_ABOUT);
}

static void open_hardware_tests_cb(lv_event_t *e)
{
    (void)e;
    push_menu(MENU_HARDWARE_TESTS);
}

static void open_hw_display_cb(lv_event_t *e)
{
    (void)e;
    push_menu(MENU_HW_DISPLAY);
}

static void open_hw_touch_cb(lv_event_t *e)
{
    (void)e;
    push_menu(MENU_HW_TOUCH);
}

static void open_hw_encoder_cb(lv_event_t *e)
{
    (void)e;
    encoder_test_count = 0;
    push_menu(MENU_HW_ENCODER);
}

static void open_hw_backlight_cb(lv_event_t *e)
{
    (void)e;
    push_menu(MENU_HW_BACKLIGHT);
}

static void open_hw_memory_cb(lv_event_t *e)
{
    (void)e;
    push_menu(MENU_HW_MEMORY);
}

static void open_unavailable_hw(const char *name)
{
    unavailable_hw_name = name;
    push_menu(MENU_HW_UNAVAILABLE);
}

static void open_hw_sd_cb(lv_event_t *e)
{
    (void)e;
    open_unavailable_hw("TF / microSD card");
}

static void open_hw_haptics_cb(lv_event_t *e)
{
    (void)e;
    push_menu(MENU_HW_HAPTICS);
}

static void open_hw_audio_mic_cb(lv_event_t *e)
{
    (void)e;
    open_unavailable_hw("PCM5100A + digital MIC");
}

static void open_hw_battery_cb(lv_event_t *e)
{
    (void)e;
    open_unavailable_hw("Battery / ADC");
}

static void open_hw_wireless_cb(lv_event_t *e)
{
    (void)e;
    open_unavailable_hw("Wi-Fi / Bluetooth / secondary ESP32");
}

static void back_cb(lv_event_t *e)
{
    (void)e;
    pop_menu();
}

static void create_main_menu(void)
{
    menu_cont = create_menu_container();

    lv_obj_t *title = lv_label_create(menu_cont);
    lv_label_set_text(title, "Main Menu");

    create_menu_button(menu_cont, "Settings", open_settings_cb);
    create_menu_button(menu_cont, "Hardware Tests", open_hardware_tests_cb);
    create_menu_button(menu_cont, "About", open_about_cb);
}

static void create_settings_menu(void)
{
    menu_cont = create_menu_container();

    lv_obj_t *title = lv_label_create(menu_cont);
    lv_label_set_text(title, "Settings");

    create_menu_button(menu_cont, "Display", open_display_cb);
    create_menu_button(menu_cont, "Audio", open_audio_cb);
    create_menu_button(menu_cont, "Input", open_input_cb);
    create_menu_button(menu_cont, "Back", back_cb);
}

static void create_display_menu(void)
{
    menu_cont = create_menu_container();

    lv_obj_t *title = lv_label_create(menu_cont);
    lv_label_set_text(title, "Display");

    lv_obj_t *brightness_label = lv_label_create(menu_cont);
    lv_label_set_text(brightness_label, "Brightness");

    lv_obj_t *brightness = lv_slider_create(menu_cont);
    lv_obj_set_size(brightness, 200, 20);
    lv_slider_set_range(brightness, 0, 100);
    lv_slider_set_value(brightness, 75, LV_ANIM_OFF);

    create_menu_button(menu_cont, "Back", back_cb);
}

static void create_audio_menu(void)
{
    menu_cont = create_menu_container();

    lv_obj_t *title = lv_label_create(menu_cont);
    lv_label_set_text(title, "Audio");

    lv_obj_t *volume_label = lv_label_create(menu_cont);
    lv_label_set_text(volume_label, "Volume");

    lv_obj_t *volume = lv_slider_create(menu_cont);
    lv_obj_set_size(volume, 200, 20);
    lv_slider_set_range(volume, 0, 100);
    lv_slider_set_value(volume, 50, LV_ANIM_OFF);

    create_menu_button(menu_cont, "Back", back_cb);
}

static void bezel_sensitivity_changed_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target_obj(e);
    encoder_scroll_per_step = lv_slider_get_value(slider);
}

static void create_input_menu(void)
{
    menu_cont = create_menu_container();

    lv_obj_t *title = lv_label_create(menu_cont);
    lv_label_set_text(title, "Input");

    lv_obj_t *label = lv_label_create(menu_cont);
    lv_label_set_text(label, "Bezel Sensitivity");

    lv_obj_t *slider = lv_slider_create(menu_cont);
    lv_obj_set_size(slider, 200, 20);
    lv_slider_set_range(slider, 5, 80);
    lv_slider_set_value(
        slider,
        encoder_scroll_per_step,
        LV_ANIM_OFF);

    lv_obj_add_event_cb(
        slider,
        bezel_sensitivity_changed_cb,
        LV_EVENT_VALUE_CHANGED,
        NULL);

    create_menu_button(menu_cont, "Back", back_cb);
}

static void create_hardware_tests_menu(void)
{
    menu_cont = create_menu_container();

    lv_obj_t *title = lv_label_create(menu_cont);
    lv_label_set_text(title, "Hardware Tests");

    create_menu_button(menu_cont, "LCD / Colors", open_hw_display_cb);
    create_menu_button(menu_cont, "Touch", open_hw_touch_cb);
    create_menu_button(menu_cont, "Bezel Encoder", open_hw_encoder_cb);
    create_menu_button(menu_cont, "Backlight", open_hw_backlight_cb);
    create_menu_button(menu_cont, "RAM / PSRAM", open_hw_memory_cb);

    create_menu_button(menu_cont, "TF / microSD", open_hw_sd_cb);
    create_menu_button(menu_cont, "Haptic Motor", open_hw_haptics_cb);
    create_menu_button(menu_cont, "Audio + MIC", open_hw_audio_mic_cb);
    create_menu_button(menu_cont, "Battery / ADC", open_hw_battery_cb);
    create_menu_button(menu_cont, "Wireless / ESP32", open_hw_wireless_cb);

    create_menu_button(menu_cont, "Back", back_cb);
}
static void haptic_strong_click_cb(lv_event_t *e)
{
    (void)e;
    ESP_ERROR_CHECK_WITHOUT_ABORT(drv2605_play_effect(1));
}
static void haptic_double_click_cb(lv_event_t *e)
{
    (void)e;
    ESP_ERROR_CHECK_WITHOUT_ABORT(drv2605_play_effect(10));
}
static void haptic_soft_bump_cb(lv_event_t *e)
{
    (void)e;
    ESP_ERROR_CHECK_WITHOUT_ABORT(drv2605_play_effect(7));
}
static void haptic_buzz_cb(lv_event_t *e)
{
    (void)e;
    ESP_ERROR_CHECK_WITHOUT_ABORT(drv2605_play_effect(47));
}
static void haptic_stop_cb(lv_event_t *e)
{
    (void)e;
    ESP_ERROR_CHECK_WITHOUT_ABORT(drv2605_stop());
}

static lv_obj_t *create_color_swatch(lv_obj_t *parent, const char *name, uint32_t color)
{
    lv_obj_t *swatch = lv_obj_create(parent);
    lv_obj_set_size(swatch, 200, 52);
    lv_obj_set_style_bg_color(swatch, lv_color_hex(color), 0);
    lv_obj_set_style_bg_opa(swatch, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(swatch, 0, 0);
    lv_obj_set_style_radius(swatch, 8, 0);

    lv_obj_t *label = lv_label_create(swatch);
    lv_label_set_text(label, name);

    if (color == 0xFFFFFF || color == 0xFFFF00 || color == 0x00FF00)
        lv_obj_set_style_text_color(label, lv_color_hex(0x000000), 0);
    else
        lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);

    lv_obj_center(label);
    return swatch;
}

static void create_hw_display_test_menu(void)
{
    menu_cont = create_menu_container();

    lv_obj_t *title = lv_label_create(menu_cont);
    lv_label_set_text(title, "LCD Color Test");

    create_color_swatch(menu_cont, "RED", 0xFF0000);
    create_color_swatch(menu_cont, "GREEN", 0x00FF00);
    create_color_swatch(menu_cont, "BLUE", 0x0000FF);
    create_color_swatch(menu_cont, "WHITE", 0xFFFFFF);
    create_color_swatch(menu_cont, "BLACK", 0x000000);
    create_color_swatch(menu_cont, "YELLOW", 0xFFFF00);
    create_color_swatch(menu_cont, "MAGENTA", 0xFF00FF);
    create_color_swatch(menu_cont, "CYAN", 0x00FFFF);

    lv_obj_t *hint = lv_label_create(menu_cont);
    lv_label_set_text(hint, "Look for bad colors,\nlines, flicker, or dead areas.");
    lv_obj_set_style_text_align(hint, LV_TEXT_ALIGN_CENTER, 0);

    create_menu_button(menu_cont, "Back", back_cb);
}

static void create_hw_touch_test_menu(void)
{
    menu_cont = create_menu_container();

    lv_obj_t *title = lv_label_create(menu_cont);
    lv_label_set_text(title, "Touch Test");

    lv_obj_t *hint = lv_label_create(menu_cont);
    lv_label_set_text(hint, "Touch around the screen.\nCoordinates update below.");
    lv_obj_set_style_text_align(hint, LV_TEXT_ALIGN_CENTER, 0);

    touch_test_label = lv_label_create(menu_cont);
    lv_label_set_text(touch_test_label, "X: ---   Y: ---\nReleased");
    lv_obj_set_style_text_align(touch_test_label, LV_TEXT_ALIGN_CENTER, 0);

    create_menu_button(menu_cont, "Back", back_cb);
}

static void create_hw_encoder_test_menu(void)
{
    menu_cont = create_menu_container();

    lv_obj_t *title = lv_label_create(menu_cont);
    lv_label_set_text(title, "Bezel Encoder Test");

    lv_obj_t *hint = lv_label_create(menu_cont);
    lv_label_set_text(hint, "Rotate the bezel.\nRaw steps are counted below.");
    lv_obj_set_style_text_align(hint, LV_TEXT_ALIGN_CENTER, 0);

    encoder_test_label = lv_label_create(menu_cont);
    lv_label_set_text_fmt(encoder_test_label, "Count: %ld", (long)encoder_test_count);

    create_menu_button(menu_cont, "Back", back_cb);
}

static void backlight_test_changed_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target_obj(e);
    int32_t value = lv_slider_get_value(slider);

    if (value < 0)
        value = 0;
    if (value > 255)
        value = 255;

    backlight_test_level = (uint8_t)value;
    setUpduty(backlight_test_level);
}

static void create_hw_backlight_test_menu(void)
{
    menu_cont = create_menu_container();

    lv_obj_t *title = lv_label_create(menu_cont);
    lv_label_set_text(title, "Backlight Test");

    lv_obj_t *slider = lv_slider_create(menu_cont);
    lv_obj_set_size(slider, 200, 24);
    lv_slider_set_range(slider, 0, 255);
    lv_slider_set_value(slider, backlight_test_level, LV_ANIM_OFF);
    lv_obj_add_event_cb(slider, backlight_test_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);

    create_menu_button(menu_cont, "Back", back_cb);
}

static void create_hw_memory_test_menu(void)
{
    menu_cont = create_menu_container();

    lv_obj_t *title = lv_label_create(menu_cont);
    lv_label_set_text(title, "RAM / PSRAM Test");

    size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t internal_largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t psram_largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);

    lv_obj_t *info = lv_label_create(menu_cont);
    lv_label_set_text_fmt(
        info,
        "Internal free: %u KiB\n"
        "Largest block: %u KiB\n\n"
        "PSRAM free: %u KiB\n"
        "Largest block: %u KiB",
        (unsigned)(internal_free / 1024),
        (unsigned)(internal_largest / 1024),
        (unsigned)(psram_free / 1024),
        (unsigned)(psram_largest / 1024));

    lv_obj_set_style_text_align(info, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *status = lv_label_create(menu_cont);
    lv_label_set_text(status, psram_free > 0 ? "PSRAM: detected" : "PSRAM: NOT detected");

    create_menu_button(menu_cont, "Back", back_cb);
}
static void create_hw_haptics_test_menu(void)
{
    menu_cont = create_menu_container();

    lv_obj_t *title =
        lv_label_create(menu_cont);

    lv_label_set_text(
        title,
        "Haptic Motor Test");

    lv_obj_t *info =
        lv_label_create(menu_cont);

    lv_label_set_text(
        info,
        "Select a DRV2605\n"
        "built-in effect");

    lv_obj_set_style_text_align(
        info,
        LV_TEXT_ALIGN_CENTER,
        0);

    create_menu_button(
        menu_cont,
        "Direct RTP Test",
        haptic_rtp_test_cb);

    create_menu_button(
        menu_cont,
        "Strong Click",
        haptic_strong_click_cb);

    create_menu_button(
        menu_cont,
        "Double Click",
        haptic_double_click_cb);

    create_menu_button(
        menu_cont,
        "Soft Bump",
        haptic_soft_bump_cb);

    create_menu_button(
        menu_cont,
        "Buzz",
        haptic_buzz_cb);

    create_menu_button(
        menu_cont,
        "Stop",
        haptic_stop_cb);

    create_menu_button(
        menu_cont,
        "Back",
        back_cb);
}

static void create_hw_unavailable_menu(void)
{
    menu_cont = create_menu_container();

    lv_obj_t *title = lv_label_create(menu_cont);
    lv_label_set_text(title, unavailable_hw_name);

    lv_obj_t *status = lv_label_create(menu_cont);
    lv_label_set_text(
        status,
        "Hardware is present,\n"
        "but its driver is not yet\n"
        "integrated in this project.");
    lv_obj_set_style_text_align(status, LV_TEXT_ALIGN_CENTER, 0);

    create_menu_button(menu_cont, "Back", back_cb);
}

static void create_about_menu(void)
{
    menu_cont = create_menu_container();

    lv_obj_t *title = lv_label_create(menu_cont);
    lv_label_set_text(title, "About");

    lv_obj_t *info = lv_label_create(menu_cont);
    lv_label_set_text(
        info,
        "Waveshare ESP32-S3\nKnob Touch LCD 1.8\nLVGL 9");

    lv_obj_set_style_text_align(
        info,
        LV_TEXT_ALIGN_CENTER,
        0);

    create_menu_button(menu_cont, "Back", back_cb);
}

/* ============================================================================
 * 6. LVGL DISPLAY AND TOUCH BRIDGE
 * ========================================================================== */

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

    /*
     * Swap the two bytes of every RGB565 pixel before QSPI transfer.
     *
     * LVGL's RGB565 memory byte order differs from the order expected by this
     * SH8601 transfer path. Without this swap, colors appear incorrect.
     */
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

        if (current_menu == MENU_HW_TOUCH && touch_test_label != NULL)
        {
            lv_label_set_text_fmt(
                touch_test_label,
                "X: %ld   Y: %ld\nPressed",
                (long)data->point.x,
                (long)data->point.y);
        }
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;

        if (current_menu == MENU_HW_TOUCH && touch_test_label != NULL)
        {
            lv_label_set_text(touch_test_label, "Released");
        }
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

/* ============================================================================
 * 7. FREERTOS TASKS AND PERIODIC INPUT PROCESSING
 * ========================================================================== */

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
    (void)timer;

    if (menu_cont == NULL)
        return;

    int32_t steps = encoder_steps;
    if (steps == 0)
        return;

    encoder_steps = 0;

    if (current_menu == MENU_HW_ENCODER)
    {
        encoder_test_count += steps;

        if (encoder_test_label != NULL)
        {
            lv_label_set_text_fmt(
                encoder_test_label,
                "Count: %ld\nLast step: %s",
                (long)encoder_test_count,
                steps > 0 ? "+" : "-");
        }
        return;
    }

    lv_obj_scroll_by_bounded(
        menu_cont,
        0,
        -steps * encoder_scroll_per_step,
        LV_ANIM_OFF);
}
/* ============================================================================
 * 8. APPLICATION ENTRY POINT / HARDWARE INITIALIZATION
 * ========================================================================== */

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

    ESP_LOGI(TAG, "Initialize haptic motor");
    ESP_ERROR_CHECK(drv2605_init());

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
