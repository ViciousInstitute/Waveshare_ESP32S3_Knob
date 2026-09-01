/* ============================================================================
 * Waveshare ESP32-S3-Knob-Touch-LCD-1.8
 * LVGL 9 application + hardware diagnostics
 *
 * Major sections:
 *   1. Includes and global state
 *   2. LCD initialization data
 *   3. Menu/navigation declarations
 *   4. DRV2605 haptic driver
 *   5. SDMMC / microSD driver
 *   6. Microphone / audio diagnostics
 *   7. Battery / system-voltage ADC diagnostics
 *   8. BLE media-controller integration
 *   9. Menu implementation
 *  10. LVGL display/touch bridge
 *  11. FreeRTOS tasks
 *  12. app_main hardware initialization
 * ========================================================================== */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <dirent.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/sdmmc_host.h"
#include "driver/i2s_pdm.h"
#include "driver/i2s_std.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "lvgl.h"
#include "lv_demos.h"
#include "esp_lcd_sh8601.h"
#include "i2c_bsp.h"
#include "lcd_touch_bsp.h"
#include "user_config.h"
#include "lcd_bl_pwm_bsp.h"
#include "user_encoder_bsp.h"
#include "media_controller_ble.h"

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

/*
 * SD-card diagnostic state.
 *
 * The card is mounted lazily from the SD test page. A missing card therefore
 * never prevents the rest of the device, including BLE HID, from starting.
 */
static sdmmc_card_t *sd_card = NULL;
static bool sd_card_mounted = false;
static lv_obj_t *sd_test_label = NULL;


/*
 * Battery / system-voltage ADC diagnostic.
 *
 * Waveshare's official 01_ADC_Test identifies the sense input as:
 *
 *      ADC1 channel 0
 *
 * On ESP32-S3, ADC1 channel 0 is GPIO1.
 *
 * Board pin maps for this exact unit identify a 2:1 resistor divider between
 * the monitored system/battery rail and GPIO1, therefore:
 *
 *      system_voltage ~= calibrated_GPIO1_voltage * 2
 *
 * Waveshare calls this "system voltage", which is important: when USB/external
 * power is present the reading can be above the normal single-cell Li-ion
 * range. The UI therefore only shows a rough battery percentage when the rail
 * is inside a plausible battery-only window.
 */
#define BATTERY_ADC_UNIT             ADC_UNIT_1
#define BATTERY_ADC_CHANNEL          ADC_CHANNEL_0
#define BATTERY_ADC_ATTEN            ADC_ATTEN_DB_12
#define BATTERY_ADC_BITWIDTH         ADC_BITWIDTH_12
#define BATTERY_ADC_MULTISAMPLES     8
#define BATTERY_DIVIDER_NUMERATOR    2
#define BATTERY_DIVIDER_DENOMINATOR  1

static adc_oneshot_unit_handle_t battery_adc_handle = NULL;
static adc_cali_handle_t battery_adc_cali_handle = NULL;
static bool battery_adc_initialized = false;
static bool battery_adc_calibrated = false;

static lv_obj_t *battery_adc_status_label = NULL;
static lv_obj_t *battery_adc_detail_label = NULL;
static lv_obj_t *battery_adc_level_bar = NULL;

/*
 * IMPORTANT:
 * ADC reads do not run from LVGL callbacks.
 *
 * v10 performed initialization + multisampling synchronously while the
 * Battery / ADC page was being constructed from an LV_EVENT_PRESSED callback.
 * That can stall the GUI task if an ADC operation takes longer than expected.
 *
 * v11 moves all ADC hardware access into this low-priority worker. LVGL only
 * copies the latest scalar results into labels and the bar.
 */
typedef enum
{
    BATTERY_SAMPLE_IDLE = 0,
    BATTERY_SAMPLE_STARTING,
    BATTERY_SAMPLE_READY,
    BATTERY_SAMPLE_ERROR
} battery_sample_state_t;

static volatile battery_sample_state_t battery_sample_state =
    BATTERY_SAMPLE_IDLE;

static volatile int battery_latest_raw = 0;
static volatile int battery_latest_pin_mv = 0;
static volatile int battery_latest_rail_mv = 0;
static volatile bool battery_latest_calibrated = false;
static volatile esp_err_t battery_latest_error = ESP_OK;

static TaskHandle_t battery_adc_task_handle = NULL;


/*
 * ESP32-S3 microphone diagnostic state.
 *
 * IMPORTANT:
 * The onboard digital MEMS microphone is a PDM microphone, not a conventional
 * standard-I2S microphone.
 *
 * The board schematic labels the microphone nets:
 *
 *      PDM_MIC_SCK      -> GPIO45
 *      PDM_MIC_DATA     -> GPIO46
 *
 * GPIO42 belongs to SDMMC D2 and must NOT be used by the microphone test.
 *
 * The ESP32-S3 hardware PDM-to-PCM converter is used so the capture task reads
 * ordinary signed 16-bit PCM samples at 16 kHz. This mirrors the architecture
 * used by Waveshare's official 07_Audio_Test, which initializes PDM RX for the
 * microphone.
 */
#define MIC_PDM_CLK_PIN      GPIO_NUM_45
#define MIC_PDM_DATA_PIN     GPIO_NUM_46
#define MIC_SAMPLE_RATE_HZ   16000
#define MIC_TASK_STACK_SIZE  4096

typedef enum
{
    MIC_TEST_STOPPED = 0,
    MIC_TEST_STARTING,
    MIC_TEST_RUNNING,
    MIC_TEST_STOPPING,
    MIC_TEST_ERROR
} mic_test_state_t;

static i2s_chan_handle_t mic_rx_chan = NULL;
static TaskHandle_t mic_task_handle = NULL;
static volatile bool mic_stop_requested = false;
static volatile mic_test_state_t mic_test_state = MIC_TEST_STOPPED;
static volatile esp_err_t mic_last_error = ESP_OK;
static volatile int mic_level_percent = 0;

/*
 * DC-corrected microphone diagnostics.
 *
 * mic_rms_raw:
 *     RMS amplitude of the selected PDM slot after subtracting that slot's
 *     average (DC offset).
 *
 * mic_dc_raw:
 *     Signed average sample value of the selected slot. A digital MEMS/PDM
 *     microphone may have a substantial DC offset; this value is intentionally
 *     displayed so we can distinguish DC bias from real audio energy.
 *
 * mic_active_slot:
 *     0 = PDM line 0 right slot
 *     1 = PDM line 0 left slot
 */
static volatile uint32_t mic_rms_raw = 0;
static volatile int32_t mic_dc_raw = 0;
static volatile int mic_active_slot = 0;
static volatile uint32_t mic_blocks_read = 0;

static lv_obj_t *audio_mic_status_label = NULL;
static lv_obj_t *audio_mic_level_bar = NULL;
static lv_obj_t *audio_mic_peak_label = NULL;
static lv_obj_t *audio_dac_info_label = NULL;

/*
 * PCM5100A / 3.5 mm output diagnostic.
 *
 * The board schematic confirms that the ESP32-S3 has its own audio route into
 * the PCM5100A through the CH445P 4-channel analog switch:
 *
 *      S3 BCLK       GPIO39
 *      S3 LRCK / WS  GPIO44
 *      S3 DATA       GPIO45
 *      CH445P IN     GPIO40
 *
 * The S3 signals are wired to the CH445P S2 inputs. The CH445P datasheet says
 * IN=HIGH selects S2, so GPIO40 must be HIGH while the S3 is driving the DAC.
 *
 * GPIO45 is shared with the microphone PDM clock. The tone task therefore waits
 * for the microphone task to fully stop and release its I2S channel before
 * configuring standard-I2S TX.
 */
#define DAC_I2S_BCLK_PIN       GPIO_NUM_39
#define DAC_I2S_WS_PIN         GPIO_NUM_44
#define DAC_I2S_DATA_PIN       GPIO_NUM_45
#define DAC_I2S_SWITCH_PIN     GPIO_NUM_40
#define DAC_SAMPLE_RATE_HZ     32000
#define DAC_TONE_HZ            1000
#define DAC_TONE_TASK_STACK    4096

typedef enum
{
    DAC_TEST_STOPPED = 0,
    DAC_TEST_WAITING_FOR_MIC,
    DAC_TEST_STARTING,
    DAC_TEST_RUNNING,
    DAC_TEST_STOPPING,
    DAC_TEST_ERROR
} dac_test_state_t;

static i2s_chan_handle_t dac_tx_chan = NULL;
static TaskHandle_t dac_tone_task_handle = NULL;
static volatile bool dac_tone_stop_requested = false;
static volatile dac_test_state_t dac_test_state = DAC_TEST_STOPPED;
static volatile esp_err_t dac_last_error = ESP_OK;

/*
 * Media-controller page state. Bluetooth callbacks never update LVGL directly;
 * an LVGL timer polls BLE state and refreshes this label safely.
 */
static lv_obj_t *media_status_label = NULL;

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
    MENU_MEDIA_CONTROLLER,
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
    MENU_HW_SD,
    MENU_HW_HAPTICS,
    MENU_HW_AUDIO_MIC,
    MENU_HW_BATTERY,
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
static void create_media_controller_menu(void);
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
static void create_hw_sd_test_menu(void);
static void create_hw_haptics_test_menu(void);
static void create_hw_audio_mic_test_menu(void);
static void create_hw_battery_test_menu(void);
static void create_hw_unavailable_menu(void);

static void open_media_controller_cb(lv_event_t *e);
static void media_previous_cb(lv_event_t *e);
static void media_play_pause_cb(lv_event_t *e);
static void media_next_cb(lv_event_t *e);
static void media_mute_cb(lv_event_t *e);
static void media_status_timer_cb(lv_timer_t *timer);

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

/* SD-card diagnostic page callbacks */
static void sd_mount_refresh_cb(lv_event_t *e);
static void sd_read_write_test_cb(lv_event_t *e);
static void sd_list_root_cb(lv_event_t *e);
static void sd_unmount_cb(lv_event_t *e);

/* Audio / microphone diagnostic callbacks and helpers */
static void audio_mic_start_cb(lv_event_t *e);
static void audio_mic_stop_cb(lv_event_t *e);
static void audio_dac_start_tone_cb(lv_event_t *e);
static void audio_dac_stop_tone_cb(lv_event_t *e);
static void audio_mic_output_info_cb(lv_event_t *e);
static void audio_mic_back_cb(lv_event_t *e);
static void audio_mic_ui_timer_cb(lv_timer_t *timer);

static void mic_capture_task(void *arg);
static esp_err_t mic_test_start(void);
static void mic_test_request_stop(void);
static uint32_t mic_isqrt_u64(uint64_t value);
static int mic_level_from_rms(uint32_t rms);

static void dac_tone_task(void *arg);
static esp_err_t dac_tone_start(void);
static void dac_tone_request_stop(void);

/* Battery / ADC diagnostic callbacks and helpers */
static esp_err_t battery_adc_init(void);
static esp_err_t battery_adc_read(
    int *raw_average,
    int *adc_pin_mv,
    int *system_mv,
    bool *used_calibration);
static int battery_percent_from_mv(int system_mv);
static void battery_adc_worker_task(void *arg);
static void battery_adc_worker_start(void);
static void battery_adc_ui_timer_cb(lv_timer_t *timer);

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

/* SDMMC / FatFs driver helpers */
static esp_err_t sd_card_mount(void);
static esp_err_t sd_card_unmount(void);
static esp_err_t sd_card_read_write_test(void);
static esp_err_t sd_card_list_root(size_t *entry_count);
static void sd_card_update_status_label(const char *message);

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
 * 5. SDMMC / microSD DRIVER
 * ============================================================================
 *
 * This board exposes its TF/microSD slot through the ESP32-S3 SDMMC peripheral.
 *
 * Board-specific 4-bit SDMMC wiring:
 *
 *      SD signal       ESP32-S3 GPIO
 *      ---------       --------------
 *      CLK             GPIO4
 *      CMD             GPIO3
 *      D0              GPIO5
 *      D1              GPIO6
 *      D2              GPIO42
 *      D3              GPIO2
 *
 * Using SDMMC instead of SDSPI is useful on this board because the LCD already
 * uses the general-purpose SPI peripheral for its QSPI display interface.
 *
 * The driver is intentionally NON-DESTRUCTIVE:
 *
 *   - It never formats the card automatically.
 *   - A mount failure leaves the card contents untouched.
 *   - The read/write diagnostic creates one temporary file, verifies it, and
 *     removes it again.
 *
 * The card is mounted at /sdcard. Once mounted, ordinary C/POSIX file APIs such
 * as fopen(), fread(), fwrite(), remove(), opendir(), and readdir() can access
 * files beneath that path.
 * ========================================================================== */

#define SD_MOUNT_POINT       "/sdcard"
#define SD_TEST_FILE_PATH    SD_MOUNT_POINT "/waveshare_sd_test.txt"

/*
 * Exact SDMMC GPIO assignment for the Waveshare/Guition knob board.
 *
 * GPIO_NUM_* is used instead of raw integers so the compiler can type-check
 * these assignments against gpio_num_t fields in sdmmc_slot_config_t.
 */
#define SD_PIN_CLK           GPIO_NUM_4
#define SD_PIN_CMD           GPIO_NUM_3
#define SD_PIN_D0            GPIO_NUM_5
#define SD_PIN_D1            GPIO_NUM_6
#define SD_PIN_D2            GPIO_NUM_42
#define SD_PIN_D3            GPIO_NUM_2

/**
 * @brief Mount the TF/microSD card using the ESP32-S3 SDMMC peripheral.
 *
 * The helper is idempotent: calling it when the card is already mounted simply
 * returns ESP_OK.
 *
 * format_if_mount_failed is deliberately false. A diagnostic page should never
 * erase somebody's card merely because the filesystem could not be mounted.
 *
 * disk_status_check_enable asks FatFs to perform real media-status checks. It
 * has a small performance cost but is useful on a device where the user may
 * physically remove the card.
 */
static esp_err_t sd_card_mount(void)
{
    /*
     * The current board integration uses GPIO42 in both the SD diagnostic
     * wiring and the microphone clock path. Refuse a new SD mount while the
     * microphone task still owns its I2S channel.
     */
    if (mic_task_handle != NULL || mic_test_state == MIC_TEST_RUNNING ||
        mic_test_state == MIC_TEST_STARTING || mic_test_state == MIC_TEST_STOPPING)
    {
        ESP_LOGW(TAG, "SD mount blocked while microphone test is active");
        return ESP_ERR_INVALID_STATE;
    }

    if (sd_card_mounted && sd_card != NULL)
    {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Mounting microSD card at %s", SD_MOUNT_POINT);

    const esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
        .disk_status_check_enable = true,
        .use_one_fat = false,
    };

    /*
     * SDMMC_HOST_DEFAULT() configures the native SD/MMC host peripheral.
     * The default clock is conservative enough for initial board bring-up.
     */
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    /*
     * The board is wired for a full 4-bit SD bus, matching Waveshare's own
     * 4-wire SDMMC demo. Explicitly assigning every signal is important on
     * ESP32-S3 because the SDMMC peripheral can route signals to arbitrary
     * GPIOs through the GPIO matrix.
     */
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;
    slot_config.clk = SD_PIN_CLK;
    slot_config.cmd = SD_PIN_CMD;
    slot_config.d0 = SD_PIN_D0;
    slot_config.d1 = SD_PIN_D1;
    slot_config.d2 = SD_PIN_D2;
    slot_config.d3 = SD_PIN_D3;

    /*
     * Enable the ESP32-S3's internal pull-ups as supplemental pull-ups.
     * The production board should already contain the required external SD
     * pull-ups; ESP-IDF explicitly warns that internal pull-ups alone are not
     * sufficient for a robust physical design.
     */
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    sdmmc_card_t *mounted_card = NULL;

    esp_err_t err = esp_vfs_fat_sdmmc_mount(
        SD_MOUNT_POINT,
        &host,
        &slot_config,
        &mount_config,
        &mounted_card);

    if (err != ESP_OK)
    {
        sd_card = NULL;
        sd_card_mounted = false;

        if (err == ESP_FAIL)
        {
            ESP_LOGE(
                TAG,
                "SD mount failed. Card may be absent or filesystem may not be FAT-formatted.");
        }
        else
        {
            ESP_LOGE(
                TAG,
                "SD initialization failed: %s",
                esp_err_to_name(err));
        }

        return err;
    }

    sd_card = mounted_card;
    sd_card_mounted = true;

    ESP_LOGI(TAG, "microSD card mounted successfully");

    /*
     * Print the complete ESP-IDF card report to the serial monitor. This
     * includes useful protocol details that would be too verbose for the
     * 360x360 diagnostic screen.
     */
    sdmmc_card_print_info(stdout, sd_card);

    return ESP_OK;
}

/**
 * @brief Unmount the SD card and release the SDMMC/FatFs resources.
 *
 * The helper is also idempotent. Calling it when nothing is mounted succeeds
 * without doing any work.
 */
static esp_err_t sd_card_unmount(void)
{
    if (!sd_card_mounted || sd_card == NULL)
    {
        sd_card = NULL;
        sd_card_mounted = false;
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Unmounting microSD card");

    esp_err_t err = esp_vfs_fat_sdcard_unmount(
        SD_MOUNT_POINT,
        sd_card);

    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Failed to unmount microSD card: %s",
            esp_err_to_name(err));
        return err;
    }

    sd_card = NULL;
    sd_card_mounted = false;

    ESP_LOGI(TAG, "microSD card unmounted");
    return ESP_OK;
}

/**
 * @brief Perform a write -> read -> compare -> delete integrity test.
 *
 * This verifies more than "the card mounted." It confirms that:
 *   1. FatFs can create/open a file.
 *   2. Data can be written to the card.
 *   3. The same data can be read back.
 *   4. The returned bytes exactly match what was written.
 *   5. The temporary file can be deleted.
 *
 * Existing user files are not touched because the diagnostic uses its own
 * dedicated filename at the root of the mounted card.
 */
static esp_err_t sd_card_read_write_test(void)
{
    esp_err_t err = sd_card_mount();
    if (err != ESP_OK)
    {
        return err;
    }

    static const char expected[] =
        "Waveshare ESP32-S3 SDMMC read/write test OK\n";

    ESP_LOGI(TAG, "Writing SD test file: %s", SD_TEST_FILE_PATH);

    FILE *file = fopen(SD_TEST_FILE_PATH, "wb");
    if (file == NULL)
    {
        ESP_LOGE(
            TAG,
            "Could not open SD test file for writing: errno=%d (%s)",
            errno,
            strerror(errno));
        return ESP_FAIL;
    }

    const size_t expected_len = strlen(expected);
    const size_t written = fwrite(expected, 1, expected_len, file);

    if (fclose(file) != 0)
    {
        ESP_LOGE(
            TAG,
            "Failed closing SD test file after write: errno=%d (%s)",
            errno,
            strerror(errno));
        remove(SD_TEST_FILE_PATH);
        return ESP_FAIL;
    }

    if (written != expected_len)
    {
        ESP_LOGE(
            TAG,
            "Short SD write: expected %u bytes, wrote %u",
            (unsigned)expected_len,
            (unsigned)written);
        remove(SD_TEST_FILE_PATH);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Reading SD test file back");

    file = fopen(SD_TEST_FILE_PATH, "rb");
    if (file == NULL)
    {
        ESP_LOGE(
            TAG,
            "Could not open SD test file for reading: errno=%d (%s)",
            errno,
            strerror(errno));
        remove(SD_TEST_FILE_PATH);
        return ESP_FAIL;
    }

    char read_buffer[sizeof(expected) + 8];
    memset(read_buffer, 0, sizeof(read_buffer));

    const size_t bytes_read = fread(
        read_buffer,
        1,
        sizeof(read_buffer) - 1,
        file);

    if (fclose(file) != 0)
    {
        ESP_LOGW(
            TAG,
            "SD test file close after read returned errno=%d (%s)",
            errno,
            strerror(errno));
    }

    /*
     * Delete the temporary file regardless of whether the comparison succeeds.
     * The diagnostic should clean up after itself whenever possible.
     */
    if (remove(SD_TEST_FILE_PATH) != 0)
    {
        ESP_LOGW(
            TAG,
            "Could not remove SD test file: errno=%d (%s)",
            errno,
            strerror(errno));
    }

    if (bytes_read != expected_len ||
        memcmp(read_buffer, expected, expected_len) != 0)
    {
        ESP_LOGE(
            TAG,
            "SD read-back verification failed: expected %u bytes, read %u",
            (unsigned)expected_len,
            (unsigned)bytes_read);
        return ESP_FAIL;
    }

    ESP_LOGI(
        TAG,
        "SD read/write verification PASSED (%u bytes)",
        (unsigned)expected_len);

    return ESP_OK;
}

/**
 * @brief Enumerate the root directory and print its entries to the monitor.
 *
 * The 360x360 display is too small for a useful file browser, so this test
 * reports the total count on-screen and prints individual names to the serial
 * monitor. That still verifies directory traversal through FatFs/VFS.
 */
static esp_err_t sd_card_list_root(size_t *entry_count)
{
    if (entry_count != NULL)
    {
        *entry_count = 0;
    }

    esp_err_t err = sd_card_mount();
    if (err != ESP_OK)
    {
        return err;
    }

    DIR *dir = opendir(SD_MOUNT_POINT);
    if (dir == NULL)
    {
        ESP_LOGE(
            TAG,
            "Could not open SD root directory: errno=%d (%s)",
            errno,
            strerror(errno));
        return ESP_FAIL;
    }

    size_t count = 0;
    struct dirent *entry = NULL;

    ESP_LOGI(TAG, "SD root directory:");

    while ((entry = readdir(dir)) != NULL)
    {
        /*
         * Skip the synthetic current/parent entries if the filesystem returns
         * them. They are not useful when counting actual card contents.
         */
        if (strcmp(entry->d_name, ".") == 0 ||
            strcmp(entry->d_name, "..") == 0)
        {
            continue;
        }

        count++;
        ESP_LOGI(TAG, "  %s", entry->d_name);
    }

    closedir(dir);

    if (entry_count != NULL)
    {
        *entry_count = count;
    }

    ESP_LOGI(TAG, "SD root contains %u entries", (unsigned)count);
    return ESP_OK;
}

/**
 * @brief Refresh the SD diagnostic status label.
 *
 * message may be NULL. When supplied, it is appended beneath the basic card
 * information so button callbacks can report results such as "Read/write PASS."
 */
static void sd_card_update_status_label(const char *message)
{
    if (sd_test_label == NULL)
    {
        return;
    }

    if (!sd_card_mounted || sd_card == NULL)
    {
        if (message != NULL)
        {
            lv_label_set_text_fmt(
                sd_test_label,
                "SD: not mounted\n%s",
                message);
        }
        else
        {
            lv_label_set_text(
                sd_test_label,
                "SD: not mounted");
        }

        return;
    }

    /*
     * csd.capacity is expressed in sectors and csd.sector_size is bytes per
     * sector. Use 64-bit arithmetic so large SDXC capacities cannot overflow.
     */
    const uint64_t capacity_bytes =
        (uint64_t)sd_card->csd.capacity *
        (uint64_t)sd_card->csd.sector_size;

    const uint64_t capacity_mib =
        capacity_bytes / (1024ULL * 1024ULL);

    if (message != NULL)
    {
        lv_label_set_text_fmt(
            sd_test_label,
            "SD: mounted\n"
            "Name: %s\n"
            "Capacity: %llu MiB\n"
            "%s",
            sd_card->cid.name,
            (unsigned long long)capacity_mib,
            message);
    }
    else
    {
        lv_label_set_text_fmt(
            sd_test_label,
            "SD: mounted\n"
            "Name: %s\n"
            "Capacity: %llu MiB",
            sd_card->cid.name,
            (unsigned long long)capacity_mib);
    }
}

/* ============================================================================
 * 6. MICROPHONE / AUDIO DIAGNOSTICS
 * ============================================================================
 *
 * This integrated test intentionally focuses on the microphone that is directly
 * reachable from the ESP32-S3 while the LVGL display is running.
 *
 * The board's PCM5100A 3.5 mm output lives on the companion/shared audio path.
 * On common JC3636K518/Waveshare wiring, the S3-side PCM5100A I2S signals use
 * GPIO16/17/18, which overlap the active LCD QSPI data pins. Driving those pins
 * as I2S from this firmware would corrupt or disable the display. Therefore the
 * Audio + MIC page reports that topology instead of performing an unsafe tone
 * test from the S3.
 *
 * The microphone itself is a two-signal PDM device (clock + data) and can be
 * sampled safely. The capture task calculates a simple peak-based level and
 * publishes only scalar values. An LVGL timer turns those values into the live
 * meter on screen.
 * ========================================================================== */

/**
 * @brief Convert a signed 16-bit PCM peak into a useful 0..100 meter value.
 *
 * The PDM hardware converter returns 16-bit PCM. A logarithmic-ish mapping is
 * more useful than a linear full-scale mapping for speech, because normal room
 * sound occupies only a fraction of +/-32767.
 *
 * Peaks below 64 are treated as near-silence. Roughly bit 6 through bit 14 is
 * then spread across the meter. The raw peak remains visible on screen for
 * diagnostics, so this display scaling can be tuned later without obscuring
 * whether the microphone itself is producing data.
 */
/**
 * @brief Integer square root for a 64-bit value.
 *
 * This avoids pulling floating-point/logarithm code into the real-time capture
 * task merely to draw a diagnostic audio meter.
 */
static uint32_t mic_isqrt_u64(uint64_t value)
{
    uint64_t result = 0;
    uint64_t bit = (uint64_t)1 << 62;

    while (bit > value)
    {
        bit >>= 2;
    }

    while (bit != 0)
    {
        if (value >= result + bit)
        {
            value -= result + bit;
            result = (result >> 1) + bit;
        }
        else
        {
            result >>= 1;
        }

        bit >>= 2;
    }

    return (uint32_t)result;
}

/**
 * @brief Convert DC-corrected 16-bit RMS amplitude to a 0..100 meter value.
 *
 * Each doubling in RMS amplitude is about +6 dB. Mapping power-of-two ranges
 * across ten 10-point meter bands approximates a -60 dBFS .. 0 dBFS meter:
 *
 *      RMS <=    32  ->   0%
 *      RMS ~=    64  ->  10%
 *      RMS ~=   512  ->  40%
 *      RMS ~=  4096  ->  70%
 *      RMS ~= 16384  ->  90%
 *      RMS ~= 32767  -> 100%
 *
 * Unlike the v8 peak meter, a large constant DC offset does not inflate this
 * value because RMS is calculated only after subtracting each slot's mean.
 */
static int mic_level_from_rms(uint32_t rms)
{
    if (rms <= 32U)
    {
        return 0;
    }

    if (rms >= 32767U)
    {
        return 100;
    }

    int highest_bit = 31 - __builtin_clz(rms);

    /*
     * rms=32 has highest_bit=5. Every next bit represents one approximate
     * +6 dB interval, which maps to another 10 meter points.
     */
    int level = (highest_bit - 5) * 10;

    uint32_t base = 1U << highest_bit;
    uint32_t remainder = rms - base;

    /*
     * Interpolate inside the current power-of-two band rather than making the
     * bar jump in coarse 10-point steps.
     */
    level += (int)((remainder * 10U) / base);

    if (level < 0)
    {
        level = 0;
    }
    else if (level > 100)
    {
        level = 100;
    }

    return level;
}

/**
 * @brief FreeRTOS task that owns and reads the microphone I2S RX channel.
 */
static void mic_capture_task(void *arg)
{
    (void)arg;

    esp_err_t err = ESP_OK;

    /*
     * PDM RX on ESP32-S3 is provided by I2S0. Use it explicitly rather than
     * I2S_NUM_AUTO so the intent is clear and deterministic.
     */
    i2s_chan_config_t channel_config =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);

    channel_config.dma_desc_num = 6;
    channel_config.dma_frame_num = 256;

    err = i2s_new_channel(
        &channel_config,
        NULL,
        &mic_rx_chan);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "MIC PDM: i2s_new_channel failed: %s",
                 esp_err_to_name(err));
        mic_last_error = err;
        mic_test_state = MIC_TEST_ERROR;
        goto cleanup;
    }

    /*
     * The ESP32-S3 supports hardware PDM-to-PCM conversion. The microphone is
     * connected to the first PDM data input line.
     *
     * ESP32-S3 exposes four possible PDM input lines in the driver structure,
     * so unused DIN entries are explicitly marked I2S_GPIO_UNUSED.
     */
    i2s_pdm_rx_config_t pdm_config = {
        .clk_cfg =
            I2S_PDM_RX_CLK_DEFAULT_CONFIG(MIC_SAMPLE_RATE_HZ),

        .slot_cfg =
            I2S_PDM_RX_SLOT_PCM_FMT_DEFAULT_CONFIG(
                I2S_DATA_BIT_WIDTH_16BIT,
                I2S_SLOT_MODE_MONO),

        .gpio_cfg = {
            .clk = MIC_PDM_CLK_PIN,
            .dins = {
                MIC_PDM_DATA_PIN,
                I2S_GPIO_UNUSED,
                I2S_GPIO_UNUSED,
                I2S_GPIO_UNUSED,
            },
            .invert_flags = {
                .clk_inv = false,
            },
        },
    };

    /*
     * Only physical PDM data line 0 is wired on this board. Do not enable all
     * four ESP32-S3 PDM input lines, because the unused line slots can inject
     * meaningless samples into a diagnostic meter.
     *
     * Capture both left/right slots on line 0. The microphone's hardware SEL
     * state determines which slot actually contains useful audio. The capture
     * loop measures both independently and automatically selects the slot with
     * the larger DC-corrected RMS signal.
     */
    pdm_config.slot_cfg.slot_mode = I2S_SLOT_MODE_STEREO;
    pdm_config.slot_cfg.slot_mask =
        I2S_PDM_RX_LINE0_SLOT_RIGHT |
        I2S_PDM_RX_LINE0_SLOT_LEFT;

    err = i2s_channel_init_pdm_rx_mode(
        mic_rx_chan,
        &pdm_config);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "MIC PDM: init failed: %s",
                 esp_err_to_name(err));
        mic_last_error = err;
        mic_test_state = MIC_TEST_ERROR;
        goto cleanup;
    }

    err = i2s_channel_enable(mic_rx_chan);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "MIC PDM: enable failed: %s",
                 esp_err_to_name(err));
        mic_last_error = err;
        mic_test_state = MIC_TEST_ERROR;
        goto cleanup;
    }

    ESP_LOGI(
        TAG,
        "MIC PDM started: CLK=%d DATA=%d, %d Hz, 16-bit PCM",
        MIC_PDM_CLK_PIN,
        MIC_PDM_DATA_PIN,
        MIC_SAMPLE_RATE_HZ);

    mic_last_error = ESP_OK;
    mic_test_state = MIC_TEST_RUNNING;

    int16_t samples[512];
    uint32_t log_divider = 0;

    while (!mic_stop_requested)
    {
        size_t bytes_read = 0;

        err = i2s_channel_read(
            mic_rx_chan,
            samples,
            sizeof(samples),
            &bytes_read,
            pdMS_TO_TICKS(100));

        if (err == ESP_ERR_TIMEOUT)
        {
            continue;
        }

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "MIC PDM read failed: %s",
                     esp_err_to_name(err));
            mic_last_error = err;
            mic_test_state = MIC_TEST_ERROR;
            break;
        }

        size_t sample_count =
            bytes_read / sizeof(samples[0]);

        /*
         * Stereo PDM line-0 data is interleaved:
         *
         *      samples[0], samples[2], ...  -> slot 0
         *      samples[1], samples[3], ...  -> slot 1
         *
         * Calculate each slot independently. This is important because the
         * microphone may sit on either the LEFT or RIGHT PDM slot depending on
         * its board-level SEL connection.
         */
        int64_t sum[2] = {0, 0};
        uint32_t count[2] = {0, 0};

        for (size_t i = 0; i < sample_count; ++i)
        {
            int slot = (int)(i & 1U);
            sum[slot] += samples[i];
            count[slot]++;
        }

        int32_t mean[2] = {0, 0};

        for (int slot = 0; slot < 2; ++slot)
        {
            if (count[slot] != 0)
            {
                mean[slot] =
                    (int32_t)(sum[slot] / (int64_t)count[slot]);
            }
        }

        uint64_t square_sum[2] = {0, 0};

        for (size_t i = 0; i < sample_count; ++i)
        {
            int slot = (int)(i & 1U);

            int32_t centered =
                (int32_t)samples[i] - mean[slot];

            square_sum[slot] +=
                (uint64_t)((int64_t)centered * (int64_t)centered);
        }

        uint32_t rms[2] = {0, 0};

        for (int slot = 0; slot < 2; ++slot)
        {
            if (count[slot] != 0)
            {
                uint64_t mean_square =
                    square_sum[slot] / count[slot];

                rms[slot] =
                    mic_isqrt_u64(mean_square);
            }
        }

        /*
         * Whichever left/right slot contains the actual microphone should have
         * the stronger AC component. Select it automatically.
         */
        int active_slot =
            (rms[1] > rms[0]) ? 1 : 0;

        uint32_t selected_rms =
            rms[active_slot];

        int32_t selected_dc =
            mean[active_slot];

        int new_level =
            mic_level_from_rms(selected_rms);

        int old_level =
            mic_level_percent;

        /* Fast attack with a gentle decay makes speech visually readable. */
        if (new_level >= old_level)
        {
            mic_level_percent = new_level;
        }
        else
        {
            mic_level_percent =
                (old_level * 3 + new_level) / 4;
        }

        mic_rms_raw = selected_rms;
        mic_dc_raw = selected_dc;
        mic_active_slot = active_slot;
        mic_blocks_read++;

        /*
         * Log both slots. If the visual result is still strange, these values
         * immediately show whether one slot is real audio, a constant offset,
         * or garbage from the peripheral configuration.
         */
        if (++log_divider >= 50)
        {
            log_divider = 0;

            ESP_LOGI(
                TAG,
                "MIC PDM: blocks=%lu bytes=%u "
                "R[rms=%lu dc=%ld] L[rms=%lu dc=%ld] "
                "use=%c level=%d%%",
                (unsigned long)mic_blocks_read,
                (unsigned)bytes_read,
                (unsigned long)rms[0],
                (long)mean[0],
                (unsigned long)rms[1],
                (long)mean[1],
                active_slot == 0 ? 'R' : 'L',
                mic_level_percent);
        }
    }

cleanup:
    if (mic_rx_chan != NULL)
    {
        esp_err_t disable_err =
            i2s_channel_disable(mic_rx_chan);

        if (disable_err != ESP_OK &&
            disable_err != ESP_ERR_INVALID_STATE)
        {
            ESP_LOGW(TAG, "MIC PDM disable: %s",
                     esp_err_to_name(disable_err));
        }

        esp_err_t delete_err =
            i2s_del_channel(mic_rx_chan);

        if (delete_err != ESP_OK)
        {
            ESP_LOGW(TAG, "MIC PDM delete channel: %s",
                     esp_err_to_name(delete_err));
        }

        mic_rx_chan = NULL;
    }

    if (mic_test_state != MIC_TEST_ERROR)
    {
        mic_test_state = MIC_TEST_STOPPED;
        mic_last_error = ESP_OK;
    }

    mic_level_percent = 0;
    mic_rms_raw = 0;
    mic_dc_raw = 0;
    mic_active_slot = 0;
    mic_stop_requested = false;
    mic_task_handle = NULL;

    ESP_LOGI(TAG, "MIC PDM capture task stopped");
    vTaskDelete(NULL);
}

/**
 * @brief Start the live microphone test.
 */
static esp_err_t mic_test_start(void)
{
    /*
     * GPIO45 cannot simultaneously be the microphone PDM clock and the DAC
     * standard-I2S data output.
     */
    if (dac_tone_task_handle != NULL ||
        dac_test_state == DAC_TEST_WAITING_FOR_MIC ||
        dac_test_state == DAC_TEST_STARTING ||
        dac_test_state == DAC_TEST_RUNNING ||
        dac_test_state == DAC_TEST_STOPPING)
    {
        ESP_LOGW(TAG, "MIC start rejected: DAC tone currently owns shared GPIO45");
        return ESP_ERR_INVALID_STATE;
    }

    if (mic_task_handle != NULL || mic_test_state == MIC_TEST_STARTING ||
        mic_test_state == MIC_TEST_RUNNING || mic_test_state == MIC_TEST_STOPPING)
    {
        return ESP_ERR_INVALID_STATE;
    }

    mic_stop_requested = false;
    mic_level_percent = 0;
    mic_rms_raw = 0;
    mic_dc_raw = 0;
    mic_active_slot = 0;
    mic_blocks_read = 0;
    mic_last_error = ESP_OK;
    mic_test_state = MIC_TEST_STARTING;

    BaseType_t created = xTaskCreate(
        mic_capture_task,
        "mic_capture",
        MIC_TASK_STACK_SIZE,
        NULL,
        4,
        &mic_task_handle);

    if (created != pdPASS)
    {
        mic_task_handle = NULL;
        mic_last_error = ESP_ERR_NO_MEM;
        mic_test_state = MIC_TEST_ERROR;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

/**
 * @brief Ask the capture task to stop and release I2S asynchronously.
 */
static void mic_test_request_stop(void)
{
    if (mic_task_handle == NULL)
    {
        if (mic_test_state != MIC_TEST_ERROR)
        {
            mic_test_state = MIC_TEST_STOPPED;
        }
        return;
    }

    mic_test_state = MIC_TEST_STOPPING;
    mic_stop_requested = true;
}

/**
 * @brief PCM5100A test-tone worker.
 *
 * This task performs all potentially blocking I2S work outside LVGL context.
 * It first waits for the microphone to release GPIO45, routes the CH445P mux
 * to the S3, initializes standard-I2S TX, and continuously writes a quiet
 * 1 kHz stereo sine wave until stopped.
 */
static void dac_tone_task(void *arg)
{
    (void)arg;

    dac_test_state = DAC_TEST_WAITING_FOR_MIC;
    dac_last_error = ESP_OK;

    /*
     * GPIO45 is shared. Ask the MIC worker to stop, then wait for it to delete
     * its PDM I2S channel. The wait is bounded so a broken mic task cannot trap
     * this worker forever.
     */
    mic_test_request_stop();

    const TickType_t wait_step = pdMS_TO_TICKS(20);
    const int max_wait_steps = 100; /* 2 seconds */

    int wait_steps = 0;

    while ((mic_task_handle != NULL || mic_rx_chan != NULL) &&
           !dac_tone_stop_requested &&
           wait_steps < max_wait_steps)
    {
        vTaskDelay(wait_step);
        wait_steps++;
    }

    if (dac_tone_stop_requested)
    {
        goto cleanup;
    }

    if (mic_task_handle != NULL || mic_rx_chan != NULL)
    {
        dac_last_error = ESP_ERR_TIMEOUT;
        dac_test_state = DAC_TEST_ERROR;

        ESP_LOGE(
            TAG,
            "DAC tone: MIC did not release GPIO45 within timeout");

        goto cleanup;
    }

    dac_test_state = DAC_TEST_STARTING;

    /*
     * CH445P:
     *   IN LOW  -> S1 inputs -> companion ESP32
     *   IN HIGH -> S2 inputs -> ESP32-S3
     *
     * The schematic wires the S3 I2S signals to S2, so select HIGH.
     */
    gpio_config_t switch_cfg = {
        .pin_bit_mask = 1ULL << DAC_I2S_SWITCH_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err =
        gpio_config(&switch_cfg);

    if (err != ESP_OK)
    {
        dac_last_error = err;
        dac_test_state = DAC_TEST_ERROR;
        goto cleanup;
    }

    gpio_set_level(DAC_I2S_SWITCH_PIN, 1);

    /*
     * Use I2S1 for DAC TX. The mic test uses I2S0. They still may not run
     * simultaneously because of shared GPIO45, but separate peripherals make
     * ownership and cleanup unambiguous.
     */
    i2s_chan_config_t chan_cfg =
        I2S_CHANNEL_DEFAULT_CONFIG(
            I2S_NUM_1,
            I2S_ROLE_MASTER);

    chan_cfg.dma_desc_num = 6;
    chan_cfg.dma_frame_num = 256;

    err =
        i2s_new_channel(
            &chan_cfg,
            &dac_tx_chan,
            NULL);

    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "DAC tone: i2s_new_channel failed: %s",
            esp_err_to_name(err));

        dac_last_error = err;
        dac_test_state = DAC_TEST_ERROR;
        goto cleanup;
    }

    /*
     * PCM5100A accepts standard Philips I2S and does not require MCLK when its
     * internal PLL can lock to BCLK/LRCK.
     */
    i2s_std_config_t std_cfg = {
        .clk_cfg =
            I2S_STD_CLK_DEFAULT_CONFIG(DAC_SAMPLE_RATE_HZ),

        .slot_cfg =
            I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                I2S_DATA_BIT_WIDTH_16BIT,
                I2S_SLOT_MODE_STEREO),

        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = DAC_I2S_BCLK_PIN,
            .ws = DAC_I2S_WS_PIN,
            .dout = DAC_I2S_DATA_PIN,
            .din = I2S_GPIO_UNUSED,

            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    err =
        i2s_channel_init_std_mode(
            dac_tx_chan,
            &std_cfg);

    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "DAC tone: standard-I2S init failed: %s",
            esp_err_to_name(err));

        dac_last_error = err;
        dac_test_state = DAC_TEST_ERROR;
        goto cleanup;
    }

    err =
        i2s_channel_enable(dac_tx_chan);

    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "DAC tone: channel enable failed: %s",
            esp_err_to_name(err));

        dac_last_error = err;
        dac_test_state = DAC_TEST_ERROR;
        goto cleanup;
    }

    /*
     * 32-sample sine, peak amplitude ~= 4096 / 32767 = 12.5% full scale.
     * This intentionally starts quiet for headphones and powered speakers.
     */
    static const int16_t sine32[32] = {
           0,   799,  1567,  2276,  2896,  3406,  3784,  4017,
        4096,  4017,  3784,  3406,  2896,  2276,  1567,   799,
           0,  -799, -1567, -2276, -2896, -3406, -3784, -4017,
       -4096, -4017, -3784, -3406, -2896, -2276, -1567,  -799,
    };

    int16_t stereo_buffer[32 * 2];

    for (int i = 0; i < 32; ++i)
    {
        stereo_buffer[i * 2 + 0] = sine32[i];
        stereo_buffer[i * 2 + 1] = sine32[i];
    }

    dac_test_state = DAC_TEST_RUNNING;

    ESP_LOGI(
        TAG,
        "DAC tone started: %d Hz, %d Hz sample rate, "
        "BCLK=%d WS=%d DATA=%d MUX=%d(HIGH)",
        DAC_TONE_HZ,
        DAC_SAMPLE_RATE_HZ,
        DAC_I2S_BCLK_PIN,
        DAC_I2S_WS_PIN,
        DAC_I2S_DATA_PIN,
        DAC_I2S_SWITCH_PIN);

    while (!dac_tone_stop_requested)
    {
        size_t bytes_written = 0;

        err =
            i2s_channel_write(
                dac_tx_chan,
                stereo_buffer,
                sizeof(stereo_buffer),
                &bytes_written,
                pdMS_TO_TICKS(250));

        if (err != ESP_OK)
        {
            ESP_LOGE(
                TAG,
                "DAC tone write failed: %s",
                esp_err_to_name(err));

            dac_last_error = err;
            dac_test_state = DAC_TEST_ERROR;
            break;
        }
    }

cleanup:
    if (dac_tx_chan != NULL)
    {
        esp_err_t disable_err =
            i2s_channel_disable(dac_tx_chan);

        if (disable_err != ESP_OK &&
            disable_err != ESP_ERR_INVALID_STATE)
        {
            ESP_LOGW(
                TAG,
                "DAC tone disable: %s",
                esp_err_to_name(disable_err));
        }

        esp_err_t delete_err =
            i2s_del_channel(dac_tx_chan);

        if (delete_err != ESP_OK)
        {
            ESP_LOGW(
                TAG,
                "DAC tone delete: %s",
                esp_err_to_name(delete_err));
        }

        dac_tx_chan = NULL;
    }

    /*
     * Return the DAC mux to the companion ESP32 when our test is not using it.
     * That matches the board's normal stock-audio ownership.
     */
    gpio_set_level(DAC_I2S_SWITCH_PIN, 0);

    if (dac_test_state != DAC_TEST_ERROR)
    {
        dac_test_state = DAC_TEST_STOPPED;
        dac_last_error = ESP_OK;
    }

    dac_tone_stop_requested = false;
    dac_tone_task_handle = NULL;

    ESP_LOGI(TAG, "DAC tone task stopped");

    vTaskDelete(NULL);
}

/**
 * @brief Start the PCM5100A tone worker.
 */
static esp_err_t dac_tone_start(void)
{
    if (dac_tone_task_handle != NULL ||
        dac_test_state == DAC_TEST_WAITING_FOR_MIC ||
        dac_test_state == DAC_TEST_STARTING ||
        dac_test_state == DAC_TEST_RUNNING ||
        dac_test_state == DAC_TEST_STOPPING)
    {
        return ESP_ERR_INVALID_STATE;
    }

    dac_tone_stop_requested = false;
    dac_last_error = ESP_OK;
    dac_test_state = DAC_TEST_WAITING_FOR_MIC;

    BaseType_t created =
        xTaskCreate(
            dac_tone_task,
            "dac_tone",
            DAC_TONE_TASK_STACK,
            NULL,
            4,
            &dac_tone_task_handle);

    if (created != pdPASS)
    {
        dac_tone_task_handle = NULL;
        dac_last_error = ESP_ERR_NO_MEM;
        dac_test_state = DAC_TEST_ERROR;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

/**
 * @brief Ask the DAC tone task to stop asynchronously.
 */
static void dac_tone_request_stop(void)
{
    if (dac_tone_task_handle == NULL)
    {
        if (dac_test_state != DAC_TEST_ERROR)
        {
            dac_test_state = DAC_TEST_STOPPED;
        }

        /*
         * Ensure the companion ESP32 owns the audio mux while our S3 test is
         * idle even if a previous start failed before creating an I2S channel.
         */
        gpio_set_direction(DAC_I2S_SWITCH_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level(DAC_I2S_SWITCH_PIN, 0);
        return;
    }

    dac_test_state = DAC_TEST_STOPPING;
    dac_tone_stop_requested = true;
}

static void audio_mic_start_cb(lv_event_t *e)
{
    (void)e;

    esp_err_t err = mic_test_start();

    if (err != ESP_OK && audio_mic_status_label != NULL)
    {
        lv_label_set_text_fmt(
            audio_mic_status_label,
            "MIC start failed\n%s",
            esp_err_to_name(err));
    }
}

static void audio_mic_stop_cb(lv_event_t *e)
{
    (void)e;
    mic_test_request_stop();
}

static void audio_dac_start_tone_cb(lv_event_t *e)
{
    (void)e;

    esp_err_t err =
        dac_tone_start();

    if (err != ESP_OK &&
        audio_dac_info_label != NULL)
    {
        lv_label_set_text_fmt(
            audio_dac_info_label,
            "Tone start failed\n%s",
            esp_err_to_name(err));
    }
}

static void audio_dac_stop_tone_cb(lv_event_t *e)
{
    (void)e;
    dac_tone_request_stop();
}

/**
 * @brief Show the verified board-level DAC routing.
 */
static void audio_mic_output_info_cb(lv_event_t *e)
{
    (void)e;

    if (audio_dac_info_label != NULL)
    {
        lv_label_set_text(
            audio_dac_info_label,
            "PCM5100A via CH445P\n"
            "BCLK39  WS44  DATA45\n"
            "MUX GPIO40 HIGH = S3");
    }
}

static void audio_mic_back_cb(lv_event_t *e)
{
    (void)e;
    mic_test_request_stop();
    dac_tone_request_stop();
    pop_menu();
}

/**
 * @brief Refresh the microphone meter from LVGL context.
 */
static void audio_mic_ui_timer_cb(lv_timer_t *timer)
{
    (void)timer;

    if (current_menu != MENU_HW_AUDIO_MIC)
    {
        return;
    }

    if (audio_mic_level_bar != NULL)
    {
        lv_bar_set_value(
            audio_mic_level_bar,
            mic_level_percent,
            LV_ANIM_OFF);
    }

    if (audio_mic_peak_label != NULL)
    {
        lv_label_set_text_fmt(
            audio_mic_peak_label,
            "Level %d%%  RMS %lu\nDC %ld  Slot %c  Blocks %lu",
            mic_level_percent,
            (unsigned long)mic_rms_raw,
            (long)mic_dc_raw,
            mic_active_slot == 0 ? 'R' : 'L',
            (unsigned long)mic_blocks_read);
    }

    if (audio_mic_status_label == NULL)
    {
        return;
    }

    if (audio_dac_info_label != NULL)
    {
        switch (dac_test_state)
        {
        case DAC_TEST_STOPPED:
            lv_label_set_text(
                audio_dac_info_label,
                "PCM5100A: stopped\n"
                "Connect headphones / powered speaker");
            break;

        case DAC_TEST_WAITING_FOR_MIC:
            lv_label_set_text(
                audio_dac_info_label,
                "PCM5100A: waiting for MIC\n"
                "Releasing shared GPIO45...");
            break;

        case DAC_TEST_STARTING:
            lv_label_set_text(
                audio_dac_info_label,
                "PCM5100A: starting...");
            break;

        case DAC_TEST_RUNNING:
            lv_label_set_text(
                audio_dac_info_label,
                "PCM5100A: LIVE\n"
                "1 kHz tone @ 12.5% full scale");
            break;

        case DAC_TEST_STOPPING:
            lv_label_set_text(
                audio_dac_info_label,
                "PCM5100A: stopping...");
            break;

        case DAC_TEST_ERROR:
            lv_label_set_text_fmt(
                audio_dac_info_label,
                "PCM5100A error\n%s",
                esp_err_to_name(dac_last_error));
            break;

        default:
            break;
        }
    }

    switch (mic_test_state)
    {
    case MIC_TEST_STOPPED:
        lv_label_set_text(audio_mic_status_label, "MIC: stopped");
        break;

    case MIC_TEST_STARTING:
        lv_label_set_text(audio_mic_status_label, "MIC: starting...");
        break;

    case MIC_TEST_RUNNING:
        lv_label_set_text(
            audio_mic_status_label,
            "MIC: LIVE  PDM -> 16 kHz PCM\n"
            "Speak near the microphone");
        break;

    case MIC_TEST_STOPPING:
        lv_label_set_text(audio_mic_status_label, "MIC: stopping...");
        break;

    case MIC_TEST_ERROR:
        lv_label_set_text_fmt(
            audio_mic_status_label,
            "MIC error\n%s",
            esp_err_to_name(mic_last_error));
        break;

    default:
        break;
    }
}

/* ============================================================================
 * 7. BATTERY / SYSTEM-VOLTAGE ADC DIAGNOSTIC
 * ============================================================================
 *
 * Uses ESP-IDF's ADC oneshot driver on ADC1 channel 0 (GPIO1).
 *
 * The S3 curve-fitting calibration scheme is used when available. ESP32 ADC
 * reference voltage varies between chips, so calibrated millivolts are much
 * more useful than pretending a raw 12-bit code maps perfectly to 3.3 V.
 *
 * Eight samples are averaged per worker acquisition to reduce display jitter.
 * ========================================================================== */

/**
 * @brief Lazily initialize ADC1 channel 0 and its calibration scheme.
 */
static esp_err_t battery_adc_init(void)
{
    if (battery_adc_initialized)
    {
        return ESP_OK;
    }

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = BATTERY_ADC_UNIT,
    };

    esp_err_t err =
        adc_oneshot_new_unit(
            &unit_cfg,
            &battery_adc_handle);

    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Battery ADC: adc_oneshot_new_unit failed: %s",
            esp_err_to_name(err));

        return err;
    }

    adc_oneshot_chan_cfg_t channel_cfg = {
        .atten = BATTERY_ADC_ATTEN,
        .bitwidth = BATTERY_ADC_BITWIDTH,
    };

    err =
        adc_oneshot_config_channel(
            battery_adc_handle,
            BATTERY_ADC_CHANNEL,
            &channel_cfg);

    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Battery ADC: channel config failed: %s",
            esp_err_to_name(err));

        adc_oneshot_del_unit(battery_adc_handle);
        battery_adc_handle = NULL;
        return err;
    }

    /*
     * ESP32-S3 supports the curve-fitting calibration scheme.
     *
     * Calibration failure is not fatal to the diagnostic page. The raw ADC
     * remains useful, and a clearly-labelled approximate voltage fallback is
     * provided rather than disabling the test entirely.
     */
    adc_cali_curve_fitting_config_t cal_cfg = {
        .unit_id = BATTERY_ADC_UNIT,
        .chan = BATTERY_ADC_CHANNEL,
        .atten = BATTERY_ADC_ATTEN,
        .bitwidth = BATTERY_ADC_BITWIDTH,
    };

    err =
        adc_cali_create_scheme_curve_fitting(
            &cal_cfg,
            &battery_adc_cali_handle);

    if (err == ESP_OK)
    {
        battery_adc_calibrated = true;
        ESP_LOGI(TAG, "Battery ADC: curve-fitting calibration enabled");
    }
    else
    {
        battery_adc_cali_handle = NULL;
        battery_adc_calibrated = false;

        ESP_LOGW(
            TAG,
            "Battery ADC: calibration unavailable (%s); using approximate voltage",
            esp_err_to_name(err));
    }

    battery_adc_initialized = true;

    ESP_LOGI(
        TAG,
        "Battery ADC ready: ADC1 CH0 / GPIO1 / 2:1 divider");

    return ESP_OK;
}

/**
 * @brief Read a multisampled ADC value and calculate monitored rail voltage.
 */
static esp_err_t battery_adc_read(
    int *raw_average,
    int *adc_pin_mv,
    int *system_mv,
    bool *used_calibration)
{
    if (raw_average == NULL ||
        adc_pin_mv == NULL ||
        system_mv == NULL ||
        used_calibration == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = battery_adc_init();

    if (err != ESP_OK)
    {
        return err;
    }

    int64_t raw_sum = 0;

    for (int i = 0; i < BATTERY_ADC_MULTISAMPLES; ++i)
    {
        int raw = 0;

        err =
            adc_oneshot_read(
                battery_adc_handle,
                BATTERY_ADC_CHANNEL,
                &raw);

        if (err != ESP_OK)
        {
            return err;
        }

        raw_sum += raw;
    }

    int raw =
        (int)(raw_sum / BATTERY_ADC_MULTISAMPLES);

    int pin_mv = 0;
    bool calibrated = false;

    if (battery_adc_calibrated &&
        battery_adc_cali_handle != NULL)
    {
        err =
            adc_cali_raw_to_voltage(
                battery_adc_cali_handle,
                raw,
                &pin_mv);

        if (err == ESP_OK)
        {
            calibrated = true;
        }
        else
        {
            ESP_LOGW(
                TAG,
                "Battery ADC: calibrated conversion failed: %s",
                esp_err_to_name(err));
        }
    }

    if (!calibrated)
    {
        /*
         * Fallback only.
         *
         * This intentionally mirrors the simple approximation commonly used
         * in board demos. ADC nonlinearity/reference variation means this is
         * less trustworthy than the calibration path.
         */
        pin_mv =
            (raw * 3300) / 4095;
    }

    int rail_mv =
        (pin_mv * BATTERY_DIVIDER_NUMERATOR) /
        BATTERY_DIVIDER_DENOMINATOR;

    *raw_average = raw;
    *adc_pin_mv = pin_mv;
    *system_mv = rail_mv;
    *used_calibration = calibrated;

    return ESP_OK;
}

/**
 * @brief Rough single-cell Li-ion state-of-charge estimate.
 *
 * Voltage-only battery percentages are inherently approximate and vary with
 * load, temperature, cell chemistry, and charging state. This piecewise curve
 * is therefore used only as a human-friendly diagnostic indicator.
 */
static int battery_percent_from_mv(int mv)
{
    typedef struct
    {
        int mv;
        int pct;
    } point_t;

    static const point_t curve[] = {
        {3300,   0},
        {3450,  10},
        {3600,  20},
        {3700,  40},
        {3800,  60},
        {3900,  75},
        {4000,  85},
        {4100,  95},
        {4200, 100},
    };

    if (mv <= curve[0].mv)
    {
        return 0;
    }

    const int point_count =
        (int)(sizeof(curve) / sizeof(curve[0]));

    if (mv >= curve[point_count - 1].mv)
    {
        return 100;
    }

    for (int i = 1; i < point_count; ++i)
    {
        if (mv <= curve[i].mv)
        {
            int span_mv =
                curve[i].mv - curve[i - 1].mv;

            int into_mv =
                mv - curve[i - 1].mv;

            int span_pct =
                curve[i].pct - curve[i - 1].pct;

            return curve[i - 1].pct +
                (into_mv * span_pct) / span_mv;
        }
    }

    return 0;
}

/**
 * @brief Low-priority ADC acquisition task.
 *
 * The worker intentionally owns every call into the ADC driver. It publishes
 * only scalar values, so LVGL never waits for ADC initialization, calibration,
 * a driver mutex, or a hardware conversion.
 *
 * The task remains alive while the Battery page is open. When the user leaves
 * the page it exits automatically. The ADC unit itself remains initialized for
 * reuse on a later visit.
 */
static void battery_adc_worker_task(void *arg)
{
    (void)arg;

    battery_sample_state = BATTERY_SAMPLE_STARTING;
    battery_latest_error = ESP_OK;

    ESP_LOGI(TAG, "Battery ADC worker started");

    uint32_t sample_counter = 0;

    while (current_menu == MENU_HW_BATTERY)
    {
        int raw = 0;
        int pin_mv = 0;
        int rail_mv = 0;
        bool calibrated = false;

        /*
         * This log is deliberately before the read. If a board/driver issue
         * ever stalls inside adc_oneshot_read(), the serial monitor will make
         * the exact location obvious while the LVGL UI remains responsive.
         */
        if (sample_counter == 0)
        {
            ESP_LOGI(TAG, "Battery ADC: requesting first sample");
        }

        esp_err_t err =
            battery_adc_read(
                &raw,
                &pin_mv,
                &rail_mv,
                &calibrated);

        if (err != ESP_OK)
        {
            battery_latest_error = err;
            battery_sample_state = BATTERY_SAMPLE_ERROR;

            ESP_LOGE(
                TAG,
                "Battery ADC worker read failed: %s",
                esp_err_to_name(err));

            /*
             * Keep the task alive so a transient ADC error can recover without
             * forcing the user to leave and re-enter the page.
             */
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        battery_latest_raw = raw;
        battery_latest_pin_mv = pin_mv;
        battery_latest_rail_mv = rail_mv;
        battery_latest_calibrated = calibrated;
        battery_latest_error = ESP_OK;
        battery_sample_state = BATTERY_SAMPLE_READY;

        if (sample_counter == 0 || (sample_counter % 4U) == 0U)
        {
            ESP_LOGI(
                TAG,
                "Battery ADC sample: raw=%d pin=%d mV rail=%d mV calibrated=%d",
                raw,
                pin_mv,
                rail_mv,
                calibrated ? 1 : 0);
        }

        sample_counter++;

        /* The UI refreshes at 2 Hz, so there is no reason to sample faster. */
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGI(TAG, "Battery ADC worker stopped");

    battery_adc_task_handle = NULL;
    battery_sample_state = BATTERY_SAMPLE_IDLE;

    vTaskDelete(NULL);
}

/**
 * @brief Start the ADC worker if it is not already running.
 */
static void battery_adc_worker_start(void)
{
    if (battery_adc_task_handle != NULL)
    {
        return;
    }

    battery_sample_state = BATTERY_SAMPLE_STARTING;
    battery_latest_error = ESP_OK;

    BaseType_t result =
        xTaskCreate(
            battery_adc_worker_task,
            "battery_adc",
            4096,
            NULL,
            2,
            &battery_adc_task_handle);

    if (result != pdPASS)
    {
        battery_adc_task_handle = NULL;
        battery_latest_error = ESP_ERR_NO_MEM;
        battery_sample_state = BATTERY_SAMPLE_ERROR;

        ESP_LOGE(TAG, "Battery ADC: failed to create worker task");
    }
}

/**
 * @brief Update the Battery / ADC page from LVGL context.
 *
 * This callback performs NO ADC hardware access. It only renders the most
 * recent values published by battery_adc_worker_task().
 */
static void battery_adc_ui_timer_cb(lv_timer_t *timer)
{
    (void)timer;

    if (current_menu != MENU_HW_BATTERY ||
        battery_adc_status_label == NULL ||
        battery_adc_detail_label == NULL ||
        battery_adc_level_bar == NULL)
    {
        return;
    }

    battery_sample_state_t state =
        battery_sample_state;

    if (state == BATTERY_SAMPLE_IDLE ||
        state == BATTERY_SAMPLE_STARTING)
    {
        lv_label_set_text(
            battery_adc_status_label,
            "Starting ADC...");

        lv_label_set_text(
            battery_adc_detail_label,
            "GPIO1 / ADC1 CH0\nWaiting for first sample");

        lv_bar_set_value(
            battery_adc_level_bar,
            0,
            LV_ANIM_OFF);

        return;
    }

    if (state == BATTERY_SAMPLE_ERROR)
    {
        esp_err_t err =
            battery_latest_error;

        lv_label_set_text_fmt(
            battery_adc_status_label,
            "ADC read error\n%s",
            esp_err_to_name(err));

        lv_label_set_text(
            battery_adc_detail_label,
            "GPIO1 / ADC1 CH0\nUI remains responsive");

        lv_bar_set_value(
            battery_adc_level_bar,
            0,
            LV_ANIM_OFF);

        return;
    }

    /*
     * Copy volatile values once so a worker update cannot create a visibly
     * inconsistent label half-way through formatting.
     */
    int raw = battery_latest_raw;
    int pin_mv = battery_latest_pin_mv;
    int rail_mv = battery_latest_rail_mv;
    bool calibrated = battery_latest_calibrated;

    /*
     * Use integer millivolt formatting instead of %.3f.
     *
     * Besides avoiding unnecessary floating-point formatting in the GUI path,
     * this prints the ADC result exactly to the precision the calibration API
     * actually provides.
     */
    int rail_whole = rail_mv / 1000;
    int rail_frac = rail_mv % 1000;
    int pin_whole = pin_mv / 1000;
    int pin_frac = pin_mv % 1000;

    if (rail_frac < 0)
    {
        rail_frac = -rail_frac;
    }

    if (pin_frac < 0)
    {
        pin_frac = -pin_frac;
    }

    if (rail_mv <= 4350)
    {
        int percent =
            battery_percent_from_mv(rail_mv);

        lv_label_set_text_fmt(
            battery_adc_status_label,
            "System %d.%03d V\nBattery est. %d%%",
            rail_whole,
            rail_frac,
            percent);

        lv_bar_set_value(
            battery_adc_level_bar,
            percent,
            LV_ANIM_OFF);
    }
    else
    {
        lv_label_set_text_fmt(
            battery_adc_status_label,
            "System %d.%03d V\nUSB / external power likely",
            rail_whole,
            rail_frac);

        lv_bar_set_value(
            battery_adc_level_bar,
            100,
            LV_ANIM_OFF);
    }

    lv_label_set_text_fmt(
        battery_adc_detail_label,
        "Raw %d\nGPIO1 %d.%03d V\n%s",
        raw,
        pin_whole,
        pin_frac,
        calibrated ? "Calibrated" : "Approximate");
}

/* ============================================================================
 * 8. BLE MEDIA-CONTROLLER INTEGRATION
 * ============================================================================
 * Bluetooth/HID details live in media_controller_ble.c. This section only
 * translates UI input into logical media commands.
 * ========================================================================== */

static void media_send_from_ui(media_control_key_t key, const char *name)
{
    esp_err_t err = media_controller_send(key);

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Media command queued: %s", name);
        return;
    }

    if (err == ESP_ERR_INVALID_STATE)
    {
        ESP_LOGW(TAG, "Media command ignored (%s): no BLE HID host connected", name);
        return;
    }

    ESP_LOGW(TAG, "Media command failed (%s): %s", name, esp_err_to_name(err));
}

static void open_media_controller_cb(lv_event_t *e)
{
    (void)e;
    push_menu(MENU_MEDIA_CONTROLLER);
}

static void media_previous_cb(lv_event_t *e)
{
    (void)e;
    media_send_from_ui(MEDIA_CONTROL_PREVIOUS_TRACK, "previous track");
}

static void media_play_pause_cb(lv_event_t *e)
{
    (void)e;
    media_send_from_ui(MEDIA_CONTROL_PLAY_PAUSE, "play/pause");
}

static void media_next_cb(lv_event_t *e)
{
    (void)e;
    media_send_from_ui(MEDIA_CONTROL_NEXT_TRACK, "next track");
}

static void media_mute_cb(lv_event_t *e)
{
    (void)e;
    media_send_from_ui(MEDIA_CONTROL_MUTE, "mute");
}

static void media_status_timer_cb(lv_timer_t *timer)
{
    (void)timer;

    if (current_menu != MENU_MEDIA_CONTROLLER || media_status_label == NULL)
        return;

    lv_label_set_text(media_status_label, media_controller_status_text());
}

/* ============================================================================
 * 9. MENU / UI IMPLEMENTATION
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
    /* Stop microphone capture whenever navigation leaves its diagnostic page. */
    if (current_menu == MENU_HW_AUDIO_MIC && menu != MENU_HW_AUDIO_MIC)
    {
        mic_test_request_stop();
    }

    /*
     * Clear test-widget pointers before deleting menu_cont. Deleting the parent
     * also deletes all child LVGL objects.
     */
    touch_test_label = NULL;
    encoder_test_label = NULL;
    sd_test_label = NULL;
    media_status_label = NULL;
    audio_mic_status_label = NULL;
    audio_mic_level_bar = NULL;
    audio_mic_peak_label = NULL;
    audio_dac_info_label = NULL;
    battery_adc_status_label = NULL;
    battery_adc_detail_label = NULL;
    battery_adc_level_bar = NULL;

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
    case MENU_MEDIA_CONTROLLER:
        create_media_controller_menu();
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
    case MENU_HW_SD:
        create_hw_sd_test_menu();
        break;
    case MENU_HW_HAPTICS:
        create_hw_haptics_test_menu();
        break;
    case MENU_HW_AUDIO_MIC:
        create_hw_audio_mic_test_menu();
        break;
    case MENU_HW_BATTERY:
        create_hw_battery_test_menu();
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
    push_menu(MENU_HW_SD);
}

static void open_hw_haptics_cb(lv_event_t *e)
{
    (void)e;
    push_menu(MENU_HW_HAPTICS);
}

static void open_hw_audio_mic_cb(lv_event_t *e)
{
    (void)e;
    push_menu(MENU_HW_AUDIO_MIC);
}

static void open_hw_battery_cb(lv_event_t *e)
{
    (void)e;
    push_menu(MENU_HW_BATTERY);
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

    create_menu_button(menu_cont, "Media Controller", open_media_controller_cb);
    create_menu_button(menu_cont, "Settings", open_settings_cb);
    create_menu_button(menu_cont, "Hardware Tests", open_hardware_tests_cb);
    create_menu_button(menu_cont, "About", open_about_cb);
}

static void create_media_controller_menu(void)
{
    menu_cont = create_menu_container();

    lv_obj_t *title = lv_label_create(menu_cont);
    lv_label_set_text(title, "PC Media Controller");

    media_status_label = lv_label_create(menu_cont);
    lv_label_set_text(media_status_label, media_controller_status_text());

    lv_obj_t *hint = lv_label_create(menu_cont);
    lv_label_set_text(hint, "Bezel: Volume\nPair in Windows Bluetooth");
    lv_obj_set_style_text_align(hint, LV_TEXT_ALIGN_CENTER, 0);

    create_menu_button(menu_cont, "Previous", media_previous_cb);
    create_menu_button(menu_cont, "Play / Pause", media_play_pause_cb);
    create_menu_button(menu_cont, "Next", media_next_cb);
    create_menu_button(menu_cont, "Mute", media_mute_cb);
    create_menu_button(menu_cont, "Back", back_cb);
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
/**
 * @brief Mount (or refresh) the SD card and update the on-screen status.
 */
static void sd_mount_refresh_cb(lv_event_t *e)
{
    (void)e;

    esp_err_t err = sd_card_mount();

    if (err == ESP_OK)
    {
        sd_card_update_status_label("Mount: OK");
    }
    else
    {
        char message[96];
        snprintf(
            message,
            sizeof(message),
            "Mount failed:\n%s",
            esp_err_to_name(err));

        sd_card_update_status_label(message);
    }
}

/**
 * @brief Run the temporary-file read/write verification from the UI.
 */
static void sd_read_write_test_cb(lv_event_t *e)
{
    (void)e;

    sd_card_update_status_label("Running R/W test...");

    esp_err_t err = sd_card_read_write_test();

    if (err == ESP_OK)
    {
        sd_card_update_status_label("Read/write: PASS");
    }
    else
    {
        char message[96];
        snprintf(
            message,
            sizeof(message),
            "Read/write: FAIL\n%s",
            esp_err_to_name(err));

        sd_card_update_status_label(message);
    }
}

/**
 * @brief Enumerate the root directory and report the number of entries.
 */
static void sd_list_root_cb(lv_event_t *e)
{
    (void)e;

    size_t entry_count = 0;
    esp_err_t err = sd_card_list_root(&entry_count);

    if (err == ESP_OK)
    {
        char message[96];
        snprintf(
            message,
            sizeof(message),
            "Root entries: %u\nSee serial monitor",
            (unsigned)entry_count);

        sd_card_update_status_label(message);
    }
    else
    {
        char message[96];
        snprintf(
            message,
            sizeof(message),
            "List root: FAIL\n%s",
            esp_err_to_name(err));

        sd_card_update_status_label(message);
    }
}

/**
 * @brief Unmount the card from the diagnostic page.
 */
static void sd_unmount_cb(lv_event_t *e)
{
    (void)e;

    esp_err_t err = sd_card_unmount();

    if (err == ESP_OK)
    {
        sd_card_update_status_label("Unmount: OK");
    }
    else
    {
        char message[96];
        snprintf(
            message,
            sizeof(message),
            "Unmount failed:\n%s",
            esp_err_to_name(err));

        sd_card_update_status_label(message);
    }
}

/**
 * @brief Build the microSD hardware-test page.
 *
 * Entering this page attempts a mount immediately. Failure is non-fatal and is
 * shown on-screen; the user can insert a card and press Mount / Refresh again.
 */
static void create_hw_sd_test_menu(void)
{
    menu_cont = create_menu_container();

    lv_obj_t *title = lv_label_create(menu_cont);
    lv_label_set_text(title, "TF / microSD Test");

    sd_test_label = lv_label_create(menu_cont);
    lv_obj_set_width(sd_test_label, 230);
    lv_obj_set_style_text_align(
        sd_test_label,
        LV_TEXT_ALIGN_CENTER,
        0);

    /*
     * Try to mount as soon as the page opens. This keeps SD initialization out
     * of app_main(), so a missing card can never prevent the main UI from
     * starting.
     */
    esp_err_t mount_err = sd_card_mount();

    if (mount_err == ESP_OK)
    {
        sd_card_update_status_label("Ready");
    }
    else
    {
        char message[96];
        snprintf(
            message,
            sizeof(message),
            "Insert FAT-formatted card\n%s",
            esp_err_to_name(mount_err));

        sd_card_update_status_label(message);
    }

    create_menu_button(
        menu_cont,
        "Mount / Refresh",
        sd_mount_refresh_cb);

    create_menu_button(
        menu_cont,
        "Read / Write Test",
        sd_read_write_test_cb);

    create_menu_button(
        menu_cont,
        "List Root",
        sd_list_root_cb);

    create_menu_button(
        menu_cont,
        "Unmount",
        sd_unmount_cb);

    create_menu_button(
        menu_cont,
        "Back",
        back_cb);
}

/**
 * @brief Build the integrated Audio + MIC hardware-test page.
 *
 * Both sides are now real hardware tests:
 *
 *   - onboard PDM microphone -> live RMS meter
 *   - ESP32-S3 -> CH445P -> PCM5100A -> 3.5 mm 1 kHz tone
 *
 * GPIO45 is shared between PDM MIC clock and DAC data, so the firmware never
 * allows the microphone and DAC tone to own the pin at the same time.
 */
static void create_hw_audio_mic_test_menu(void)
{
    menu_cont = create_menu_container();

    lv_obj_t *title = lv_label_create(menu_cont);
    lv_label_set_text(title, "Audio + MIC Test");

    audio_mic_status_label = lv_label_create(menu_cont);
    lv_obj_set_width(audio_mic_status_label, 240);
    lv_obj_set_style_text_align(
        audio_mic_status_label,
        LV_TEXT_ALIGN_CENTER,
        0);

    switch (mic_test_state)
    {
    case MIC_TEST_RUNNING:
        lv_label_set_text(audio_mic_status_label, "MIC: LIVE");
        break;
    case MIC_TEST_STARTING:
        lv_label_set_text(audio_mic_status_label, "MIC: starting...");
        break;
    case MIC_TEST_STOPPING:
        lv_label_set_text(audio_mic_status_label, "MIC: stopping...");
        break;
    case MIC_TEST_ERROR:
        lv_label_set_text_fmt(
            audio_mic_status_label,
            "MIC error\n%s",
            esp_err_to_name(mic_last_error));
        break;
    default:
        lv_label_set_text(audio_mic_status_label, "MIC: stopped");
        break;
    }

    audio_mic_level_bar = lv_bar_create(menu_cont);
    lv_obj_set_size(audio_mic_level_bar, 220, 22);
    lv_bar_set_range(audio_mic_level_bar, 0, 100);
    lv_bar_set_value(
        audio_mic_level_bar,
        mic_level_percent,
        LV_ANIM_OFF);

    audio_mic_peak_label = lv_label_create(menu_cont);
    lv_obj_set_width(audio_mic_peak_label, 240);
    lv_obj_set_style_text_align(
        audio_mic_peak_label,
        LV_TEXT_ALIGN_CENTER,
        0);
    lv_label_set_text(audio_mic_peak_label, "Level 0%  RMS 0\nDC 0  Slot ?");

    create_menu_button(
        menu_cont,
        "Start MIC Meter",
        audio_mic_start_cb);

    create_menu_button(
        menu_cont,
        "Stop MIC Meter",
        audio_mic_stop_cb);

    audio_dac_info_label = lv_label_create(menu_cont);
    lv_obj_set_width(audio_dac_info_label, 240);
    lv_obj_set_style_text_align(
        audio_dac_info_label,
        LV_TEXT_ALIGN_CENTER,
        0);

    switch (dac_test_state)
    {
    case DAC_TEST_RUNNING:
        lv_label_set_text(
            audio_dac_info_label,
            "PCM5100A: LIVE\n1 kHz test tone");
        break;

    case DAC_TEST_ERROR:
        lv_label_set_text_fmt(
            audio_dac_info_label,
            "PCM5100A error\n%s",
            esp_err_to_name(dac_last_error));
        break;

    default:
        lv_label_set_text(
            audio_dac_info_label,
            "PCM5100A: stopped\n"
            "Connect headphones / powered speaker");
        break;
    }

    create_menu_button(
        menu_cont,
        "Start 1 kHz Tone",
        audio_dac_start_tone_cb);

    create_menu_button(
        menu_cont,
        "Stop Tone",
        audio_dac_stop_tone_cb);

    create_menu_button(
        menu_cont,
        "3.5mm Routing Info",
        audio_mic_output_info_cb);

    create_menu_button(
        menu_cont,
        "Back",
        audio_mic_back_cb);
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

/**
 * @brief Build the live Battery / ADC diagnostic page.
 */
static void create_hw_battery_test_menu(void)
{
    menu_cont = create_menu_container();

    lv_obj_t *title =
        lv_label_create(menu_cont);

    lv_label_set_text(
        title,
        "Battery / ADC");

    lv_obj_t *hardware =
        lv_label_create(menu_cont);

    lv_label_set_text(
        hardware,
        "ADC1 CH0 / GPIO1\n2:1 voltage divider");

    lv_obj_set_style_text_align(
        hardware,
        LV_TEXT_ALIGN_CENTER,
        0);

    battery_adc_status_label =
        lv_label_create(menu_cont);

    lv_label_set_text(
        battery_adc_status_label,
        "Reading ADC...");

    lv_obj_set_style_text_align(
        battery_adc_status_label,
        LV_TEXT_ALIGN_CENTER,
        0);

    battery_adc_level_bar =
        lv_bar_create(menu_cont);

    lv_obj_set_size(
        battery_adc_level_bar,
        200,
        20);

    lv_bar_set_range(
        battery_adc_level_bar,
        0,
        100);

    lv_bar_set_value(
        battery_adc_level_bar,
        0,
        LV_ANIM_OFF);

    battery_adc_detail_label =
        lv_label_create(menu_cont);

    lv_label_set_text(
        battery_adc_detail_label,
        "Raw ---\nGPIO1 --- V\nInitializing");

    lv_obj_set_style_text_align(
        battery_adc_detail_label,
        LV_TEXT_ALIGN_CENTER,
        0);

    lv_obj_t *note =
        lv_label_create(menu_cont);

    lv_label_set_text(
        note,
        "Waveshare monitors system voltage.\n"
        "Battery % is only a rough estimate.");

    lv_obj_set_style_text_align(
        note,
        LV_TEXT_ALIGN_CENTER,
        0);

    create_menu_button(
        menu_cont,
        "Back",
        back_cb);

    /*
     * Do not perform ADC work from this LV_EVENT_PRESSED call path.
     *
     * The worker runs independently. The normal 500 ms LVGL timer will update
     * this page when the first result becomes available.
     */
    battery_adc_worker_start();
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
 * 10. LVGL DISPLAY AND TOUCH BRIDGE
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
 * 11. FREERTOS TASKS AND PERIODIC INPUT PROCESSING
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

    /*
     * Media Controller page: consume bezel motion as Windows volume commands
     * instead of scrolling the menu. Swap UP/DOWN below if physical direction
     * feels reversed on your hardware.
     */
    if (current_menu == MENU_MEDIA_CONTROLLER)
    {
        media_control_key_t command =
            (steps > 0) ? MEDIA_CONTROL_VOLUME_UP : MEDIA_CONTROL_VOLUME_DOWN;

        int32_t count = (steps > 0) ? steps : -steps;

        for (int32_t i = 0; i < count; i++)
        {
            if (media_controller_send(command) == ESP_OK)
            {
                ESP_ERROR_CHECK_WITHOUT_ABORT(drv2605_play_effect(1));
            }
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
 * 12. APPLICATION ENTRY POINT / HARDWARE INITIALIZATION
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

    /* BLE HID is optional to the local UI; log failure instead of rebooting. */
    ESP_LOGI(TAG, "Initialize BLE HID media controller");
    esp_err_t media_init_err = media_controller_init();
    if (media_init_err != ESP_OK)
    {
        ESP_LOGW(TAG, "BLE media controller unavailable: %s", esp_err_to_name(media_init_err));
    }

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

    /* Refresh BLE connection text from LVGL context. */
    lv_timer_create(media_status_timer_cb, 250, NULL);

    /* Refresh live microphone level/status from LVGL context. */
    lv_timer_create(audio_mic_ui_timer_cb, 100, NULL);

    /* Refresh Battery / ADC values twice per second while that page is open. */
    lv_timer_create(battery_adc_ui_timer_cb, 500, NULL);


        lvgl_unlock();
    }
}
