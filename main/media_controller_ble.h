#pragma once

/*
 * BLE HID media-controller abstraction for the Waveshare ESP32-S3 knob.
 *
 * This module deliberately exposes a very small API to the UI:
 *
 *     media_controller_init()
 *     media_controller_send(...)
 *     media_controller_is_connected()
 *     media_controller_is_ready()
 *
 * main.c does not need to know anything about Bluetooth GATT, HID report
 * descriptors, advertising, bonding, or press/release report timing.
 */

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Logical consumer-control commands used by the application.
 *
 * The values are application-level identifiers, not USB HID usage IDs.
 * media_controller_ble.c translates them into bits in its HID report.
 */
typedef enum
{
    MEDIA_CONTROL_NEXT_TRACK = 0,
    MEDIA_CONTROL_PREVIOUS_TRACK,
    MEDIA_CONTROL_STOP,
    MEDIA_CONTROL_PLAY_PAUSE,
    MEDIA_CONTROL_MUTE,
    MEDIA_CONTROL_VOLUME_UP,
    MEDIA_CONTROL_VOLUME_DOWN
} media_control_key_t;

/**
 * @brief Initialize NVS, the BLE controller, Bluedroid, BLE HID and advertising.
 *
 * Safe to call once during app_main().
 *
 * @return ESP_OK on success, otherwise the ESP-IDF error that stopped startup.
 */
esp_err_t media_controller_init(void);

/**
 * @brief Queue one media-key press/release for transmission to the PC.
 *
 * This call is non-blocking. A dedicated FreeRTOS task sends the press report,
 * waits briefly, then sends the release report.
 *
 * @return
 *   ESP_OK if queued,
 *   ESP_ERR_INVALID_STATE if BLE HID is not ready/connected,
 *   ESP_ERR_TIMEOUT if the command queue is full.
 */
esp_err_t media_controller_send(media_control_key_t key);

/**
 * @brief True after the BLE HID profile has initialized successfully.
 */
bool media_controller_is_ready(void);

/**
 * @brief True while a BLE HID host is connected.
 */
bool media_controller_is_connected(void);

/**
 * @brief Human-readable state for the LVGL status label.
 */
const char *media_controller_status_text(void);

#ifdef __cplusplus
}
#endif
