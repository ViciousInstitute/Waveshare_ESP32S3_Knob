/*
 * BLE HID consumer-control implementation for ESP-IDF 6.0.x.
 *
 * This implementation uses Espressif's supported esp_hid component and the
 * Bluedroid BLE stack. The report descriptor is intentionally small: one byte
 * contains seven Consumer Control buttons.
 *
 * Bit layout of HID report ID 1:
 *
 *   bit 0 : Scan Next Track
 *   bit 1 : Scan Previous Track
 *   bit 2 : Stop
 *   bit 3 : Play/Pause
 *   bit 4 : Mute
 *   bit 5 : Volume Increment
 *   bit 6 : Volume Decrement
 *   bit 7 : constant padding
 *
 * A dedicated queue/task converts each application command into:
 *
 *      press report -> short delay -> all-zero release report
 *
 * This keeps LVGL and the encoder path responsive even when many volume steps
 * arrive quickly.
 */

#include "media_controller_ble.h"

#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"

#include "esp_hid_common.h"
#include "esp_hidd.h"

#include "esp_log.h"
#include "esp_check.h"

#define MEDIA_DEVICE_NAME           "Waveshare Media Knob"
#define MEDIA_MANUFACTURER_NAME     "ESP32-S3"
#define MEDIA_SERIAL_NUMBER         "WS-MEDIA-001"

#define MEDIA_REPORT_ID             1
#define MEDIA_REPORT_MAP_INDEX      0
#define MEDIA_REPORT_LENGTH         1

#define MEDIA_COMMAND_QUEUE_LENGTH  24
#define MEDIA_COMMAND_TASK_STACK    3072
#define MEDIA_COMMAND_TASK_PRIORITY 5
#define MEDIA_KEY_HOLD_MS           18

/*
 * Report-bit definitions matching the order of usages in the HID descriptor.
 */
#define MEDIA_BIT_NEXT_TRACK        (1U << 0)
#define MEDIA_BIT_PREVIOUS_TRACK    (1U << 1)
#define MEDIA_BIT_STOP              (1U << 2)
#define MEDIA_BIT_PLAY_PAUSE        (1U << 3)
#define MEDIA_BIT_MUTE              (1U << 4)
#define MEDIA_BIT_VOLUME_UP         (1U << 5)
#define MEDIA_BIT_VOLUME_DOWN       (1U << 6)

static const char *TAG = "media_ble";

static esp_hidd_dev_t *s_hid_device = NULL;
static QueueHandle_t s_command_queue = NULL;

static volatile bool s_ready = false;
static volatile bool s_connected = false;
static volatile bool s_hid_started = false;
static volatile bool s_adv_data_ready = false;
static volatile bool s_advertising = false;

/*
 * BLE HID Consumer Control report descriptor.
 *
 * Windows sees this as a standard HID Consumer Control device. No custom PC
 * driver is needed for the usages listed here.
 */
static const uint8_t s_media_report_map[] = {
    0x05, 0x0C,       /* Usage Page (Consumer) */
    0x09, 0x01,       /* Usage (Consumer Control) */
    0xA1, 0x01,       /* Collection (Application) */

    0x85, MEDIA_REPORT_ID, /* Report ID */

    0x15, 0x00,       /* Logical Minimum (0) */
    0x25, 0x01,       /* Logical Maximum (1) */
    0x75, 0x01,       /* Report Size (1 bit) */
    0x95, 0x07,       /* Report Count (7 buttons) */

    0x09, 0xB5,       /* Scan Next Track */
    0x09, 0xB6,       /* Scan Previous Track */
    0x09, 0xB7,       /* Stop */
    0x09, 0xCD,       /* Play/Pause */
    0x09, 0xE2,       /* Mute */
    0x09, 0xE9,       /* Volume Increment */
    0x09, 0xEA,       /* Volume Decrement */

    0x81, 0x02,       /* Input (Data,Var,Abs) */

    0x75, 0x01,       /* Report Size (1 bit) */
    0x95, 0x01,       /* Report Count (1 padding bit) */
    0x81, 0x03,       /* Input (Const,Var,Abs) */

    0xC0              /* End Collection */
};

static esp_hid_raw_report_map_t s_report_maps[] = {
    {
        .data = s_media_report_map,
        .len = sizeof(s_media_report_map),
    },
};

static esp_hid_device_config_t s_hid_config = {
    /*
     * These IDs are sufficient for a personal/test HID device. They are not a
     * claim of a commercial USB/Bluetooth VID/PID assignment.
     */
    .vendor_id = 0x16C0,
    .product_id = 0x05DF,
    .version = 0x0100,
    .device_name = MEDIA_DEVICE_NAME,
    .manufacturer_name = MEDIA_MANUFACTURER_NAME,
    .serial_number = MEDIA_SERIAL_NUMBER,
    .report_maps = s_report_maps,
    .report_maps_len = 1,
};

/*
 * HID over GATT service UUID (0x1812), represented in the 128-bit Bluetooth
 * base-UUID form expected by esp_ble_adv_data_t.
 */
static uint8_t s_hid_service_uuid128[] = {
    0xFB, 0x34, 0x9B, 0x5F,
    0x80, 0x00,
    0x00, 0x80,
    0x00, 0x10,
    0x00, 0x00,
    0x12, 0x18,
    0x00, 0x00
};

static esp_ble_adv_params_t s_adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x30,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_ble_adv_data_t s_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = ESP_HID_APPEARANCE_GENERIC,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(s_hid_service_uuid128),
    .p_service_uuid = s_hid_service_uuid128,
    .flag = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT,
};

/**
 * @brief Translate an application command to one bit in report ID 1.
 */
static uint8_t media_key_to_report_bit(media_control_key_t key)
{
    switch (key)
    {
    case MEDIA_CONTROL_NEXT_TRACK:
        return MEDIA_BIT_NEXT_TRACK;

    case MEDIA_CONTROL_PREVIOUS_TRACK:
        return MEDIA_BIT_PREVIOUS_TRACK;

    case MEDIA_CONTROL_STOP:
        return MEDIA_BIT_STOP;

    case MEDIA_CONTROL_PLAY_PAUSE:
        return MEDIA_BIT_PLAY_PAUSE;

    case MEDIA_CONTROL_MUTE:
        return MEDIA_BIT_MUTE;

    case MEDIA_CONTROL_VOLUME_UP:
        return MEDIA_BIT_VOLUME_UP;

    case MEDIA_CONTROL_VOLUME_DOWN:
        return MEDIA_BIT_VOLUME_DOWN;

    default:
        return 0;
    }
}

/**
 * @brief Start connectable BLE advertising once both prerequisites are ready.
 *
 * Advertising data configuration and HID profile startup are asynchronous.
 * Whichever callback happens second will start advertising.
 */
static void media_try_start_advertising(void)
{
    if (!s_hid_started ||
        !s_adv_data_ready ||
        s_connected ||
        s_advertising)
    {
        return;
    }

    esp_err_t err = esp_ble_gap_start_advertising(&s_adv_params);

    if (err == ESP_OK)
    {
        s_advertising = true;
        ESP_LOGI(TAG, "BLE HID advertising requested");
    }
    else
    {
        ESP_LOGE(
            TAG,
            "Could not start BLE advertising: %s",
            esp_err_to_name(err));
    }
}

/**
 * @brief BLE GAP callback: advertising state and pairing/bonding.
 */
static void media_gap_event_handler(
    esp_gap_ble_cb_event_t event,
    esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        s_adv_data_ready = true;
        ESP_LOGI(TAG, "BLE advertising data configured");
        media_try_start_advertising();
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS)
        {
            s_advertising = true;
            ESP_LOGI(TAG, "Advertising as \"%s\"", MEDIA_DEVICE_NAME);
        }
        else
        {
            s_advertising = false;
            ESP_LOGE(
                TAG,
                "BLE advertising start failed, status=%d",
                param->adv_start_cmpl.status);
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        s_advertising = false;
        break;

    case ESP_GAP_BLE_SEC_REQ_EVT:
        /*
         * Accept the host's security request. Bonding keys are saved by the
         * Bluetooth stack through NVS, allowing Windows to reconnect later.
         */
        esp_ble_gap_security_rsp(
            param->ble_security.ble_req.bd_addr,
            true);
        break;

    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        if (param->ble_security.auth_cmpl.success)
        {
            ESP_LOGI(TAG, "BLE pairing/bonding succeeded");
        }
        else
        {
            ESP_LOGE(
                TAG,
                "BLE authentication failed, reason=0x%02X",
                param->ble_security.auth_cmpl.fail_reason);
        }
        break;

    default:
        break;
    }
}

/**
 * @brief esp_hid device callback.
 */
static void media_hidd_event_handler(
    void *handler_args,
    esp_event_base_t base,
    int32_t id,
    void *event_data)
{
    (void)handler_args;
    (void)base;

    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param =
        (esp_hidd_event_data_t *)event_data;

    switch (event)
    {
    case ESP_HIDD_START_EVENT:
        s_hid_started = true;
        s_ready = true;
        ESP_LOGI(TAG, "BLE HID profile started");
        media_try_start_advertising();
        break;

    case ESP_HIDD_CONNECT_EVENT:
        s_connected = true;
        s_advertising = false;
        ESP_LOGI(TAG, "PC/host connected to BLE HID");
        break;

    case ESP_HIDD_DISCONNECT_EVENT:
        s_connected = false;
        s_advertising = false;

        ESP_LOGI(
            TAG,
            "BLE HID disconnected, reason=%d",
            param != NULL ? param->disconnect.reason : -1);

        media_try_start_advertising();
        break;

    case ESP_HIDD_STOP_EVENT:
        s_connected = false;
        s_ready = false;
        s_hid_started = false;
        s_advertising = false;
        ESP_LOGI(TAG, "BLE HID profile stopped");
        break;

    default:
        break;
    }
}

/**
 * @brief Dedicated worker that sends HID press + release reports.
 */
static void media_command_task(void *arg)
{
    (void)arg;

    media_control_key_t key;

    for (;;)
    {
        if (xQueueReceive(
                s_command_queue,
                &key,
                portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        /*
         * A queued command can outlive a connection by a few milliseconds.
         * Re-check here instead of blindly calling into the HID stack.
         */
        if (!s_connected ||
            s_hid_device == NULL ||
            !esp_hidd_dev_connected(s_hid_device))
        {
            continue;
        }

        const uint8_t pressed_bit =
            media_key_to_report_bit(key);

        if (pressed_bit == 0)
        {
            continue;
        }

        uint8_t report[MEDIA_REPORT_LENGTH] = {
            pressed_bit
        };

        esp_err_t err = esp_hidd_dev_input_set(
            s_hid_device,
            MEDIA_REPORT_MAP_INDEX,
            MEDIA_REPORT_ID,
            report,
            sizeof(report));

        if (err != ESP_OK)
        {
            ESP_LOGW(
                TAG,
                "Media key press failed: %s",
                esp_err_to_name(err));
            continue;
        }

        /*
         * Hosts expect momentary media keys to be released. A small hold time
         * also prevents extremely fast press/release packets from being folded
         * together by the BLE/host stack.
         */
        vTaskDelay(pdMS_TO_TICKS(MEDIA_KEY_HOLD_MS));

        report[0] = 0;

        err = esp_hidd_dev_input_set(
            s_hid_device,
            MEDIA_REPORT_MAP_INDEX,
            MEDIA_REPORT_ID,
            report,
            sizeof(report));

        if (err != ESP_OK)
        {
            ESP_LOGW(
                TAG,
                "Media key release failed: %s",
                esp_err_to_name(err));
        }
    }
}

/**
 * @brief Initialize NVS for BLE bonding.
 */
static esp_err_t media_nvs_init(void)
{
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "NVS needs erase/reinitialization");
        ESP_RETURN_ON_ERROR(
            nvs_flash_erase(),
            TAG,
            "Could not erase NVS");

        err = nvs_flash_init();
    }

    return err;
}

/**
 * @brief Initialize the ESP32-S3 BLE controller and Bluedroid host.
 */
static esp_err_t media_bluetooth_stack_init(void)
{
    /*
     * ESP32-S3 is used here as BLE-only. Releasing Classic-BT memory leaves
     * more RAM available to LVGL, PSRAM bookkeeping, SD/FatFs, etc.
     */
    esp_err_t release_err =
        esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

    if (release_err != ESP_OK &&
        release_err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGW(
            TAG,
            "Classic BT memory release returned %s",
            esp_err_to_name(release_err));
    }

    esp_bt_controller_config_t bt_cfg =
        BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ESP_RETURN_ON_ERROR(
        esp_bt_controller_init(&bt_cfg),
        TAG,
        "Bluetooth controller init failed");

    ESP_RETURN_ON_ERROR(
        esp_bt_controller_enable(ESP_BT_MODE_BLE),
        TAG,
        "Bluetooth controller enable failed");

    esp_bluedroid_config_t bluedroid_cfg =
        BT_BLUEDROID_INIT_CONFIG_DEFAULT();

    ESP_RETURN_ON_ERROR(
        esp_bluedroid_init_with_cfg(&bluedroid_cfg),
        TAG,
        "Bluedroid init failed");

    ESP_RETURN_ON_ERROR(
        esp_bluedroid_enable(),
        TAG,
        "Bluedroid enable failed");

    return ESP_OK;
}

/**
 * @brief Configure BLE advertising and pairing policy.
 */
static esp_err_t media_gap_init(void)
{
    ESP_RETURN_ON_ERROR(
        esp_ble_gap_register_callback(media_gap_event_handler),
        TAG,
        "BLE GAP callback registration failed");

    /*
     * Bond with Windows, but use NoInputNoOutput pairing. This gives the device
     * simple "pair once, reconnect later" behavior without needing a passkey UI.
     */
    esp_ble_auth_req_t auth_req =
        ESP_LE_AUTH_REQ_SC_BOND;

    esp_ble_io_cap_t io_cap =
        ESP_IO_CAP_NONE;

    uint8_t key_size = 16;

    uint8_t init_key =
        ESP_BLE_ENC_KEY_MASK |
        ESP_BLE_ID_KEY_MASK;

    uint8_t response_key =
        ESP_BLE_ENC_KEY_MASK |
        ESP_BLE_ID_KEY_MASK;

    ESP_RETURN_ON_ERROR(
        esp_ble_gap_set_security_param(
            ESP_BLE_SM_AUTHEN_REQ_MODE,
            &auth_req,
            sizeof(auth_req)),
        TAG,
        "Could not set BLE auth mode");

    ESP_RETURN_ON_ERROR(
        esp_ble_gap_set_security_param(
            ESP_BLE_SM_IOCAP_MODE,
            &io_cap,
            sizeof(io_cap)),
        TAG,
        "Could not set BLE IO capability");

    ESP_RETURN_ON_ERROR(
        esp_ble_gap_set_security_param(
            ESP_BLE_SM_MAX_KEY_SIZE,
            &key_size,
            sizeof(key_size)),
        TAG,
        "Could not set BLE key size");

    ESP_RETURN_ON_ERROR(
        esp_ble_gap_set_security_param(
            ESP_BLE_SM_SET_INIT_KEY,
            &init_key,
            sizeof(init_key)),
        TAG,
        "Could not set BLE initiator keys");

    ESP_RETURN_ON_ERROR(
        esp_ble_gap_set_security_param(
            ESP_BLE_SM_SET_RSP_KEY,
            &response_key,
            sizeof(response_key)),
        TAG,
        "Could not set BLE response keys");

    ESP_RETURN_ON_ERROR(
        esp_ble_gap_set_device_name(MEDIA_DEVICE_NAME),
        TAG,
        "Could not set BLE device name");

    /*
     * esp_ble_gap_config_adv_data() completes asynchronously. Its GAP callback
     * sets s_adv_data_ready, and media_try_start_advertising() starts the radio
     * once the HID profile has also reached ESP_HIDD_START_EVENT.
     */
    ESP_RETURN_ON_ERROR(
        esp_ble_gap_config_adv_data(&s_adv_data),
        TAG,
        "Could not configure BLE advertising data");

    return ESP_OK;
}

esp_err_t media_controller_init(void)
{
    if (s_ready || s_hid_device != NULL)
    {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing BLE HID media controller");

    ESP_RETURN_ON_ERROR(
        media_nvs_init(),
        TAG,
        "NVS initialization failed");

    ESP_RETURN_ON_ERROR(
        media_bluetooth_stack_init(),
        TAG,
        "Bluetooth stack initialization failed");

    ESP_RETURN_ON_ERROR(
        media_gap_init(),
        TAG,
        "BLE GAP initialization failed");

    /*
     * esp_hid's BLE transport receives the raw GATTS callbacks through this
     * handler. Without it, the HID service cannot function.
     */
    ESP_RETURN_ON_ERROR(
        esp_ble_gatts_register_callback(
            esp_hidd_gatts_event_handler),
        TAG,
        "HID GATTS callback registration failed");

    ESP_RETURN_ON_ERROR(
        esp_hidd_dev_init(
            &s_hid_config,
            ESP_HID_TRANSPORT_BLE,
            media_hidd_event_handler,
            &s_hid_device),
        TAG,
        "BLE HID device initialization failed");

    /*
     * A media controller connected to a desktop does not have a meaningful
     * battery percentage yet. Report 100% until the board's battery/ADC driver
     * is integrated.
     */
    ESP_ERROR_CHECK_WITHOUT_ABORT(
        esp_hidd_dev_battery_set(
            s_hid_device,
            100));

    s_command_queue = xQueueCreate(
        MEDIA_COMMAND_QUEUE_LENGTH,
        sizeof(media_control_key_t));

    if (s_command_queue == NULL)
    {
        ESP_LOGE(TAG, "Could not allocate media command queue");
        return ESP_ERR_NO_MEM;
    }

    BaseType_t task_ok = xTaskCreate(
        media_command_task,
        "media_hid_tx",
        MEDIA_COMMAND_TASK_STACK,
        NULL,
        MEDIA_COMMAND_TASK_PRIORITY,
        NULL);

    if (task_ok != pdPASS)
    {
        vQueueDelete(s_command_queue);
        s_command_queue = NULL;

        ESP_LOGE(TAG, "Could not create media HID transmit task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(
        TAG,
        "BLE media controller initialized; waiting for advertising/pairing");

    return ESP_OK;
}

esp_err_t media_controller_send(media_control_key_t key)
{
    if (!s_ready ||
        !s_connected ||
        s_hid_device == NULL ||
        s_command_queue == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (media_key_to_report_bit(key) == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (xQueueSend(
            s_command_queue,
            &key,
            0) != pdTRUE)
    {
        ESP_LOGW(TAG, "Media HID command queue is full");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

bool media_controller_is_ready(void)
{
    return s_ready;
}

bool media_controller_is_connected(void)
{
    if (!s_connected || s_hid_device == NULL)
    {
        return false;
    }

    return esp_hidd_dev_connected(s_hid_device);
}

const char *media_controller_status_text(void)
{
    if (!s_ready)
    {
        return "BLE: unavailable";
    }

    if (media_controller_is_connected())
    {
        return "BLE: connected";
    }

    if (s_advertising)
    {
        return "BLE: advertising";
    }

    return "BLE: starting";
}
