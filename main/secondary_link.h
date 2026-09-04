#ifndef SECONDARY_LINK_H
#define SECONDARY_LINK_H

/*
 * ESP32-S3 <-> ESP32-U4WDH companion UART service.
 *
 * VERIFIED WAVESHARE SCHEMATIC WIRING
 *   S3 GPIO38 TX  -> U4WDH GPIO18 RX
 *   S3 GPIO48 RX  <- U4WDH GPIO23 TX
 *
 * v24 / protocol v4 adds live PCM5100A render telemetry from companion v7.
 * The U4WDH now owns the normal Bluetooth audio path; the S3 remains the UI,
 * BLE-HID controller, diagnostics host, and explicit alternate DAC owner.
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SECONDARY_LINK_UART_BAUD 115200
#define SECONDARY_LINK_S3_TX_GPIO 38
#define SECONDARY_LINK_S3_RX_GPIO 48
#define SECONDARY_LINK_LAST_LINE_MAX 320
#define SECONDARY_LINK_MODE_MAX 16
#define SECONDARY_LINK_BT_STATE_MAX 16
#define SECONDARY_LINK_A2DP_STATE_MAX 16
#define SECONDARY_LINK_STREAM_STATE_MAX 16
#define SECONDARY_LINK_PEER_MAX 18

#define SECONDARY_CAP_XSMT_CONTROL (1UL << 0)
#define SECONDARY_CAP_STATE_QUERY  (1UL << 1)
#define SECONDARY_CAP_CAPS_QUERY   (1UL << 2)
#define SECONDARY_CAP_BT_CLASSIC   (1UL << 8)
#define SECONDARY_CAP_A2DP_SINK    (1UL << 9)
#define SECONDARY_CAP_AVRCP        (1UL << 10)
#define SECONDARY_CAP_DAC_AUDIO    (1UL << 11)

typedef struct
{
    bool initialized;
    bool companion_seen;

    uint32_t companion_fw;
    uint32_t protocol_version;
    uint32_t capabilities;
    char companion_mode[SECONDARY_LINK_MODE_MAX];

    bool cap_xsmt_control;
    bool cap_state_query;
    bool cap_caps_query;
    bool cap_bt_classic;
    bool cap_a2dp_sink;
    bool cap_avrcp;
    bool cap_dac_audio;

    /*
     * Physical XSMT readback reported by the companion. -1 means no valid
     * readback has arrived yet. xsmt_requested remains separate so the UI can
     * expose a command/readback mismatch instead of assuming a write worked.
     */
    int xsmt_level;
    int xsmt_requested;

    /*
     * v24 runtime Bluetooth/A2DP/audio telemetry copied from protocol-v4 frames.
     * Raw string tokens stay available for diagnostics, while the booleans are
     * convenience interpretations for later control logic. a2dp_pcm_bytes is
     * deliberately 64-bit because a normal PCM stream crosses 4 GiB quickly.
     */
    char bt_state[SECONDARY_LINK_BT_STATE_MAX];
    char a2dp_state[SECONDARY_LINK_A2DP_STATE_MAX];
    char stream_state[SECONDARY_LINK_STREAM_STATE_MAX];
    char peer_bda[SECONDARY_LINK_PEER_MAX];
    bool bt_ready;
    bool a2dp_initialized;
    bool a2dp_connected;
    bool a2dp_streaming;
    uint64_t a2dp_pcm_bytes;
    uint32_t bt_errors;

    uint32_t audio_sample_rate_hz;
    uint8_t audio_channels;
    uint64_t audio_output_bytes;
    uint64_t audio_drop_bytes;
    uint32_t audio_errors;

    uint32_t tx_lines;
    uint32_t rx_lines;
    uint32_t pings_sent;
    uint32_t pongs_received;
    /* STATE and legacy STATUS replies received from the companion. */
    uint32_t status_received;
    uint32_t caps_received;
    uint32_t xsmt_commands_sent;
    uint32_t xsmt_acks_received;
    uint32_t parse_errors;

    uint32_t last_ping_sequence;
    uint32_t last_pong_sequence;
    uint32_t last_rtt_ms;
    /* UINT32_MAX means no recognized companion frame has arrived yet. */
    uint32_t last_rx_age_ms;
    uint32_t companion_uptime_ms;

    char last_line[SECONDARY_LINK_LAST_LINE_MAX];
} secondary_link_status_t;

esp_err_t secondary_link_init(void);
esp_err_t secondary_link_send_ping(void);
esp_err_t secondary_link_request_status(void);
esp_err_t secondary_link_request_capabilities(void);
esp_err_t secondary_link_set_xsmt(bool enabled);
void secondary_link_get_status(secondary_link_status_t *out_status);

#ifdef __cplusplus
}
#endif

#endif
