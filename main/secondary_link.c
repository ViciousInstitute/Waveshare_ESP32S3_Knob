/*
 * Waveshare ESP32-S3-Knob-Touch-LCD-1.8
 * ESP32-S3 <-> ESP32-U4WDH UART diagnostic/service
 *
 * WHY THIS IS ITS OWN MODULE
 * --------------------------
 * main.c already owns LVGL navigation and presentation. It should not also own
 * byte-stream framing and a long-running UART receive loop. This file therefore
 * owns the hardware transport and publishes only a copied status snapshot.
 *
 * VERIFIED PHYSICAL LINK
 * ----------------------
 *   ESP32-S3 GPIO38 TX  ----> ESP32-U4WDH GPIO18 RX
 *   ESP32-S3 GPIO48 RX  <---- ESP32-U4WDH GPIO23 TX
 *
 * Both sides use UART2. UART0 remains available for USB flashing/monitoring.
 *
 * PROTOCOL v4 (v24 / companion v7)
 * -----------------------------------
 * S3 -> U4:  PING <sequence>\n
 * S3 -> U4:  GET STATE\n
 * S3 -> U4:  GET CAPS\n
 * S3 -> U4:  SET XSMT 0|1\n
 * U4 -> S3:  STATE / CAPS / ACK XSMT / PONG / HELLO frames\n
 *
 * Companion v7 retains STATUS and XSMT as compatibility aliases.
 *
 * ASCII remains intentional while the companion grows: every frame stays readable
 * in a terminal/logical analyzer, while protocol versioning and capability bits
 * now give us enough structure to evolve without guessing feature availability.
 *
 * THREAD OWNERSHIP
 * ----------------
 * A FreeRTOS worker owns RX parsing and periodic PING generation. LVGL never
 * reads the UART directly. State is copied through a mutex-protected snapshot.
 */

#include "secondary_link.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define LINK_UART UART_NUM_2
#define LINK_RX_BUFFER 512
#define LINK_TX_BUFFER 512
#define LINK_RX_POLL_MS 20
#define LINK_HEARTBEAT_MS 1000
#define LINK_TASK_STACK 4096
#define LINK_TASK_PRIORITY 4
#define LINK_WIRE_LINE_MAX 448

static const char *TAG = "secondary_link";

static SemaphoreHandle_t state_mutex = NULL;
static SemaphoreHandle_t tx_mutex = NULL;
static secondary_link_status_t state;
static int64_t last_recognized_rx_us = 0;

static uint32_t next_ping_sequence = 1;
static uint32_t pending_ping_sequence = 0;
static int64_t pending_ping_sent_us = 0;

static bool lock_state(TickType_t ticks)
{
    return state_mutex != NULL && xSemaphoreTake(state_mutex, ticks) == pdTRUE;
}

static void unlock_state(void)
{
    if (state_mutex != NULL)
    {
        xSemaphoreGive(state_mutex);
    }
}

/* Store a bounded copy of the last complete line for the diagnostic UI. */
static void remember_line(const char *line)
{
    if (line == NULL)
    {
        return;
    }

    if (lock_state(portMAX_DELAY))
    {
        snprintf(state.last_line, sizeof(state.last_line), "%s", line);
        state.rx_lines++;
        unlock_state();
    }
}

/* A recognized frame is also our liveness timestamp. */
static void mark_seen(void)
{
    int64_t now_us = esp_timer_get_time();

    if (lock_state(portMAX_DELAY))
    {
        state.companion_seen = true;
        last_recognized_rx_us = now_us;
        unlock_state();
    }
}

static void parse_error(const char *line)
{
    if (lock_state(portMAX_DELAY))
    {
        state.parse_errors++;
        unlock_state();
    }

    ESP_LOGW(TAG, "Malformed/unknown frame: '%s'", line ? line : "(null)");
}

/*
 * Complete protocol lines can be written by the heartbeat task or an LVGL
 * button callback. tx_mutex guarantees those lines can never interleave bytes.
 */
static esp_err_t write_line(const char *line)
{
    if (line == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!state.initialized || tx_mutex == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(tx_mutex, pdMS_TO_TICKS(250)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    size_t len = strlen(line);
    int written = uart_write_bytes(LINK_UART, line, len);

    xSemaphoreGive(tx_mutex);

    if (written < 0 || (size_t)written != len)
    {
        ESP_LOGE(TAG, "UART write wanted=%u wrote=%d", (unsigned)len, written);
        return ESP_FAIL;
    }

    if (lock_state(portMAX_DELAY))
    {
        state.tx_lines++;
        unlock_state();
    }

    return ESP_OK;
}

static esp_err_t send_ping_internal(void)
{
    uint32_t sequence = next_ping_sequence++;
    if (next_ping_sequence == 0)
    {
        next_ping_sequence = 1;
    }

    char frame[48];
    snprintf(frame, sizeof(frame), "PING %lu\n", (unsigned long)sequence);

    int64_t sent_us = esp_timer_get_time();
    esp_err_t err = write_line(frame);
    if (err != ESP_OK)
    {
        return err;
    }

    pending_ping_sequence = sequence;
    pending_ping_sent_us = sent_us;

    if (lock_state(portMAX_DELAY))
    {
        state.pings_sent++;
        state.last_ping_sequence = sequence;
        unlock_state();
    }

    return ESP_OK;
}

/*
 * Extract an unsigned key=value field. Base 0 intentionally accepts both normal
 * decimal fields (FW=5) and capability masks (CAPS=0x00000007).
 */
static bool parse_named_u32(const char *line, const char *key, uint32_t *value)
{
    if (line == NULL || key == NULL || value == NULL)
    {
        return false;
    }

    const char *p = strstr(line, key);
    if (p == NULL)
    {
        return false;
    }

    p += strlen(key);
    char *end = NULL;
    unsigned long parsed = strtoul(p, &end, 0);
    if (end == p)
    {
        return false;
    }

    *value = (uint32_t)parsed;
    return true;
}

/* PCM byte totals can exceed 32 bits during long-running streams. */
static bool parse_named_u64(const char *line, const char *key, uint64_t *value)
{
    if (line == NULL || key == NULL || value == NULL)
    {
        return false;
    }

    const char *p = strstr(line, key);
    if (p == NULL)
    {
        return false;
    }

    p += strlen(key);
    char *end = NULL;
    unsigned long long parsed = strtoull(p, &end, 0);
    if (end == p)
    {
        return false;
    }

    *value = (uint64_t)parsed;
    return true;
}

/* Copy a single non-space key=value token into a bounded destination string. */
static bool parse_named_token(
    const char *line,
    const char *key,
    char *out,
    size_t out_size)
{
    if (line == NULL || key == NULL || out == NULL || out_size == 0)
    {
        return false;
    }

    const char *p = strstr(line, key);
    if (p == NULL)
    {
        return false;
    }

    p += strlen(key);
    size_t n = 0;
    while (p[n] != '\0' && p[n] != ' ' && n + 1 < out_size)
    {
        out[n] = p[n];
        n++;
    }

    out[n] = '\0';
    return n > 0;
}

static void apply_capabilities_locked(uint32_t mask)
{
    state.capabilities = mask;
    state.cap_xsmt_control = (mask & SECONDARY_CAP_XSMT_CONTROL) != 0;
    state.cap_state_query = (mask & SECONDARY_CAP_STATE_QUERY) != 0;
    state.cap_caps_query = (mask & SECONDARY_CAP_CAPS_QUERY) != 0;
    state.cap_bt_classic = (mask & SECONDARY_CAP_BT_CLASSIC) != 0;
    state.cap_a2dp_sink = (mask & SECONDARY_CAP_A2DP_SINK) != 0;
    state.cap_avrcp = (mask & SECONDARY_CAP_AVRCP) != 0;
    state.cap_dac_audio = (mask & SECONDARY_CAP_DAC_AUDIO) != 0;
}

/*
 * Decode protocol-v4 Bluetooth/audio fields. The raw tokens are retained for the UI,
 * while convenience booleans make future behavior checks less string-heavy.
 * Call only while state_mutex is held.
 */
static void apply_bt_runtime_locked(
    const char *bt,
    const char *a2dp,
    const char *stream,
    const char *peer,
    bool have_pcm,
    uint64_t pcm_bytes,
    bool have_bt_errors,
    uint32_t bt_errors,
    bool have_rate,
    uint32_t sample_rate_hz,
    bool have_channels,
    uint32_t channels,
    bool have_output,
    uint64_t output_bytes,
    bool have_drop,
    uint64_t drop_bytes,
    bool have_audio_errors,
    uint32_t audio_errors)
{
    if (bt != NULL && bt[0] != '\0')
    {
        snprintf(state.bt_state, sizeof(state.bt_state), "%s", bt);
        state.bt_ready = strcmp(bt, "READY") == 0;
    }

    if (a2dp != NULL && a2dp[0] != '\0')
    {
        snprintf(state.a2dp_state, sizeof(state.a2dp_state), "%s", a2dp);
        state.a2dp_initialized =
            strcmp(a2dp, "READY") == 0 ||
            strcmp(a2dp, "CONNECTING") == 0 ||
            strcmp(a2dp, "CONNECTED") == 0 ||
            strcmp(a2dp, "DISCONNECTING") == 0;
        state.a2dp_connected = strcmp(a2dp, "CONNECTED") == 0;
    }

    if (stream != NULL && stream[0] != '\0')
    {
        snprintf(state.stream_state, sizeof(state.stream_state), "%s", stream);
        state.a2dp_streaming = strcmp(stream, "STARTED") == 0;
    }

    if (peer != NULL && peer[0] != '\0')
    {
        snprintf(state.peer_bda, sizeof(state.peer_bda), "%s", peer);
    }

    if (have_pcm) state.a2dp_pcm_bytes = pcm_bytes;
    if (have_bt_errors) state.bt_errors = bt_errors;
    if (have_rate) state.audio_sample_rate_hz = sample_rate_hz;
    if (have_channels && channels <= UINT8_MAX) state.audio_channels = (uint8_t)channels;
    if (have_output) state.audio_output_bytes = output_bytes;
    if (have_drop) state.audio_drop_bytes = drop_bytes;
    if (have_audio_errors) state.audio_errors = audio_errors;
}

static void parse_bt_runtime_fields(
    const char *line,
    char *bt, size_t bt_size,
    char *a2dp, size_t a2dp_size,
    char *stream, size_t stream_size,
    char *peer, size_t peer_size,
    bool *have_pcm, uint64_t *pcm_bytes,
    bool *have_bt_errors, uint32_t *bt_errors,
    bool *have_rate, uint32_t *sample_rate_hz,
    bool *have_channels, uint32_t *channels,
    bool *have_output, uint64_t *output_bytes,
    bool *have_drop, uint64_t *drop_bytes,
    bool *have_audio_errors, uint32_t *audio_errors)
{
    if (bt != NULL) bt[0] = '\0';
    if (a2dp != NULL) a2dp[0] = '\0';
    if (stream != NULL) stream[0] = '\0';
    if (peer != NULL) peer[0] = '\0';

    if (bt != NULL) (void)parse_named_token(line, "BT=", bt, bt_size);
    if (a2dp != NULL) (void)parse_named_token(line, "A2DP=", a2dp, a2dp_size);
    if (stream != NULL) (void)parse_named_token(line, "STREAM=", stream, stream_size);
    if (peer != NULL) (void)parse_named_token(line, "PEER=", peer, peer_size);

    if (have_pcm != NULL && pcm_bytes != NULL)
        *have_pcm = parse_named_u64(line, "PCM=", pcm_bytes);
    if (have_bt_errors != NULL && bt_errors != NULL)
        *have_bt_errors = parse_named_u32(line, "BTERR=", bt_errors);
    if (have_rate != NULL && sample_rate_hz != NULL)
        *have_rate = parse_named_u32(line, "RATE=", sample_rate_hz);
    if (have_channels != NULL && channels != NULL)
        *have_channels = parse_named_u32(line, "CH=", channels);
    if (have_output != NULL && output_bytes != NULL)
        *have_output = parse_named_u64(line, "OUT=", output_bytes);
    if (have_drop != NULL && drop_bytes != NULL)
        *have_drop = parse_named_u64(line, "DROP=", drop_bytes);
    if (have_audio_errors != NULL && audio_errors != NULL)
        *have_audio_errors = parse_named_u32(line, "AERR=", audio_errors);
}

static void process_line(const char *line)
{
    if (line == NULL || line[0] == '\0')
    {
        return;
    }

    remember_line(line);

    if (strncmp(line, "HELLO ", 6) == 0)
    {
        uint32_t xsmt = 0;
        uint32_t fw = 0;
        uint32_t proto = 0;
        uint32_t caps = 0;
        char mode[SECONDARY_LINK_MODE_MAX] = {0};
        char bt[SECONDARY_LINK_BT_STATE_MAX] = {0};
        char a2dp[SECONDARY_LINK_A2DP_STATE_MAX] = {0};
        char stream[SECONDARY_LINK_STREAM_STATE_MAX] = {0};
        char peer[SECONDARY_LINK_PEER_MAX] = {0};
        uint64_t pcm = 0;
        uint32_t bt_errors = 0;
        uint32_t sample_rate_hz = 0;
        uint32_t channels = 0;
        uint64_t output_bytes = 0;
        uint64_t drop_bytes = 0;
        uint32_t audio_errors = 0;
        bool have_pcm = false;
        bool have_bt_errors = false;
        bool have_rate = false;
        bool have_channels = false;
        bool have_output = false;
        bool have_drop = false;
        bool have_audio_errors = false;

        bool have_xsmt = parse_named_u32(line, "XSMT=", &xsmt);
        bool have_fw = parse_named_u32(line, "FW=", &fw);
        bool have_proto = parse_named_u32(line, "PROTO=", &proto);
        bool have_caps = parse_named_u32(line, "CAPS=", &caps);
        bool have_mode = parse_named_token(line, "MODE=", mode, sizeof(mode));
        parse_bt_runtime_fields(
            line,
            bt, sizeof(bt),
            a2dp, sizeof(a2dp),
            stream, sizeof(stream),
            peer, sizeof(peer),
            &have_pcm, &pcm,
            &have_bt_errors, &bt_errors,
            &have_rate, &sample_rate_hz,
            &have_channels, &channels,
            &have_output, &output_bytes,
            &have_drop, &drop_bytes,
            &have_audio_errors, &audio_errors);

        mark_seen();

        if (lock_state(portMAX_DELAY))
        {
            if (have_xsmt) state.xsmt_level = (int)xsmt;
            if (have_fw) state.companion_fw = fw;
            if (have_proto) state.protocol_version = proto;
            if (have_caps) apply_capabilities_locked(caps);
            if (have_mode) snprintf(state.companion_mode, sizeof(state.companion_mode), "%s", mode);
            apply_bt_runtime_locked(
                bt, a2dp, stream, peer,
                have_pcm, pcm,
                have_bt_errors, bt_errors,
                have_rate, sample_rate_hz,
                have_channels, channels,
                have_output, output_bytes,
                have_drop, drop_bytes,
                have_audio_errors, audio_errors);
            unlock_state();
        }

        ESP_LOGI(TAG, "Companion HELLO: %s", line);
        return;
    }

    if (strncmp(line, "PONG ", 5) == 0)
    {
        char *end = NULL;
        unsigned long seq_ul = strtoul(line + 5, &end, 10);
        if (end == line + 5)
        {
            parse_error(line);
            return;
        }

        uint32_t sequence = (uint32_t)seq_ul;
        uint32_t xsmt = 0;
        uint32_t uptime = 0;
        uint32_t fw = 0;
        uint32_t proto = 0;
        char mode[SECONDARY_LINK_MODE_MAX] = {0};
        char bt[SECONDARY_LINK_BT_STATE_MAX] = {0};
        char a2dp[SECONDARY_LINK_A2DP_STATE_MAX] = {0};
        char stream[SECONDARY_LINK_STREAM_STATE_MAX] = {0};
        char peer[SECONDARY_LINK_PEER_MAX] = {0};
        uint64_t pcm = 0;
        uint32_t bt_errors = 0;
        uint32_t sample_rate_hz = 0;
        uint32_t channels = 0;
        uint64_t output_bytes = 0;
        uint64_t drop_bytes = 0;
        uint32_t audio_errors = 0;
        bool have_pcm = false;
        bool have_bt_errors = false;
        bool have_rate = false;
        bool have_channels = false;
        bool have_output = false;
        bool have_drop = false;
        bool have_audio_errors = false;
        bool have_xsmt = parse_named_u32(line, "XSMT=", &xsmt);
        bool have_uptime = parse_named_u32(line, "UPTIME=", &uptime);
        bool have_fw = parse_named_u32(line, "FW=", &fw);
        bool have_proto = parse_named_u32(line, "PROTO=", &proto);
        bool have_mode = parse_named_token(line, "MODE=", mode, sizeof(mode));
        parse_bt_runtime_fields(
            line,
            bt, sizeof(bt),
            a2dp, sizeof(a2dp),
            stream, sizeof(stream),
            peer, sizeof(peer),
            &have_pcm, &pcm,
            &have_bt_errors, &bt_errors,
            &have_rate, &sample_rate_hz,
            &have_channels, &channels,
            &have_output, &output_bytes,
            &have_drop, &drop_bytes,
            &have_audio_errors, &audio_errors);

        mark_seen();

        uint32_t rtt_ms = 0;
        if (sequence == pending_ping_sequence && pending_ping_sent_us != 0)
        {
            int64_t elapsed_us = esp_timer_get_time() - pending_ping_sent_us;
            if (elapsed_us > 0)
            {
                rtt_ms = (uint32_t)(elapsed_us / 1000);
            }
        }

        if (lock_state(portMAX_DELAY))
        {
            state.pongs_received++;
            state.last_pong_sequence = sequence;
            state.last_rtt_ms = rtt_ms;
            if (have_xsmt) state.xsmt_level = (int)xsmt;
            if (have_uptime) state.companion_uptime_ms = uptime;
            if (have_fw) state.companion_fw = fw;
            if (have_proto) state.protocol_version = proto;
            if (have_mode) snprintf(state.companion_mode, sizeof(state.companion_mode), "%s", mode);
            apply_bt_runtime_locked(
                bt, a2dp, stream, peer,
                have_pcm, pcm,
                have_bt_errors, bt_errors,
                have_rate, sample_rate_hz,
                have_channels, channels,
                have_output, output_bytes,
                have_drop, drop_bytes,
                have_audio_errors, audio_errors);
            unlock_state();
        }
        return;
    }

    /* v22 canonical STATE; legacy companion v4 STATUS is still accepted. */
    if (strncmp(line, "STATE ", 6) == 0 || strncmp(line, "STATUS ", 7) == 0)
    {
        uint32_t xsmt = 0;
        uint32_t uptime = 0;
        uint32_t fw = 0;
        uint32_t proto = 0;
        char mode[SECONDARY_LINK_MODE_MAX] = {0};
        char bt[SECONDARY_LINK_BT_STATE_MAX] = {0};
        char a2dp[SECONDARY_LINK_A2DP_STATE_MAX] = {0};
        char stream[SECONDARY_LINK_STREAM_STATE_MAX] = {0};
        char peer[SECONDARY_LINK_PEER_MAX] = {0};
        uint64_t pcm = 0;
        uint32_t bt_errors = 0;
        uint32_t sample_rate_hz = 0;
        uint32_t channels = 0;
        uint64_t output_bytes = 0;
        uint64_t drop_bytes = 0;
        uint32_t audio_errors = 0;
        bool have_pcm = false;
        bool have_bt_errors = false;
        bool have_rate = false;
        bool have_channels = false;
        bool have_output = false;
        bool have_drop = false;
        bool have_audio_errors = false;

        bool have_xsmt = parse_named_u32(line, "XSMT=", &xsmt);
        bool have_uptime = parse_named_u32(line, "UPTIME=", &uptime);
        bool have_fw = parse_named_u32(line, "FW=", &fw);
        bool have_proto = parse_named_u32(line, "PROTO=", &proto);
        bool have_mode = parse_named_token(line, "MODE=", mode, sizeof(mode));
        parse_bt_runtime_fields(
            line,
            bt, sizeof(bt),
            a2dp, sizeof(a2dp),
            stream, sizeof(stream),
            peer, sizeof(peer),
            &have_pcm, &pcm,
            &have_bt_errors, &bt_errors,
            &have_rate, &sample_rate_hz,
            &have_channels, &channels,
            &have_output, &output_bytes,
            &have_drop, &drop_bytes,
            &have_audio_errors, &audio_errors);

        mark_seen();

        if (lock_state(portMAX_DELAY))
        {
            state.status_received++;
            if (have_xsmt) state.xsmt_level = (int)xsmt;
            if (have_uptime) state.companion_uptime_ms = uptime;
            if (have_fw) state.companion_fw = fw;
            if (have_proto) state.protocol_version = proto;
            if (have_mode) snprintf(state.companion_mode, sizeof(state.companion_mode), "%s", mode);
            apply_bt_runtime_locked(
                bt, a2dp, stream, peer,
                have_pcm, pcm,
                have_bt_errors, bt_errors,
                have_rate, sample_rate_hz,
                have_channels, channels,
                have_output, output_bytes,
                have_drop, drop_bytes,
                have_audio_errors, audio_errors);
            unlock_state();
        }
        return;
    }

    if (strncmp(line, "CAPS ", 5) == 0)
    {
        uint32_t fw = 0;
        uint32_t proto = 0;
        uint32_t mask = 0;
        bool have_fw = parse_named_u32(line, "FW=", &fw);
        bool have_proto = parse_named_u32(line, "PROTO=", &proto);
        bool have_mask = parse_named_u32(line, "MASK=", &mask);

        if (!have_mask)
        {
            parse_error(line);
            return;
        }

        mark_seen();

        if (lock_state(portMAX_DELAY))
        {
            state.caps_received++;
            apply_capabilities_locked(mask);
            if (have_fw) state.companion_fw = fw;
            if (have_proto) state.protocol_version = proto;
            unlock_state();
        }

        ESP_LOGI(TAG, "Companion CAPS: proto=%lu mask=0x%08lX",
                 (unsigned long)proto, (unsigned long)mask);
        return;
    }

    /*
     * v21/v4 state-changing acknowledgement. SET is what the S3 asked for;
     * READ is the companion's immediate GPIO32 pad readback after applying it.
     * Keeping both values makes an electrical/configuration mismatch visible
     * instead of assuming gpio_set_level() necessarily reached the pad.
     */
    if (strncmp(line, "ACK XSMT ", 9) == 0)
    {
        uint32_t requested = 0;
        uint32_t readback = 0;
        uint32_t fw = 0;
        uint32_t proto = 0;
        bool have_requested = parse_named_u32(line, "SET=", &requested);
        bool have_readback = parse_named_u32(line, "READ=", &readback);
        bool have_fw = parse_named_u32(line, "FW=", &fw);
        bool have_proto = parse_named_u32(line, "PROTO=", &proto);

        if (!have_requested || !have_readback || requested > 1U || readback > 1U)
        {
            parse_error(line);
            return;
        }

        mark_seen();

        if (lock_state(portMAX_DELAY))
        {
            state.xsmt_acks_received++;
            state.xsmt_requested = (int)requested;
            state.xsmt_level = (int)readback;
            if (have_fw) state.companion_fw = fw;
            if (have_proto) state.protocol_version = proto;
            unlock_state();
        }

        ESP_LOGI(
            TAG,
            "Companion XSMT ACK: requested=%lu readback=%lu",
            (unsigned long)requested,
            (unsigned long)readback);
        return;
    }

    if (strncmp(line, "ERR ", 4) == 0)
    {
        mark_seen();
        parse_error(line);
        return;
    }

    parse_error(line);
}

/*
 * Assemble a stream of arbitrary UART chunks into newline-delimited frames.
 * Oversized lines are discarded safely until the next newline.
 */
static void link_task(void *arg)
{
    (void)arg;

    uint8_t rx[96];
    char line[LINK_WIRE_LINE_MAX];
    size_t line_len = 0;

    /* First automatic PING shortly after boot, then once per second. */
    int64_t next_heartbeat_us = esp_timer_get_time() + 500000;

    while (true)
    {
        int n = uart_read_bytes(
            LINK_UART,
            rx,
            sizeof(rx),
            pdMS_TO_TICKS(LINK_RX_POLL_MS));

        if (n < 0)
        {
            ESP_LOGW(TAG, "uart_read_bytes returned %d", n);
        }
        else
        {
            for (int i = 0; i < n; ++i)
            {
                char c = (char)rx[i];

                if (c == '\r')
                {
                    continue;
                }

                if (c == '\n')
                {
                    line[line_len] = '\0';
                    if (line_len > 0)
                    {
                        process_line(line);
                    }
                    line_len = 0;
                    continue;
                }

                if (line_len < sizeof(line) - 1)
                {
                    line[line_len++] = c;
                }
                else
                {
                    line_len = 0;
                    parse_error("<RX line overflow>");
                }
            }
        }

        int64_t now_us = esp_timer_get_time();
        if (now_us >= next_heartbeat_us)
        {
            esp_err_t err = send_ping_internal();
            if (err != ESP_OK)
            {
                ESP_LOGW(TAG, "Periodic PING failed: %s", esp_err_to_name(err));
            }

            next_heartbeat_us = now_us + ((int64_t)LINK_HEARTBEAT_MS * 1000);
        }
    }
}

esp_err_t secondary_link_init(void)
{
    if (state.initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&state, 0, sizeof(state));
    state.xsmt_level = -1;
    state.xsmt_requested = -1;
    snprintf(state.bt_state, sizeof(state.bt_state), "?");
    snprintf(state.a2dp_state, sizeof(state.a2dp_state), "?");
    snprintf(state.stream_state, sizeof(state.stream_state), "?");
    snprintf(state.peer_bda, sizeof(state.peer_bda), "-");
    state.last_rx_age_ms = UINT32_MAX;

    state_mutex = xSemaphoreCreateMutex();
    tx_mutex = xSemaphoreCreateMutex();
    if (state_mutex == NULL || tx_mutex == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    const uart_config_t config = {
        .baud_rate = SECONDARY_LINK_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_param_config(LINK_UART, &config);
    if (err != ESP_OK) return err;

    /* uart_set_pin arguments are TX first, then RX. */
    err = uart_set_pin(
        LINK_UART,
        SECONDARY_LINK_S3_TX_GPIO,
        SECONDARY_LINK_S3_RX_GPIO,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;

    err = uart_driver_install(
        LINK_UART,
        LINK_RX_BUFFER,
        LINK_TX_BUFFER,
        0,
        NULL,
        0);
    if (err != ESP_OK) return err;

    if (lock_state(portMAX_DELAY))
    {
        state.initialized = true;
        unlock_state();
    }

    if (xTaskCreate(
            link_task,
            "secondary_link",
            LINK_TASK_STACK,
            NULL,
            LINK_TASK_PRIORITY,
            NULL) != pdPASS)
    {
        if (lock_state(portMAX_DELAY))
        {
            state.initialized = false;
            unlock_state();
        }
        uart_driver_delete(LINK_UART);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(
        TAG,
        "UART2: S3 TX GPIO%d -> U4 RX18, S3 RX GPIO%d <- U4 TX23, %d baud",
        SECONDARY_LINK_S3_TX_GPIO,
        SECONDARY_LINK_S3_RX_GPIO,
        SECONDARY_LINK_UART_BAUD);

    return ESP_OK;
}

esp_err_t secondary_link_send_ping(void)
{
    return state.initialized ? send_ping_internal() : ESP_ERR_INVALID_STATE;
}

esp_err_t secondary_link_request_status(void)
{
    return state.initialized ? write_line("GET STATE\n") : ESP_ERR_INVALID_STATE;
}

esp_err_t secondary_link_request_capabilities(void)
{
    return state.initialized ? write_line("GET CAPS\n") : ESP_ERR_INVALID_STATE;
}

esp_err_t secondary_link_set_xsmt(bool enabled)
{
    if (!state.initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Keep the wire representation deliberately boring: one ASCII digit. This
     * makes the first state-changing command just as easy to inspect as PING.
     */
    char frame[16];
    snprintf(frame, sizeof(frame), "SET XSMT %d\n", enabled ? 1 : 0);

    esp_err_t err = write_line(frame);
    if (err != ESP_OK)
    {
        return err;
    }

    /*
     * Record what we requested immediately. xsmt_level is NOT changed here; it
     * remains the last physical readback reported by the companion. That
     * separation lets the UI expose request/readback mismatches honestly.
     */
    if (lock_state(portMAX_DELAY))
    {
        state.xsmt_requested = enabled ? 1 : 0;
        state.xsmt_commands_sent++;
        unlock_state();
    }

    return ESP_OK;
}

void secondary_link_get_status(secondary_link_status_t *out)
{
    if (out == NULL)
    {
        return;
    }

    memset(out, 0, sizeof(*out));
    out->xsmt_level = -1;
    out->xsmt_requested = -1;
    out->last_rx_age_ms = UINT32_MAX;

    if (!lock_state(pdMS_TO_TICKS(50)))
    {
        return;
    }

    *out = state;

    if (state.companion_seen && last_recognized_rx_us != 0)
    {
        int64_t elapsed_us = esp_timer_get_time() - last_recognized_rx_us;
        if (elapsed_us <= 0)
        {
            out->last_rx_age_ms = 0;
        }
        else
        {
            uint64_t age_ms = (uint64_t)elapsed_us / 1000ULL;
            out->last_rx_age_ms = age_ms > UINT32_MAX ? UINT32_MAX : (uint32_t)age_ms;
        }
    }

    unlock_state();
}
