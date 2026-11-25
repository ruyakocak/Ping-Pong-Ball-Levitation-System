/*
 * M3 control board firmware — ESP32 glue only. All decisions live in lib/m3_core.
 *
 * Links (Section 3.4):
 *   USB serial (UART0)  : height from M2 (debug output goes over UDP, never here)
 *   Serial2 (UART2)     : blower controller M1   RX=GPIO16 TX=GPIO17
 *   Serial1 (UART1)     : game board             RX=GPIO26 TX=GPIO27 (remapped; default pins = flash)
 *
 * Tasks:
 *   control_task  core 1, highest priority. Single owner of m3_ctrl_t. Waits on one
 *                 event queue with the timeout m3_ctrl_next_wait_ms() (<= 10 ms).
 *   rx tasks      core 0. Decode frames and post events to the queue.
 *   net task      core 0. Wi-Fi + UDP port 5005: parses command / target_points JSON
 *                 (m3_net_parse) and hands them to the control task; formats and sends
 *                 the acks and the 30 Hz status. No JSON work ever runs on core 1 (R3.3).
 *
 * R3.3 instrumentation: every height carries its receive time in us; the control task
 * measures receive -> fan_cmd written and reports the worst case and the count above
 * 5 ms in status.diag (lat_max_us, lat_over_5ms), plus heights lost on a full queue.
 *
 * Build env esp32_simball (M3_SIM_BALL=1) replaces the USB heights and fan_state with
 * the built-in simulated ball; everything else is identical.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_random.h>
#include <esp_timer.h>

extern "C" {
#include "m3_contract.h"
#include "m3_ctrl.h"
#include "m3_net.h"
#include "m3_plant.h"
}
#include "m3_config.h"

namespace {

constexpr int M1_RX = 16, M1_TX = 17;
constexpr int GAME_RX = 26, GAME_TX = 27;
constexpr uint32_t BAUD = 115200;

enum EvKind : uint8_t { EV_HEIGHT, EV_FAN_STATE, EV_NET };

struct Event {
    EvKind kind;
    uint32_t session;
    uint32_t seq;
    uint32_t rx_ms;
    int64_t rx_us;              // EV_HEIGHT: arrival, for the R3.3 latency
    union {
        ct_height_t height;
        ct_fan_state_t fan_state;
    };
};

struct NetIn {                  // net task -> control task
    m3_net_msg_t msg;
    uint32_t ip;
    uint16_t port;
};
struct NetAck {                 // control task -> net task
    m3_net_ack_t ack;
    uint32_t ip;
    uint16_t port;
};

QueueHandle_t g_events;         // small, frequent: heights, fan_state, "net message waiting"
QueueHandle_t g_net_in;         // parsed command / target_points
QueueHandle_t g_acks;
QueueHandle_t g_status;         // length 1, latest snapshot only
m3_ctrl_t g_ctrl;               // owned by control_task only
m3_net_t g_net;                 // owned by control_task only
volatile uint32_t g_heights_lost;
volatile uint32_t g_net_bad;
volatile uint16_t g_last_rpm;   // for the simulated M1 (simball build)
volatile uint8_t g_last_flags;

uint32_t now_ms() { return (uint32_t)(esp_timer_get_time() / 1000); }

void send_fan_cmd(const ct_fan_cmd_t &cmd, uint32_t seq)
{
    uint8_t payload[CT_FAN_CMD_LEN];
    uint8_t frame[CT_MAX_FRAME];
    uint8_t n = ct_pack_fan_cmd(payload, &cmd);
    size_t len = ct_encode_frame(frame, CT_FAN_CMD, g_ctrl.session, seq, payload, n);
    Serial2.write(frame, len);
    g_last_rpm = cmd.bottom_rpm;
    g_last_flags = cmd.flags;
}

void control_task(void *)
{
    Event ev;
    static NetIn in;
    static NetAck out;
    ct_fan_cmd_t cmd;
    uint32_t seq;
    uint32_t heights_used = 0, lat_max_us = 0, lat_over_5ms = 0;
    uint32_t next_status = now_ms();
    for (;;) {
        uint32_t wait = m3_ctrl_next_wait_ms(&g_ctrl, now_ms());
        if (xQueueReceive(g_events, &ev, pdMS_TO_TICKS(wait)) == pdTRUE) {
            switch (ev.kind) {
            case EV_HEIGHT:
                if (m3_ctrl_on_height(&g_ctrl, ev.session, &ev.height, ev.rx_ms, &cmd, &seq)) {
                    send_fan_cmd(cmd, seq);
                    uint32_t lat = (uint32_t)(esp_timer_get_time() - ev.rx_us);
                    heights_used++;
                    if (lat > lat_max_us) lat_max_us = lat;
                    if (lat > 5000) lat_over_5ms++;
                }
                break;
            case EV_FAN_STATE:
                m3_ctrl_on_fan_state(&g_ctrl, ev.session, ev.seq, &ev.fan_state, ev.rx_ms);
                break;
            case EV_NET: // only wakes us; the queue is read below on every pass
                break;
            }
        }
        // One network message per pass, after any fan_cmd of this pass has gone out,
        // so a burst of messages never delays a height (R3.3).
        if (xQueueReceive(g_net_in, &in, 0) == pdTRUE &&
            m3_net_apply(&g_net, &g_ctrl, &in.msg, now_ms(), &out.ack)) {
            out.ip = in.ip;
            out.port = in.port;
            xQueueSend(g_acks, &out, 0);
        }
        uint32_t now = now_ms();
        if (m3_ctrl_on_tick(&g_ctrl, now, &cmd, &seq)) send_fan_cmd(cmd, seq);
        if ((int32_t)(now - next_status) >= 0) {
            next_status = now + M3_STATUS_PERIOD_MS;
            m3_net_status_t st;
            m3_net_status_fill(&g_net, &g_ctrl, &st);
            st.heights_used = heights_used;
            st.lat_max_us = lat_max_us;
            st.lat_over_5ms = lat_over_5ms;
            st.heights_lost = g_heights_lost;
            xQueueOverwrite(g_status, &st);
        }
    }
}

/* Wi-Fi + UDP on core 0. Never touches g_ctrl except its constant session id. */
void net_task(void *)
{
    static char buf[M3_NET_MAX_DATAGRAM + 1];
    static NetIn in;
    static NetAck ack;
    WiFiUDP udp;
    IPAddress gw, m4;
    gw.fromString(M3_GATEWAY_IP);
    m4.fromString(M3_M4_IP);

    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(M3_WIFI_SSID, M3_WIFI_PASS);
    bool bound = false;
    for (;;) {
        if (WiFi.status() != WL_CONNECTED) { // the rig keeps flying without the network
            if (bound) udp.stop();
            bound = false;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if (!bound) bound = udp.begin(M3_UDP_PORT);

        for (int k = 0; k < 8; k++) { // bounded per loop so status is never starved
            int n = udp.parsePacket();
            if (n <= 0) break;
            int len = udp.read(buf, sizeof buf - 1);
            if (n > (int)M3_NET_MAX_DATAGRAM || len <= 0) {
                g_net_bad++;
                continue;
            }
            m3_netmsg_kind_t kind = m3_net_parse(buf, (size_t)len, &in.msg);
            if (kind == NETMSG_BAD) {
                g_net_bad++;
                continue;
            }
            if (kind == NETMSG_OTHER) continue;
            in.ip = (uint32_t)udp.remoteIP();
            in.port = udp.remotePort();
            if (xQueueSend(g_net_in, &in, 0) == pdTRUE) {
                Event ev = {};
                ev.kind = EV_NET;
                xQueueSend(g_events, &ev, 0);
            }
        }

        while (xQueueReceive(g_acks, &ack, 0) == pdTRUE) {
            size_t n = m3_net_format_ack(buf, sizeof buf, g_ctrl.session, now_ms(), &ack.ack);
            if (!n) continue;
            udp.beginPacket(IPAddress(ack.ip), ack.port);
            udp.write((const uint8_t *)buf, n);
            udp.endPacket();
        }

        m3_net_status_t st;
        if (xQueueReceive(g_status, &st, 0) == pdTRUE) {
            st.net_bad = g_net_bad;
            size_t n = m3_net_format_status(buf, sizeof buf, g_ctrl.session, now_ms(), &st);
            if (n) {
                udp.beginPacket(gw, M3_GATEWAY_PORT);
                udp.write((const uint8_t *)buf, n);
                udp.endPacket();
                udp.beginPacket(m4, M3_M4_PORT);
                udp.write((const uint8_t *)buf, n);
                udp.endPacket();
            }
        }
        vTaskDelay(1);
    }
}

#if !M3_SIM_BALL
/* Reads one serial port, posts the frames of one type. */
struct RxArgs {
    HardwareSerial *port;
    uint8_t type;
};

void rx_task(void *arg)
{
    RxArgs *a = static_cast<RxArgs *>(arg);
    ct_decoder_t dec;
    ct_decoder_init(&dec);
    ct_frame_t f;
    Event ev;
    for (;;) {
        while (a->port->available()) {
            if (!ct_decoder_feed(&dec, (uint8_t)a->port->read(), &f) || f.type != a->type) continue;
            ev.session = f.session;
            ev.seq = f.seq;
            ev.rx_us = esp_timer_get_time();
            ev.rx_ms = (uint32_t)(ev.rx_us / 1000); // timeouts are measured on arrival, with our own clock
            bool ok = false;
            if (f.type == CT_HEIGHT) { ev.kind = EV_HEIGHT; ok = ct_unpack_height(f.payload, f.len, &ev.height); }
            if (f.type == CT_FAN_STATE) { ev.kind = EV_FAN_STATE; ok = ct_unpack_fan_state(f.payload, f.len, &ev.fan_state); }
            if (ok && xQueueSend(g_events, &ev, 0) != pdTRUE && f.type == CT_HEIGHT) g_heights_lost++;
        }
        vTaskDelay(1);
    }
}
RxArgs g_usb_rx{&Serial, CT_HEIGHT};
RxArgs g_m1_rx{&Serial2, CT_FAN_STATE};
#else
/* Built-in simulated ball + a minimal simulated M1 (armed on re-arm, run on enable). */
void sim_task(void *)
{
    m3_plant_params_t pp;
    m3_plant_default_params(&pp);
    m3_plant_t plant;
    m3_plant_init(&plant, &pp);
    const uint32_t sim_session = esp_random();
    uint32_t frame = 0, fs_seq = 0;
    uint8_t m1_state = M1_OFF;
    int64_t t0 = esp_timer_get_time();
    int64_t next_frame = t0, next_fs = t0;
    Event ev;
    for (;;) {
        vTaskDelay(1);
        int64_t now = esp_timer_get_time();
        uint8_t flags = g_last_flags;
        if (flags & CT_FCMD_REARM) m1_state = M1_ARMED;
        else if ((flags & CT_FCMD_ENABLE) && m1_state != M1_OFF) m1_state = M1_RUN;
        else if (m1_state == M1_RUN) m1_state = M1_ARMED;
        m3_plant_step(&plant, (flags & CT_FCMD_ENABLE) ? g_last_rpm : 0, 0.001f);

        if (now >= next_frame) { // 60 fps camera
            next_frame += 16667;
            ev = {};
            ev.kind = EV_HEIGHT;
            ev.session = sim_session;
            ev.seq = frame;
            ev.rx_us = esp_timer_get_time();
            ev.rx_ms = (uint32_t)(ev.rx_us / 1000);
            ev.height.raw_mm = (int16_t)plant.h_mm;
            ev.height.filt_mm = (int16_t)plant.h_mm;
            ev.height.vel_mm_s = (int16_t)plant.v_mm_s;
            ev.height.flags = CT_HFLAG_RAW_VALID | CT_HFLAG_FILT_VALID;
            ev.height.frame_no = frame++;
            ev.height.capture_us = (uint64_t)now;
            ev.height.fps_x10 = 600;
            xQueueSend(g_events, &ev, 0);
        }
        if (now >= next_fs) { // 100 Hz fan_state
            next_fs += 10000;
            ev = {};
            ev.kind = EV_FAN_STATE;
            ev.session = sim_session;
            ev.seq = fs_seq++;
            ev.rx_us = esp_timer_get_time();
            ev.rx_ms = (uint32_t)(ev.rx_us / 1000);
            ev.fan_state.bottom_rpm = (uint16_t)plant.rpm;
            ev.fan_state.state = m1_state;
            xQueueSend(g_events, &ev, 0);
        }
    }
}
#endif

} // namespace

void setup()
{
    Serial.begin(BAUD);
    Serial2.begin(BAUD, SERIAL_8N1, M1_RX, M1_TX);
    Serial1.begin(BAUD, SERIAL_8N1, GAME_RX, GAME_TX);

    m3_ctrl_cfg_t cfg;
    m3_ctrl_default_cfg(&cfg);
    m3_ctrl_init(&g_ctrl, &cfg, esp_random(), now_ms()); // starts in Fault (power-up)
    m3_ctrl_set_hold_target(&g_ctrl, 500.0f);            // held until target_points arrive

    m3_net_init(&g_net);
    g_events = xQueueCreate(64, sizeof(Event));
    g_net_in = xQueueCreate(4, sizeof(NetIn));
    g_acks = xQueueCreate(8, sizeof(NetAck));
    g_status = xQueueCreate(1, sizeof(m3_net_status_t));

    xTaskCreatePinnedToCore(control_task, "control", 8192, nullptr, configMAX_PRIORITIES - 1, nullptr, 1);
#if !M3_SIM_BALL
    xTaskCreatePinnedToCore(rx_task, "rx_usb", 4096, &g_usb_rx, configMAX_PRIORITIES - 2, nullptr, 0);
    xTaskCreatePinnedToCore(rx_task, "rx_m1", 4096, &g_m1_rx, configMAX_PRIORITIES - 2, nullptr, 0);
#else
    xTaskCreatePinnedToCore(sim_task, "sim", 4096, nullptr, configMAX_PRIORITIES - 2, nullptr, 0);
#endif
    xTaskCreatePinnedToCore(net_task, "net", 8192, nullptr, 3, nullptr, 0);
    // TODO: game board link on Serial1 (game_in 30 Hz, game_out)
}

void loop() { vTaskDelete(nullptr); }
