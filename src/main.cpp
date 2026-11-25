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
 *   net task      core 0. TODO: Wi-Fi, UDP status (30 Hz), commands + ack, target_points.
 *
 * Build env esp32_simball (M3_SIM_BALL=1) replaces the USB heights and fan_state with
 * the built-in simulated ball; everything else is identical.
 */
#include <Arduino.h>
#include <esp_random.h>
#include <esp_timer.h>

extern "C" {
#include "m3_contract.h"
#include "m3_ctrl.h"
#include "m3_plant.h"
}

namespace {

constexpr int M1_RX = 16, M1_TX = 17;
constexpr int GAME_RX = 26, GAME_TX = 27;
constexpr uint32_t BAUD = 115200;

enum EvKind : uint8_t { EV_HEIGHT, EV_FAN_STATE, EV_COMMAND };

struct Event {
    EvKind kind;
    uint32_t session;
    uint32_t seq;
    uint32_t rx_ms;
    union {
        ct_height_t height;
        ct_fan_state_t fan_state;
        m3_event_t command;
    };
};

QueueHandle_t g_events;
m3_ctrl_t g_ctrl;               // owned by control_task only
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
    ct_fan_cmd_t cmd;
    uint32_t seq;
    for (;;) {
        uint32_t wait = m3_ctrl_next_wait_ms(&g_ctrl, now_ms());
        if (xQueueReceive(g_events, &ev, pdMS_TO_TICKS(wait)) == pdTRUE) {
            switch (ev.kind) {
            case EV_HEIGHT:
                if (m3_ctrl_on_height(&g_ctrl, ev.session, &ev.height, ev.rx_ms, &cmd, &seq))
                    send_fan_cmd(cmd, seq);
                break;
            case EV_FAN_STATE:
                m3_ctrl_on_fan_state(&g_ctrl, ev.session, ev.seq, &ev.fan_state, ev.rx_ms);
                break;
            case EV_COMMAND:
                (void)m3_ctrl_event(&g_ctrl, ev.command, now_ms()); // TODO: ack over UDP
                break;
            }
        }
        if (m3_ctrl_on_tick(&g_ctrl, now_ms(), &cmd, &seq)) send_fan_cmd(cmd, seq);
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
            ev.rx_ms = now_ms(); // timeouts are measured on arrival, with our own clock
            bool ok = false;
            if (f.type == CT_HEIGHT) { ev.kind = EV_HEIGHT; ok = ct_unpack_height(f.payload, f.len, &ev.height); }
            if (f.type == CT_FAN_STATE) { ev.kind = EV_FAN_STATE; ok = ct_unpack_fan_state(f.payload, f.len, &ev.fan_state); }
            if (ok) xQueueSend(g_events, &ev, 0);
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
            ev.rx_ms = now_ms();
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
            ev.rx_ms = now_ms();
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
    m3_ctrl_set_hold_target(&g_ctrl, 500.0f);            // TODO: from M4 target_points

    g_events = xQueueCreate(64, sizeof(Event));

    xTaskCreatePinnedToCore(control_task, "control", 8192, nullptr, configMAX_PRIORITIES - 1, nullptr, 1);
#if !M3_SIM_BALL
    xTaskCreatePinnedToCore(rx_task, "rx_usb", 4096, &g_usb_rx, configMAX_PRIORITIES - 2, nullptr, 0);
    xTaskCreatePinnedToCore(rx_task, "rx_m1", 4096, &g_m1_rx, configMAX_PRIORITIES - 2, nullptr, 0);
#else
    xTaskCreatePinnedToCore(sim_task, "sim", 4096, nullptr, configMAX_PRIORITIES - 2, nullptr, 0);
#endif
    // TODO: net task on core 0 (Wi-Fi, UDP status/command/ack/target_points, port 5005)
    // TODO: game board link on Serial1 (game_in 30 Hz, game_out)
}

void loop() { vTaskDelete(nullptr); }
