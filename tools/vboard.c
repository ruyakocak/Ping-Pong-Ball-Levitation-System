/*
 * Virtual control board (R3.13): the SAME m3_core + contract + net code as the
 * ESP32 firmware, wired like src/main.cpp, but the serial links, the UDP socket and
 * the clock come from the HIL simulator over stdin/stdout. This lets
 * sim/hil_sim.py run on Windows and Linux (and in CI) without a board. On the rig
 * the real board replaces this program and the simulator only changes its link.
 *
 *   gcc -O2 -std=gnu11 -Ilib/contract -Ilib/m3_core tools/vboard.c \
 *       lib/contract/m3_contract.c lib/m3_core/m3_[a-z_]*.c -lm -o vboard
 *
 * Lock-step link (little-endian). Simulator -> board, one record each:
 *   u8 kind, u32 now_ms, u16 len, len bytes
 *     kind 0: bytes arriving on the USB serial (height frames from M2)
 *     kind 1: bytes arriving on Serial2       (fan_state frames from M1)
 *     kind 2: one UDP datagram arriving on port 5005 (command / target_points JSON)
 *     kind 3: end of this millisecond, len 0 -> board runs its tick and answers
 * Board -> simulator, after every kind 3:
 *   u16 len, bytes written to Serial2 (fan_cmd frames)
 *   u8 fsm state, u8 stuck
 *   u16 len, UDP datagrams sent (acks, 30 Hz status), each followed by '\n'
 *
 * Argument: hold target in mm (default 500, as main.cpp).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

#include "m3_contract.h"
#include "m3_ctrl.h"
#include "m3_net.h"

#define OUT_CAP 4096
#define UDP_CAP 8192
#define STATUS_PERIOD_MS 33u

static m3_ctrl_t g_ctrl;
static m3_net_t g_net;
static uint8_t g_out[OUT_CAP];
static size_t g_out_len;
static char g_udp[UDP_CAP];
static size_t g_udp_len;
static uint32_t g_heights_used, g_net_bad;

static int read_exact(void *buf, size_t n) { return fread(buf, 1, n, stdin) == n; }

static void send_fan_cmd(const ct_fan_cmd_t *cmd, uint32_t seq)
{
    uint8_t payload[CT_FAN_CMD_LEN];
    uint8_t n = ct_pack_fan_cmd(payload, cmd);
    if (g_out_len + CT_MAX_FRAME > OUT_CAP) return; /* a real UART would drop too */
    g_out_len += ct_encode_frame(g_out + g_out_len, CT_FAN_CMD, g_ctrl.session, seq, payload, n);
}

static void send_udp(const char *s, size_t n)
{
    if (!n || g_udp_len + n + 1 > UDP_CAP) return;
    memcpy(g_udp + g_udp_len, s, n);
    g_udp_len += n;
    g_udp[g_udp_len++] = '\n';
}

/* Same per-port filtering as rx_task in main.cpp: a port only accepts its own type. */
static void rx_bytes(ct_decoder_t *dec, uint8_t want, const uint8_t *p, size_t n, uint32_t now)
{
    ct_frame_t f;
    ct_fan_cmd_t cmd;
    uint32_t seq;
    for (size_t i = 0; i < n; i++) {
        if (!ct_decoder_feed(dec, p[i], &f) || f.type != want) continue;
        if (want == CT_HEIGHT) {
            ct_height_t h;
            if (ct_unpack_height(f.payload, f.len, &h) && m3_ctrl_on_height(&g_ctrl, f.session, &h, now, &cmd, &seq)) {
                send_fan_cmd(&cmd, seq);
                g_heights_used++;
            }
        } else {
            ct_fan_state_t s;
            if (ct_unpack_fan_state(f.payload, f.len, &s)) m3_ctrl_on_fan_state(&g_ctrl, f.session, f.seq, &s, now);
        }
    }
}

/* net_task + control_task of main.cpp in one: parse, apply, ack. */
static void rx_udp(const char *p, size_t n, uint32_t now)
{
    static m3_net_msg_t msg;
    static char buf[M3_NET_MAX_DATAGRAM];
    m3_netmsg_kind_t k = m3_net_parse(p, n, &msg);
    if (k == NETMSG_BAD) g_net_bad++;
    if (k != NETMSG_COMMAND && k != NETMSG_TARGETS) return;
    m3_net_ack_t ack;
    if (m3_net_apply(&g_net, &g_ctrl, &msg, now, &ack))
        send_udp(buf, m3_net_format_ack(buf, sizeof buf, g_ctrl.session, now, &ack));
}

int main(int argc, char **argv)
{
#ifdef _WIN32
    _setmode(_fileno(stdin), _O_BINARY);
    _setmode(_fileno(stdout), _O_BINARY);
#endif
    float target = argc > 1 ? (float)atof(argv[1]) : 500.0f;
    m3_ctrl_cfg_t cfg;
    m3_ctrl_default_cfg(&cfg);
    m3_ctrl_init(&g_ctrl, &cfg, 0x5EB0A4D1u, 0); /* starts in Fault, like power-up */
    m3_ctrl_set_hold_target(&g_ctrl, target);
    m3_net_init(&g_net);

    ct_decoder_t usb, m1;
    ct_decoder_init(&usb);
    ct_decoder_init(&m1);
    static uint8_t data[65535];
    static char sbuf[M3_NET_MAX_DATAGRAM];
    uint32_t next_status = 0;

    for (;;) {
        uint8_t hdr[7];
        if (!read_exact(hdr, sizeof hdr)) return 0; /* simulator closed the link */
        uint8_t kind = hdr[0];
        uint32_t now = (uint32_t)hdr[1] | (uint32_t)hdr[2] << 8 | (uint32_t)hdr[3] << 16 | (uint32_t)hdr[4] << 24;
        uint16_t len = (uint16_t)(hdr[5] | hdr[6] << 8);
        if (len && !read_exact(data, len)) return 1;

        switch (kind) {
        case 0: rx_bytes(&usb, CT_HEIGHT, data, len, now); break;
        case 1: rx_bytes(&m1, CT_FAN_STATE, data, len, now); break;
        case 2: rx_udp((const char *)data, len, now); break;
        case 3: {
            ct_fan_cmd_t cmd;
            uint32_t seq;
            if (m3_ctrl_on_tick(&g_ctrl, now, &cmd, &seq)) send_fan_cmd(&cmd, seq);
            if ((int32_t)(now - next_status) >= 0) {
                next_status = now + STATUS_PERIOD_MS;
                m3_net_status_t st;
                m3_net_status_fill(&g_net, &g_ctrl, &st);
                st.heights_used = g_heights_used;
                st.net_bad = g_net_bad; /* latency is a property of the real board: 0 here */
                send_udp(sbuf, m3_net_format_status(sbuf, sizeof sbuf, g_ctrl.session, now, &st));
            }
            uint8_t lenb[2] = {(uint8_t)g_out_len, (uint8_t)(g_out_len >> 8)};
            uint8_t tail[2] = {(uint8_t)g_ctrl.fsm.state, (uint8_t)(g_ctrl.stuck.warning ? 1 : 0)};
            uint8_t udpb[2] = {(uint8_t)g_udp_len, (uint8_t)(g_udp_len >> 8)};
            fwrite(lenb, 1, 2, stdout);
            fwrite(g_out, 1, g_out_len, stdout);
            fwrite(tail, 1, 2, stdout);
            fwrite(udpb, 1, 2, stdout);
            fwrite(g_udp, 1, g_udp_len, stdout);
            fflush(stdout);
            g_out_len = 0;
            g_udp_len = 0;
            break;
        }
        default: return 2;
        }
    }
}
