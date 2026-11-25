#include "m3_contract.h"

#include <string.h>

uint16_t ct_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

/* ---- little-endian helpers ---- */
static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v) { for (int i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (8 * i)); }
static void put64(uint8_t *p, uint64_t v) { for (int i = 0; i < 8; i++) p[i] = (uint8_t)(v >> (8 * i)); }
static uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static uint64_t get64(const uint8_t *p)
{
    uint64_t v = 0;
    for (int i = 7; i >= 0; i--) v = (v << 8) | p[i];
    return v;
}

size_t ct_encode_frame(uint8_t *out, uint8_t type, uint32_t session, uint32_t seq,
                       const uint8_t *payload, uint8_t len)
{
    if (!out || (len && !payload)) return 0;
    out[0] = CT_START0;
    out[1] = CT_START1;
    out[2] = type;
    out[3] = len;
    put32(&out[4], session);
    put32(&out[8], seq);
    if (len) memcpy(&out[12], payload, len);
    uint16_t crc = ct_crc16(&out[2], CT_HEADER_LEN + len);
    put16(&out[12 + len], crc);
    return 2u + CT_HEADER_LEN + len + 2u;
}

enum { S_START0 = 0, S_START1, S_HEAD, S_BODY };

void ct_decoder_init(ct_decoder_t *d)
{
    memset(d, 0, sizeof(*d));
    d->state = S_START0;
}

bool ct_decoder_feed(ct_decoder_t *d, uint8_t byte, ct_frame_t *out)
{
    switch (d->state) {
    case S_START0:
        if (byte == CT_START0) d->state = S_START1;
        return false;
    case S_START1:
        if (byte == CT_START1) { d->state = S_HEAD; d->pos = 0; }
        else d->state = (byte == CT_START0) ? S_START1 : S_START0;
        return false;
    case S_HEAD:
        d->buf[d->pos++] = byte;
        if (d->pos == CT_HEADER_LEN) {
            d->need = (uint16_t)(CT_HEADER_LEN + d->buf[1] + 2u);
            d->state = S_BODY;
        }
        return false;
    case S_BODY:
    default:
        d->buf[d->pos++] = byte;
        if (d->pos < d->need) return false;
        d->state = S_START0;
        {
            uint8_t len = d->buf[1];
            uint16_t crc_rx = get16(&d->buf[CT_HEADER_LEN + len]);
            if (ct_crc16(d->buf, CT_HEADER_LEN + len) != crc_rx) {
                d->crc_errors++;
                return false;
            }
            out->type = d->buf[0];
            out->len = len;
            out->session = get32(&d->buf[2]);
            out->seq = get32(&d->buf[6]);
            memcpy(out->payload, &d->buf[CT_HEADER_LEN], len);
            d->frames_ok++;
            return true;
        }
    }
}

ct_seq_result_t ct_seq_accept(ct_seq_track_t *t, uint32_t session, uint32_t seq)
{
    if (!t->have || session != t->session) {
        t->have = true;
        t->session = session;
        t->last_seq = seq;
        return CT_SEQ_NEW_SESSION;
    }
    if (seq <= t->last_seq) return CT_SEQ_OLD;
    t->last_seq = seq;
    return CT_SEQ_NEW;
}

/* ---- payloads ---- */

uint8_t ct_pack_height(uint8_t *p, const ct_height_t *m)
{
    put16(&p[0], (uint16_t)m->raw_mm);
    put16(&p[2], (uint16_t)m->filt_mm);
    put16(&p[4], (uint16_t)m->vel_mm_s);
    p[6] = m->flags;
    put32(&p[7], m->frame_no);
    put64(&p[11], m->capture_us);
    put16(&p[19], m->fps_x10);
    return CT_HEIGHT_LEN;
}

bool ct_unpack_height(const uint8_t *p, uint8_t len, ct_height_t *m)
{
    if (len != CT_HEIGHT_LEN) return false;
    m->raw_mm = (int16_t)get16(&p[0]);
    m->filt_mm = (int16_t)get16(&p[2]);
    m->vel_mm_s = (int16_t)get16(&p[4]);
    m->flags = p[6];
    m->frame_no = get32(&p[7]);
    m->capture_us = get64(&p[11]);
    m->fps_x10 = get16(&p[19]);
    return true;
}

uint8_t ct_pack_fan_cmd(uint8_t *p, const ct_fan_cmd_t *m)
{
    put16(&p[0], m->bottom_rpm);
    p[2] = m->top_level;
    p[3] = m->flags;
    return CT_FAN_CMD_LEN;
}

bool ct_unpack_fan_cmd(const uint8_t *p, uint8_t len, ct_fan_cmd_t *m)
{
    if (len != CT_FAN_CMD_LEN) return false;
    m->bottom_rpm = get16(&p[0]);
    m->top_level = p[2];
    m->flags = p[3];
    return true;
}

uint8_t ct_pack_fan_config(uint8_t *p, const ct_fan_config_t *m)
{
    put16(&p[0], m->kp_x1000);
    put16(&p[2], m->ki_x1000);
    put16(&p[4], m->rate_rpm_s);
    put16(&p[6], m->power_limit_pm);
    return CT_FAN_CONFIG_LEN;
}

uint8_t ct_pack_fan_state(uint8_t *p, const ct_fan_state_t *m)
{
    put16(&p[0], m->bottom_rpm);
    put16(&p[2], m->bottom_power_pm);
    put16(&p[4], m->top_power_pm);
    p[6] = m->state;
    p[7] = m->fault_code;
    put32(&p[8], m->echo_seq);
    return CT_FAN_STATE_LEN;
}

bool ct_unpack_fan_state(const uint8_t *p, uint8_t len, ct_fan_state_t *m)
{
    if (len != CT_FAN_STATE_LEN) return false;
    m->bottom_rpm = get16(&p[0]);
    m->bottom_power_pm = get16(&p[2]);
    m->top_power_pm = get16(&p[4]);
    m->state = p[6];
    m->fault_code = p[7];
    m->echo_seq = get32(&p[8]);
    return true;
}

uint8_t ct_pack_game_in(uint8_t *p, const ct_game_in_t *m)
{
    put16(&p[0], (uint16_t)m->ball_mm);
    put16(&p[2], (uint16_t)m->range_lo_mm);
    put16(&p[4], (uint16_t)m->range_hi_mm);
    p[6] = m->game_enabled;
    p[7] = m->obstacle_speed;
    return CT_GAME_IN_LEN;
}

bool ct_unpack_game_out(const uint8_t *p, uint8_t len, ct_game_out_t *m)
{
    if (len != CT_GAME_OUT_LEN) return false;
    m->target_level = p[0];
    m->hit = p[1];
    m->phase = p[2];
    m->score = get16(&p[3]);
    return true;
}
