/*
 * Serial message contract (Section 2.3 of the project documentation).
 *
 * PROVISIONAL: the frame format below is exactly as specified in the
 * document; the PAYLOAD layouts are our own placeholders until M5 delivers
 * the frozen contract header. When it arrives, replace the payload structs /
 * pack functions here and keep the rest of M3 unchanged.
 *
 * Frame: A5 5A | type(1) | len(1) | session(4) | seq(4) | payload(len) | crc(2)
 * All multi-byte fields little-endian. CRC16/CCITT-FALSE over
 * type..payload (not the start bytes, not the CRC itself).
 */
#ifndef M3_CONTRACT_H
#define M3_CONTRACT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CT_START0 0xA5u
#define CT_START1 0x5Au
#define CT_HEADER_LEN 10u /* type, len, session, seq */
#define CT_MAX_PAYLOAD 255u
#define CT_MAX_FRAME (2u + CT_HEADER_LEN + CT_MAX_PAYLOAD + 2u)

typedef enum {
    CT_HEIGHT = 0x01,     /* M2 -> M3 */
    CT_FAN_CMD = 0x10,    /* M3 -> M1 */
    CT_FAN_CONFIG = 0x11, /* M3 -> M1 */
    CT_FAN_STATE = 0x12,  /* M1 -> M3 */
    CT_GAME_IN = 0x20,    /* M3 -> game board */
    CT_GAME_OUT = 0x21    /* game board -> M3 */
} ct_type_t;

/* ---- payloads (PROVISIONAL layouts) ---- */

#define CT_HFLAG_RAW_VALID 0x01u
#define CT_HFLAG_FILT_VALID 0x02u
#define CT_HFLAG_BRIDGED 0x04u

typedef struct {
    int16_t raw_mm;
    int16_t filt_mm;
    int16_t vel_mm_s;
    uint8_t flags;        /* CT_HFLAG_* */
    uint32_t frame_no;
    uint64_t capture_us;  /* camera clock */
    uint16_t fps_x10;
} ct_height_t;
#define CT_HEIGHT_LEN 21u

#define CT_FCMD_ENABLE 0x01u
#define CT_FCMD_REARM 0x02u

typedef struct {
    uint16_t bottom_rpm;
    uint8_t top_level; /* 0..3 */
    uint8_t flags;     /* CT_FCMD_* */
} ct_fan_cmd_t;
#define CT_FAN_CMD_LEN 4u

typedef struct {
    uint16_t kp_x1000;
    uint16_t ki_x1000;
    uint16_t rate_rpm_s;
    uint16_t power_limit_pm;
} ct_fan_config_t;
#define CT_FAN_CONFIG_LEN 8u

typedef enum { M1_OFF = 0, M1_ARMED = 1, M1_RUN = 2, M1_FAULT = 3, M1_ESTOP = 4 } ct_m1_state_t;

typedef struct {
    uint16_t bottom_rpm;
    uint16_t bottom_power_pm;
    uint16_t top_power_pm;
    uint8_t state;      /* ct_m1_state_t */
    uint8_t fault_code;
    uint32_t echo_seq;
} ct_fan_state_t;
#define CT_FAN_STATE_LEN 12u

typedef struct {
    int16_t ball_mm;      /* INT16_MIN = invalid */
    int16_t range_lo_mm;
    int16_t range_hi_mm;
    uint8_t game_enabled;
    uint8_t obstacle_speed;
} ct_game_in_t;
#define CT_GAME_IN_LEN 8u

typedef struct {
    uint8_t target_level; /* 0..7 */
    uint8_t hit;
    uint8_t phase;
    uint16_t score;
} ct_game_out_t;
#define CT_GAME_OUT_LEN 5u

/* ---- CRC ---- */
uint16_t ct_crc16(const uint8_t *data, size_t len);

/* ---- framing ---- */

/* Encodes one frame into out (size >= CT_MAX_FRAME). Returns total length, 0 on error. */
size_t ct_encode_frame(uint8_t *out, uint8_t type, uint32_t session, uint32_t seq,
                       const uint8_t *payload, uint8_t len);

typedef struct {
    uint8_t type;
    uint8_t len;
    uint32_t session;
    uint32_t seq;
    uint8_t payload[CT_MAX_PAYLOAD];
} ct_frame_t;

/* Byte-wise streaming decoder; resynchronises on the start bytes. */
typedef struct {
    uint8_t buf[CT_HEADER_LEN + CT_MAX_PAYLOAD + 2u];
    uint16_t pos;
    uint16_t need;
    uint8_t state;
    uint32_t crc_errors;
    uint32_t frames_ok;
} ct_decoder_t;

void ct_decoder_init(ct_decoder_t *d);
/* Feeds one byte; returns true and fills *out when a frame with a correct CRC completes. */
bool ct_decoder_feed(ct_decoder_t *d, uint8_t byte, ct_frame_t *out);

/* ---- sequence / session tracking (per sender, per receiver) ---- */
typedef struct {
    bool have;
    uint32_t session;
    uint32_t last_seq;
} ct_seq_track_t;

typedef enum { CT_SEQ_OLD = 0, CT_SEQ_NEW = 1, CT_SEQ_NEW_SESSION = 2 } ct_seq_result_t;
/* Accepts a frame only if newer than the last one in the same session, or from a new session. */
ct_seq_result_t ct_seq_accept(ct_seq_track_t *t, uint32_t session, uint32_t seq);

/* ---- payload pack / unpack (return bytes written, or false on wrong length) ---- */
uint8_t ct_pack_height(uint8_t *p, const ct_height_t *m);
bool ct_unpack_height(const uint8_t *p, uint8_t len, ct_height_t *m);
uint8_t ct_pack_fan_cmd(uint8_t *p, const ct_fan_cmd_t *m);
bool ct_unpack_fan_cmd(const uint8_t *p, uint8_t len, ct_fan_cmd_t *m);
uint8_t ct_pack_fan_config(uint8_t *p, const ct_fan_config_t *m);
uint8_t ct_pack_fan_state(uint8_t *p, const ct_fan_state_t *m);
bool ct_unpack_fan_state(const uint8_t *p, uint8_t len, ct_fan_state_t *m);
uint8_t ct_pack_game_in(uint8_t *p, const ct_game_in_t *m);
bool ct_unpack_game_out(const uint8_t *p, uint8_t len, ct_game_out_t *m);

#ifdef __cplusplus
}
#endif
#endif
