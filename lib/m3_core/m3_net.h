/*
 * Network messages of the control board (Section 2.3): JSON over UDP, one message
 * per datagram, each with type, src, session and t.
 *
 *   in : command (M5)        -> m3_command_apply -> ack
 *        target_points (M4)  -> m3_ctrl_on_target_points
 *   out: ack (one per command), status (30 Hz)
 *
 * Split so that the JSON work never runs on the control core (R3.3):
 *   net task     : m3_net_parse()          datagram -> m3_net_msg_t
 *   control task : m3_net_apply()          m3_net_msg_t -> ack (or targets pushed)
 *                  m3_net_status_fill()    snapshot of the controller
 *   net task     : m3_net_format_ack(), m3_net_format_status()
 *
 * No hardware calls, no allocation; malformed input is rejected, never trusted.
 *
 * FIELD NAMES ARE PROVISIONAL until the M5 contract freeze (the documentation
 * lists the content of each message, not the JSON keys). They are all here:
 *
 *   command       {"type":"command","src":"m5","session":7,"t":1234,"request_id":12,
 *                  "cmd":"start"|"stop"|"reset"|"mode"|"set",
 *                  "mode":"hold"|"wave"|"game",            (cmd "mode")
 *                  "param":"kp","value":1.4}               (cmd "set")
 *   ack           {"type":"ack","src":"m3","session":..,"t":..,"request_id":12,"cmd":"set",
 *                  "ok":true,"pending":false,"param":"kp","applied":1.4,"reason":"ok"}
 *   target_points {"type":"target_points","src":"m4","session":..,"t":..,
 *                  "points":[{"t":1400,"h":500,"ff":0,"top":0}, ...]}   (h mm, ff rpm, t ms)
 *   status        {"type":"status","src":"m3","session":..,"t":..,"state":"run","fault":"none",
 *                  "mode":"hold","target_mm":..,"h_used_mm":..,"n_wanted_rpm":..,"rpm":..,
 *                  "bottom_power_pm":..,"top_power_pm":..,"top_level":..,"m1_state":..,
 *                  "game_phase":..,"game_score":..,"ball_stuck":false, + diagnostics}
 */
#ifndef M3_NET_H
#define M3_NET_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "m3_command.h"
#include "m3_ctrl.h"

#ifdef __cplusplus
extern "C" {
#endif

#define M3_NET_MAX_DATAGRAM 1400u
#define M3_NET_NAME_LEN 24u

typedef enum { NETMSG_BAD = 0, NETMSG_COMMAND, NETMSG_TARGETS, NETMSG_OTHER } m3_netmsg_kind_t;

typedef struct {
    m3_netmsg_kind_t kind;
    const char *error;          /* NETMSG_BAD: why (static string) */
    uint32_t session;
    int64_t t_ms;
    /* command */
    uint32_t request_id;
    char cmd[M3_NET_NAME_LEN];
    char mode[M3_NET_NAME_LEN];
    char param[M3_NET_NAME_LEN];
    float value;                /* NaN when missing or not a number */
    /* target_points */
    uint8_t n_points;
    m3_tpoint_t pts[M3_TBUF_MAX_POINTS_PER_MSG];
} m3_net_msg_t;

/* Parses one datagram. Returns the kind; *out is always filled (kind, error). */
m3_netmsg_kind_t m3_net_parse(const char *buf, size_t len, m3_net_msg_t *out);

typedef struct {
    m3_ack_t a;                 /* reason points to a static string */
    char cmd[M3_NET_NAME_LEN];
    char param[M3_NET_NAME_LEN];
} m3_net_ack_t;

/* Control-task side state: repeated commands are answered, not applied twice. */
typedef struct {
    bool have_last;
    uint32_t last_session, last_request_id;
    m3_net_ack_t last_ack;
    uint32_t n_commands, n_repeated;
    uint32_t n_targets, n_targets_refused;   /* refused: old, bad or not the target source */
} m3_net_t;

void m3_net_init(m3_net_t *n);

/* Returns true and fills *ack when an ack must be sent (every command). */
bool m3_net_apply(m3_net_t *n, m3_ctrl_t *c, const m3_net_msg_t *m, uint32_t now_ms, m3_net_ack_t *ack);

typedef struct {
    uint8_t state, cause, mode, m1_state, top_level, game_phase;
    bool ball_stuck, target_valid;
    float target_mm, h_used_mm, n_wanted_rpm;
    uint16_t rpm, bottom_power_pm, top_power_pm, game_score;
    /* diagnostics */
    uint32_t n_commands, n_targets, n_targets_refused;
    uint32_t heights_used;      /* filled by the firmware */
    uint32_t lat_max_us;        /* height received -> fan_cmd written, worst since boot (R3.3) */
    uint32_t lat_over_5ms;      /* heights that took longer than 5 ms */
    uint32_t heights_lost;      /* heights dropped before the control task saw them */
    uint32_t net_bad;           /* malformed datagrams (net task) */
} m3_net_status_t;

void m3_net_status_fill(const m3_net_t *n, const m3_ctrl_t *c, m3_net_status_t *st);

/* Both return the JSON length written (without the terminating 0), or 0 if it does not fit. */
size_t m3_net_format_ack(char *buf, size_t cap, uint32_t session, uint32_t now_ms, const m3_net_ack_t *ack);
size_t m3_net_format_status(char *buf, size_t cap, uint32_t session, uint32_t now_ms, const m3_net_status_t *st);

const char *m3_mode_name(uint8_t mode);

#ifdef __cplusplus
}
#endif
#endif
