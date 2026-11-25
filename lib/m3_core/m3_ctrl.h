/*
 * M3 controller core: glues height gate, FSM, target buffer and PID into the
 * three entry points the firmware (and the native simulation) call.
 * No hardware calls; time is always passed in (local monotonic ms, and the
 * camera capture time inside the height message).
 *
 *   on_height    -> runs the PID once, returns a fan_cmd to send now
 *   on_fan_state -> records M1 state
 *   on_tick      -> fault checks; returns a fan_cmd if 10 ms passed without one
 *   next_wait_ms -> how long the control task may block before calling on_tick
 */
#ifndef M3_CTRL_H
#define M3_CTRL_H

#include <stdbool.h>
#include <stdint.h>

#include "m3_contract.h"
#include "m3_fsm.h"
#include "m3_height_gate.h"
#include "m3_pid.h"
#include "m3_stuck.h"
#include "m3_target_buffer.h"

#ifdef __cplusplus
extern "C" {
#endif

#define M3_FAN_CMD_PERIOD_MS 10u

typedef struct {
    m3_pid_cfg_t pid;
    float hover_rpm;
    float launch_rpm;   /* launch push starts here ... */
    float launch_ramp_rpm_s; /* ... and rises at this rate, capped at n_max, until the ball lifts
                                (so a hover speed far from the model still lifts the ball) */
    float target_rate_mm_s; /* fastest change of the target the PID sees (0 = no limit): a step
                               command becomes a ramp, so the ball does not overshoot */
    float range_lo_mm;  /* usable height range (game levels map into it) */
    float range_hi_mm;
} m3_ctrl_cfg_t;

typedef struct {
    m3_ctrl_cfg_t cfg;
    m3_pid_t pid;
    m3_hgate_t hgate;
    m3_fsm_t fsm;
    m3_tbuf_t tbuf;
    bool game_mode;         /* set only through the mode command */
    uint8_t mode;           /* m3_mode_t (hold / wave / game), for status */
    float target_mm;        /* target used at the last control step, for status */
    bool have_ref;          /* rate-limited target valid */
    uint32_t ref_ms;
    m3_stuck_t stuck;       /* stuck-ball warning: status only, never a fault */

    /* game board link (Serial1) */
    ct_seq_track_t game_seq;
    uint8_t obstacle_speed;
    uint8_t game_phase;
    uint16_t game_score;
    uint32_t n_game_ignored;   /* game_out outside game mode */

    ct_seq_track_t fs_seq;
    bool have_fs;
    uint32_t fs_rx_ms;
    ct_fan_state_t fs;

    float h_used_mm;
    float n_wanted;     /* last wanted bottom speed (rpm) */
    uint8_t top_level;

    uint32_t session;   /* our own session id for outgoing frames */
    uint32_t out_seq;
    bool have_sent;
    uint32_t last_send_ms;
} m3_ctrl_t;

void m3_ctrl_default_cfg(m3_ctrl_cfg_t *cfg);
void m3_ctrl_init(m3_ctrl_t *c, const m3_ctrl_cfg_t *cfg, uint32_t session, uint32_t now_ms);

/* Each returns true and fills *cmd and *seq when a fan_cmd must be sent now. */
bool m3_ctrl_on_height(m3_ctrl_t *c, uint32_t session, const ct_height_t *m, uint32_t now_ms,
                       ct_fan_cmd_t *cmd, uint32_t *seq);
void m3_ctrl_on_fan_state(m3_ctrl_t *c, uint32_t session, uint32_t seq, const ct_fan_state_t *m,
                          uint32_t now_ms);
bool m3_ctrl_on_tick(m3_ctrl_t *c, uint32_t now_ms, ct_fan_cmd_t *cmd, uint32_t *seq);
uint32_t m3_ctrl_next_wait_ms(const m3_ctrl_t *c, uint32_t now_ms);

m3_result_t m3_ctrl_event(m3_ctrl_t *c, m3_event_t ev, uint32_t now_ms);
void m3_ctrl_set_hold_target(m3_ctrl_t *c, float mm);

/* target_points from M4 (hold and wave modes). In game mode the game board is the
 * target source, so M4 points are ignored and TB_IGNORED is returned. */
m3_tbuf_result_t m3_ctrl_on_target_points(m3_ctrl_t *c, uint32_t session, int64_t msg_t_ms,
                                          const m3_tpoint_t *pts, uint8_t n, uint32_t now_ms);

m3_cond_t m3_ctrl_conditions(const m3_ctrl_t *c, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
#endif
