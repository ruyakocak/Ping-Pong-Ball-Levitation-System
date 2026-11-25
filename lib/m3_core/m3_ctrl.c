#include "m3_ctrl.h"

#include <string.h>

void m3_ctrl_default_cfg(m3_ctrl_cfg_t *cfg)
{
    /* Starting values for the textbook plant (m3_plant.h); tuned in simulation. */
    cfg->pid.kp = 1.4f;  /* tools/tune_gains.c, worst case over +-30 % models, noise 5 mm, 5 % drops */
    cfg->pid.ki = 0.8f;
    cfg->pid.kd = 0.13f;
    cfg->pid.i_max = 2500.0f; /* >= n_max - hover, so a stuck ball can drive the output to the limit */
    cfg->pid.n_min = 0.0f;
    cfg->pid.n_max = 5800.0f; /* below abs max 6000; covers a hover speed up to +70 % of the model */
    cfg->pid.rate_rpm_s = 8000.0f;
    cfg->hover_rpm = 3000.0f;
    cfg->launch_rpm = 3000.0f * 1.15f;
    cfg->launch_ramp_rpm_s = 2000.0f;
    cfg->range_lo_mm = 100.0f; /* provisional, until the rig is measured */
    cfg->range_hi_mm = 850.0f;
}

void m3_ctrl_init(m3_ctrl_t *c, const m3_ctrl_cfg_t *cfg, uint32_t session, uint32_t now_ms)
{
    memset(c, 0, sizeof(*c));
    c->cfg = *cfg;
    m3_pid_init(&c->pid, &cfg->pid);
    m3_pid_reset(&c->pid, 0.0f);
    m3_hgate_init(&c->hgate);
    m3_fsm_init(&c->fsm, now_ms);
    m3_tbuf_init(&c->tbuf);
    m3_stuck_init(&c->stuck);
    c->session = session;
}

m3_cond_t m3_ctrl_conditions(const m3_ctrl_t *c, uint32_t now)
{
    m3_cond_t k;
    k.height_fresh = !m3_hgate_timed_out(&c->hgate, now);
    k.fan_state_fresh = c->have_fs && (uint32_t)(now - c->fs_rx_ms) < M3_FAN_STATE_TIMEOUT_MS;
    k.m1_armed = c->have_fs && (c->fs.state == M1_ARMED || c->fs.state == M1_RUN);
    k.m1_fault = c->have_fs && c->fs.state == M1_FAULT;
    k.estop = c->have_fs && c->fs.state == M1_ESTOP;
    k.target_valid = m3_tbuf_valid(&c->tbuf);
    k.game_mode = c->game_mode;
    k.ball_mm = c->h_used_mm;
    return k;
}

/* Applies FSM side effects to the PID after any FSM call. */
static void sync_fsm(m3_ctrl_t *c)
{
    if (c->fsm.clear_integral) {
        c->fsm.clear_integral = false;
        m3_fsm_out_t o = m3_fsm_output(&c->fsm);
        float n_now = o.enable ? c->n_wanted : 0.0f;
        m3_pid_reset(&c->pid, n_now);
        if (!o.enable) c->n_wanted = 0.0f;
    }
    m3_pid_freeze_integral(&c->pid, m3_fsm_output(&c->fsm).freeze_integral);
}

static void build_cmd(m3_ctrl_t *c, uint32_t now, ct_fan_cmd_t *cmd, uint32_t *seq)
{
    m3_fsm_out_t o = m3_fsm_output(&c->fsm);
    float n = o.enable ? c->n_wanted : 0.0f;
    if (n < 0.0f) n = 0.0f;
    if (n > 65535.0f) n = 65535.0f;
    cmd->bottom_rpm = (uint16_t)(n + 0.5f);
    cmd->top_level = o.enable ? c->top_level : 0;
    cmd->flags = (uint8_t)((o.enable ? CT_FCMD_ENABLE : 0) | (o.rearm ? CT_FCMD_REARM : 0));
    *seq = c->out_seq++;
    c->have_sent = true;
    c->last_send_ms = now;
}

bool m3_ctrl_on_height(m3_ctrl_t *c, uint32_t session, const ct_height_t *m, uint32_t now,
                       ct_fan_cmd_t *cmd, uint32_t *seq)
{
    m3_hg_out_t g = m3_hgate_on_height(&c->hgate, session, m, now);
    if (!g.accepted) return false; /* repeated / old frame: no control step */

    if (g.src != HG_SRC_NONE) c->h_used_mm = g.h_mm;

    m3_cond_t k = m3_ctrl_conditions(c, now);
    m3_fsm_tick(&c->fsm, &k, now);
    sync_fsm(c);

    m3_fsm_out_t o = m3_fsm_output(&c->fsm);
    if (g.src != HG_SRC_NONE && o.enable) {
        m3_target_t t = m3_tbuf_get(&c->tbuf, now);
        if (g.src_switched) m3_pid_restart_derivative(&c->pid);
        float n_pid = m3_pid_step(&c->pid, t.target_mm, g.h_mm, g.dt_s, c->cfg.hover_rpm, t.ff_rpm);
        if (o.launch_push) {
            float push = c->cfg.launch_rpm +
                         c->cfg.launch_ramp_rpm_s * (float)(uint32_t)(now - c->fsm.launch_start_ms) * 1e-3f;
            c->n_wanted = push < c->pid.cfg.n_max ? push : c->pid.cfg.n_max;
        } else {
            c->n_wanted = n_pid;
        }
        if (o.launch_push) c->pid.n_out = c->n_wanted; /* rate limiter continues from the push */
        c->top_level = t.top_level;
        m3_stuck_update(&c->stuck, c->fsm.state == ST_RUN, t.target_mm, g.h_mm, c->n_wanted,
                        c->pid.cfg.n_max, now);
    } else if (!o.enable) {
        m3_stuck_init(&c->stuck);
    }
    build_cmd(c, now, cmd, seq);
    return true;
}

void m3_ctrl_on_fan_state(m3_ctrl_t *c, uint32_t session, uint32_t seq, const ct_fan_state_t *m,
                          uint32_t now)
{
    if (ct_seq_accept(&c->fs_seq, session, seq) == CT_SEQ_OLD) return;
    c->fs = *m;
    c->have_fs = true;
    c->fs_rx_ms = now;
}

bool m3_ctrl_on_tick(m3_ctrl_t *c, uint32_t now, ct_fan_cmd_t *cmd, uint32_t *seq)
{
    m3_cond_t k = m3_ctrl_conditions(c, now);
    m3_state_t before = c->fsm.state;
    m3_fsm_tick(&c->fsm, &k, now);
    sync_fsm(c);
    bool changed = c->fsm.state != before;
    if (changed || !c->have_sent || (uint32_t)(now - c->last_send_ms) >= M3_FAN_CMD_PERIOD_MS) {
        build_cmd(c, now, cmd, seq);
        return true;
    }
    return false;
}

uint32_t m3_ctrl_next_wait_ms(const m3_ctrl_t *c, uint32_t now)
{
    uint32_t wait = M3_FAN_CMD_PERIOD_MS;
    if (c->have_sent) {
        uint32_t since = now - c->last_send_ms;
        wait = since >= M3_FAN_CMD_PERIOD_MS ? 0 : M3_FAN_CMD_PERIOD_MS - since;
    }
    /* wake exactly at the 200 ms height deadline and the 100 ms fan_state deadline */
    if (c->hgate.ever_real) {
        uint32_t since = now - c->hgate.last_real_ms;
        uint32_t left = since >= M3_HG_TIMEOUT_MS ? 0 : M3_HG_TIMEOUT_MS - since;
        if (left < wait) wait = left;
    }
    if (c->have_fs) {
        uint32_t since = now - c->fs_rx_ms;
        uint32_t left = since >= M3_FAN_STATE_TIMEOUT_MS ? 0 : M3_FAN_STATE_TIMEOUT_MS - since;
        if (left < wait) wait = left;
    }
    return wait;
}

m3_result_t m3_ctrl_event(m3_ctrl_t *c, m3_event_t ev, uint32_t now)
{
    m3_cond_t k = m3_ctrl_conditions(c, now);
    m3_result_t r = m3_fsm_event(&c->fsm, ev, &k, now);
    sync_fsm(c);
    return r;
}

void m3_ctrl_set_hold_target(m3_ctrl_t *c, float mm) { m3_tbuf_set_hold(&c->tbuf, mm); }

m3_tbuf_result_t m3_ctrl_on_target_points(m3_ctrl_t *c, uint32_t session, int64_t msg_t_ms,
                                          const m3_tpoint_t *pts, uint8_t n, uint32_t now_ms)
{
    if (c->game_mode) return TB_IGNORED;
    return m3_tbuf_push(&c->tbuf, session, msg_t_ms, pts, n, now_ms);
}
