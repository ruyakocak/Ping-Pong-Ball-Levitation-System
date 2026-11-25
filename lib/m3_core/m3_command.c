#include "m3_command.h"

#include <string.h>

static const char *fsm_reason(m3_result_t r)
{
    switch (r) {
    case RES_OK: return "ok";
    case RES_PENDING: return "pending_m1_armed";
    case RES_REFUSED_STATE: return "wrong_state";
    case RES_REFUSED_NO_HEIGHT: return "no_height";
    case RES_REFUSED_M1_NOT_ARMED: return "m1_not_armed";
    case RES_REFUSED_M1_FAULT: return "m1_fault";
    case RES_REFUSED_ESTOP: return "estop";
    case RES_REFUSED_NO_TARGET: return "no_target";
    case RES_REFUSED_NO_FAN_STATE: return "no_fan_state";
    case RES_REFUSED_NOT_GAME: return "not_game_mode";
    }
    return "?";
}

static bool flying(const m3_ctrl_t *c) { return c->fsm.state == ST_LAUNCH || c->fsm.state == ST_RUN; }

bool m3_param_get(const m3_ctrl_t *c, const char *name, float *v)
{
    const m3_param_def_t *d = m3_param_find(name);
    if (!d) return false;
    if (!d->writable) { *v = d->min; return true; } /* safety limits are constants */
    if (!strcmp(name, "kp")) *v = c->pid.cfg.kp;
    else if (!strcmp(name, "ki")) *v = c->pid.cfg.ki;
    else if (!strcmp(name, "kd")) *v = c->pid.cfg.kd;
    else if (!strcmp(name, "hover_rpm")) *v = c->cfg.hover_rpm;
    else if (!strcmp(name, "target_mm")) *v = c->tbuf.out.target_mm;
    else if (!strcmp(name, "rate_rpm_s")) *v = c->pid.cfg.rate_rpm_s;
    else if (!strcmp(name, "n_min_rpm")) *v = c->pid.cfg.n_min;
    else if (!strcmp(name, "n_max_rpm")) *v = c->pid.cfg.n_max;
    else if (!strcmp(name, "obstacle_speed")) *v = (float)c->obstacle_speed;
    else return false;
    return true;
}

/* Cross-checks after the range check. Returns CMD_OK or the reason. */
static m3_cmd_result_t consistent(const m3_ctrl_t *c, const char *p, float v)
{
    float lo = c->pid.cfg.n_min, hi = c->pid.cfg.n_max, hover = c->cfg.hover_rpm;
    if (!strcmp(p, "n_min_rpm")) lo = v;
    else if (!strcmp(p, "n_max_rpm")) hi = v;
    else if (!strcmp(p, "hover_rpm")) hover = v;
    else if (!strcmp(p, "target_mm")) {
        if (c->game_mode) return CMD_WRONG_MODE; /* the game board sets the target */
        if (v < c->cfg.range_lo_mm || v > c->cfg.range_hi_mm) return CMD_OUT_OF_RANGE;
        return CMD_OK;
    } else
        return CMD_OK;
    return (lo < hi && hover > lo && hover < hi) ? CMD_OK : CMD_CONFLICT;
}

static void set_param(m3_ctrl_t *c, const char *p, float v)
{
    if (!strcmp(p, "kp")) m3_pid_set_gains(&c->pid, v, c->pid.cfg.ki, c->pid.cfg.kd);
    else if (!strcmp(p, "ki")) m3_pid_set_gains(&c->pid, c->pid.cfg.kp, v, c->pid.cfg.kd);
    else if (!strcmp(p, "kd")) m3_pid_set_gains(&c->pid, c->pid.cfg.kp, c->pid.cfg.ki, v);
    else if (!strcmp(p, "hover_rpm")) c->cfg.hover_rpm = v;
    else if (!strcmp(p, "target_mm")) m3_ctrl_set_hold_target(c, v);
    else if (!strcmp(p, "rate_rpm_s")) c->pid.cfg.rate_rpm_s = v;
    else if (!strcmp(p, "n_min_rpm")) c->pid.cfg.n_min = v;
    else if (!strcmp(p, "n_max_rpm")) c->pid.cfg.n_max = v;
    else if (!strcmp(p, "obstacle_speed")) c->obstacle_speed = (uint8_t)(v + 0.5f);
    c->cfg.pid = c->pid.cfg;
}

static m3_ack_t ack(const m3_command_t *cmd, bool ok, const char *reason)
{
    m3_ack_t a;
    memset(&a, 0, sizeof(a));
    a.request_id = cmd->request_id;
    a.ok = ok;
    a.reason = reason;
    return a;
}

m3_ack_t m3_command_apply(m3_ctrl_t *c, const m3_command_t *cmd, uint32_t now)
{
    switch (cmd->kind) {
    case CMDK_START:
    case CMDK_STOP:
    case CMDK_RESET: {
        m3_event_t ev = cmd->kind == CMDK_START ? EV_START : cmd->kind == CMDK_STOP ? EV_STOP : EV_RESET;
        m3_result_t r = m3_ctrl_event(c, ev, now);
        m3_ack_t a = ack(cmd, r == RES_OK || r == RES_PENDING, fsm_reason(r));
        a.pending = r == RES_PENDING;
        return a;
    }
    case CMDK_MODE:
        if (cmd->mode < 0 || cmd->mode >= MODE_COUNT) return ack(cmd, false, m3_cmd_result_name(CMD_OUT_OF_RANGE));
        if (flying(c)) return ack(cmd, false, m3_cmd_result_name(CMD_WRONG_STATE));
        c->game_mode = cmd->mode == MODE_GAME;
        return ack(cmd, true, "ok");
    case CMDK_SET: {
        m3_ack_t a = ack(cmd, false, "");
        float old = 0.0f;
        bool known = m3_param_get(c, cmd->param, &old);
        a.applied = old;
        m3_cmd_result_t r = m3_param_check(cmd->param, cmd->value);
        if (r == CMD_OK) r = consistent(c, cmd->param, cmd->value);
        if (r != CMD_OK || !known) {
            a.reason = m3_cmd_result_name(r != CMD_OK ? r : CMD_UNKNOWN_PARAM);
            return a;
        }
        set_param(c, cmd->param, cmd->value);
        m3_param_get(c, cmd->param, &a.applied);
        a.ok = true;
        a.reason = "ok";
        return a;
    }
    }
    return ack(cmd, false, m3_cmd_result_name(CMD_UNKNOWN_COMMAND));
}

float m3_game_level_mm(const m3_ctrl_t *c, uint8_t level)
{
    float lo = c->cfg.range_lo_mm, hi = c->cfg.range_hi_mm;
    return lo + (hi - lo) * (float)level / (float)(M3_GAME_LEVELS - 1u);
}

m3_game_result_t m3_command_game_out(m3_ctrl_t *c, uint32_t session, uint32_t seq, const ct_game_out_t *m,
                                     uint32_t now)
{
    if (ct_seq_accept(&c->game_seq, session, seq) == CT_SEQ_OLD) return GAME_OLD;
    if (!c->game_mode) {
        c->n_game_ignored++;
        return GAME_IGNORED_NOT_GAME;
    }
    if (m->target_level >= M3_GAME_LEVELS) return GAME_BAD_LEVEL;
    c->game_phase = m->phase;
    c->game_score = m->score;
    m3_tbuf_set_hold(&c->tbuf, m3_game_level_mm(c, m->target_level));
    if (m->hit && flying(c)) m3_ctrl_event(c, EV_GAME_HIT, now);
    return GAME_APPLIED;
}
