#include "m3_fsm.h"

static void enter(m3_fsm_t *f, m3_state_t s, m3_cause_t cause, uint32_t now)
{
    f->state = s;
    f->clear_integral = true;
    f->rearm_pending = false;
    if (s == ST_FAULT) f->cause = cause;
    else f->cause = CAUSE_NONE;
    if (s == ST_LAUNCH) f->launch_start_ms = now;
}

void m3_fsm_init(m3_fsm_t *f, uint32_t now_ms)
{
    f->launch_start_ms = 0;
    f->rearm_start_ms = 0;
    enter(f, ST_FAULT, CAUSE_POWER_UP, now_ms);
}

/* Fault condition present right now (Launch/Run row of the table). */
static m3_cause_t fault_cause(const m3_cond_t *c)
{
    if (c->estop) return CAUSE_ESTOP;
    if (c->m1_fault) return CAUSE_M1_FAULT;
    if (!c->fan_state_fresh) return CAUSE_FAN_STATE_LOST;
    if (!c->height_fresh) return CAUSE_NO_HEIGHT;
    return CAUSE_NONE;
}

m3_result_t m3_fsm_event(m3_fsm_t *f, m3_event_t ev, const m3_cond_t *c, uint32_t now)
{
    switch (ev) {
    case EV_START:
        if (f->state != ST_IDLE) return RES_REFUSED_STATE;
        if (c->estop) return RES_REFUSED_ESTOP;
        if (!c->fan_state_fresh) return RES_REFUSED_NO_FAN_STATE;
        if (c->m1_fault) return RES_REFUSED_M1_FAULT;
        if (!c->m1_armed) return RES_REFUSED_M1_NOT_ARMED;
        if (!c->height_fresh) return RES_REFUSED_NO_HEIGHT;
        if (!c->target_valid) return RES_REFUSED_NO_TARGET;
        enter(f, ST_LAUNCH, CAUSE_NONE, now);
        return RES_OK;

    case EV_GAME_HIT:
        if (!c->game_mode) return RES_REFUSED_NOT_GAME;
        return m3_fsm_event(f, EV_STOP, c, now); /* a hit acts like Stop */
    case EV_STOP:
        if (f->state == ST_LAUNCH || f->state == ST_RUN) {
            enter(f, ST_IDLE, CAUSE_NONE, now);
            return RES_OK;
        }
        if (f->state == ST_IDLE) return RES_OK; /* already stopped */
        return RES_REFUSED_STATE;             /* Stop never leaves Fault */

    case EV_RESET:
        if (f->state != ST_FAULT) return RES_REFUSED_STATE;
        if (c->estop) return RES_REFUSED_ESTOP;
        if (!c->height_fresh) return RES_REFUSED_NO_HEIGHT;
        if (!c->fan_state_fresh) return RES_REFUSED_NO_FAN_STATE;
        /* Step 3 of the reset sequence: send re-arm with blowers disabled,
         * Idle only after M1 reports armed (step 5, in m3_fsm_tick). */
        f->rearm_pending = true;
        f->rearm_start_ms = now;
        return RES_PENDING;
    }
    return RES_REFUSED_STATE;
}

void m3_fsm_tick(m3_fsm_t *f, const m3_cond_t *c, uint32_t now)
{
    switch (f->state) {
    case ST_LAUNCH:
    case ST_RUN: {
        m3_cause_t cause = fault_cause(c);
        if (cause != CAUSE_NONE) { enter(f, ST_FAULT, cause, now); return; }
        if (f->state == ST_LAUNCH) {
            if (c->ball_mm > M3_LAUNCH_LIFT_MM) enter(f, ST_RUN, CAUSE_NONE, now);
            else if ((uint32_t)(now - f->launch_start_ms) >= M3_LAUNCH_TIMEOUT_MS)
                enter(f, ST_FAULT, CAUSE_LAUNCH_FAILED, now);
        }
        return;
    }
    case ST_FAULT:
        if (!f->rearm_pending) return;
        if (c->estop || !c->height_fresh || !c->fan_state_fresh) { f->rearm_pending = false; return; }
        if (c->m1_armed && !c->m1_fault) { enter(f, ST_IDLE, CAUSE_NONE, now); return; }
        if ((uint32_t)(now - f->rearm_start_ms) >= M3_REARM_TIMEOUT_MS) f->rearm_pending = false;
        return;
    case ST_IDLE:
    default:
        return;
    }
}

m3_fsm_out_t m3_fsm_output(const m3_fsm_t *f)
{
    m3_fsm_out_t o = {false, false, false, false};
    o.enable = f->state == ST_LAUNCH || f->state == ST_RUN;
    o.rearm = f->state == ST_FAULT && f->rearm_pending;
    o.freeze_integral = f->state == ST_LAUNCH;
    o.launch_push = f->state == ST_LAUNCH;
    return o;
}

const char *m3_state_name(m3_state_t s)
{
    static const char *n[] = {"idle", "launch", "run", "fault"};
    return (unsigned)s < 4u ? n[s] : "?";
}

const char *m3_cause_name(m3_cause_t c)
{
    static const char *n[] = {"none", "power_up", "no_height", "fan_state_lost",
                              "m1_fault", "estop", "launch_failed"};
    return (unsigned)c < 7u ? n[c] : "?";
}

const char *m3_result_name(m3_result_t r)
{
    static const char *n[] = {"ok", "pending", "refused_state", "refused_no_height",
                              "refused_m1_not_armed", "refused_m1_fault", "refused_estop",
                              "refused_no_target", "refused_no_fan_state", "refused_not_game"};
    return (unsigned)r < 10u ? n[r] : "?";
}
