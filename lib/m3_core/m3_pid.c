#include "m3_pid.h"

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void m3_pid_init(m3_pid_t *c, const m3_pid_cfg_t *cfg)
{
    c->cfg = *cfg;
    c->freeze_i = false;
    m3_pid_reset(c, cfg->n_min);
    c->have_out = false;
}

void m3_pid_reset(m3_pid_t *c, float n_now)
{
    c->i = 0.0f;
    c->have_prev = false;
    c->h_prev = 0.0f;
    c->n_out = n_now;
    c->have_out = true;
    c->last_e = 0.0f;
    c->last_dhdt = 0.0f;
    c->p_term = c->d_term = c->u_raw = 0.0f;
}

void m3_pid_restart_derivative(m3_pid_t *c) { c->have_prev = false; }

void m3_pid_freeze_integral(m3_pid_t *c, bool freeze) { c->freeze_i = freeze; }

float m3_pid_step(m3_pid_t *c, float r, float h, float dt, float hover, float ff)
{
    const m3_pid_cfg_t *k = &c->cfg;
    const bool dt_ok = dt > 0.0f && dt <= M3_PID_DT_MAX_S;
    const float e = r - h;

    const float p = k->kp * e;

    float dhdt = 0.0f;
    if (dt_ok && c->have_prev) dhdt = (h - c->h_prev) / dt;
    const float d = -k->kd * dhdt;

    /* Conditional integration: do not grow I while the output is at a limit
     * and the error pushes it further into that limit. */
    float i_new = c->i;
    if (dt_ok && !c->freeze_i) {
        float cand = clampf(c->i + k->ki * e * dt, -k->i_max, k->i_max);
        float u_cand = hover + p + cand + d + ff;
        bool push_hi = u_cand > k->n_max && e > 0.0f;
        bool push_lo = u_cand < k->n_min && e < 0.0f;
        if (!(push_hi || push_lo)) i_new = cand;
    }
    c->i = i_new;

    const float u = hover + p + c->i + d + ff;
    float n = clampf(u, k->n_min, k->n_max);

    /* Rate limit, using the step time (capped). */
    if (c->have_out) {
        float step_dt = dt > 0.0f ? (dt < M3_PID_DT_MAX_S ? dt : M3_PID_DT_MAX_S) : 0.0f;
        float max_step = k->rate_rpm_s * step_dt;
        n = clampf(n, c->n_out - max_step, c->n_out + max_step);
    }

    c->n_out = n;
    c->have_out = true;
    c->h_prev = h;
    c->have_prev = true;
    c->last_e = e;
    c->last_dhdt = dhdt;
    c->p_term = p;
    c->d_term = d;
    c->u_raw = u;
    return n;
}

void m3_pid_set_gains(m3_pid_t *c, float kp, float ki, float kd)
{
    const float before = c->cfg.kp * c->last_e - c->cfg.kd * c->last_dhdt;
    const float after = kp * c->last_e - kd * c->last_dhdt;
    c->i = clampf(c->i + before - after, -c->cfg.i_max, c->cfg.i_max);
    c->cfg.kp = kp;
    c->cfg.ki = ki; /* I is stored already multiplied by Ki: no jump from a Ki change */
    c->cfg.kd = kd;
}
