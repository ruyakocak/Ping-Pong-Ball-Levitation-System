/*
 * Outer height PID (Section 6.3).
 *
 *   e = r - h
 *   P = Kp*e
 *   I = limit(I + Ki*e*dt, -I_max, I_max)        (conditional integration = anti-windup)
 *   D = -Kd*(h - h_prev)/dt                       (on measurement: no kick on target steps)
 *   u = hover + P + I + D + ff
 *   n = rate_limit(limit(u, n_min, n_max))
 *
 * dt <= 0 or dt > 100 ms: I not updated, D = 0 for that step.
 * Gain changes are bumpless: I absorbs the change of P + D at that moment.
 * Units: mm, s, rpm.
 */
#ifndef M3_PID_H
#define M3_PID_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define M3_PID_DT_MAX_S 0.100f

typedef struct {
    float kp, ki, kd;     /* rpm/mm, rpm/(mm*s), rpm*s/mm */
    float i_max;          /* rpm */
    float n_min, n_max;   /* rpm */
    float rate_rpm_s;     /* max change of the output per second */
} m3_pid_cfg_t;

typedef struct {
    m3_pid_cfg_t cfg;
    float i;
    float h_prev;
    bool have_prev;
    bool freeze_i;      /* Launch state: integral frozen */
    float n_out;        /* last output (after limits) */
    bool have_out;
    float last_e;       /* for bumpless gain change */
    float last_dhdt;
    /* last terms, for logging / tests */
    float p_term, d_term, u_raw;
} m3_pid_t;

void m3_pid_init(m3_pid_t *c, const m3_pid_cfg_t *cfg);

/* Clears I and the derivative history; the rate limiter starts from n_now. */
void m3_pid_reset(m3_pid_t *c, float n_now);

/* Next step has no derivative (e.g. height source switched filtered <-> raw). */
void m3_pid_restart_derivative(m3_pid_t *c);

void m3_pid_freeze_integral(m3_pid_t *c, bool freeze);

/* One step per new height. dt_s from camera capture times. Returns the speed to send (rpm). */
float m3_pid_step(m3_pid_t *c, float r_mm, float h_mm, float dt_s, float hover_rpm, float ff_rpm);

/* Changes gains without a jump of the output at this moment. */
void m3_pid_set_gains(m3_pid_t *c, float kp, float ki, float kd);

#ifdef __cplusplus
}
#endif
#endif
