#include "m3_stuck.h"

void m3_stuck_init(m3_stuck_t *s)
{
    s->armed = false;
    s->start_ms = 0;
    s->start_mm = 0.0f;
    s->warning = false;
}

bool m3_stuck_update(m3_stuck_t *s, bool running, float target_mm, float ball_mm, float n_wanted,
                     float n_max, uint32_t now)
{
    bool cond = running && target_mm - ball_mm >= M3_STUCK_GAP_MM &&
                n_wanted >= M3_STUCK_SPEED_FRAC * n_max;
    if (!cond) {
        m3_stuck_init(s); /* warning clears as soon as a condition is gone */
        return false;
    }
    if (!s->armed || ball_mm - s->start_mm >= M3_STUCK_RISE_MM) {
        /* first time, or the ball did rise: start a new 3 s window from here */
        s->armed = true;
        s->start_ms = now;
        s->start_mm = ball_mm;
        s->warning = false;
        return false;
    }
    if ((uint32_t)(now - s->start_ms) >= M3_STUCK_WINDOW_MS) s->warning = true;
    return s->warning;
}
