/*
 * Stuck-ball warning (Section 6.3, R3.11b/c). In Run only: if the target is at
 * least 50 mm above the ball, the wanted speed is within 10 % of the maximum,
 * and the ball rises less than 10 mm in 3 s, the warning is set. It is NOT a
 * fault: it changes neither the control nor any limit; the operator decides.
 * Holding still near the top never triggers it (the target is not above the ball).
 * Thresholds are B-level values, to be checked in the first rig week.
 */
#ifndef M3_STUCK_H
#define M3_STUCK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define M3_STUCK_GAP_MM 50.0f
#define M3_STUCK_SPEED_FRAC 0.90f
#define M3_STUCK_RISE_MM 10.0f
#define M3_STUCK_WINDOW_MS 3000u

typedef struct {
    bool armed;         /* conditions holding, window running */
    uint32_t start_ms;
    float start_mm;
    bool warning;
} m3_stuck_t;

void m3_stuck_init(m3_stuck_t *s);

/* Call at every used height. Returns the warning flag. */
bool m3_stuck_update(m3_stuck_t *s, bool running, float target_mm, float ball_mm, float n_wanted,
                     float n_max, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
#endif
