/*
 * M2 stand-in filter for the tests (not firmware). M2 sends, with every frame, the
 * raw height AND a filtered height from its own two-state Kalman filter, moved
 * forward by the camera delay (Section 5.3). M3 controls on the filtered height
 * while it is valid, so a stand-in that copied the raw height into the filtered
 * field would play a worse M2 than the contract promises.
 *
 * This is a plain alpha-beta filter, deliberately WEAKER than M2's own pass
 * conditions, so M3 is never tested against a better M2 than the real one must be
 * (checked in sim/tests/test_m2_standin.py with the same numbers):
 *   R2.5  still ball, 5 mm noise: filtered spread 0.62 x raw   (M2 must reach < 0.5)
 *   R2.4  moving ball: filtered error about 6.5 mm RMS          (M2 must reach < 5)
 *   R2.10 lag on a 15 cm 0.5 Hz sine about 6 ms                 (M2 must reach < 15)
 * Same coefficients as sim/hil_sim.py (M2Filter).
 */
#ifndef M2_STANDIN_H
#define M2_STANDIN_H

#include <stdbool.h>
#include <stdint.h>

#define M2SI_ALPHA 0.35f
#define M2SI_BETA 0.06f
#define M2SI_FRAME_S (1.0f / 60.0f)

typedef struct {
    float x, v;          /* height (mm) and velocity (mm/s) at the capture time */
    uint32_t frame;
    bool have;
} m2si_t;

static inline void m2si_init(m2si_t *f) { f->have = false; }

/* One camera frame (raw reading z, frame number); returns the filtered height,
 * moved forward by delay_s like M2's output. Lost frames are predicted over. */
static inline float m2si_update(m2si_t *f, float z, uint32_t frame, float delay_s)
{
    if (!f->have) {
        f->x = z;
        f->v = 0.0f;
        f->have = true;
    } else {
        float dt = (float)(frame - f->frame) * M2SI_FRAME_S;
        if (dt <= 0.0f || dt > 0.2f) dt = M2SI_FRAME_S;
        float xp = f->x + f->v * dt, r = z - xp;
        f->x = xp + M2SI_ALPHA * r;
        f->v = f->v + M2SI_BETA * r / dt;
    }
    f->frame = frame;
    return f->x + f->v * delay_s;
}

#endif
