#include "m3_plant.h"

#include <math.h>

void m3_plant_default_params(m3_plant_params_t *p)
{
    p->mass_kg = 0.0027f;
    p->g = 9.81f;
    p->k_air = 0.002f;                        /* 3000 rpm -> 6 m/s */
    p->c_drag = 0.0027f * 9.81f / 36.0f;      /* hover exactly at 6 m/s */
    p->tau_s = 0.15f;
    p->rpm_min_run = 800.0f;
    p->rpm_max = 6000.0f;
    p->top_mm = 960.0f;
}

void m3_plant_init(m3_plant_t *s, const m3_plant_params_t *p)
{
    s->p = *p;
    s->rpm = 0.0f;
    s->h_mm = 0.0f;
    s->v_mm_s = 0.0f;
}

float m3_plant_hover_rpm(const m3_plant_params_t *p)
{
    return sqrtf(p->mass_kg * p->g / p->c_drag) / p->k_air;
}

void m3_plant_step(m3_plant_t *s, float cmd, float dt)
{
    const m3_plant_params_t *p = &s->p;
    float target = cmd < p->rpm_min_run ? 0.0f : (cmd > p->rpm_max ? p->rpm_max : cmd);
    s->rpm += (target - s->rpm) * (dt / p->tau_s);

    float v_air = p->k_air * s->rpm;
    float v_rel = v_air - s->v_mm_s * 1e-3f;
    float f = p->c_drag * v_rel * fabsf(v_rel);
    float a = f / p->mass_kg - p->g; /* m/s^2 */

    s->v_mm_s += a * 1000.0f * dt;
    s->h_mm += s->v_mm_s * dt;
    if (s->h_mm <= 0.0f) { s->h_mm = 0.0f; if (s->v_mm_s < 0.0f) s->v_mm_s = 0.0f; }
    if (s->h_mm >= p->top_mm) { s->h_mm = p->top_mm; if (s->v_mm_s > 0.0f) s->v_mm_s = 0.0f; }
}
