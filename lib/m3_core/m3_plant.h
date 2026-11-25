/*
 * Textbook plant model (Section 6.3, "Simulated ball and simulator").
 * Used by the built-in simulated ball (esp32_simball) and the native
 * closed-loop tests. sim/m3sim/plant.py implements the SAME equations; keep
 * both in sync (test_plant_matches_python checks one reference trajectory).
 *
 *   blower:  d(rpm)/dt = (clamp(cmd, rpm_min_run, rpm_max) - rpm) / tau      (0 below rpm_min_run)
 *   air:     v_air = k_air * rpm                                            (m/s)
 *   force:   F = c_drag * (v_air - v_ball) * |v_air - v_ball|               (N)
 *   ball:    a = F / m - g ; stops on the bottom grille (0 mm) and top grille
 *
 * Coefficients are built-in textbook values until M4's fitted parameter file exists.
 */
#ifndef M3_PLANT_H
#define M3_PLANT_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float mass_kg;      /* 2.7 g */
    float g;
    float k_air;        /* (m/s) per rpm */
    float c_drag;       /* N/(m/s)^2 */
    float tau_s;        /* blower speed lag */
    float rpm_min_run;  /* lowest steady speed */
    float rpm_max;
    float top_mm;       /* highest centre position */
} m3_plant_params_t;

typedef struct {
    m3_plant_params_t p;
    float rpm;
    float h_mm;
    float v_mm_s;
} m3_plant_t;

void m3_plant_default_params(m3_plant_params_t *p);
void m3_plant_init(m3_plant_t *s, const m3_plant_params_t *p);
void m3_plant_step(m3_plant_t *s, float rpm_cmd, float dt_s);
/* Speed that holds the ball still (for the default model). */
float m3_plant_hover_rpm(const m3_plant_params_t *p);

#ifdef __cplusplus
}
#endif
#endif
