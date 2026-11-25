/*
 * Gain search (Section 6.3 "Tuning", R3.9). Runs the real m3_ctrl core against
 * the plant model with noise 5 mm, 5 % dropped frames and a 30 ms camera delay,
 * for every combination of k_air, c_drag, tau at x0.7 / x1.0 / x1.3, and keeps the
 * gains whose WORST case is best. Build and run (Windows or Linux, gcc):
 *
 *   gcc -O2 -std=gnu11 -Ilib/contract -Ilib/m3_core tools/tune_gains.c \
 *       lib/contract/m3_contract.c lib/m3_core/m3_[a-z_]*.c -lm -o tune && ./tune
 *
 * Score per case: mean |error| and peak-to-peak of the TRUE height over 20 s
 * after a 15 s launch and settle, holding 300, 500 and 750 mm. Pass: mean <= 30 mm and p2p <= 50 mm.
 */
#include <math.h>
#include <stdio.h>

#include "m3_ctrl.h"
#include "m3_plant.h"

typedef struct { double mean, p2p; bool run; } res_t;

static uint32_t rng;
static float g_noise = 5.0f;  /* sigma, mm */
static float g_drop = 0.05f;
static float urand(void)
{
    rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
    return ((rng >> 8) + 1u) / 16777216.0f;
}
static float gauss(void) { return sqrtf(-2.0f * logf(urand())) * cosf(6.2831853f * urand()); }

static res_t run_case(const m3_ctrl_cfg_t *base, const m3_plant_params_t *truth, float target, uint32_t seed)
{
    static m3_ctrl_t c;
    m3_plant_t pl;
    m3_ctrl_init(&c, base, 1, 0);
    m3_ctrl_set_hold_target(&c, target);
    m3_plant_init(&pl, truth);
    rng = seed;
    float delay[64] = {0};
    ct_fan_cmd_t last = {0, 0, 0}, cmd;
    uint32_t seq, frame = 0, fs_seq = 0;
    uint8_t m1 = M1_OFF;
    double sum = 0, lo = 1e9, hi = -1e9;
    int n = 0;
    for (uint32_t t = 1; t <= 35100; t++) {
        float rpm = (last.flags & CT_FCMD_ENABLE) ? last.bottom_rpm : 0.0f;
        m3_plant_step(&pl, rpm, 0.001f);
        delay[t % 64] = pl.h_mm;
        if (t % 10 == 0) {
            ct_fan_state_t fs = {(uint16_t)pl.rpm, 0, 0, m1, 0, 0};
            if (m1 == M1_ARMED && (last.flags & CT_FCMD_ENABLE)) fs.state = M1_RUN;
            m3_ctrl_on_fan_state(&c, 1, fs_seq++, &fs, t);
        }
        bool sent = false;
        if ((t * 3) / 50 != ((t - 1) * 3) / 50 && t > 30) {
            ++frame;
            if (urand() >= g_drop) {
                float h = delay[(t - 30) % 64] + g_noise * gauss();
                ct_height_t m = {(int16_t)lrintf(h), (int16_t)lrintf(h), 0,
                                 CT_HFLAG_RAW_VALID | CT_HFLAG_FILT_VALID, frame, (uint64_t)(t - 30) * 1000u, 600};
                sent = m3_ctrl_on_height(&c, 2, &m, t, &cmd, &seq);
                if (sent) last = cmd;
            }
        }
        if (m3_ctrl_on_tick(&c, t, &cmd, &seq)) last = cmd;
        if (last.flags & CT_FCMD_REARM) m1 = M1_ARMED;
        if (t == 50) m3_ctrl_event(&c, EV_RESET, t);
        if (t == 100) m3_ctrl_event(&c, EV_START, t);
        if (t > 15100) { /* 15 s to launch and settle, then 20 s measured */
            double h = pl.h_mm;
            sum += fabs(h - target);
            if (h < lo) lo = h;
            if (h > hi) hi = h;
            n++;
        }
    }
    res_t r = {sum / n, hi - lo, c.fsm.state == ST_RUN};
    return r;
}

/* Worst case over the 27 model corners x 3 heights. */
static res_t worst(const m3_ctrl_cfg_t *cfg, bool verbose)
{
    const float sc[3] = {0.7f, 1.0f, 1.3f};
    res_t w = {0, 0, true};
    for (int a = 0; a < 3; a++)
        for (int b = 0; b < 3; b++)
            for (int k = 0; k < 3; k++) {
                m3_plant_params_t pp;
                m3_plant_default_params(&pp);
                pp.k_air *= sc[a];
                pp.c_drag *= sc[b];
                pp.tau_s *= sc[k];
                const float H[3] = {300.0f, 500.0f, 750.0f};
                for (int h = 0; h < 3; h++) {
                    res_t r = run_case(cfg, &pp, H[h], 2463534242u + 77u * (uint32_t)(a * 27 + b * 9 + k * 3 + h));
                    if (verbose)
                        printf("  k_air x%.1f c_drag x%.1f tau x%.1f at %3.0f mm: mean %5.1f p2p %5.1f %s\n", sc[a],
                               sc[b], sc[k], H[h], r.mean, r.p2p, r.run ? "" : "NOT RUNNING");
                    if (!r.run) w.run = false;
                    if (r.mean > w.mean) w.mean = r.mean;
                    if (r.p2p > w.p2p) w.p2p = r.p2p;
                }
            }
    return w;
}

int main(void)
{
    m3_ctrl_cfg_t cfg;
    m3_ctrl_default_cfg(&cfg);
    printf("current gains kp %.2f ki %.2f kd %.2f\n", cfg.pid.kp, cfg.pid.ki, cfg.pid.kd);
    res_t cur = worst(&cfg, true);
    printf("current worst: mean %.1f p2p %.1f %s\n\n", cur.mean, cur.p2p, cur.run ? "" : "(some not running)");

    const float KP[] = {1.0f, 1.2f, 1.4f, 1.6f, 1.8f, 2.0f};
    const float KI[] = {0.6f, 0.8f, 1.0f, 1.2f, 1.5f};
    const float KD[] = {0.08f, 0.1f, 0.13f, 0.16f, 0.2f, 0.25f};
    m3_ctrl_cfg_t best = cfg;
    double best_score = 1e9;
    for (unsigned i = 0; i < sizeof KP / sizeof *KP; i++)
        for (unsigned j = 0; j < sizeof KI / sizeof *KI; j++)
            for (unsigned k = 0; k < sizeof KD / sizeof *KD; k++) {
                m3_ctrl_cfg_t c = cfg;
                c.pid.kp = KP[i];
                c.pid.ki = KI[j];
                c.pid.kd = KD[k];
                res_t w = worst(&c, false);
                double score = w.run ? fmax(w.mean / 30.0, w.p2p / 50.0) : 1e6; /* < 1 passes */
                if (score < best_score) {
                    best_score = score;
                    best = c;
                    printf("kp %.2f ki %.2f kd %.2f -> worst mean %.1f mm, p2p %.1f mm (score %.2f)\n", c.pid.kp,
                           c.pid.ki, c.pid.kd, w.mean, w.p2p, score);
                    fflush(stdout);
                }
            }
    printf("\nbest kp %.2f ki %.2f kd %.2f, score %.2f (%s)\n", best.pid.kp, best.pid.ki, best.pid.kd, best_score,
           best_score <= 1.0 ? "meets R3.9" : "does NOT meet R3.9");
    worst(&best, true);
    return best_score <= 1.0 ? 0 : 1;
}
