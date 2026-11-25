/*
 * Controller core, closed loop on the native build (layer 2 of Figure 11):
 * real m3_ctrl code + simulated ball and blower + simulated M1 state.
 * Time advances in 1 ms steps; camera at 60 fps with a 30 ms delay.
 */
#include <math.h>
#include <stdio.h>
#include <unity.h>

#include "m3_ctrl.h"
#include "m3_plant.h"

void setUp(void) {}
void tearDown(void) {}

typedef struct {
    m3_ctrl_t ctl;
    m3_plant_t plant;
    uint32_t t_ms;
    uint32_t frame, fs_seq;
    uint8_t m1_state;
    ct_fan_cmd_t last_cmd;
    uint32_t last_seq;
    int cmds_sent;
    uint32_t max_gap_ms, last_cmd_ms;
    bool seq_ok;
    /* height pipeline with delay */
    float delayed_h[64];
    bool heights_on;
    bool bridged_only;
    uint32_t fault_at_ms;
    /* fault injection (R3.11) */
    bool fs_on;          /* M1 sends fan_state */
    uint8_t m1_inject;   /* 0 or M1_FAULT / M1_ESTOP reported in fan_state */
    float stuck_mm;      /* > 0: ball held at this height */
    uint32_t warn_at_ms;
    /* bad data (R3.9) */
    float noise_mm;      /* Gaussian sigma on the measured height */
    int drop_pct;        /* frames lost, percent */
    uint32_t rng;
    int dropped;
} sim_t;

static sim_t S;

static void on_cmd(const ct_fan_cmd_t *cmd, uint32_t seq)
{
    if (S.cmds_sent && seq != S.last_seq + 1) S.seq_ok = false;
    if (S.cmds_sent && S.t_ms - S.last_cmd_ms > S.max_gap_ms) S.max_gap_ms = S.t_ms - S.last_cmd_ms;
    S.last_cmd = *cmd;
    S.last_seq = seq;
    S.last_cmd_ms = S.t_ms;
    S.cmds_sent++;
    if (cmd->flags & CT_FCMD_REARM) S.m1_state = M1_ARMED;
}

/* truth = the ball the controller really meets; the controller is always set up
 * from the textbook (nominal) model, as it will be on the rig before fitting. */
static void sim_init_model(float target, const m3_plant_params_t *truth)
{
    m3_ctrl_cfg_t cfg;
    m3_ctrl_default_cfg(&cfg);
    m3_plant_params_t pp;
    m3_plant_default_params(&pp);
    cfg.hover_rpm = m3_plant_hover_rpm(&pp);
    cfg.launch_rpm = cfg.hover_rpm * 1.15f;
    m3_ctrl_init(&S.ctl, &cfg, 0xC0FFEE, 0);
    m3_ctrl_set_hold_target(&S.ctl, target);
    m3_plant_init(&S.plant, truth ? truth : &pp);
    S.t_ms = 0;
    S.frame = S.fs_seq = 0;
    S.m1_state = M1_OFF;
    S.cmds_sent = 0;
    S.max_gap_ms = 0;
    S.seq_ok = true;
    S.heights_on = true;
    S.bridged_only = false;
    S.fault_at_ms = 0;
    S.fs_on = true;
    S.m1_inject = 0;
    S.stuck_mm = 0.0f;
    S.warn_at_ms = 0;
    S.noise_mm = 0.0f;
    S.drop_pct = 0;
    S.rng = 2463534242u;
    S.dropped = 0;
    for (int i = 0; i < 64; i++) S.delayed_h[i] = 0.0f;
}

static void sim_init(float target) { sim_init_model(target, 0); }

static float urand(void) /* (0,1] */
{
    S.rng ^= S.rng << 13; S.rng ^= S.rng >> 17; S.rng ^= S.rng << 5;
    return ((S.rng >> 8) + 1u) / 16777216.0f;
}
static float gauss(void) { return sqrtf(-2.0f * logf(urand())) * cosf(6.2831853f * urand()); }

static void sim_step(void)
{
    ct_fan_cmd_t cmd;
    uint32_t seq;
    S.t_ms++;
    float rpm = (S.last_cmd.flags & CT_FCMD_ENABLE) ? S.last_cmd.bottom_rpm : 0.0f;
    m3_plant_step(&S.plant, rpm, 0.001f);
    if (S.stuck_mm > 0.0f) { S.plant.h_mm = S.stuck_mm; S.plant.v_mm_s = 0.0f; }
    S.delayed_h[S.t_ms % 64] = S.plant.h_mm;

    if (S.fs_on && S.t_ms % 10 == 0) { /* M1 fan_state 100 Hz */
        ct_fan_state_t fs = {(uint16_t)S.plant.rpm, 0, 0, S.m1_state, 0, S.last_seq};
        if (S.m1_state == M1_ARMED && (S.last_cmd.flags & CT_FCMD_ENABLE)) fs.state = M1_RUN;
        if (S.m1_inject) { fs.state = S.m1_inject; S.m1_state = S.m1_inject; fs.fault_code = 7; }
        m3_ctrl_on_fan_state(&S.ctl, 1, S.fs_seq++, &fs, S.t_ms);
    }
    /* camera 60 fps (frames at 16/17 ms), measurement 30 ms old */
    if (S.heights_on && (S.t_ms * 3) / 50 != ((S.t_ms - 1) * 3) / 50) {
        float h = S.delayed_h[(S.t_ms - 30) % 64] + S.noise_mm * gauss();
        ++S.frame;
        if (S.drop_pct && urand() * 100.0f < (float)S.drop_pct) { S.dropped++; goto tick; } /* frame lost */
        uint8_t flags = S.bridged_only ? (CT_HFLAG_FILT_VALID | CT_HFLAG_BRIDGED)
                                       : (CT_HFLAG_RAW_VALID | CT_HFLAG_FILT_VALID);
        ct_height_t m = {(int16_t)lrintf(h), (int16_t)lrintf(h), 0, flags, S.frame,
                         (uint64_t)(S.t_ms - 30) * 1000u, 600};
        if (m3_ctrl_on_height(&S.ctl, 2, &m, S.t_ms, &cmd, &seq)) on_cmd(&cmd, seq);
    }
tick:
    if (m3_ctrl_on_tick(&S.ctl, S.t_ms, &cmd, &seq)) on_cmd(&cmd, seq);
    if (S.ctl.fsm.state == ST_FAULT && S.fault_at_ms == 0 && S.t_ms > 1) S.fault_at_ms = S.t_ms;
    if (S.ctl.stuck.warning && S.warn_at_ms == 0) S.warn_at_ms = S.t_ms;
}

static void run_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) sim_step();
}

static void reset_and_start(void)
{
    run_ms(50);
    TEST_ASSERT_EQUAL(RES_PENDING, m3_ctrl_event(&S.ctl, EV_RESET, S.t_ms));
    run_ms(50);
    TEST_ASSERT_EQUAL(ST_IDLE, S.ctl.fsm.state);
    TEST_ASSERT_EQUAL(RES_OK, m3_ctrl_event(&S.ctl, EV_START, S.t_ms));
}

void test_R3_4_fan_cmd_every_10ms_with_new_sequence(void)
{
    sim_init(500);
    reset_and_start();
    run_ms(3000);
    S.heights_on = false; /* frames stopped: still sent every 10 ms */
    run_ms(1000);
    TEST_ASSERT_TRUE(S.seq_ok);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(10, S.max_gap_ms);
}

void test_R3_5_heights_stop_fault_within_200ms(void)
{
    sim_init(500);
    reset_and_start();
    run_ms(3000);
    TEST_ASSERT_EQUAL(ST_RUN, S.ctl.fsm.state);
    uint32_t last_real = S.ctl.hgate.last_real_ms;
    S.heights_on = false;
    S.fault_at_ms = 0;
    run_ms(400);
    TEST_ASSERT_EQUAL(ST_FAULT, S.ctl.fsm.state);
    TEST_ASSERT_EQUAL(CAUSE_NO_HEIGHT, S.ctl.fsm.cause);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(200, S.fault_at_ms - last_real);
    TEST_ASSERT_FALSE(S.last_cmd.flags & CT_FCMD_ENABLE);
    TEST_ASSERT_EQUAL_UINT16(0, S.last_cmd.bottom_rpm);
}

void test_R3_5b_only_bridged_heights_fault_within_200ms(void)
{
    sim_init(500);
    reset_and_start();
    run_ms(3000);
    uint32_t last_real = S.ctl.hgate.last_real_ms;
    S.bridged_only = true;
    S.fault_at_ms = 0;
    run_ms(400);
    TEST_ASSERT_EQUAL(ST_FAULT, S.ctl.fsm.state);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(200, S.fault_at_ms - last_real);
}

void test_R3_8_holds_simulated_ball(void)
{
    sim_init(500);
    reset_and_start();
    run_ms(6000); /* settle */
    double sum = 0, lo = 1e9, hi = -1e9;
    int n = 0;
    for (int i = 0; i < 20000; i++) {
        sim_step();
        double e = S.plant.h_mm - 500.0;
        sum += fabs(e);
        if (S.plant.h_mm < lo) lo = S.plant.h_mm;
        if (S.plant.h_mm > hi) hi = S.plant.h_mm;
        n++;
    }
    char msg[96];
    snprintf(msg, sizeof msg, "mean |e| = %.1f mm, peak-to-peak = %.1f mm", sum / n, hi - lo);
    TEST_MESSAGE(msg);
    TEST_ASSERT_EQUAL(ST_RUN, S.ctl.fsm.state);
    TEST_ASSERT_TRUE_MESSAGE(sum / n <= 20.0, msg);   /* mean error <= 2 cm */
    TEST_ASSERT_TRUE_MESSAGE(hi - lo <= 30.0, msg);   /* oscillation within 3 cm */
}

/* ---------- R3.11 (a): each fault 10 times -> fault within 100 ms ---------- */

typedef enum { INJ_M1_FAULT, INJ_ESTOP, INJ_FS_LOST } inj_t;

static void fault_10_times(inj_t kind, m3_cause_t want)
{
    sim_init(500);
    uint32_t worst = 0;
    for (int i = 0; i < 10; i++) {
        reset_and_start();
        run_ms(1500 + 7u * (uint32_t)i); /* vary the phase against the 10 ms periods */
        TEST_ASSERT_EQUAL_MESSAGE(ST_RUN, S.ctl.fsm.state, "not flying before the fault");
        uint32_t t0 = S.t_ms;
        S.fault_at_ms = 0;
        if (kind == INJ_M1_FAULT) S.m1_inject = M1_FAULT;
        else if (kind == INJ_ESTOP) S.m1_inject = M1_ESTOP;
        else S.fs_on = false;
        run_ms(300);
        TEST_ASSERT_EQUAL(ST_FAULT, S.ctl.fsm.state);
        TEST_ASSERT_EQUAL(want, S.ctl.fsm.cause);
        TEST_ASSERT_NOT_EQUAL(0, S.fault_at_ms);
        uint32_t lat = S.fault_at_ms - t0;
        if (lat > worst) worst = lat;
        TEST_ASSERT_LESS_OR_EQUAL_UINT32(100, lat);
        TEST_ASSERT_FALSE(S.last_cmd.flags & CT_FCMD_ENABLE); /* blowers commanded off */
        TEST_ASSERT_EQUAL_UINT16(0, S.last_cmd.bottom_rpm);

        /* data returning never leaves Fault by itself */
        S.m1_inject = 0;
        S.m1_state = M1_OFF;
        S.fs_on = true;
        run_ms(500);
        TEST_ASSERT_EQUAL(ST_FAULT, S.ctl.fsm.state);
    }
    char msg[64];
    snprintf(msg, sizeof msg, "worst fault latency %u ms over 10 runs", (unsigned)worst);
    TEST_MESSAGE(msg);
}

void test_R3_11_m1_fault_10_times_fault_within_100ms(void) { fault_10_times(INJ_M1_FAULT, CAUSE_M1_FAULT); }
void test_R3_11_estop_10_times_fault_within_100ms(void) { fault_10_times(INJ_ESTOP, CAUSE_ESTOP); }
void test_R3_11_fan_state_lost_10_times_fault_within_100ms(void)
{
    fault_10_times(INJ_FS_LOST, CAUSE_FAN_STATE_LOST);
}

/* ---------- R3.11 (b): ball held stuck, target 10 cm above ---------- */

void test_R3_11_stuck_ball_warning_within_3500ms_no_fault(void)
{
    sim_init(400);
    reset_and_start();
    run_ms(5000); /* holding at 400 */
    TEST_ASSERT_EQUAL(ST_RUN, S.ctl.fsm.state);
    TEST_ASSERT_FALSE(S.ctl.stuck.warning);

    S.stuck_mm = 300.0f; /* ball held 10 cm below the target */
    uint32_t t0 = S.t_ms, speed_at = 0;
    while (S.t_ms - t0 < 60000 && S.warn_at_ms == 0) {
        sim_step();
        if (!speed_at && S.ctl.n_wanted >= M3_STUCK_SPEED_FRAC * S.ctl.pid.cfg.n_max) speed_at = S.t_ms;
    }
    char msg[96];
    snprintf(msg, sizeof msg, "speed >= 90%% max after %u ms, warning after %u ms",
             (unsigned)(speed_at ? speed_at - t0 : 0), (unsigned)(S.warn_at_ms ? S.warn_at_ms - t0 : 0));
    TEST_MESSAGE(msg);
    TEST_ASSERT_NOT_EQUAL_MESSAGE(0, speed_at, msg);
    TEST_ASSERT_NOT_EQUAL_MESSAGE(0, S.warn_at_ms, msg);
    /* the 3 s window runs from the moment all conditions hold (speed near max) */
    TEST_ASSERT_LESS_OR_EQUAL_UINT32_MESSAGE(3500, S.warn_at_ms - speed_at, msg);
    TEST_ASSERT_EQUAL(ST_RUN, S.ctl.fsm.state); /* a warning, not a fault */
    run_ms(2000);
    TEST_ASSERT_EQUAL(ST_RUN, S.ctl.fsm.state);
    TEST_ASSERT_TRUE(S.ctl.stuck.warning);

    S.stuck_mm = 0.0f; /* released: ball rises, warning clears */
    run_ms(3000);
    TEST_ASSERT_FALSE(S.ctl.stuck.warning);
    TEST_ASSERT_EQUAL(ST_RUN, S.ctl.fsm.state);
}

/* ---------- R3.11 (c): steady hold near the top -> no warning ---------- */

void test_R3_11_steady_hold_near_top_no_warning(void)
{
    sim_init(850);
    reset_and_start();
    run_ms(30000);
    TEST_ASSERT_EQUAL(ST_RUN, S.ctl.fsm.state);
    TEST_ASSERT_EQUAL_UINT32(0, S.warn_at_ms);
}

/* ---------- R3.9: model coefficients +-30 %, noise 5 mm, 5 % dropped frames ---------- */

typedef struct { double mean_abs, p2p; int dropped; bool run; } hold_res_t;

static hold_res_t hold_bad_data(float target, const m3_plant_params_t *truth)
{
    sim_init_model(target, truth);
    S.noise_mm = 5.0f;
    S.drop_pct = 5;
    reset_and_start();
    run_ms(15000); /* launch + settle: with hover up to 70 % off the integral needs ~10 s */
    double sum = 0, lo = 1e9, hi = -1e9;
    int n = 0;
    for (int i = 0; i < 20000; i++, n++) { /* 20 s, true height (the simulator knows it) */
        sim_step();
        double h = S.plant.h_mm;
        sum += fabs(h - target);
        if (h < lo) lo = h;
        if (h > hi) hi = h;
    }
    hold_res_t r = {sum / n, hi - lo, S.dropped, S.ctl.fsm.state == ST_RUN};
    return r;
}

void test_R3_9_gains_robust_to_30pct_model_error_noise_and_drops(void)
{
    const float sc[3] = {0.7f, 1.0f, 1.3f};
    const float heights[3] = {300.0f, 500.0f, 750.0f};
    double worst_mean = 0, worst_p2p = 0;
    char where[160] = "";
    int cases = 0, failed = 0;
    for (int a = 0; a < 3; a++)
        for (int c = 0; c < 3; c++)
            for (int t = 0; t < 3; t++)
                for (int m = 1; m < 2; m++) { /* mass is weighed (2.7 g), not fitted: kept exact */
                    m3_plant_params_t pp;
                    m3_plant_default_params(&pp);
                    pp.k_air *= sc[a];
                    pp.c_drag *= sc[c];
                    pp.tau_s *= sc[t];
                    pp.mass_kg *= sc[m];
                    for (int h = 0; h < 3; h++) {
                        hold_res_t r = hold_bad_data(heights[h], &pp);
                        cases++;
                        char buf[160];
                        snprintf(buf, sizeof buf,
                                 "k_air x%.1f c_drag x%.1f tau x%.1f mass x%.1f at %.0f mm: mean %.1f mm, p2p %.1f mm%s",
                                 sc[a], sc[c], sc[t], sc[m], heights[h], r.mean_abs, r.p2p, r.run ? "" : " NOT RUNNING");
                        if (!r.run || r.mean_abs > 30.0 || r.p2p > 50.0) {
                            failed++;
                            printf("FAILCASE %s\n", buf);
                            continue;
                        }
                        if (r.mean_abs > worst_mean) worst_mean = r.mean_abs;
                        if (r.p2p > worst_p2p) { worst_p2p = r.p2p; snprintf(where, sizeof where, "%s", buf); }
                    }
                }
    char msg[256];
    snprintf(msg, sizeof msg, "%d cases, %d failed; worst passing mean %.1f mm; worst passing p2p: %s", cases,
             failed, worst_mean, where);
    TEST_MESSAGE(msg);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, failed, msg);
}

void test_R3_9_dropped_frames_really_dropped(void)
{
    hold_res_t r = hold_bad_data(500.0f, 0);
    /* 35 s at 60 fps = ~2100 frames; 5 % -> about 105 */
    TEST_ASSERT_INT_WITHIN(40, 105, r.dropped);
    TEST_ASSERT_TRUE(r.run);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_R3_4_fan_cmd_every_10ms_with_new_sequence);
    RUN_TEST(test_R3_5_heights_stop_fault_within_200ms);
    RUN_TEST(test_R3_5b_only_bridged_heights_fault_within_200ms);
    RUN_TEST(test_R3_8_holds_simulated_ball);
    RUN_TEST(test_R3_9_gains_robust_to_30pct_model_error_noise_and_drops);
    RUN_TEST(test_R3_9_dropped_frames_really_dropped);
    RUN_TEST(test_R3_11_m1_fault_10_times_fault_within_100ms);
    RUN_TEST(test_R3_11_estop_10_times_fault_within_100ms);
    RUN_TEST(test_R3_11_fan_state_lost_10_times_fault_within_100ms);
    RUN_TEST(test_R3_11_stuck_ball_warning_within_3500ms_no_fault);
    RUN_TEST(test_R3_11_steady_hold_near_top_no_warning);
    return UNITY_END();
}
