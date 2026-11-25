/*
 * R3.10: commands are checked, safety limits cannot be changed, game messages
 * act only in game mode. T3.10: out-of-range values, safety limits, game_out
 * outside game mode -> refused with a reason, or ignored. A refused command
 * must leave every setting exactly as it was.
 */
#include <math.h>
#include <string.h>
#include <unity.h>

#include "m3_command.h"

static m3_ctrl_t C;
static uint32_t req;

void setUp(void)
{
    m3_ctrl_cfg_t cfg;
    m3_ctrl_default_cfg(&cfg);
    m3_ctrl_init(&C, &cfg, 1, 0);
    req = 100;
}
void tearDown(void) {}

static const char *ALL[] = {"kp", "ki", "kd", "hover_rpm", "target_mm", "rate_rpm_s", "n_min_rpm",
                            "n_max_rpm", "obstacle_speed", "height_timeout_ms", "fan_state_timeout_ms",
                            "abs_max_rpm", "max_power_pm", "launch_push_ms"};
#define N_ALL (sizeof(ALL) / sizeof(ALL[0]))

typedef struct { float v[N_ALL]; } snap_t;
static snap_t snap(void)
{
    snap_t s;
    for (unsigned i = 0; i < N_ALL; i++) TEST_ASSERT_TRUE_MESSAGE(m3_param_get(&C, ALL[i], &s.v[i]), ALL[i]);
    return s;
}
static void assert_unchanged(const snap_t *a, const char *what)
{
    snap_t b = snap();
    for (unsigned i = 0; i < N_ALL; i++)
        if (!(a->v[i] == b.v[i] || (isnan(a->v[i]) && isnan(b.v[i])))) TEST_FAIL_MESSAGE(what);
}

static m3_ack_t set(const char *p, float v, uint32_t now)
{
    m3_command_t c = {CMDK_SET, ++req, 0, p, v};
    return m3_command_apply(&C, &c, now);
}
static m3_ack_t cmd(m3_cmd_kind_t k, int mode)
{
    m3_command_t c = {k, ++req, mode, 0, 0};
    return m3_command_apply(&C, &c, 0);
}

/* ---------- parameter checks ---------- */

void test_R3_10_valid_change_applied_and_acked(void)
{
    m3_ack_t a = set("kp", 5.0f, 0);
    TEST_ASSERT_TRUE(a.ok);
    TEST_ASSERT_EQUAL_STRING("ok", a.reason);
    TEST_ASSERT_EQUAL_UINT32(req, a.request_id);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, a.applied);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, C.pid.cfg.kp);
    a = set("hover_rpm", 3200.0f, 0);
    TEST_ASSERT_TRUE(a.ok);
    TEST_ASSERT_EQUAL_FLOAT(3200.0f, C.cfg.hover_rpm);
}

void test_R3_10_out_of_range_refused_with_reason_nothing_changes(void)
{
    const char *w[] = {"kp", "ki", "kd", "hover_rpm", "target_mm", "rate_rpm_s", "n_min_rpm", "n_max_rpm",
                       "obstacle_speed"};
    for (unsigned i = 0; i < sizeof(w) / sizeof(w[0]); i++) {
        const m3_param_def_t *d = m3_param_find(w[i]);
        TEST_ASSERT_NOT_NULL(d);
        const float bad[] = {d->min - 1.0f, d->max + 1.0f, -1e9f, 1e9f};
        for (unsigned k = 0; k < 4; k++) {
            snap_t s = snap();
            m3_ack_t a = set(w[i], bad[k], 0);
            TEST_ASSERT_FALSE_MESSAGE(a.ok, w[i]);
            TEST_ASSERT_EQUAL_STRING_MESSAGE("out_of_range", a.reason, w[i]);
            float now_v;
            m3_param_get(&C, w[i], &now_v);
            TEST_ASSERT_EQUAL_FLOAT(now_v, a.applied); /* ack reports the value still in use */
            assert_unchanged(&s, w[i]);
        }
    }
}

void test_R3_10_not_a_number_refused(void)
{
    snap_t s = snap();
    TEST_ASSERT_EQUAL_STRING("not_a_number", set("kp", NAN, 0).reason);
    TEST_ASSERT_EQUAL_STRING("not_a_number", set("hover_rpm", INFINITY, 0).reason);
    TEST_ASSERT_EQUAL_STRING("not_a_number", set("target_mm", -INFINITY, 0).reason);
    assert_unchanged(&s, "nan");
}

void test_R3_10_safety_limits_cannot_be_changed(void)
{
    const char *ro[] = {"height_timeout_ms", "fan_state_timeout_ms", "abs_max_rpm", "max_power_pm",
                        "launch_push_ms"};
    for (unsigned i = 0; i < sizeof(ro) / sizeof(ro[0]); i++) {
        float v;
        TEST_ASSERT_TRUE(m3_param_get(&C, ro[i], &v)); /* readable */
        const float tries[] = {v, v * 2.0f, 0.0f, 1e6f, NAN};
        for (unsigned k = 0; k < 5; k++) {
            snap_t s = snap();
            m3_ack_t a = set(ro[i], tries[k], 0);
            TEST_ASSERT_FALSE_MESSAGE(a.ok, ro[i]);
            TEST_ASSERT_EQUAL_STRING_MESSAGE("read_only", a.reason, ro[i]);
            assert_unchanged(&s, ro[i]);
        }
    }
    /* the fixed values the rest of the firmware relies on */
    float v;
    m3_param_get(&C, "height_timeout_ms", &v);
    TEST_ASSERT_EQUAL_FLOAT(200.0f, v);
}

void test_R3_10_unknown_parameter_or_command_refused(void)
{
    snap_t s = snap();
    TEST_ASSERT_EQUAL_STRING("unknown_param", set("kP", 1.0f, 0).reason);
    TEST_ASSERT_EQUAL_STRING("unknown_param", set("", 1.0f, 0).reason);
    TEST_ASSERT_EQUAL_STRING("unknown_param", set(0, 1.0f, 0).reason);
    m3_command_t c = {(m3_cmd_kind_t)99, ++req, 0, 0, 0};
    m3_ack_t a = m3_command_apply(&C, &c, 0);
    TEST_ASSERT_FALSE(a.ok);
    TEST_ASSERT_EQUAL_STRING("unknown_command", a.reason);
    assert_unchanged(&s, "unknown");
}

void test_R3_10_inconsistent_limits_refused(void)
{
    snap_t s = snap(); /* defaults: n 0..5800, hover 3000 */
    TEST_ASSERT_EQUAL_FLOAT(5800.0f, C.pid.cfg.n_max);
    TEST_ASSERT_EQUAL_STRING("conflict", set("n_max_rpm", 2500.0f, 0).reason);  /* below hover */
    TEST_ASSERT_EQUAL_STRING("conflict", set("n_min_rpm", 3000.0f, 0).reason);  /* = hover */
    TEST_ASSERT_EQUAL_STRING("conflict", set("hover_rpm", 5900.0f, 0).reason);  /* above n_max */
    assert_unchanged(&s, "conflict");
    TEST_ASSERT_TRUE(set("n_max_rpm", 5950.0f, 0).ok);
    TEST_ASSERT_TRUE(set("hover_rpm", 5900.0f, 0).ok);
}

void test_R3_10_target_outside_usable_range_refused(void)
{
    snap_t s = snap();
    TEST_ASSERT_EQUAL_STRING("out_of_range", set("target_mm", C.cfg.range_lo_mm - 1.0f, 0).reason);
    TEST_ASSERT_EQUAL_STRING("out_of_range", set("target_mm", C.cfg.range_hi_mm + 1.0f, 0).reason);
    assert_unchanged(&s, "range");
    TEST_ASSERT_TRUE(set("target_mm", 400.0f, 0).ok);
    TEST_ASSERT_EQUAL_FLOAT(400.0f, C.tbuf.out.target_mm);
}

void test_R3_10_twenty_bad_commands_change_nothing(void)
{
    struct { const char *p; float v; } bad[20] = {
        {"kp", -0.1f}, {"kp", 201}, {"ki", -5}, {"ki", 1e4f}, {"kd", 51},
        {"kd", NAN}, {"hover_rpm", 0}, {"hover_rpm", 9000}, {"target_mm", 0}, {"target_mm", 2000},
        {"rate_rpm_s", 0}, {"rate_rpm_s", 1e6f}, {"n_min_rpm", -1}, {"n_max_rpm", 7000}, {"n_max_rpm", 900},
        {"obstacle_speed", 0}, {"obstacle_speed", 11}, {"abs_max_rpm", 9000}, {"max_power_pm", 1000},
        {"height_timeout_ms", 5000}};
    snap_t s = snap();
    for (int i = 0; i < 20; i++) {
        m3_ack_t a = set(bad[i].p, bad[i].v, 0);
        TEST_ASSERT_FALSE_MESSAGE(a.ok, bad[i].p);
        TEST_ASSERT_TRUE(a.reason && a.reason[0] && strcmp(a.reason, "ok") != 0);
    }
    assert_unchanged(&s, "bad batch");
}

/* ---------- state commands carry the reason ---------- */

void test_R3_10_refused_start_has_reason(void)
{
    m3_ack_t a = cmd(CMDK_START, 0); /* power-up state is Fault */
    TEST_ASSERT_FALSE(a.ok);
    TEST_ASSERT_EQUAL_STRING("wrong_state", a.reason);
    TEST_ASSERT_EQUAL(ST_FAULT, C.fsm.state);
}

void test_R3_10_mode_change_refused_while_flying(void)
{
    TEST_ASSERT_TRUE(cmd(CMDK_MODE, MODE_GAME).ok); /* allowed in Fault / Idle */
    TEST_ASSERT_TRUE(C.game_mode);
    TEST_ASSERT_TRUE(cmd(CMDK_MODE, MODE_HOLD).ok);
    C.fsm.state = ST_RUN;
    m3_ack_t a = cmd(CMDK_MODE, MODE_GAME);
    TEST_ASSERT_FALSE(a.ok);
    TEST_ASSERT_EQUAL_STRING("wrong_state", a.reason);
    TEST_ASSERT_FALSE(C.game_mode);
    C.fsm.state = ST_IDLE;
    TEST_ASSERT_EQUAL_STRING("out_of_range", cmd(CMDK_MODE, 7).reason);
    TEST_ASSERT_EQUAL_STRING("out_of_range", cmd(CMDK_MODE, -1).reason);
}

/* ---------- game messages only in game mode ---------- */

void test_R3_10_game_out_outside_game_mode_ignored(void)
{
    TEST_ASSERT_TRUE(set("target_mm", 400.0f, 0).ok);
    C.fsm.state = ST_RUN;
    ct_game_out_t g = {7, 1, 2, 55}; /* top level + hit */
    TEST_ASSERT_EQUAL(GAME_IGNORED_NOT_GAME, m3_command_game_out(&C, 9, 1, &g, 0));
    TEST_ASSERT_EQUAL(ST_RUN, C.fsm.state);                  /* hit did not stop the ball */
    TEST_ASSERT_EQUAL_FLOAT(400.0f, C.tbuf.out.target_mm);   /* target unchanged */
    TEST_ASSERT_EQUAL_UINT16(0, C.game_score);
    TEST_ASSERT_EQUAL_UINT32(1, C.n_game_ignored);
}

void test_R3_10_game_out_in_game_mode_sets_target_and_hit_stops(void)
{
    TEST_ASSERT_TRUE(cmd(CMDK_MODE, MODE_GAME).ok);
    ct_game_out_t g = {0, 0, 1, 10};
    TEST_ASSERT_EQUAL(GAME_APPLIED, m3_command_game_out(&C, 9, 1, &g, 0));
    TEST_ASSERT_EQUAL_FLOAT(C.cfg.range_lo_mm, C.tbuf.out.target_mm);
    g.target_level = 7;
    TEST_ASSERT_EQUAL(GAME_APPLIED, m3_command_game_out(&C, 9, 2, &g, 0));
    TEST_ASSERT_EQUAL_FLOAT(C.cfg.range_hi_mm, C.tbuf.out.target_mm);
    TEST_ASSERT_EQUAL_UINT16(10, C.game_score);

    g.target_level = 8;
    TEST_ASSERT_EQUAL(GAME_BAD_LEVEL, m3_command_game_out(&C, 9, 3, &g, 0));
    TEST_ASSERT_EQUAL_FLOAT(C.cfg.range_hi_mm, C.tbuf.out.target_mm);

    g.target_level = 3;
    TEST_ASSERT_EQUAL(GAME_OLD, m3_command_game_out(&C, 9, 2, &g, 0)); /* repeated sequence */
    TEST_ASSERT_EQUAL_FLOAT(C.cfg.range_hi_mm, C.tbuf.out.target_mm);

    /* operator target and M4 targets are not the source in game mode */
    TEST_ASSERT_EQUAL_STRING("wrong_mode", set("target_mm", 400.0f, 0).reason);
    m3_tpoint_t p[2] = {{0, 300, 0, 0}, {200, 300, 0, 0}};
    TEST_ASSERT_EQUAL(TB_IGNORED, m3_ctrl_on_target_points(&C, 5, 0, p, 2, 0));

    C.fsm.state = ST_RUN;
    g.hit = 1;
    TEST_ASSERT_EQUAL(GAME_APPLIED, m3_command_game_out(&C, 9, 4, &g, 0));
    TEST_ASSERT_EQUAL(ST_IDLE, C.fsm.state); /* a hit acts like Stop */
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_R3_10_valid_change_applied_and_acked);
    RUN_TEST(test_R3_10_out_of_range_refused_with_reason_nothing_changes);
    RUN_TEST(test_R3_10_not_a_number_refused);
    RUN_TEST(test_R3_10_safety_limits_cannot_be_changed);
    RUN_TEST(test_R3_10_unknown_parameter_or_command_refused);
    RUN_TEST(test_R3_10_inconsistent_limits_refused);
    RUN_TEST(test_R3_10_target_outside_usable_range_refused);
    RUN_TEST(test_R3_10_twenty_bad_commands_change_nothing);
    RUN_TEST(test_R3_10_refused_start_has_reason);
    RUN_TEST(test_R3_10_mode_change_refused_while_flying);
    RUN_TEST(test_R3_10_game_out_outside_game_mode_ignored);
    RUN_TEST(test_R3_10_game_out_in_game_mode_sets_target_and_hit_stops);
    return UNITY_END();
}
