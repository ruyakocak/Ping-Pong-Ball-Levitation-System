/*
 * Target buffer (R3.7): clock offset, on-time playback with network delay,
 * hold when targets stop, new M4 session, old messages, source selection.
 * The HIL form of T3.7 (real board, UDP delays) comes later; this is the logic.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unity.h>

#include "m3_ctrl.h"
#include "m3_target_buffer.h"

void setUp(void) {}
void tearDown(void) {}

static m3_tpoint_t P(int64_t t, float mm) { m3_tpoint_t p = {t, mm, 0.0f, 0}; return p; }

/* ---------- offset estimate ---------- */

void test_R3_7_offset_is_smallest_delay_in_window(void)
{
    m3_tbuf_t b;
    m3_tbuf_init(&b);
    const int64_t D = 777000; /* sender clock = local + D */
    const int delays[] = {50, 12, 80, 30};
    uint32_t now = 1000;
    for (int i = 0; i < 4; i++, now += 33) {
        m3_tpoint_t p = P(now + D + 200, 400);
        m3_tbuf_push(&b, 1, (int64_t)now - delays[i] + D, &p, 1, now);
    }
    int64_t off;
    TEST_ASSERT_TRUE(m3_tbuf_offset(&b, now, &off));
    TEST_ASSERT_EQUAL_INT64(-D + 12, off);

    /* the 12 ms sample leaves the ~10 s window; only 40 ms samples remain */
    for (; now < 13000; now += 33) {
        m3_tpoint_t p = P(now + D + 200, 400);
        m3_tbuf_push(&b, 1, (int64_t)now - 40 + D, &p, 1, now);
    }
    TEST_ASSERT_TRUE(m3_tbuf_offset(&b, now, &off));
    TEST_ASSERT_EQUAL_INT64(-D + 40, off);
}

/* ---------- linear interpolation ---------- */

void test_R3_7_points_interpolated_linearly(void)
{
    m3_tbuf_t b;
    m3_tbuf_init(&b);
    m3_tpoint_t pts[2] = {{1000, 100.0f, 50.0f, 1}, {1100, 200.0f, 150.0f, 3}};
    TEST_ASSERT_EQUAL(TB_ACCEPTED, m3_tbuf_push(&b, 7, 0, pts, 2, 0)); /* offset 0 */
    m3_target_t t = m3_tbuf_get(&b, 1050);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 150.0f, t.target_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 100.0f, t.ff_rpm);
    t = m3_tbuf_get(&b, 1075);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 175.0f, t.target_mm);
    TEST_ASSERT_EQUAL_UINT8(3, t.top_level);
}

/* ---------- T3.7: wave with random network delay up to 150 ms ---------- */

typedef struct { uint32_t arrive; int64_t send_t; } msg_t;

static uint32_t rng = 12345u;
static uint32_t rnd(uint32_t n) { rng = rng * 1664525u + 1013904223u; return (rng >> 8) % n; }

static int cmp_arrive(const void *a, const void *b)
{
    const msg_t *x = a, *y = b;
    return (x->arrive > y->arrive) - (x->arrive < y->arrive);
}

#define WAVE_D 123456     /* sender clock = local + WAVE_D */
static float wave(int64_t sender_t)
{
    return 400.0f + 100.0f * sinf(2.0f * 3.14159265f * 0.5f * (float)sender_t / 1000.0f);
}

void test_R3_7_wave_on_time_with_random_delay_up_to_150ms(void)
{
    enum { N_MSG = 600, AHEAD_MS = 200, STEP_MS = 20, T_END = 20000, T_CHECK = 3000 };
    static msg_t msgs[N_MSG];
    for (int i = 0; i < N_MSG; i++) {
        uint32_t send_local = (uint32_t)(i * 100 / 3); /* 30 Hz */
        msgs[i].send_t = (int64_t)send_local + WAVE_D;
        msgs[i].arrive = send_local + rnd(151);        /* 0..150 ms, reorders packets */
    }
    qsort(msgs, N_MSG, sizeof(msgs[0]), cmp_arrive);

    m3_tbuf_t b;
    m3_tbuf_init(&b);
    m3_tbuf_set_hold(&b, 400.0f);
    int next = 0, old = 0;
    float max_err = 0.0f;
    for (uint32_t now = 0; now < T_END; now++) {
        while (next < N_MSG && msgs[next].arrive <= now) {
            m3_tpoint_t pts[AHEAD_MS / STEP_MS + 1];
            uint8_t n = 0;
            for (int k = 0; k <= AHEAD_MS; k += STEP_MS, n++) {
                int64_t t = msgs[next].send_t + k;
                pts[n] = P(t, wave(t));
            }
            if (m3_tbuf_push(&b, 42, msgs[next].send_t, pts, n, now) == TB_OLD) old++;
            next++;
        }
        m3_target_t t = m3_tbuf_get(&b, now);
        if (now >= T_CHECK) {
            float e = fabsf(t.target_mm - wave((int64_t)now + WAVE_D));
            if (e > max_err) max_err = e;
        }
    }
    printf("wave: max target error %.2f mm, %d reordered packets ignored\n", max_err, old);
    TEST_ASSERT_TRUE(old > 0); /* the delay really reordered packets */
    TEST_ASSERT_TRUE(max_err <= 5.0f);
}

/* ---------- targets stop -> hold the last one ---------- */

void test_R3_7_targets_stop_last_target_held(void)
{
    m3_tbuf_t b;
    m3_tbuf_init(&b);
    uint32_t now = 0;
    for (; now < 2000; now++) {
        if (now % 33 == 0) {
            m3_tpoint_t p[2] = {P(now, 300.0f + now * 0.05f), P(now + 200, 300.0f + (now + 200) * 0.05f)};
            m3_tbuf_push(&b, 1, now, p, 2, now);
        }
        m3_tbuf_get(&b, now);
    }
    float last = 0;
    for (; now < 2400; now++) last = m3_tbuf_get(&b, now).target_mm;
    for (; now < 6000; now++) TEST_ASSERT_EQUAL_FLOAT(last, m3_tbuf_get(&b, now).target_mm);
    TEST_ASSERT_TRUE(m3_tbuf_valid(&b));
    TEST_ASSERT_EQUAL_UINT32(1, b.n_dry);
}

/* ---------- new M4 session: hold, re-estimate, blend smoothly ---------- */

static void feed_const(m3_tbuf_t *b, uint32_t session, int64_t D, float mm, uint32_t now)
{
    m3_tpoint_t p[2] = {P((int64_t)now + D, mm), P((int64_t)now + D + 200, mm)};
    m3_tbuf_push(b, session, (int64_t)now + D - 5, p, 2, now);
}

void test_R3_7_new_session_holds_then_moves_smoothly(void)
{
    m3_tbuf_t b;
    m3_tbuf_init(&b);
    uint32_t now = 0;
    for (; now < 5000; now++) {
        if (now % 33 == 0) feed_const(&b, 1, 1000000, 400.0f, now);
        m3_tbuf_get(&b, now);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 400.0f, m3_tbuf_get(&b, now).target_mm);

    /* M4 restarted: different session, different clock origin, new trajectory */
    float prev = 400.0f, max_step = 0.0f;
    uint32_t t_switch = now;
    bool new_seen = false;
    for (; now < 9000; now++) {
        if (now % 33 == 0) {
            if (m3_tbuf_push(&b, 2, (int64_t)now - 5, (m3_tpoint_t[]){P(now, 300), P(now + 200, 300)}, 2,
                             now) == TB_NEW_SESSION)
                new_seen = true;
        }
        float v = m3_tbuf_get(&b, now).target_mm;
        if (now < t_switch + M3_TBUF_SETTLE_MS) TEST_ASSERT_EQUAL_FLOAT(400.0f, v); /* holding */
        if (fabsf(v - prev) > max_step) max_step = fabsf(v - prev);
        prev = v;
    }
    TEST_ASSERT_TRUE(new_seen);
    TEST_ASSERT_TRUE(max_step <= 100.0f / M3_TBUF_BLEND_MS + 0.01f); /* no jump */
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 300.0f, prev);
}

void test_R3_7_resume_after_stop_blends_without_jump(void)
{
    m3_tbuf_t b;
    m3_tbuf_init(&b);
    uint32_t now = 0;
    for (; now < 2000; now++) {
        if (now % 33 == 0) feed_const(&b, 1, 0, 400.0f, now);
        m3_tbuf_get(&b, now);
    }
    for (; now < 3000; now++) m3_tbuf_get(&b, now); /* stream stopped, holding */
    float prev = 400.0f, max_step = 0.0f;
    for (; now < 6000; now++) {
        if (now % 33 == 0) feed_const(&b, 1, 0, 250.0f, now);
        float v = m3_tbuf_get(&b, now).target_mm;
        if (fabsf(v - prev) > max_step) max_step = fabsf(v - prev);
        prev = v;
    }
    TEST_ASSERT_TRUE(max_step <= 150.0f / M3_TBUF_BLEND_MS + 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 250.0f, prev);
}

/* ---------- old / bad messages ---------- */

void test_R3_7_old_or_repeated_message_ignored(void)
{
    m3_tbuf_t b;
    m3_tbuf_init(&b);
    m3_tpoint_t a[2] = {P(1000, 100), P(1200, 100)};
    m3_tpoint_t z[2] = {P(900, 999), P(1100, 999)};
    TEST_ASSERT_EQUAL(TB_ACCEPTED, m3_tbuf_push(&b, 1, 1000, a, 2, 1000));
    TEST_ASSERT_EQUAL(TB_OLD, m3_tbuf_push(&b, 1, 1000, a, 2, 1001)); /* repeated */
    TEST_ASSERT_EQUAL(TB_OLD, m3_tbuf_push(&b, 1, 900, z, 2, 1002));  /* older */
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 100.0f, m3_tbuf_get(&b, 1600).target_mm);
}

void test_R3_7_points_out_of_order_refused(void)
{
    m3_tbuf_t b;
    m3_tbuf_init(&b);
    m3_tpoint_t p[2] = {P(200, 1), P(100, 2)};
    TEST_ASSERT_EQUAL(TB_BAD, m3_tbuf_push(&b, 1, 0, p, 2, 0));
    TEST_ASSERT_EQUAL(TB_BAD, m3_tbuf_push(&b, 1, 0, p, 0, 0));
    TEST_ASSERT_FALSE(m3_tbuf_valid(&b));
}

/* ---------- source: game mode ignores M4 ---------- */

void test_R3_7_game_mode_ignores_m4_targets(void)
{
    m3_ctrl_cfg_t cfg;
    m3_ctrl_default_cfg(&cfg);
    static m3_ctrl_t c;
    m3_ctrl_init(&c, &cfg, 1, 0);
    m3_tpoint_t p[2] = {P(0, 300), P(200, 300)};
    c.game_mode = true;
    TEST_ASSERT_EQUAL(TB_IGNORED, m3_ctrl_on_target_points(&c, 5, 0, p, 2, 0));
    TEST_ASSERT_FALSE(m3_tbuf_valid(&c.tbuf));
    c.game_mode = false;
    TEST_ASSERT_EQUAL(TB_ACCEPTED, m3_ctrl_on_target_points(&c, 5, 0, p, 2, 0));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_R3_7_offset_is_smallest_delay_in_window);
    RUN_TEST(test_R3_7_points_interpolated_linearly);
    RUN_TEST(test_R3_7_wave_on_time_with_random_delay_up_to_150ms);
    RUN_TEST(test_R3_7_targets_stop_last_target_held);
    RUN_TEST(test_R3_7_new_session_holds_then_moves_smoothly);
    RUN_TEST(test_R3_7_resume_after_stop_blends_without_jump);
    RUN_TEST(test_R3_7_old_or_repeated_message_ignored);
    RUN_TEST(test_R3_7_points_out_of_order_refused);
    RUN_TEST(test_R3_7_game_mode_ignores_m4_targets);
    return UNITY_END();
}
