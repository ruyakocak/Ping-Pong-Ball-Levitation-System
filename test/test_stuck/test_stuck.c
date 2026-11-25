/* Stuck-ball detector (R3.11 b/c): each condition and the 3 s window on its own. */
#include <unity.h>

#include "m3_stuck.h"

void setUp(void) {}
void tearDown(void) {}

#define NMAX 5000.0f
#define HIGH (0.95f * NMAX)

/* Feeds 60 fps updates for ms milliseconds; returns the first time the warning was set (0 = never). */
static uint32_t feed(m3_stuck_t *s, uint32_t *now, uint32_t ms, bool run, float target, float ball,
                     float rise_mm_s, float n)
{
    uint32_t first = 0, end = *now + ms;
    for (; *now < end; *now += 17) {
        if (m3_stuck_update(s, run, target, ball, n, NMAX, *now) && !first) first = *now;
        ball += rise_mm_s * 0.017f;
    }
    return first;
}

void test_R3_11_stuck_warning_after_3s_when_all_conditions_hold(void)
{
    m3_stuck_t s;
    m3_stuck_init(&s);
    uint32_t now = 1000;
    uint32_t w = feed(&s, &now, 5000, true, 400, 300, 0, HIGH);
    TEST_ASSERT_NOT_EQUAL(0, w);
    TEST_ASSERT_UINT32_WITHIN(20, 1000 + M3_STUCK_WINDOW_MS, w); /* 3 s, not before */
}

void test_R3_11_no_warning_if_target_less_than_50mm_above(void)
{
    m3_stuck_t s;
    m3_stuck_init(&s);
    uint32_t now = 0;
    TEST_ASSERT_EQUAL_UINT32(0, feed(&s, &now, 6000, true, 349, 300, 0, HIGH));
}

void test_R3_11_no_warning_if_speed_below_90_percent(void)
{
    m3_stuck_t s;
    m3_stuck_init(&s);
    uint32_t now = 0;
    TEST_ASSERT_EQUAL_UINT32(0, feed(&s, &now, 6000, true, 400, 300, 0, 0.89f * NMAX));
}

void test_R3_11_no_warning_if_ball_rises_10mm_in_3s(void)
{
    m3_stuck_t s;
    m3_stuck_init(&s);
    uint32_t now = 0;
    /* slow climb, 4 mm/s = 12 mm per 3 s: not stuck. Target stays far enough above. */
    TEST_ASSERT_EQUAL_UINT32(0, feed(&s, &now, 9000, true, 800, 300, 4.0f, HIGH));
}

void test_R3_11_no_warning_outside_run(void)
{
    m3_stuck_t s;
    m3_stuck_init(&s);
    uint32_t now = 0;
    TEST_ASSERT_EQUAL_UINT32(0, feed(&s, &now, 6000, false, 400, 300, 0, HIGH));
}

void test_R3_11_holding_near_top_no_warning(void)
{
    m3_stuck_t s;
    m3_stuck_init(&s);
    uint32_t now = 0;
    /* ball on target near the top, blower near max: target is not above the ball */
    TEST_ASSERT_EQUAL_UINT32(0, feed(&s, &now, 10000, true, 850, 850, 0, HIGH));
}

void test_R3_11_warning_clears_when_condition_goes(void)
{
    m3_stuck_t s;
    m3_stuck_init(&s);
    uint32_t now = 0;
    TEST_ASSERT_NOT_EQUAL(0, feed(&s, &now, 4000, true, 400, 300, 0, HIGH));
    TEST_ASSERT_TRUE(s.warning);
    feed(&s, &now, 100, true, 400, 380, 0, HIGH); /* ball came up: target only 20 mm above */
    TEST_ASSERT_FALSE(s.warning);
}

void test_R3_11_short_interruption_restarts_window(void)
{
    m3_stuck_t s;
    m3_stuck_init(&s);
    uint32_t now = 0;
    TEST_ASSERT_EQUAL_UINT32(0, feed(&s, &now, 2500, true, 400, 300, 0, HIGH));
    feed(&s, &now, 50, true, 400, 300, 0, 0.5f * NMAX); /* speed dipped */
    uint32_t t = now;
    uint32_t w = feed(&s, &now, 4000, true, 400, 300, 0, HIGH);
    TEST_ASSERT_UINT32_WITHIN(20, t + M3_STUCK_WINDOW_MS, w);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_R3_11_stuck_warning_after_3s_when_all_conditions_hold);
    RUN_TEST(test_R3_11_no_warning_if_target_less_than_50mm_above);
    RUN_TEST(test_R3_11_no_warning_if_speed_below_90_percent);
    RUN_TEST(test_R3_11_no_warning_if_ball_rises_10mm_in_3s);
    RUN_TEST(test_R3_11_no_warning_outside_run);
    RUN_TEST(test_R3_11_holding_near_top_no_warning);
    RUN_TEST(test_R3_11_warning_clears_when_condition_goes);
    RUN_TEST(test_R3_11_short_interruption_restarts_window);
    return UNITY_END();
}
