/* R3.1 — the PID computes exactly as designed (values computed by hand from Section 6.3). */
#include <math.h>
#include <unity.h>

#include "m3_pid.h"

void setUp(void) {}
void tearDown(void) {}

/* within 0.1 % (or 1e-3 absolute near zero) */
#define ASSERT_REL(exp, act) TEST_ASSERT_FLOAT_WITHIN(fmaxf(fabsf(exp) * 1e-3f, 1e-3f), (exp), (act))

static m3_pid_t c;

static void make(float kp, float ki, float kd)
{
    m3_pid_cfg_t k = {kp, ki, kd, 1000.0f, 0.0f, 10000.0f, 1e9f};
    m3_pid_init(&c, &k);
    m3_pid_reset(&c, 3000.0f);
}

void test_R3_1_proportional(void)
{
    make(2.0f, 0.0f, 0.0f);
    /* e = 500-400 = 100 -> u = 3000 + 200 */
    ASSERT_REL(3200.0f, m3_pid_step(&c, 500, 400, 0.0167f, 3000, 0));
}

void test_R3_1_integral_uneven_dt(void)
{
    make(0.0f, 2.0f, 0.0f);
    m3_pid_step(&c, 500, 400, 0.010f, 3000, 0); /* I = 2*100*0.010 = 2 */
    m3_pid_step(&c, 500, 450, 0.025f, 3000, 0); /* I = 2 + 2*50*0.025 = 4.5 */
    ASSERT_REL(4.5f, c.i);
    ASSERT_REL(3004.5f, c.n_out);
}

void test_R3_1_invalid_dt_no_integral_no_derivative(void)
{
    make(1.0f, 2.0f, 5.0f);
    m3_pid_step(&c, 500, 400, 0.016f, 3000, 0); /* I = 3.2 */
    float i_before = c.i;
    float n = m3_pid_step(&c, 500, 300, 0.150f, 3000, 0); /* gap > 100 ms */
    ASSERT_REL(i_before, c.i);
    ASSERT_REL(0.0f, c.d_term);
    ASSERT_REL(3000.0f + 200.0f + i_before, n);
    m3_pid_step(&c, 500, 300, 0.0f, 3000, 0);  /* dt = 0 */
    m3_pid_step(&c, 500, 300, -0.01f, 3000, 0); /* dt < 0 */
    ASSERT_REL(i_before, c.i);
}

void test_R3_1_derivative_on_measurement_no_target_kick(void)
{
    make(0.0f, 0.0f, 2.0f);
    m3_pid_step(&c, 500, 400, 0.020f, 3000, 0);
    /* target jumps 500 -> 800, height moves 400 -> 410 in 20 ms: D = -2*(10/0.02) = -1000 */
    float n = m3_pid_step(&c, 800, 410, 0.020f, 3000, 0);
    ASSERT_REL(-1000.0f, c.d_term);
    ASSERT_REL(2000.0f, n);
}

void test_R3_1_derivative_restart(void)
{
    make(0.0f, 0.0f, 2.0f);
    m3_pid_step(&c, 500, 400, 0.020f, 3000, 0);
    m3_pid_restart_derivative(&c);
    m3_pid_step(&c, 500, 450, 0.020f, 3000, 0);
    ASSERT_REL(0.0f, c.d_term);
}

void test_R3_1_integral_limit(void)
{
    make(0.0f, 100.0f, 0.0f);
    for (int i = 0; i < 50; i++) m3_pid_step(&c, 500, 0, 0.05f, 3000, 0);
    ASSERT_REL(1000.0f, c.i); /* I_max */
}

void test_R3_1_anti_windup_at_output_limit(void)
{
    m3_pid_cfg_t k = {1.0f, 10.0f, 0.0f, 5000.0f, 0.0f, 3500.0f, 1e9f};
    m3_pid_init(&c, &k);
    m3_pid_reset(&c, 3000.0f);
    /* e = 600 -> P alone gives 3600 > n_max: integral must not grow */
    for (int i = 0; i < 20; i++) m3_pid_step(&c, 600, 0, 0.02f, 3000, 0);
    ASSERT_REL(0.0f, c.i);
    ASSERT_REL(3500.0f, c.n_out);
    /* error reverses: integral may move again (downwards) */
    m3_pid_step(&c, 0, 100, 0.02f, 3000, 0);
    TEST_ASSERT_TRUE(c.i < 0.0f);
}

void test_R3_1_output_limits(void)
{
    m3_pid_cfg_t k = {50.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 4000.0f, 1e9f};
    m3_pid_init(&c, &k);
    m3_pid_reset(&c, 3000.0f);
    ASSERT_REL(4000.0f, m3_pid_step(&c, 900, 0, 0.016f, 3000, 0));
    ASSERT_REL(1000.0f, m3_pid_step(&c, 0, 900, 0.016f, 3000, 0));
}

void test_R3_1_rate_limit(void)
{
    m3_pid_cfg_t k = {50.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10000.0f, 6000.0f};
    m3_pid_init(&c, &k);
    m3_pid_reset(&c, 3000.0f);
    /* wants +5000, may move 6000 rpm/s * 0.02 s = 120 rpm */
    ASSERT_REL(3120.0f, m3_pid_step(&c, 500, 400, 0.02f, 3000, 0));
    ASSERT_REL(3240.0f, m3_pid_step(&c, 500, 400, 0.02f, 3000, 0));
    /* step time capped at 100 ms after a gap */
    ASSERT_REL(3840.0f, m3_pid_step(&c, 500, 400, 0.5f, 3000, 0));
}

void test_R3_1_feed_forward_added(void)
{
    make(1.0f, 0.0f, 0.0f);
    ASSERT_REL(3000.0f + 100.0f + 250.0f, m3_pid_step(&c, 500, 400, 0.016f, 3000, 250));
}

void test_R3_1_gain_change_without_jump(void)
{
    make(2.0f, 1.0f, 1.0f);
    m3_pid_step(&c, 500, 400, 0.02f, 3000, 0);
    m3_pid_step(&c, 500, 410, 0.02f, 3000, 0);
    float u_before = 3000.0f + c.cfg.kp * c.last_e + c.i - c.cfg.kd * c.last_dhdt;
    m3_pid_set_gains(&c, 5.0f, 3.0f, 0.5f);
    float u_after = 3000.0f + c.cfg.kp * c.last_e + c.i - c.cfg.kd * c.last_dhdt;
    ASSERT_REL(u_before, u_after);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_R3_1_proportional);
    RUN_TEST(test_R3_1_integral_uneven_dt);
    RUN_TEST(test_R3_1_invalid_dt_no_integral_no_derivative);
    RUN_TEST(test_R3_1_derivative_on_measurement_no_target_kick);
    RUN_TEST(test_R3_1_derivative_restart);
    RUN_TEST(test_R3_1_integral_limit);
    RUN_TEST(test_R3_1_anti_windup_at_output_limit);
    RUN_TEST(test_R3_1_output_limits);
    RUN_TEST(test_R3_1_rate_limit);
    RUN_TEST(test_R3_1_feed_forward_added);
    RUN_TEST(test_R3_1_gain_change_without_jump);
    return UNITY_END();
}
