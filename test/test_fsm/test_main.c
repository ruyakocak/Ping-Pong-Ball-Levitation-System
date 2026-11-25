/* R3.2 — every row of the transition table and the reset sequence (Section 6.3). */
#include <unity.h>

#include "m3_fsm.h"

void setUp(void) {}
void tearDown(void) {}

static m3_fsm_t f;

static m3_cond_t healthy(void)
{
    m3_cond_t c = {true, true, true, false, false, true, false, 0.0f};
    return c;
}

/* power-up -> Fault -> Reset -> (armed) -> Idle */
static void to_idle(void)
{
    m3_fsm_init(&f, 0);
    m3_cond_t c = healthy();
    m3_fsm_event(&f, EV_RESET, &c, 10);
    m3_fsm_tick(&f, &c, 20);
    TEST_ASSERT_EQUAL(ST_IDLE, f.state);
}

static void to_run(void)
{
    to_idle();
    m3_cond_t c = healthy();
    TEST_ASSERT_EQUAL(RES_OK, m3_fsm_event(&f, EV_START, &c, 100));
    c.ball_mm = 40.0f;
    m3_fsm_tick(&f, &c, 200);
    TEST_ASSERT_EQUAL(ST_RUN, f.state);
}

void test_R3_2_power_up_is_fault(void)
{
    m3_fsm_init(&f, 0);
    TEST_ASSERT_EQUAL(ST_FAULT, f.state);
    TEST_ASSERT_EQUAL(CAUSE_POWER_UP, f.cause);
    TEST_ASSERT_FALSE(m3_fsm_output(&f).enable);
}

void test_R3_2_fault_refuses_start(void)
{
    m3_fsm_init(&f, 0);
    m3_cond_t c = healthy();
    TEST_ASSERT_EQUAL(RES_REFUSED_STATE, m3_fsm_event(&f, EV_START, &c, 5));
    TEST_ASSERT_EQUAL(ST_FAULT, f.state);
}

void test_R3_2_reset_sequence_waits_for_armed(void)
{
    m3_fsm_init(&f, 0);
    m3_cond_t c = healthy();
    c.m1_armed = false;
    TEST_ASSERT_EQUAL(RES_PENDING, m3_fsm_event(&f, EV_RESET, &c, 10));
    m3_fsm_out_t o = m3_fsm_output(&f);
    TEST_ASSERT_TRUE(o.rearm);
    TEST_ASSERT_FALSE(o.enable);
    m3_fsm_tick(&f, &c, 20);
    TEST_ASSERT_EQUAL(ST_FAULT, f.state); /* not armed yet */
    c.m1_armed = true;
    m3_fsm_tick(&f, &c, 30);
    TEST_ASSERT_EQUAL(ST_IDLE, f.state);
    TEST_ASSERT_FALSE(m3_fsm_output(&f).enable); /* only a separate Start enables */
}

void test_R3_2_reset_refused_while_cause_present(void)
{
    m3_fsm_init(&f, 0);
    m3_cond_t c = healthy();
    c.estop = true;
    TEST_ASSERT_EQUAL(RES_REFUSED_ESTOP, m3_fsm_event(&f, EV_RESET, &c, 10));
    c = healthy();
    c.height_fresh = false;
    TEST_ASSERT_EQUAL(RES_REFUSED_NO_HEIGHT, m3_fsm_event(&f, EV_RESET, &c, 10));
    c = healthy();
    c.fan_state_fresh = false;
    TEST_ASSERT_EQUAL(RES_REFUSED_NO_FAN_STATE, m3_fsm_event(&f, EV_RESET, &c, 10));
    TEST_ASSERT_EQUAL(ST_FAULT, f.state);
}

void test_R3_2_rearm_times_out(void)
{
    m3_fsm_init(&f, 0);
    m3_cond_t c = healthy();
    c.m1_armed = false;
    m3_fsm_event(&f, EV_RESET, &c, 0);
    m3_fsm_tick(&f, &c, M3_REARM_TIMEOUT_MS);
    TEST_ASSERT_FALSE(f.rearm_pending);
    c.m1_armed = true;
    m3_fsm_tick(&f, &c, M3_REARM_TIMEOUT_MS + 10);
    TEST_ASSERT_EQUAL(ST_FAULT, f.state); /* never left automatically */
}

void test_R3_2_start_refused_with_reason_when_condition_missing(void)
{
    m3_cond_t c;
    to_idle();
    c = healthy(); c.height_fresh = false;
    TEST_ASSERT_EQUAL(RES_REFUSED_NO_HEIGHT, m3_fsm_event(&f, EV_START, &c, 50));
    c = healthy(); c.m1_armed = false;
    TEST_ASSERT_EQUAL(RES_REFUSED_M1_NOT_ARMED, m3_fsm_event(&f, EV_START, &c, 50));
    c = healthy(); c.m1_fault = true;
    TEST_ASSERT_EQUAL(RES_REFUSED_M1_FAULT, m3_fsm_event(&f, EV_START, &c, 50));
    c = healthy(); c.estop = true;
    TEST_ASSERT_EQUAL(RES_REFUSED_ESTOP, m3_fsm_event(&f, EV_START, &c, 50));
    c = healthy(); c.target_valid = false;
    TEST_ASSERT_EQUAL(RES_REFUSED_NO_TARGET, m3_fsm_event(&f, EV_START, &c, 50));
    TEST_ASSERT_EQUAL(ST_IDLE, f.state);
}

void test_R3_2_launch_to_run_above_30mm(void)
{
    to_idle();
    m3_cond_t c = healthy();
    m3_fsm_event(&f, EV_START, &c, 100);
    TEST_ASSERT_EQUAL(ST_LAUNCH, f.state);
    TEST_ASSERT_TRUE(m3_fsm_output(&f).freeze_integral);
    f.clear_integral = false;
    c.ball_mm = 30.0f;
    m3_fsm_tick(&f, &c, 150);
    TEST_ASSERT_EQUAL(ST_LAUNCH, f.state);
    c.ball_mm = 31.0f;
    m3_fsm_tick(&f, &c, 160);
    TEST_ASSERT_EQUAL(ST_RUN, f.state);
    TEST_ASSERT_TRUE(f.clear_integral); /* integral starts from zero */
}

void test_R3_2_launch_without_lift_faults_after_1500ms(void)
{
    to_idle();
    m3_cond_t c = healthy();
    m3_fsm_event(&f, EV_START, &c, 100);
    m3_fsm_tick(&f, &c, 100 + 1499);
    TEST_ASSERT_EQUAL(ST_LAUNCH, f.state);
    m3_fsm_tick(&f, &c, 100 + 1500);
    TEST_ASSERT_EQUAL(ST_FAULT, f.state);
    TEST_ASSERT_EQUAL(CAUSE_LAUNCH_FAILED, f.cause);
}

void test_R3_2_stop_from_run_goes_idle(void)
{
    to_run();
    m3_cond_t c = healthy();
    TEST_ASSERT_EQUAL(RES_OK, m3_fsm_event(&f, EV_STOP, &c, 300));
    TEST_ASSERT_EQUAL(ST_IDLE, f.state);
    TEST_ASSERT_FALSE(m3_fsm_output(&f).enable);
}

void test_R3_2_game_hit_only_in_game_mode(void)
{
    to_run();
    m3_cond_t c = healthy();
    TEST_ASSERT_EQUAL(RES_REFUSED_NOT_GAME, m3_fsm_event(&f, EV_GAME_HIT, &c, 300));
    TEST_ASSERT_EQUAL(ST_RUN, f.state);
    c.game_mode = true;
    TEST_ASSERT_EQUAL(RES_OK, m3_fsm_event(&f, EV_GAME_HIT, &c, 310));
    TEST_ASSERT_EQUAL(ST_IDLE, f.state);
}

static void check_run_fault(m3_cond_t c, m3_cause_t expected)
{
    to_run();
    c.ball_mm = 400.0f;
    m3_fsm_tick(&f, &c, 400);
    TEST_ASSERT_EQUAL(ST_FAULT, f.state);
    TEST_ASSERT_EQUAL(expected, f.cause);
    TEST_ASSERT_FALSE(m3_fsm_output(&f).enable);
}

void test_R3_2_run_faults(void)
{
    m3_cond_t c;
    c = healthy(); c.height_fresh = false;   check_run_fault(c, CAUSE_NO_HEIGHT);
    c = healthy(); c.fan_state_fresh = false; check_run_fault(c, CAUSE_FAN_STATE_LOST);
    c = healthy(); c.m1_fault = true;         check_run_fault(c, CAUSE_M1_FAULT);
    c = healthy(); c.estop = true;            check_run_fault(c, CAUSE_ESTOP);
}

void test_R3_2_network_lost_keeps_run(void)
{
    to_run();
    m3_cond_t c = healthy(); /* targets stopped: buffer still valid (last target held) */
    c.ball_mm = 400.0f;
    m3_fsm_tick(&f, &c, 5000);
    TEST_ASSERT_EQUAL(ST_RUN, f.state);
}

void test_R3_2_fault_not_left_by_returning_data_or_start_or_stop(void)
{
    to_run();
    m3_cond_t c = healthy();
    c.height_fresh = false;
    m3_fsm_tick(&f, &c, 400);
    c = healthy();
    m3_fsm_tick(&f, &c, 500); /* data returns */
    TEST_ASSERT_EQUAL(ST_FAULT, f.state);
    TEST_ASSERT_EQUAL(RES_REFUSED_STATE, m3_fsm_event(&f, EV_START, &c, 510));
    TEST_ASSERT_EQUAL(RES_REFUSED_STATE, m3_fsm_event(&f, EV_STOP, &c, 520));
    TEST_ASSERT_EQUAL(ST_FAULT, f.state);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_R3_2_power_up_is_fault);
    RUN_TEST(test_R3_2_fault_refuses_start);
    RUN_TEST(test_R3_2_reset_sequence_waits_for_armed);
    RUN_TEST(test_R3_2_reset_refused_while_cause_present);
    RUN_TEST(test_R3_2_rearm_times_out);
    RUN_TEST(test_R3_2_start_refused_with_reason_when_condition_missing);
    RUN_TEST(test_R3_2_launch_to_run_above_30mm);
    RUN_TEST(test_R3_2_launch_without_lift_faults_after_1500ms);
    RUN_TEST(test_R3_2_stop_from_run_goes_idle);
    RUN_TEST(test_R3_2_game_hit_only_in_game_mode);
    RUN_TEST(test_R3_2_run_faults);
    RUN_TEST(test_R3_2_network_lost_keeps_run);
    RUN_TEST(test_R3_2_fault_not_left_by_returning_data_or_start_or_stop);
    return UNITY_END();
}
