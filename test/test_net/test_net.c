/* Network messages (JSON over UDP): command -> ack (R3.10), target_points (R3.7), status. */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "m3_net.h"

void setUp(void) {}
void tearDown(void) {}

static m3_ctrl_t c;
static m3_net_t net;
static char out[1400];

static void fresh(void)
{
    m3_ctrl_cfg_t cfg;
    m3_ctrl_default_cfg(&cfg);
    m3_ctrl_init(&c, &cfg, 99, 0);
    m3_net_init(&net);
}

/* Parses + applies one datagram; returns true if an ack was produced (formatted into out). */
static bool send(const char *json, m3_net_ack_t *ack)
{
    m3_net_msg_t m;
    m3_net_parse(json, strlen(json), &m);
    m3_net_ack_t a;
    bool r = m3_net_apply(&net, &c, &m, 1000, &a);
    if (r) {
        TEST_ASSERT_NOT_EQUAL(0, m3_net_format_ack(out, sizeof out, c.session, 1000, &a));
        if (ack) *ack = a;
    }
    return r;
}

#define CMD(id, rest) "{\"type\":\"command\",\"src\":\"m5\",\"session\":7,\"t\":5,\"request_id\":" #id "," rest "}"

void test_R3_10_net_set_parsed_applied_and_acked(void)
{
    fresh();
    m3_net_ack_t a;
    TEST_ASSERT_TRUE(send(CMD(12, "\"cmd\":\"set\",\"param\":\"kp\",\"value\":2.25"), &a));
    TEST_ASSERT_TRUE(a.a.ok);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.25f, c.pid.cfg.kp);
    TEST_ASSERT_NOT_NULL(strstr(out, "\"type\":\"ack\""));
    TEST_ASSERT_NOT_NULL(strstr(out, "\"request_id\":12"));
    TEST_ASSERT_NOT_NULL(strstr(out, "\"ok\":true"));
    TEST_ASSERT_NOT_NULL(strstr(out, "\"param\":\"kp\",\"applied\":2.2500"));
    TEST_ASSERT_NOT_NULL(strstr(out, "\"reason\":\"ok\""));
}

void test_R3_10_net_refusals_carry_reason(void)
{
    fresh();
    float kp = c.pid.cfg.kp;
    const struct { const char *json, *reason; } cases[] = {
        {CMD(1, "\"cmd\":\"set\",\"param\":\"kp\",\"value\":\"abc\""), "\"reason\":\"not_a_number\""},
        {CMD(2, "\"cmd\":\"set\",\"param\":\"kp\""), "\"reason\":\"not_a_number\""},
        {CMD(3, "\"cmd\":\"set\",\"param\":\"kp\",\"value\":999"), "\"reason\":\"out_of_range\""},
        {CMD(4, "\"cmd\":\"set\",\"param\":\"abs_max_rpm\",\"value\":7000"), "\"reason\":\"read_only\""},
        {CMD(5, "\"cmd\":\"set\",\"param\":\"nope\",\"value\":1"), "\"reason\":\"unknown_param\""},
        {CMD(6, "\"cmd\":\"fly\""), "\"reason\":\"unknown_command\""},
        {CMD(7, "\"cmd\":\"start\""), "\"reason\":\"wrong_state\""},   /* power-up Fault */
        {CMD(8, "\"cmd\":\"mode\",\"mode\":\"turbo\""), "\"reason\":\"out_of_range\""},
    };
    for (unsigned i = 0; i < sizeof cases / sizeof *cases; i++) {
        m3_net_ack_t a;
        TEST_ASSERT_TRUE_MESSAGE(send(cases[i].json, &a), cases[i].json);
        TEST_ASSERT_FALSE_MESSAGE(a.a.ok, cases[i].json);
        TEST_ASSERT_NOT_NULL_MESSAGE(strstr(out, cases[i].reason), out);
        TEST_ASSERT_NOT_NULL(strstr(out, "\"ok\":false"));
    }
    TEST_ASSERT_EQUAL_FLOAT(kp, c.pid.cfg.kp); /* nothing changed */
}

void test_R3_10_net_mode_command_changes_mode(void)
{
    fresh();
    m3_net_ack_t a;
    TEST_ASSERT_TRUE(send(CMD(1, "\"cmd\":\"mode\",\"mode\":\"game\""), &a));
    TEST_ASSERT_TRUE(a.a.ok);
    TEST_ASSERT_TRUE(c.game_mode);
    TEST_ASSERT_EQUAL_UINT8(MODE_GAME, c.mode);
    TEST_ASSERT_TRUE(send(CMD(2, "\"cmd\":\"mode\",\"mode\":\"wave\""), &a));
    TEST_ASSERT_FALSE(c.game_mode);
    TEST_ASSERT_EQUAL_UINT8(MODE_WAVE, c.mode);
}

void test_R3_10_net_repeated_request_answered_not_applied_twice(void)
{
    fresh();
    m3_net_ack_t a1, a2;
    TEST_ASSERT_TRUE(send(CMD(5, "\"cmd\":\"set\",\"param\":\"kp\",\"value\":2"), &a1));
    /* same request id again (resent after a lost ack), even with other content */
    TEST_ASSERT_TRUE(send(CMD(5, "\"cmd\":\"set\",\"param\":\"kp\",\"value\":3"), &a2));
    TEST_ASSERT_EQUAL_FLOAT(2.0f, c.pid.cfg.kp);
    TEST_ASSERT_TRUE(a2.a.ok);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, a2.a.applied);
    TEST_ASSERT_EQUAL_UINT32(1, net.n_repeated);
    TEST_ASSERT_TRUE(send(CMD(6, "\"cmd\":\"set\",\"param\":\"kp\",\"value\":3"), &a2));
    TEST_ASSERT_EQUAL_FLOAT(3.0f, c.pid.cfg.kp);
}

void test_R3_10_net_command_without_request_id_dropped(void)
{
    fresh();
    m3_net_msg_t m;
    const char *j = "{\"type\":\"command\",\"session\":7,\"t\":5,\"cmd\":\"stop\"}";
    TEST_ASSERT_EQUAL(NETMSG_BAD, m3_net_parse(j, strlen(j), &m));
    TEST_ASSERT_EQUAL_STRING("no_request_id", m.error);
    TEST_ASSERT_FALSE(send(j, NULL));
}

void test_R3_10_net_malformed_datagrams_never_trusted(void)
{
    fresh();
    const char *bad[] = {
        "", "null", "[]", "{", "{\"type\":\"command\"}", "{\"type\":5,\"session\":1,\"t\":1}",
        "{\"type\":\"command\",\"session\":-1,\"t\":1,\"request_id\":1}",
        "{\"type\":\"command\",\"session\":1.5,\"t\":1,\"request_id\":1}",
        "{\"type\":\"command\",\"session\":1,\"t\":1,\"request_id\":1}x",
        "{\"type\":\"command\",\"session\":1,\"t\":1,\"request_id\":1,}",
        "{\"type\":\"command\" \"session\":1}",
        "{\"type\":\"comm\\qand\",\"session\":1,\"t\":1,\"request_id\":1}",
        "{\"a\":[[[[[[[[[[[[1]]]]]]]]]]]],\"type\":\"command\",\"session\":1,\"t\":1,\"request_id\":1}",
        "{\"type\":\"target_points\",\"session\":1,\"t\":1,\"points\":[]}",
        "{\"type\":\"target_points\",\"session\":1,\"t\":1,\"points\":[{\"h\":500}]}",
        "{\"type\":\"target_points\",\"session\":1,\"t\":1,\"points\":[{\"t\":1,\"h\":\"x\"}]}",
        "{\"type\":\"target_points\",\"session\":1,\"t\":1,\"points\":[{\"t\":1,\"h\":1e999}]}",
    };
    for (unsigned i = 0; i < sizeof bad / sizeof *bad; i++) {
        m3_net_msg_t m;
        TEST_ASSERT_EQUAL_MESSAGE(NETMSG_BAD, m3_net_parse(bad[i], strlen(bad[i]), &m), bad[i]);
        TEST_ASSERT_NOT_NULL(m.error);
    }
    TEST_ASSERT_EQUAL_UINT32(0, net.n_commands);
}

void test_R3_10_net_every_truncation_and_random_bytes_rejected_safely(void)
{
    const char *full = CMD(9, "\"cmd\":\"set\",\"param\":\"kd\",\"value\":0.2");
    size_t n = strlen(full);
    m3_net_msg_t m;
    TEST_ASSERT_EQUAL(NETMSG_COMMAND, m3_net_parse(full, n, &m));
    for (size_t k = 0; k < n; k++) TEST_ASSERT_EQUAL(NETMSG_BAD, m3_net_parse(full, k, &m));

    uint32_t x = 12345;
    char buf[200];
    unsigned accepted = 0;
    for (int it = 0; it < 20000; it++) {
        size_t len = 1 + it % sizeof buf;
        memcpy(buf, full, n < len ? n : len);
        for (size_t i = 0; i < len; i++) {
            x ^= x << 13; x ^= x >> 17; x ^= x << 5;
            if (i >= n || (x & 7) == 0) buf[i] = (char)(x >> 9); /* corrupt ~1/8 of the bytes */
        }
        if (m3_net_parse(buf, len, &m) == NETMSG_COMMAND) accepted++;
    }
    TEST_ASSERT_LESS_THAN_UINT(200, accepted); /* corrupt input almost never parses... */
    /* ...and anything that does parse still goes through the range checks of R3.10 */
}

void test_R3_7_net_target_points_reach_the_buffer(void)
{
    fresh();
    const char *j = "{\"type\":\"target_points\",\"src\":\"m4\",\"session\":3,\"t\":100,\"points\":["
                    "{\"t\":100,\"h\":400,\"ff\":10},{\"t\":133,\"h\":410,\"top\":2},{\"t\":166,\"h\":420.5}]}";
    m3_net_msg_t m;
    TEST_ASSERT_EQUAL(NETMSG_TARGETS, m3_net_parse(j, strlen(j), &m));
    TEST_ASSERT_EQUAL_UINT8(3, m.n_points);
    TEST_ASSERT_EQUAL_INT64(133, m.pts[1].t_ms);
    TEST_ASSERT_EQUAL_FLOAT(420.5f, m.pts[2].mm);
    TEST_ASSERT_EQUAL_FLOAT(10.0f, m.pts[0].ff_rpm);
    TEST_ASSERT_EQUAL_UINT8(2, m.pts[1].top_level);
    m3_net_ack_t a;
    TEST_ASSERT_FALSE(m3_net_apply(&net, &c, &m, 1000, &a)); /* no ack for targets */
    TEST_ASSERT_EQUAL_UINT32(1, net.n_targets);
    TEST_ASSERT_EQUAL_UINT32(0, net.n_targets_refused);
    TEST_ASSERT_EQUAL_UINT32(1, c.tbuf.n_sessions);
}

void test_R3_7_net_target_points_ignored_in_game_mode_and_too_many_refused(void)
{
    fresh();
    TEST_ASSERT_TRUE(send(CMD(1, "\"cmd\":\"mode\",\"mode\":\"game\""), NULL));
    const char *j = "{\"type\":\"target_points\",\"session\":3,\"t\":100,\"points\":[{\"t\":100,\"h\":400}]}";
    TEST_ASSERT_FALSE(send(j, NULL));
    TEST_ASSERT_EQUAL_UINT32(1, net.n_targets_refused);

    char big[1400] = "{\"type\":\"target_points\",\"session\":3,\"t\":100,\"points\":[";
    for (unsigned i = 0; i <= M3_TBUF_MAX_POINTS_PER_MSG; i++)
        sprintf(big + strlen(big), "%s{\"t\":%u,\"h\":400}", i ? "," : "", 100 + 33 * i);
    strcat(big, "]}");
    m3_net_msg_t m;
    TEST_ASSERT_EQUAL(NETMSG_BAD, m3_net_parse(big, strlen(big), &m));
    TEST_ASSERT_EQUAL_STRING("too_many_points", m.error);
}

void test_R3_11_net_status_carries_state_fault_and_stuck_warning(void)
{
    fresh();
    c.stuck.warning = true;
    c.h_used_mm = 312.0f;
    m3_net_status_t st;
    m3_net_status_fill(&net, &c, &st);
    st.lat_max_us = 812;
    size_t n = m3_net_format_status(out, sizeof out, c.session, 4321, &st);
    TEST_ASSERT_NOT_EQUAL(0, n);
    TEST_ASSERT_EQUAL(strlen(out), n);
    TEST_ASSERT_NOT_NULL(strstr(out, "\"type\":\"status\",\"src\":\"m3\",\"session\":99,\"t\":4321"));
    TEST_ASSERT_NOT_NULL(strstr(out, "\"state\":\"fault\""));
    TEST_ASSERT_NOT_NULL(strstr(out, "\"fault\":\"power_up\""));
    TEST_ASSERT_NOT_NULL(strstr(out, "\"mode\":\"hold\""));
    TEST_ASSERT_NOT_NULL(strstr(out, "\"h_used_mm\":312.0"));
    TEST_ASSERT_NOT_NULL(strstr(out, "\"ball_stuck\":true"));
    TEST_ASSERT_NOT_NULL(strstr(out, "\"lat_max_us\":812"));
    TEST_ASSERT_EQUAL(0, m3_net_format_status(out, 50, c.session, 4321, &st)); /* never a cut-off JSON */
}

void test_R3_10_net_ack_escapes_echoed_text(void)
{
    fresh();
    TEST_ASSERT_TRUE(send(CMD(3, "\"cmd\":\"set\",\"param\":\"k\\\"p\",\"value\":1"), NULL));
    TEST_ASSERT_NOT_NULL_MESSAGE(strstr(out, "\"param\":\"k\\\"p\""), out);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_R3_10_net_set_parsed_applied_and_acked);
    RUN_TEST(test_R3_10_net_refusals_carry_reason);
    RUN_TEST(test_R3_10_net_mode_command_changes_mode);
    RUN_TEST(test_R3_10_net_repeated_request_answered_not_applied_twice);
    RUN_TEST(test_R3_10_net_command_without_request_id_dropped);
    RUN_TEST(test_R3_10_net_malformed_datagrams_never_trusted);
    RUN_TEST(test_R3_10_net_every_truncation_and_random_bytes_rejected_safely);
    RUN_TEST(test_R3_7_net_target_points_reach_the_buffer);
    RUN_TEST(test_R3_7_net_target_points_ignored_in_game_mode_and_too_many_refused);
    RUN_TEST(test_R3_11_net_status_carries_state_fault_and_stuck_warning);
    RUN_TEST(test_R3_10_net_ack_escapes_echoed_text);
    return UNITY_END();
}
