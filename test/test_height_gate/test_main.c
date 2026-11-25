/* R3.5 / R3.6 — safety timer and height source selection. */
#include <unity.h>

#include "m3_height_gate.h"

void setUp(void) {}
void tearDown(void) {}

static m3_hgate_t g;

static ct_height_t h(uint32_t frame, uint8_t flags, int16_t raw, int16_t filt)
{
    ct_height_t m = {raw, filt, 0, flags, frame, (uint64_t)frame * 16667u, 600};
    return m;
}

#define REAL (CT_HFLAG_RAW_VALID | CT_HFLAG_FILT_VALID)
#define BRIDGED (CT_HFLAG_FILT_VALID | CT_HFLAG_BRIDGED)

void test_R3_5_timer_starts_timed_out(void)
{
    m3_hgate_init(&g);
    TEST_ASSERT_TRUE(m3_hgate_timed_out(&g, 0));
}

void test_R3_5_real_height_refreshes_timer_until_200ms(void)
{
    m3_hgate_init(&g);
    ct_height_t m = h(1, REAL, 500, 501);
    m3_hg_out_t o = m3_hgate_on_height(&g, 7, &m, 1000);
    TEST_ASSERT_TRUE(o.real);
    TEST_ASSERT_FALSE(m3_hgate_timed_out(&g, 1199));
    TEST_ASSERT_TRUE(m3_hgate_timed_out(&g, 1200));
}

void test_R3_5b_bridged_heights_never_refresh_timer(void)
{
    m3_hgate_init(&g);
    ct_height_t m = h(1, REAL, 500, 500);
    m3_hgate_on_height(&g, 7, &m, 0);
    for (uint32_t f = 2; f < 20; f++) {
        m = h(f, BRIDGED, 0, 500);
        m3_hg_out_t o = m3_hgate_on_height(&g, 7, &m, f * 16);
        TEST_ASSERT_FALSE(o.real);
    }
    TEST_ASSERT_TRUE(m3_hgate_timed_out(&g, 200));
}

void test_R3_5c_repeated_frame_number_ignored(void)
{
    m3_hgate_init(&g);
    ct_height_t m = h(5, REAL, 500, 500);
    m3_hgate_on_height(&g, 7, &m, 0);
    m3_hg_out_t o = m3_hgate_on_height(&g, 7, &m, 150); /* same frame again */
    TEST_ASSERT_FALSE(o.accepted);
    m = h(4, REAL, 500, 500); /* older */
    o = m3_hgate_on_height(&g, 7, &m, 160);
    TEST_ASSERT_FALSE(o.accepted);
    TEST_ASSERT_TRUE(m3_hgate_timed_out(&g, 200));
}

void test_R3_5d_new_session_accepts_numbers_from_zero(void)
{
    m3_hgate_init(&g);
    ct_height_t m = h(1000, REAL, 500, 500);
    m3_hgate_on_height(&g, 7, &m, 0);
    m = h(0, REAL, 500, 500);
    m3_hg_out_t o = m3_hgate_on_height(&g, 8, &m, 100);
    TEST_ASSERT_TRUE(o.accepted);
    TEST_ASSERT_TRUE(o.new_session);
    TEST_ASSERT_TRUE(o.real);
}

void test_R3_6_filtered_preferred_raw_fallback_with_switch_flag(void)
{
    m3_hgate_init(&g);
    ct_height_t m = h(1, REAL, 498, 500);
    m3_hg_out_t o = m3_hgate_on_height(&g, 7, &m, 0);
    TEST_ASSERT_EQUAL(HG_SRC_FILTERED, o.src);
    TEST_ASSERT_EQUAL_FLOAT(500.0f, o.h_mm);
    m = h(2, CT_HFLAG_RAW_VALID, 497, 0); /* filter invalid */
    o = m3_hgate_on_height(&g, 7, &m, 16);
    TEST_ASSERT_EQUAL(HG_SRC_RAW, o.src);
    TEST_ASSERT_EQUAL_FLOAT(497.0f, o.h_mm);
    TEST_ASSERT_TRUE(o.src_switched);
    TEST_ASSERT_TRUE(o.real);
}

void test_R3_6_only_five_bridged_in_a_row_used(void)
{
    m3_hgate_init(&g);
    ct_height_t m = h(1, REAL, 500, 500);
    m3_hgate_on_height(&g, 7, &m, 0);
    int used = 0;
    for (uint32_t f = 2; f < 12; f++) { /* 10 bridged */
        m = h(f, BRIDGED, 0, 500);
        if (m3_hgate_on_height(&g, 7, &m, f * 16).src != HG_SRC_NONE) used++;
    }
    TEST_ASSERT_EQUAL(5, used);
}

void test_dt_from_capture_times(void)
{
    m3_hgate_init(&g);
    ct_height_t m = h(1, REAL, 500, 500);
    m.capture_us = 1000000;
    TEST_ASSERT_TRUE(m3_hgate_on_height(&g, 7, &m, 0).dt_s <= 0.0f);
    m = h(2, REAL, 500, 500);
    m.capture_us = 1021000;
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.021f, m3_hgate_on_height(&g, 7, &m, 21).dt_s);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_R3_5_timer_starts_timed_out);
    RUN_TEST(test_R3_5_real_height_refreshes_timer_until_200ms);
    RUN_TEST(test_R3_5b_bridged_heights_never_refresh_timer);
    RUN_TEST(test_R3_5c_repeated_frame_number_ignored);
    RUN_TEST(test_R3_5d_new_session_accepts_numbers_from_zero);
    RUN_TEST(test_R3_6_filtered_preferred_raw_fallback_with_switch_flag);
    RUN_TEST(test_R3_6_only_five_bridged_in_a_row_used);
    RUN_TEST(test_dt_from_capture_times);
    return UNITY_END();
}
