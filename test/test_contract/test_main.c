/* Contract layer: CRC, framing, sequence tracking. Vectors identical to contract/test_vectors.json. */
#include <stdlib.h>
#include <string.h>
#include <unity.h>

#include "m3_contract.h"

void setUp(void) {}
void tearDown(void) {}

static size_t from_hex(const char *hex, uint8_t *out)
{
    size_t n = strlen(hex) / 2;
    for (size_t i = 0; i < n; i++) {
        char b[3] = {hex[2 * i], hex[2 * i + 1], 0};
        out[i] = (uint8_t)strtoul(b, NULL, 16);
    }
    return n;
}

static void assert_vector(const char *hex, uint8_t type, uint32_t session, uint32_t seq,
                          const uint8_t *payload, uint8_t len)
{
    uint8_t want[CT_MAX_FRAME], got[CT_MAX_FRAME];
    size_t wn = from_hex(hex, want);
    size_t gn = ct_encode_frame(got, type, session, seq, payload, len);
    TEST_ASSERT_EQUAL(wn, gn);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(want, got, wn);
}

void test_crc_check_value(void)
{
    TEST_ASSERT_EQUAL_HEX16(0x29B1, ct_crc16((const uint8_t *)"123456789", 9));
}

void test_vector_height(void)
{
    ct_height_t m = {512, 505, -120, CT_HFLAG_RAW_VALID | CT_HFLAG_FILT_VALID, 1234, 9876543210ull, 601};
    uint8_t p[CT_MAX_PAYLOAD];
    uint8_t n = ct_pack_height(p, &m);
    assert_vector("a55a011544332211070000000002f90188ff03d2040000ea16b04c020000005902aa36",
                  CT_HEIGHT, 0x11223344u, 7, p, n);
    ct_height_t back;
    TEST_ASSERT_TRUE(ct_unpack_height(p, n, &back));
    TEST_ASSERT_EQUAL_INT16(-120, back.vel_mm_s);
    TEST_ASSERT_TRUE(back.capture_us == 9876543210ull);
}

void test_vector_fan_cmd(void)
{
    ct_fan_cmd_t m = {3150, 2, CT_FCMD_ENABLE};
    uint8_t p[CT_MAX_PAYLOAD];
    uint8_t n = ct_pack_fan_cmd(p, &m);
    assert_vector("a55a1004efbeadde2a0000004e0c020115f5", CT_FAN_CMD, 0xDEADBEEFu, 42, p, n);
}

void test_vector_fan_state(void)
{
    ct_fan_state_t m = {3001, 512, 0, M1_ARMED, 0, 41};
    uint8_t p[CT_MAX_PAYLOAD];
    uint8_t n = ct_pack_fan_state(p, &m);
    assert_vector("a55a120c0403020163000000b90b000200000100290000009541", CT_FAN_STATE, 0x01020304u, 99, p, n);
}

void test_decoder_roundtrip_with_garbage_and_resync(void)
{
    uint8_t frame[CT_MAX_FRAME];
    size_t n = from_hex("a55a1004efbeadde2a0000004e0c020115f5", frame);
    uint8_t stream[64] = {0x00, 0xA5, 0x13, 0xA5};
    memcpy(&stream[4], frame, n);
    ct_decoder_t d;
    ct_decoder_init(&d);
    ct_frame_t f;
    int got = 0;
    for (size_t i = 0; i < 4 + n; i++)
        if (ct_decoder_feed(&d, stream[i], &f)) got++;
    TEST_ASSERT_EQUAL(1, got);
    TEST_ASSERT_EQUAL_UINT32(42, f.seq);
    TEST_ASSERT_EQUAL_UINT8(CT_FAN_CMD, f.type);
}

void test_corrupted_frames_rejected_and_counted(void)
{
    uint8_t frame[CT_MAX_FRAME];
    size_t n = from_hex("a55a120c0403020163000000b90b000200000100290000009541", frame);
    ct_decoder_t d;
    ct_decoder_init(&d);
    ct_frame_t f;
    srand(1);
    int accepted = 0;
    for (int k = 0; k < 1000; k++) {
        uint8_t bad[CT_MAX_FRAME];
        memcpy(bad, frame, n);
        size_t pos = 4 + (size_t)rand() % (n - 4); /* corrupt after the length byte */
        bad[pos] ^= (uint8_t)(1u << (rand() % 8));
        for (size_t i = 0; i < n; i++)
            if (ct_decoder_feed(&d, bad[i], &f)) accepted++;
    }
    TEST_ASSERT_EQUAL(0, accepted);
    TEST_ASSERT_EQUAL_UINT32(1000, d.crc_errors);
}

void test_sequence_tracking(void)
{
    ct_seq_track_t t = {0};
    TEST_ASSERT_EQUAL(CT_SEQ_NEW_SESSION, ct_seq_accept(&t, 5, 10));
    TEST_ASSERT_EQUAL(CT_SEQ_NEW, ct_seq_accept(&t, 5, 11));
    TEST_ASSERT_EQUAL(CT_SEQ_OLD, ct_seq_accept(&t, 5, 11)); /* repeated */
    TEST_ASSERT_EQUAL(CT_SEQ_OLD, ct_seq_accept(&t, 5, 3));  /* older */
    TEST_ASSERT_EQUAL(CT_SEQ_NEW_SESSION, ct_seq_accept(&t, 6, 0));
    TEST_ASSERT_EQUAL(CT_SEQ_NEW, ct_seq_accept(&t, 6, 1));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_crc_check_value);
    RUN_TEST(test_vector_height);
    RUN_TEST(test_vector_fan_cmd);
    RUN_TEST(test_vector_fan_state);
    RUN_TEST(test_decoder_roundtrip_with_garbage_and_resync);
    RUN_TEST(test_corrupted_frames_rejected_and_counted);
    RUN_TEST(test_sequence_tracking);
    return UNITY_END();
}
