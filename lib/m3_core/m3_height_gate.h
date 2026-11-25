/*
 * Height source selection and the 200 ms safety timer (Sections 2.6, 6.3).
 *
 * - Frames whose frame number is not newer than the last accepted one (same
 *   M2 session) are ignored completely: not used, never refresh the timer.
 * - A new M2 session id resets the frame tracking (numbers from 0 accepted);
 *   it does NOT change the FSM state.
 * - "New real measurement" = raw valid AND not bridged AND newer frame.
 *   Only that resets the safety timer.
 * - Control uses the filtered height while valid; bridged filtered heights at
 *   most M3_HG_MAX_BRIDGED in a row; otherwise the raw height.
 */
#ifndef M3_HEIGHT_GATE_H
#define M3_HEIGHT_GATE_H

#include <stdbool.h>
#include <stdint.h>

#include "m3_contract.h"

#ifdef __cplusplus
extern "C" {
#endif

#define M3_HG_TIMEOUT_MS 200u
#define M3_HG_MAX_BRIDGED 5u

typedef enum { HG_SRC_NONE = 0, HG_SRC_FILTERED, HG_SRC_RAW } m3_hg_src_t;

typedef struct {
    bool have_session;
    uint32_t session;
    bool have_frame;
    uint32_t last_frame;
    bool ever_real;
    uint32_t last_real_ms;
    uint8_t bridged_run;
    m3_hg_src_t last_src;
    bool have_capture;
    uint64_t last_capture_us;
} m3_hgate_t;

typedef struct {
    bool accepted;        /* false: old/duplicate frame, ignored entirely */
    bool new_session;
    bool real;            /* refreshed the safety timer */
    m3_hg_src_t src;      /* HG_SRC_NONE: nothing usable for control in this frame */
    float h_mm;
    float dt_s;           /* from capture times; <= 0 if unknown */
    bool src_switched;    /* filtered <-> raw: restart the derivative */
} m3_hg_out_t;

void m3_hgate_init(m3_hgate_t *g);
m3_hg_out_t m3_hgate_on_height(m3_hgate_t *g, uint32_t session, const ct_height_t *m, uint32_t now_ms);
/* True if no new real measurement for >= 200 ms (or never). */
bool m3_hgate_timed_out(const m3_hgate_t *g, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
#endif
