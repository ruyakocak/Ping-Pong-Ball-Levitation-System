#include "m3_height_gate.h"

#include <string.h>

void m3_hgate_init(m3_hgate_t *g) { memset(g, 0, sizeof(*g)); }

m3_hg_out_t m3_hgate_on_height(m3_hgate_t *g, uint32_t session, const ct_height_t *m, uint32_t now_ms)
{
    m3_hg_out_t o;
    memset(&o, 0, sizeof(o));

    if (!g->have_session || session != g->session) {
        g->have_session = true;
        g->session = session;
        g->have_frame = false;
        g->have_capture = false;
        g->bridged_run = 0;
        o.new_session = true;
    }
    if (g->have_frame && m->frame_no <= g->last_frame) return o; /* old or repeated */
    g->have_frame = true;
    g->last_frame = m->frame_no;
    o.accepted = true;

    const bool raw_valid = (m->flags & CT_HFLAG_RAW_VALID) != 0;
    const bool filt_valid = (m->flags & CT_HFLAG_FILT_VALID) != 0;
    const bool bridged = (m->flags & CT_HFLAG_BRIDGED) != 0;

    if (raw_valid && !bridged) {
        o.real = true;
        g->ever_real = true;
        g->last_real_ms = now_ms;
    }

    if (filt_valid && bridged) g->bridged_run++;
    else g->bridged_run = 0;

    if (filt_valid && (!bridged || g->bridged_run <= M3_HG_MAX_BRIDGED)) {
        o.src = HG_SRC_FILTERED;
        o.h_mm = (float)m->filt_mm;
    } else if (raw_valid && !bridged) {
        o.src = HG_SRC_RAW;
        o.h_mm = (float)m->raw_mm;
    } else {
        o.src = HG_SRC_NONE;
    }

    if (o.src != HG_SRC_NONE) {
        o.src_switched = g->last_src != HG_SRC_NONE && o.src != g->last_src;
        g->last_src = o.src;
    }

    o.dt_s = g->have_capture ? (float)((int64_t)(m->capture_us - g->last_capture_us)) * 1e-6f : -1.0f;
    g->have_capture = true;
    g->last_capture_us = m->capture_us;
    return o;
}

bool m3_hgate_timed_out(const m3_hgate_t *g, uint32_t now_ms)
{
    if (!g->ever_real) return true;
    return (uint32_t)(now_ms - g->last_real_ms) >= M3_HG_TIMEOUT_MS;
}
