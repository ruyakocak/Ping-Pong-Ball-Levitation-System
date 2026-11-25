#include "m3_target_buffer.h"

#include <string.h>

static void forget_session(m3_tbuf_t *b)
{
    b->n = 0;
    memset(b->bucket, 0, sizeof(b->bucket));
    b->have_offset = false;
    b->have_msg_t = false;
    b->playing = false;
    b->dry = false;
}

void m3_tbuf_init(m3_tbuf_t *b)
{
    memset(b, 0, sizeof(*b));
}

void m3_tbuf_set_hold(m3_tbuf_t *b, float mm)
{
    b->out.target_mm = mm;
    b->out.ff_rpm = 0.0f;
    b->out.top_level = 0;
    b->valid = true;
    b->n = 0;
    b->playing = false;
    b->dry = false;
}

bool m3_tbuf_valid(const m3_tbuf_t *b) { return b->valid; }

static void add_offset_sample(m3_tbuf_t *b, int64_t off, uint32_t now)
{
    uint32_t slot = now / M3_TBUF_BUCKET_MS;
    m3_tbuf_bucket_t *k = &b->bucket[slot % M3_TBUF_OFFSET_BUCKETS];
    if (!k->used || k->slot != slot) {
        k->used = true;
        k->slot = slot;
        k->min_off = off;
    } else if (off < k->min_off) {
        k->min_off = off;
    }
    if (!b->have_offset) {
        b->have_offset = true;
        b->first_sample_ms = now;
    }
}

bool m3_tbuf_offset(const m3_tbuf_t *b, uint32_t now, int64_t *off_ms)
{
    uint32_t slot = now / M3_TBUF_BUCKET_MS;
    bool found = false;
    int64_t best = 0;
    for (unsigned i = 0; i < M3_TBUF_OFFSET_BUCKETS; i++) {
        const m3_tbuf_bucket_t *k = &b->bucket[i];
        if (!k->used || (uint32_t)(slot - k->slot) >= M3_TBUF_OFFSET_BUCKETS) continue;
        if (!found || k->min_off < best) best = k->min_off;
        found = true;
    }
    if (found && off_ms) *off_ms = best;
    return found;
}

m3_tbuf_result_t m3_tbuf_push(m3_tbuf_t *b, uint32_t session, int64_t msg_t, const m3_tpoint_t *pts,
                              uint8_t n, uint32_t now)
{
    if (n == 0 || n > M3_TBUF_MAX_POINTS_PER_MSG) return TB_BAD;
    for (uint8_t i = 1; i < n; i++)
        if (pts[i].t_ms <= pts[i - 1].t_ms) return TB_BAD;

    m3_tbuf_result_t res = TB_ACCEPTED;
    if (!b->have_session || session != b->session) {
        if (b->have_session) res = TB_NEW_SESSION;
        forget_session(b);  /* the held output (b->out) stays */
        b->have_session = true;
        b->session = session;
        b->n_sessions++;
    }
    if (b->have_msg_t && msg_t <= b->last_msg_t) {
        b->n_old++;
        return TB_OLD;
    }
    b->have_msg_t = true;
    b->last_msg_t = msg_t;

    add_offset_sample(b, (int64_t)now - msg_t, now);

    /* Newer message wins where it overlaps: drop buffered points from its first time on. */
    uint8_t keep = 0;
    while (keep < b->n && b->pts[keep].t_ms < pts[0].t_ms) keep++;
    b->n = keep;
    for (uint8_t i = 0; i < n; i++) {
        if (b->n == M3_TBUF_CAP) { /* full: drop the oldest */
            memmove(&b->pts[0], &b->pts[1], (M3_TBUF_CAP - 1u) * sizeof(b->pts[0]));
            b->n--;
        }
        b->pts[b->n++] = pts[i];
    }

    if (b->dry) { /* stream resumed after running dry: blend in again */
        b->dry = false;
        b->playing = false;
    }
    return res;
}

static float lerp(float a, float b, float t) { return a + (b - a) * t; }

m3_target_t m3_tbuf_get(m3_tbuf_t *b, uint32_t now)
{
    int64_t off = 0;
    bool have_off = m3_tbuf_offset(b, now, &off);

    if (!b->playing) {
        bool settled = b->have_offset && (uint32_t)(now - b->first_sample_ms) >= M3_TBUF_SETTLE_MS;
        if (!(have_off && settled && b->n > 0)) return b->out; /* keep holding */
        b->playing = true;
        b->blend_start_ms = now;
        b->blend_from = b->out;
        if (!b->valid) b->blend_start_ms = now - M3_TBUF_BLEND_MS; /* nothing held: no blend */
    }
    if (b->n == 0 || !have_off) return b->out;

    int64_t s = (int64_t)now - off; /* now on the sender's clock */
    while (b->n >= 2 && b->pts[1].t_ms <= s) {
        memmove(&b->pts[0], &b->pts[1], (size_t)(b->n - 1u) * sizeof(b->pts[0]));
        b->n--;
    }

    m3_target_t tr;
    const m3_tpoint_t *p0 = &b->pts[0];
    if (b->n == 1 || s <= p0->t_ms) {
        tr.target_mm = p0->mm;
        tr.ff_rpm = p0->ff_rpm;
        tr.top_level = p0->top_level;
        if (b->n == 1 && s > p0->t_ms && !b->dry) {
            b->dry = true; /* targets stopped: hold the last one */
            b->n_dry++;
        }
    } else {
        const m3_tpoint_t *p1 = &b->pts[1];
        float a = (float)(s - p0->t_ms) / (float)(p1->t_ms - p0->t_ms);
        tr.target_mm = lerp(p0->mm, p1->mm, a);
        tr.ff_rpm = lerp(p0->ff_rpm, p1->ff_rpm, a);
        tr.top_level = a < 0.5f ? p0->top_level : p1->top_level;
    }

    uint32_t since = now - b->blend_start_ms;
    if (since < M3_TBUF_BLEND_MS) {
        float a = (float)since / (float)M3_TBUF_BLEND_MS;
        tr.target_mm = lerp(b->blend_from.target_mm, tr.target_mm, a);
        tr.ff_rpm = lerp(b->blend_from.ff_rpm, tr.ff_rpm, a);
    }
    b->out = tr;
    b->valid = true;
    return tr;
}
