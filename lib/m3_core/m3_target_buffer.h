/*
 * Target buffer (Section 6.3, "Targets"; R3.7).
 *
 * target_points from M4 carry the trajectory program's clock. M3 never compares
 * clocks directly; it estimates
 *     offset = min(local receive time - sender time)   over the last ~10 s
 * (the fastest packet had the least network delay). A point with sender time t
 * is played at local time t + offset, interpolated linearly between points.
 * Because M4 sends points 200 ms ahead, a packet that is late by up to that much
 * does not disturb the target.
 *
 *  - targets stop          -> the last target is held
 *  - new M4 session id     -> buffer and offset cleared, last target held; once
 *                             the offset has been estimated again (SETTLE_MS) the
 *                             output blends into the new trajectory (BLEND_MS)
 *  - stream resumes after running dry -> same blend, so there is no jump
 *  - older or repeated message (sender time not newer) -> ignored
 *
 * Times: local = this board's monotonic ms (uint32, wraps after 49 days);
 * sender = M4's clock in ms (int64, any origin).
 * Nothing here touches hardware; time always comes in as a parameter.
 */
#ifndef M3_TARGET_BUFFER_H
#define M3_TARGET_BUFFER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define M3_TBUF_CAP 32u
#define M3_TBUF_MAX_POINTS_PER_MSG 16u
#define M3_TBUF_OFFSET_BUCKETS 10u      /* 10 x 1 s -> offset window ~10 s */
#define M3_TBUF_BUCKET_MS 1000u
#define M3_TBUF_SETTLE_MS 500u          /* offset samples needed before playing */
#define M3_TBUF_BLEND_MS 1000u          /* hold -> trajectory blend time */

typedef struct {
    int64_t t_ms;       /* sender clock */
    float mm;           /* target height */
    float ff_rpm;       /* bottom feed-forward, extra rpm above hover */
    uint8_t top_level;  /* top-blower assist level */
} m3_tpoint_t;

typedef struct {
    float target_mm;
    float ff_rpm;
    uint8_t top_level;
} m3_target_t;

typedef enum {
    TB_ACCEPTED = 0,
    TB_NEW_SESSION,     /* accepted, previous session forgotten */
    TB_OLD,             /* sender time not newer than the last message: ignored */
    TB_BAD,             /* no points / too many / not in time order */
    TB_IGNORED          /* not the active target source (game mode) */
} m3_tbuf_result_t;

typedef struct {
    uint32_t slot;      /* local_ms / BUCKET_MS of this bucket */
    int64_t min_off;
    bool used;
} m3_tbuf_bucket_t;

typedef struct {
    bool valid;             /* any target known (hold or played) */
    m3_target_t out;        /* last output = what is held */

    bool have_session;
    uint32_t session;
    bool have_msg_t;
    int64_t last_msg_t;

    m3_tpoint_t pts[M3_TBUF_CAP];
    uint8_t n;

    m3_tbuf_bucket_t bucket[M3_TBUF_OFFSET_BUCKETS];
    bool have_offset;
    uint32_t first_sample_ms;   /* local time of the first offset sample this session */

    bool playing;           /* output currently follows the points */
    bool dry;               /* ran past the last point (holding) */
    uint32_t blend_start_ms;
    m3_target_t blend_from;

    /* diagnostics */
    uint32_t n_sessions;
    uint32_t n_old;
    uint32_t n_dry;
} m3_tbuf_t;

void m3_tbuf_init(m3_tbuf_t *b);

/* Direct hold (operator command or tests): clears the points, keeps the offset. */
void m3_tbuf_set_hold(m3_tbuf_t *b, float mm);

/* One target_points message: sender send time + its points (in time order). */
m3_tbuf_result_t m3_tbuf_push(m3_tbuf_t *b, uint32_t session, int64_t msg_t_ms,
                              const m3_tpoint_t *pts, uint8_t n, uint32_t now_ms);

bool m3_tbuf_valid(const m3_tbuf_t *b);

/* Current offset estimate (local - sender, ms); false while none. */
bool m3_tbuf_offset(const m3_tbuf_t *b, uint32_t now_ms, int64_t *off_ms);

/* Target to use now. Advances playback, so call it with non-decreasing now_ms. */
m3_target_t m3_tbuf_get(m3_tbuf_t *b, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
#endif
