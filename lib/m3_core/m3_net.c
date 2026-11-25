#include "m3_net.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---------- minimal JSON reader (bounded, no allocation) ---------- */

typedef struct {
    const char *p, *end;
    int depth;
} rd_t;

#define MAX_DEPTH 8

static void ws(rd_t *r)
{
    while (r->p < r->end && (*r->p == ' ' || *r->p == '\t' || *r->p == '\n' || *r->p == '\r')) r->p++;
}

static bool eat(rd_t *r, char c)
{
    ws(r);
    if (r->p < r->end && *r->p == c) {
        r->p++;
        return true;
    }
    return false;
}

/* Reads a string into out (truncated to cap-1, always terminated). */
static bool str(rd_t *r, char *out, size_t cap)
{
    size_t n = 0;
    ws(r);
    if (r->p >= r->end || *r->p != '"') return false;
    r->p++;
    while (r->p < r->end && *r->p != '"') {
        char c = *r->p++;
        if ((unsigned char)c < 0x20) return false;
        if (c == '\\') {
            if (r->p >= r->end) return false;
            char e = *r->p++;
            switch (e) {
            case '"': case '\\': case '/': c = e; break;
            case 'b': c = '\b'; break;
            case 'f': c = '\f'; break;
            case 'n': c = '\n'; break;
            case 'r': c = '\r'; break;
            case 't': c = '\t'; break;
            case 'u':
                for (int i = 0; i < 4; i++) {
                    if (r->p >= r->end) return false;
                    char h = *r->p++;
                    if (!((h >= '0' && h <= '9') || (h >= 'a' && h <= 'f') || (h >= 'A' && h <= 'F'))) return false;
                }
                c = '?'; /* names are ASCII; anything else just will not match */
                break;
            default: return false;
            }
        }
        if (out && n + 1 < cap) out[n++] = c;
    }
    if (r->p >= r->end) return false;
    r->p++; /* closing quote */
    if (out && cap) out[n] = 0;
    return true;
}

static bool num(rd_t *r, double *v)
{
    ws(r);
    const char *s = r->p;
    char tmp[40];
    size_t n = 0;
    if (s < r->end && (*s == '-' || *s == '+')) tmp[n++] = *s++;
    bool digits = false;
    while (s < r->end && n < sizeof tmp - 1 &&
           ((*s >= '0' && *s <= '9') || *s == '.' || *s == 'e' || *s == 'E' || *s == '-' || *s == '+')) {
        if (*s >= '0' && *s <= '9') digits = true;
        tmp[n++] = *s++;
    }
    if (!digits || n >= sizeof tmp - 1) return false;
    tmp[n] = 0;
    char *e;
    double d = strtod(tmp, &e);
    if (*e != 0 || !isfinite(d)) return false;
    r->p = s;
    *v = d;
    return true;
}

static bool lit(rd_t *r, const char *w)
{
    ws(r);
    size_t n = strlen(w);
    if ((size_t)(r->end - r->p) < n || memcmp(r->p, w, n) != 0) return false;
    r->p += n;
    return true;
}

static bool skip_value(rd_t *r);

static bool skip_container(rd_t *r, char open, char close)
{
    if (!eat(r, open)) return false;
    if (++r->depth > MAX_DEPTH) return false;
    if (eat(r, close)) {
        r->depth--;
        return true;
    }
    for (;;) {
        if (open == '{') {
            if (!str(r, NULL, 0) || !eat(r, ':')) return false;
        }
        if (!skip_value(r)) return false;
        if (eat(r, ',')) continue;
        if (!eat(r, close)) return false;
        r->depth--;
        return true;
    }
}

static bool skip_value(rd_t *r)
{
    ws(r);
    if (r->p >= r->end) return false;
    double d;
    switch (*r->p) {
    case '"': return str(r, NULL, 0);
    case '{': return skip_container(r, '{', '}');
    case '[': return skip_container(r, '[', ']');
    case 't': return lit(r, "true");
    case 'f': return lit(r, "false");
    case 'n': return lit(r, "null");
    default: return num(r, &d);
    }
}

static bool is_u32(double d) { return d >= 0.0 && d <= 4294967295.0 && d == floor(d); }

/* One target point: {"t":..,"h":..,"ff":..,"top":..}; t and h required. */
static bool point(rd_t *r, m3_tpoint_t *pt)
{
    bool have_t = false, have_h = false;
    memset(pt, 0, sizeof *pt);
    if (!eat(r, '{')) return false;
    if (eat(r, '}')) return false;
    for (;;) {
        char key[8];
        double d;
        if (!str(r, key, sizeof key) || !eat(r, ':')) return false;
        if (!strcmp(key, "t")) {
            if (!num(r, &d) || d != floor(d) || fabs(d) > 9.0e15) return false;
            pt->t_ms = (int64_t)d;
            have_t = true;
        } else if (!strcmp(key, "h")) {
            if (!num(r, &d)) return false;
            pt->mm = (float)d;
            have_h = true;
        } else if (!strcmp(key, "ff")) {
            if (!num(r, &d)) return false;
            pt->ff_rpm = (float)d;
        } else if (!strcmp(key, "top")) {
            if (!num(r, &d) || d < 0 || d > 255 || d != floor(d)) return false;
            pt->top_level = (uint8_t)d;
        } else if (!skip_value(r)) {
            return false;
        }
        if (eat(r, ',')) continue;
        if (!eat(r, '}')) return false;
        return have_t && have_h;
    }
}

static m3_netmsg_kind_t bad(m3_net_msg_t *m, const char *why)
{
    m->kind = NETMSG_BAD;
    m->error = why;
    return NETMSG_BAD;
}

m3_netmsg_kind_t m3_net_parse(const char *buf, size_t len, m3_net_msg_t *m)
{
    memset(m, 0, sizeof *m);
    m->value = NAN;
    if (!buf || len == 0 || len > M3_NET_MAX_DATAGRAM) return bad(m, "size");
    rd_t r = {buf, buf + len, 0};
    char type[M3_NET_NAME_LEN] = "";
    bool have_session = false, have_t = false, have_req = false, have_points = false;

    if (!eat(&r, '{')) return bad(m, "not_an_object");
    if (!eat(&r, '}')) {
        for (;;) {
            char key[M3_NET_NAME_LEN];
            double d;
            if (!str(&r, key, sizeof key) || !eat(&r, ':')) return bad(m, "syntax");
            if (!strcmp(key, "type")) {
                if (!str(&r, type, sizeof type)) return bad(m, "type");
            } else if (!strcmp(key, "session")) {
                if (!num(&r, &d) || !is_u32(d)) return bad(m, "session");
                m->session = (uint32_t)d;
                have_session = true;
            } else if (!strcmp(key, "t")) {
                if (!num(&r, &d) || d != floor(d) || fabs(d) > 9.0e15) return bad(m, "t");
                m->t_ms = (int64_t)d;
                have_t = true;
            } else if (!strcmp(key, "request_id")) {
                if (!num(&r, &d) || !is_u32(d)) return bad(m, "request_id");
                m->request_id = (uint32_t)d;
                have_req = true;
            } else if (!strcmp(key, "cmd")) {
                if (!str(&r, m->cmd, sizeof m->cmd)) return bad(m, "cmd");
            } else if (!strcmp(key, "mode")) {
                if (!str(&r, m->mode, sizeof m->mode)) return bad(m, "mode");
            } else if (!strcmp(key, "param")) {
                if (!str(&r, m->param, sizeof m->param)) return bad(m, "param");
            } else if (!strcmp(key, "value")) {
                if (!num(&r, &d)) {
                    if (!skip_value(&r)) return bad(m, "syntax");
                    d = NAN; /* present but not a number: the ack says so */
                }
                m->value = (float)d;
            } else if (!strcmp(key, "points")) {
                if (!eat(&r, '[')) return bad(m, "points");
                have_points = true;
                if (!eat(&r, ']')) {
                    for (;;) {
                        if (m->n_points >= M3_TBUF_MAX_POINTS_PER_MSG) return bad(m, "too_many_points");
                        if (!point(&r, &m->pts[m->n_points])) return bad(m, "point");
                        m->n_points++;
                        if (eat(&r, ',')) continue;
                        if (!eat(&r, ']')) return bad(m, "points");
                        break;
                    }
                }
            } else if (!skip_value(&r)) {
                return bad(m, "syntax");
            }
            if (eat(&r, ',')) continue;
            if (!eat(&r, '}')) return bad(m, "syntax");
            break;
        }
    }
    ws(&r);
    if (r.p != r.end) return bad(m, "trailing_data");
    if (!type[0]) return bad(m, "no_type");
    if (!have_session || !have_t) return bad(m, "no_session_or_t");

    if (!strcmp(type, "command")) {
        if (!have_req) return bad(m, "no_request_id");
        m->kind = NETMSG_COMMAND;
    } else if (!strcmp(type, "target_points")) {
        if (!have_points || m->n_points == 0) return bad(m, "no_points");
        m->kind = NETMSG_TARGETS;
    } else {
        m->kind = NETMSG_OTHER; /* valid JSON, not for us: ignored */
    }
    return m->kind;
}

/* ---------- control-task side ---------- */

void m3_net_init(m3_net_t *n) { memset(n, 0, sizeof *n); }

static const char *const MODE_NAMES[MODE_COUNT] = {"hold", "wave", "game"};

const char *m3_mode_name(uint8_t mode) { return mode < MODE_COUNT ? MODE_NAMES[mode] : "?"; }

static void copy_name(char *dst, const char *src)
{
    size_t n = strlen(src);
    if (n > M3_NET_NAME_LEN - 1) n = M3_NET_NAME_LEN - 1;
    memcpy(dst, src, n);
    dst[n] = 0;
}

bool m3_net_apply(m3_net_t *n, m3_ctrl_t *c, const m3_net_msg_t *m, uint32_t now_ms, m3_net_ack_t *ack)
{
    if (m->kind == NETMSG_TARGETS) {
        m3_tbuf_result_t r = m3_ctrl_on_target_points(c, m->session, m->t_ms, m->pts, m->n_points, now_ms);
        n->n_targets++;
        if (r != TB_ACCEPTED && r != TB_NEW_SESSION) n->n_targets_refused++;
        return false;
    }
    if (m->kind != NETMSG_COMMAND) return false;

    /* The same request again (lost ack, resend): answer it, do not apply it twice. */
    if (n->have_last && n->last_session == m->session && n->last_request_id == m->request_id) {
        n->n_repeated++;
        *ack = n->last_ack;
        return true;
    }

    m3_command_t cmd;
    memset(&cmd, 0, sizeof cmd);
    cmd.request_id = m->request_id;
    cmd.param = m->param;
    cmd.value = m->value;
    cmd.mode = -1;
    if (!strcmp(m->cmd, "start")) cmd.kind = CMDK_START;
    else if (!strcmp(m->cmd, "stop")) cmd.kind = CMDK_STOP;
    else if (!strcmp(m->cmd, "reset")) cmd.kind = CMDK_RESET;
    else if (!strcmp(m->cmd, "mode")) cmd.kind = CMDK_MODE;
    else if (!strcmp(m->cmd, "set")) cmd.kind = CMDK_SET;
    else cmd.kind = (m3_cmd_kind_t)-1; /* -> unknown_command */
    for (int i = 0; i < MODE_COUNT; i++)
        if (!strcmp(m->mode, MODE_NAMES[i])) cmd.mode = i;

    memset(ack, 0, sizeof *ack);
    ack->a = m3_command_apply(c, &cmd, now_ms);
    copy_name(ack->cmd, m->cmd);
    copy_name(ack->param, cmd.kind == CMDK_SET ? m->param : "");
    n->n_commands++;
    n->have_last = true;
    n->last_session = m->session;
    n->last_request_id = m->request_id;
    n->last_ack = *ack;
    return true;
}

void m3_net_status_fill(const m3_net_t *n, const m3_ctrl_t *c, m3_net_status_t *st)
{
    memset(st, 0, sizeof *st);
    st->state = (uint8_t)c->fsm.state;
    st->cause = (uint8_t)c->fsm.cause;
    st->mode = c->mode;
    st->m1_state = c->have_fs ? c->fs.state : 0;
    st->top_level = c->top_level;
    st->game_phase = c->game_phase;
    st->game_score = c->game_score;
    st->ball_stuck = c->stuck.warning;
    st->target_valid = m3_tbuf_valid(&c->tbuf);
    st->target_mm = c->target_mm;
    st->h_used_mm = c->h_used_mm;
    st->n_wanted_rpm = c->n_wanted;
    st->rpm = c->have_fs ? c->fs.bottom_rpm : 0;
    st->bottom_power_pm = c->have_fs ? c->fs.bottom_power_pm : 0;
    st->top_power_pm = c->have_fs ? c->fs.top_power_pm : 0;
    st->n_commands = n->n_commands;
    st->n_targets = n->n_targets;
    st->n_targets_refused = n->n_targets_refused;
}

/* ---------- JSON writer ---------- */

typedef struct {
    char *buf;
    size_t cap, n;
    bool overflow;
} wr_t;

static void put(wr_t *w, const char *fmt, ...)
{
    if (w->overflow) return;
    va_list ap;
    va_start(ap, fmt);
    int k = vsnprintf(w->buf + w->n, w->cap - w->n, fmt, ap);
    va_end(ap);
    if (k < 0 || (size_t)k >= w->cap - w->n) w->overflow = true;
    else w->n += (size_t)k;
}

static void put_str(wr_t *w, const char *key, const char *s)
{
    put(w, ",\"%s\":\"", key);
    for (; *s && !w->overflow; s++) {
        unsigned char ch = (unsigned char)*s;
        if (ch == '"' || ch == '\\') put(w, "\\%c", ch);
        else if (ch < 0x20 || ch >= 0x7f) put(w, "\\u%04x", ch);
        else put(w, "%c", ch);
    }
    put(w, "\"");
}

static void put_num(wr_t *w, const char *key, float v, int decimals)
{
    if (isfinite(v)) put(w, ",\"%s\":%.*f", key, decimals, (double)v);
    else put(w, ",\"%s\":null", key);
}

static size_t finish(wr_t *w)
{
    put(w, "}");
    if (w->overflow) {
        if (w->cap) w->buf[0] = 0;
        return 0;
    }
    return w->n;
}

size_t m3_net_format_ack(char *buf, size_t cap, uint32_t session, uint32_t now_ms, const m3_net_ack_t *a)
{
    wr_t w = {buf, cap, 0, cap == 0};
    put(&w, "{\"type\":\"ack\",\"src\":\"m3\",\"session\":%lu,\"t\":%lu,\"request_id\":%lu", (unsigned long)session,
        (unsigned long)now_ms, (unsigned long)a->a.request_id);
    put_str(&w, "cmd", a->cmd);
    put(&w, ",\"ok\":%s,\"pending\":%s", a->a.ok ? "true" : "false", a->a.pending ? "true" : "false");
    if (a->param[0]) {
        put_str(&w, "param", a->param);
        put_num(&w, "applied", a->a.applied, 4);
    }
    put_str(&w, "reason", a->a.reason ? a->a.reason : "?");
    return finish(&w);
}

size_t m3_net_format_status(char *buf, size_t cap, uint32_t session, uint32_t now_ms, const m3_net_status_t *s)
{
    wr_t w = {buf, cap, 0, cap == 0};
    put(&w, "{\"type\":\"status\",\"src\":\"m3\",\"session\":%lu,\"t\":%lu", (unsigned long)session,
        (unsigned long)now_ms);
    put_str(&w, "state", m3_state_name((m3_state_t)s->state));
    put_str(&w, "fault", s->state == ST_FAULT ? m3_cause_name((m3_cause_t)s->cause) : "none");
    put_str(&w, "mode", m3_mode_name(s->mode));
    put(&w, ",\"target_valid\":%s", s->target_valid ? "true" : "false");
    put_num(&w, "target_mm", s->target_mm, 1);
    put_num(&w, "h_used_mm", s->h_used_mm, 1);
    put_num(&w, "n_wanted_rpm", s->n_wanted_rpm, 0);
    put(&w, ",\"rpm\":%u,\"bottom_power_pm\":%u,\"top_power_pm\":%u,\"top_level\":%u,\"m1_state\":%u",
        (unsigned)s->rpm, (unsigned)s->bottom_power_pm, (unsigned)s->top_power_pm, (unsigned)s->top_level,
        (unsigned)s->m1_state);
    put(&w, ",\"game_phase\":%u,\"game_score\":%u,\"ball_stuck\":%s", (unsigned)s->game_phase,
        (unsigned)s->game_score, s->ball_stuck ? "true" : "false");
    put(&w, ",\"diag\":{\"commands\":%lu,\"targets\":%lu,\"targets_refused\":%lu,\"heights_used\":%lu,"
            "\"lat_max_us\":%lu,\"lat_over_5ms\":%lu,\"heights_lost\":%lu,\"net_bad\":%lu}",
        (unsigned long)s->n_commands, (unsigned long)s->n_targets, (unsigned long)s->n_targets_refused,
        (unsigned long)s->heights_used, (unsigned long)s->lat_max_us, (unsigned long)s->lat_over_5ms,
        (unsigned long)s->heights_lost, (unsigned long)s->net_bad);
    return finish(&w);
}
