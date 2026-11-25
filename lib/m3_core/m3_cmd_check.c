#include "m3_cmd_check.h"

#include <math.h>
#include <string.h>

static const m3_param_def_t PARAMS[] = {
    /* name               min      max       writable */
    {"kp",                0.0f,    200.0f,   true},
    {"ki",                0.0f,    200.0f,   true},
    {"kd",                0.0f,    50.0f,    true},
    {"hover_rpm",         500.0f,  6000.0f,  true},
    {"target_mm",         40.0f,   900.0f,   true},
    {"rate_rpm_s",        100.0f,  20000.0f, true},
    {"n_min_rpm",         0.0f,    3000.0f,  true},
    {"n_max_rpm",         1000.0f, 6000.0f,  true},
    {"obstacle_speed",    1.0f,    10.0f,    true},
    /* safety limits: visible, never writable */
    {"height_timeout_ms",    200.0f,  200.0f,  false},
    {"fan_state_timeout_ms", 100.0f,  100.0f,  false},
    {"abs_max_rpm",          6000.0f, 6000.0f, false},
    {"max_power_pm",         900.0f,  900.0f,  false},
    {"launch_push_ms",       1500.0f, 1500.0f, false},
};

const m3_param_def_t *m3_param_find(const char *name)
{
    if (!name) return 0;
    for (unsigned i = 0; i < sizeof(PARAMS) / sizeof(PARAMS[0]); i++)
        if (strcmp(PARAMS[i].name, name) == 0) return &PARAMS[i];
    return 0;
}

m3_cmd_result_t m3_param_check(const char *name, float value)
{
    const m3_param_def_t *p = m3_param_find(name);
    if (!p) return CMD_UNKNOWN_PARAM;
    if (!p->writable) return CMD_READ_ONLY;
    if (!isfinite(value)) return CMD_NOT_A_NUMBER;
    if (value < p->min || value > p->max) return CMD_OUT_OF_RANGE;
    return CMD_OK;
}

const char *m3_cmd_result_name(m3_cmd_result_t r)
{
    static const char *n[CMD_COUNT] = {"ok",           "unknown_param", "read_only",
                                       "out_of_range", "not_a_number",  "conflict",
                                       "wrong_state",  "wrong_mode",    "unknown_command"};
    return (unsigned)r < (unsigned)CMD_COUNT ? n[r] : "?";
}
