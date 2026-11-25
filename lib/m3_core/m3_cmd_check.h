/*
 * Live-parameter table and range checks (R3.10). Ranges are PROVISIONAL until
 * the contract freeze. Safety limits (timeouts, absolute max speed and power,
 * launch push) are in the table as read-only: they can be read, never written
 * by a message.
 *
 * TODO: names must match the M5 contract parameter names.
 */
#ifndef M3_CMD_CHECK_H
#define M3_CMD_CHECK_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CMD_OK = 0,
    CMD_UNKNOWN_PARAM,
    CMD_READ_ONLY,      /* safety limit */
    CMD_OUT_OF_RANGE,
    CMD_NOT_A_NUMBER,
    CMD_CONFLICT,       /* in range, but inconsistent with another setting */
    CMD_WRONG_STATE,    /* not allowed while the ball is flying */
    CMD_WRONG_MODE,     /* e.g. target_mm in game mode */
    CMD_UNKNOWN_COMMAND,
    CMD_COUNT
} m3_cmd_result_t;

typedef struct {
    const char *name;
    float min, max;
    bool writable;
} m3_param_def_t;

const m3_param_def_t *m3_param_find(const char *name);
m3_cmd_result_t m3_param_check(const char *name, float value);
const char *m3_cmd_result_name(m3_cmd_result_t r);

#ifdef __cplusplus
}
#endif
#endif
