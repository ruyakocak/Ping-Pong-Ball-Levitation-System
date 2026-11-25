/*
 * Commands from M5 (UDP `command` -> `ack`) and game_out from the game board
 * (R3.10). Every command is checked before anything changes; a refused command
 * changes nothing and its ack carries the reason. The network task decodes the
 * JSON into m3_command_t and encodes the ack; this file has no hardware or JSON.
 */
#ifndef M3_COMMAND_H
#define M3_COMMAND_H

#include <stdbool.h>
#include <stdint.h>

#include "m3_cmd_check.h"
#include "m3_ctrl.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { CMDK_START = 0, CMDK_STOP, CMDK_RESET, CMDK_MODE, CMDK_SET } m3_cmd_kind_t;
typedef enum { MODE_HOLD = 0, MODE_WAVE, MODE_GAME, MODE_COUNT } m3_mode_t;

typedef struct {
    m3_cmd_kind_t kind;
    uint32_t request_id;
    int mode;            /* CMDK_MODE: m3_mode_t */
    const char *param;   /* CMDK_SET */
    float value;         /* CMDK_SET */
} m3_command_t;

typedef struct {
    uint32_t request_id;
    bool ok;             /* applied (or Reset accepted and pending) */
    bool pending;        /* Reset: waiting for M1 armed */
    float applied;       /* CMDK_SET: value now in use (the old one if refused) */
    const char *reason;  /* "ok" or why it was refused */
} m3_ack_t;

m3_ack_t m3_command_apply(m3_ctrl_t *c, const m3_command_t *cmd, uint32_t now_ms);

/* Current value of a parameter (writable or safety limit); false if unknown. */
bool m3_param_get(const m3_ctrl_t *c, const char *name, float *value);

typedef enum {
    GAME_APPLIED = 0,
    GAME_IGNORED_NOT_GAME,  /* outside game mode: ignored, counted */
    GAME_OLD,               /* sequence not newer */
    GAME_BAD_LEVEL          /* target level outside 0..7 */
} m3_game_result_t;

#define M3_GAME_LEVELS 8u

m3_game_result_t m3_command_game_out(m3_ctrl_t *c, uint32_t session, uint32_t seq,
                                     const ct_game_out_t *m, uint32_t now_ms);

/* Height of a game level inside the usable range. */
float m3_game_level_mm(const m3_ctrl_t *c, uint8_t level);

#ifdef __cplusplus
}
#endif
#endif
