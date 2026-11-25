/*
 * Control-board state machine (Section 6.3, transition table).
 * Anything not in the table is refused. Fault is never left automatically.
 *
 * The FSM is pure: the caller passes the current conditions and the local
 * monotonic time; the FSM returns what the blowers may do.
 */
#ifndef M3_FSM_H
#define M3_FSM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define M3_LAUNCH_TIMEOUT_MS 1500u
#define M3_LAUNCH_LIFT_MM 30.0f
#define M3_FAN_STATE_TIMEOUT_MS 100u
#define M3_REARM_TIMEOUT_MS 1000u

typedef enum { ST_IDLE = 0, ST_LAUNCH, ST_RUN, ST_FAULT } m3_state_t;

typedef enum {
    CAUSE_NONE = 0,
    CAUSE_POWER_UP,
    CAUSE_NO_HEIGHT,     /* no new real height for 200 ms */
    CAUSE_FAN_STATE_LOST,
    CAUSE_M1_FAULT,
    CAUSE_ESTOP,
    CAUSE_LAUNCH_FAILED
} m3_cause_t;

typedef enum {
    EV_START = 0,
    EV_STOP,
    EV_RESET,
    EV_GAME_HIT
} m3_event_t;

typedef enum {
    RES_OK = 0,
    RES_PENDING,          /* Reset accepted, waiting for M1 armed */
    RES_REFUSED_STATE,    /* not allowed in this state */
    RES_REFUSED_NO_HEIGHT,
    RES_REFUSED_M1_NOT_ARMED,
    RES_REFUSED_M1_FAULT,
    RES_REFUSED_ESTOP,
    RES_REFUSED_NO_TARGET,
    RES_REFUSED_NO_FAN_STATE,
    RES_REFUSED_NOT_GAME
} m3_result_t;

/* Conditions sampled by the caller at the moment of the call. */
typedef struct {
    bool height_fresh;     /* new real height within 200 ms */
    bool fan_state_fresh;  /* fan_state within 100 ms */
    bool m1_armed;         /* fan_state.state == ARMED (or RUN) */
    bool m1_fault;         /* fan_state.state == FAULT */
    bool estop;            /* fan_state.state == ESTOP */
    bool target_valid;
    bool game_mode;
    float ball_mm;         /* height used for control */
} m3_cond_t;

typedef struct {
    m3_state_t state;
    m3_cause_t cause;
    uint32_t launch_start_ms;
    bool rearm_pending;
    uint32_t rearm_start_ms;
    /* set on transitions, cleared by the caller after reading */
    bool clear_integral;
} m3_fsm_t;

/* Output of the FSM for the next fan_cmd. */
typedef struct {
    bool enable;          /* blowers enabled (Launch, Run) */
    bool rearm;           /* re-arm flag in fan_cmd */
    bool freeze_integral; /* Launch */
    bool launch_push;     /* use the limited launch push instead of the PID */
} m3_fsm_out_t;

void m3_fsm_init(m3_fsm_t *f, uint32_t now_ms); /* starts in Fault, cause power-up */
m3_result_t m3_fsm_event(m3_fsm_t *f, m3_event_t ev, const m3_cond_t *c, uint32_t now_ms);
/* Call on every control step: checks fault conditions, launch progress, re-arm progress. */
void m3_fsm_tick(m3_fsm_t *f, const m3_cond_t *c, uint32_t now_ms);
m3_fsm_out_t m3_fsm_output(const m3_fsm_t *f);

const char *m3_state_name(m3_state_t s);
const char *m3_cause_name(m3_cause_t c);
const char *m3_result_name(m3_result_t r);

#ifdef __cplusplus
}
#endif
#endif
