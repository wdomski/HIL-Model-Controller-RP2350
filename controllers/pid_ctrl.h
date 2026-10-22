#ifndef PID_CTRL_H
#define PID_CTRL_H

#include "controller.h"

#define PID_MAX_CHANNELS 4

/* Per-channel PID gains and output saturation (time-independent). */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float out_min;
    float out_max;
    float integ_min; /* anti-windup: lower clamp on the integral sum */
    float integ_max; /* anti-windup: upper clamp on the integral sum */
} pid_gains_t;

/*
 * Per-channel effective gains with the loop timestep dt folded in, so the hot
 * update path needs no multiply/divide by dt. Computed by pid_controller_init().
 */
typedef struct
{
    float kp;        /* kp                       */
    float ki_dt;     /* ki * dt                  */
    float kd_inv_dt; /* kd / dt                  */
    float out_min;
    float out_max;
    float integ_min; /* clamp on the raw error sum (integ_min / dt) */
    float integ_max; /* clamp on the raw error sum (integ_max / dt) */
} pid_eff_gains_t;

typedef struct
{
    uint32_t n;
    pid_eff_gains_t gains[PID_MAX_CHANNELS];
    float integ[PID_MAX_CHANNELS];
    float prev_err[PID_MAX_CHANNELS];
} pid_ctrl_t;

/*
 * Bind a controller_t to a per-channel PID implementation.
 *
 * `gains` are time-independent; `dt` is the fixed control-loop timestep [s].
 * The integral/derivative gains are pre-scaled by dt at init time so the update
 * step stays branch-light and free of runtime dt arithmetic.
 */
void pid_controller_init(controller_t *ctrl, pid_ctrl_t *pid,
                         const pid_gains_t *gains, uint32_t n, float dt);

#endif /* PID_CTRL_H */
