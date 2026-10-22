#include "pid_ctrl.h"

#ifndef __weak
#define __weak __attribute__((weak))
#endif

/* Publish-mode weak stub for pid_ctrl.c: update() outputs 0; init()/reset() are
 * complete. Overridden by the real pid_ctrl.c when present (HIL_REAL_CONTROLLERS). */

static void pid_update_empty(controller_t *c, const float *setpoint,
                             const float *measured, float *control, float dt)
{
    (void)setpoint;
    (void)measured;
    (void)dt;

    /*
     * TODO: per-channel PID law -- write control[0..c->n-1].
     *   pid_ctrl_t *p = c->params;
     *     effective gains p->gains[i]: kp, ki_dt, kd_inv_dt, out_min, out_max,
     *                                 integ_min, integ_max;
     *     state p->integ[i], p->prev_err[i].
     *   per channel i:
     *     e = setpoint[i] - measured[i];
     *     p->integ[i] += e;                  clamp to [integ_min, integ_max];
     *     de = e - p->prev_err[i];           p->prev_err[i] = e;
     *     u = kp*e + ki_dt*p->integ[i] + kd_inv_dt*de;
     *     clamp u to [out_min, out_max];     control[i] = u;
     *   (ki/kd are pre-scaled by dt in pid_controller_init().)
     * Placeholder below writes 0 to every channel.
     */
    for (uint32_t i = 0; i < c->n; i++)
    {
        control[i] = 0.0f;
    }
}

static void pid_reset_empty(controller_t *c)
{
    pid_ctrl_t *p = (pid_ctrl_t *)c->params;
    for (uint32_t i = 0; i < p->n; i++)
    {
        p->integ[i] = 0.0f;
        p->prev_err[i] = 0.0f;
    }
}

__weak void pid_controller_init(controller_t *ctrl, pid_ctrl_t *pid,
                                const pid_gains_t *gains, uint32_t n, float dt)
{
    if (n > PID_MAX_CHANNELS)
    {
        n = PID_MAX_CHANNELS;
    }
    /* Fold dt into the integral/derivative gains once, up front. */
    float inv_dt = (dt > 0.0f) ? (1.0f / dt) : 0.0f;
    pid->n = n;
    for (uint32_t i = 0; i < n; i++)
    {
        pid->gains[i].kp        = gains[i].kp;
        pid->gains[i].ki_dt     = gains[i].ki * dt;
        pid->gains[i].kd_inv_dt = gains[i].kd * inv_dt;
        pid->gains[i].out_min   = gains[i].out_min;
        pid->gains[i].out_max   = gains[i].out_max;
        pid->gains[i].integ_min = gains[i].integ_min * inv_dt;
        pid->gains[i].integ_max = gains[i].integ_max * inv_dt;
        pid->integ[i] = 0.0f;
        pid->prev_err[i] = 0.0f;
    }

    ctrl->n = n;
    ctrl->update = pid_update_empty;
    ctrl->reset = pid_reset_empty;
    ctrl->params = pid;
}
