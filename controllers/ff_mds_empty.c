#include "ff_mds.h"

#ifndef __weak
#define __weak __attribute__((weak))
#endif

/* Publish-mode weak stub for ff_mds.c: update() outputs 0; init()/reset() are
 * complete. Overridden by the real ff_mds.c when present (HIL_REAL_CONTROLLERS). */

static void ff_mds_update_empty(controller_t *c, const float *setpoint,
                                const float *measured, float *control, float dt)
{
    (void)setpoint;
    (void)measured;
    (void)dt;

    /*
     * TODO: full-knowledge computed force for the mass-spring-damper --
     *       write control[0] = F.
     *   ff_mds_t *s = c->params;
     *     known model s->model (mass, damping, stiffness), gains s->kp, s->kd,
     *     history s->prev_x, s->prev_sp, s->prev_sp_d1, s->have_prev.
     *   measured[0] = x;  setpoint[0] = x_d.
     *   steps:
     *     1. x' via finite difference of measured; x_d', x_d'' via double FD of setpoint;
     *     2. v = x_d'' - Kd (x'-x_d') - Kp (x-x_d);
     *     3. F = m*v + c*x' + k*x   (cancels the plant -> e'' + Kd e' + Kp e = 0).
     * Placeholder below writes 0.
     */
    for (uint32_t i = 0; i < c->n; i++)
    {
        control[i] = 0.0f;
    }
}

static void ff_mds_reset_empty(controller_t *c)
{
    ff_mds_t *s = (ff_mds_t *)c->params;
    s->have_prev = 0;
    s->prev_x = 0.0f;
    s->prev_sp = s->prev_sp_d1 = 0.0f;
}

__weak void ff_mds_init(controller_t *ctrl, ff_mds_t *c,
                        const mds_params_t *model, float kp, float kd)
{
    c->model = *model;
    c->kp = kp;
    c->kd = kd;
    c->have_prev = 0;
    c->prev_x = 0.0f;
    c->prev_sp = c->prev_sp_d1 = 0.0f;

    ctrl->n = 1;
    ctrl->update = ff_mds_update_empty;
    ctrl->reset = ff_mds_reset_empty;
    ctrl->params = c;
}
