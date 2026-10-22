#include <math.h>

#include "static_lin_unicycle_dyn.h"
#include "ealgebra.h"

#ifndef __weak
#define __weak __attribute__((weak))
#endif

/* Publish-mode weak stub for static_lin_unicycle_dyn.c: update() outputs 0;
 * init()/reset() are complete. Overridden by the real static_lin_unicycle_dyn.c
 * when present (HIL_REAL_CONTROLLERS). */

static void static_lin_update_empty(controller_t *c, const float *setpoint,
                                    const float *measured, float *control, float dt)
{
    (void)setpoint;
    (void)measured;
    (void)dt;

    /*
     * TODO: static feedback-linearization for the dynamic unicycle -- write control[0..1] (wheel torques).
     *   static_lin_unicycle_dyn_t *s = c->params;
     *     geometry s->geom, gains s->kp/s->kd,
     *     setpoint history s->prev_hd[2]/s->prev_hd_d1[2], s->have_prev.
     *   measured[0..4] = [x, y, theta, v, omega];  setpoint[0..1] = hd.
     *   steps:
     *     1. Mh/Ch/Bh via unicycle_dynamic_linearization(measured, &s->geom, &lin);
     *     2. hd'/hd'' by double finite-differencing setpoint[];
     *     3. eh = h-setpoint, eh' = h'-hd';  v = hd'' - s->kd*eh' - s->kp*eh;
     *     4. u = Bh^-1*(Mh*v + Ch*h');  control[0..1] = u.
     * Placeholder below writes 0.
     */
    for (uint32_t i = 0; i < c->n; i++)
    {
        control[i] = 0.0f;
    }
}

static void static_lin_reset_empty(controller_t *c)
{
    static_lin_unicycle_dyn_t *s = (static_lin_unicycle_dyn_t *)c->params;
    s->have_prev = 0;
    s->prev_hd[0] = 0.0f;
    s->prev_hd[1] = 0.0f;
    s->prev_hd_d1[0] = 0.0f;
    s->prev_hd_d1[1] = 0.0f;
}

__weak void static_lin_unicycle_dyn_init(controller_t *ctrl, static_lin_unicycle_dyn_t *sl,
                                         const unicycle_params_t *geom, float kp, float kd)
{
    sl->geom = *geom;
    sl->kp = kp;
    sl->kd = kd;
    sl->have_prev = 0;
    sl->prev_hd[0] = 0.0f;
    sl->prev_hd[1] = 0.0f;
    sl->prev_hd_d1[0] = 0.0f;
    sl->prev_hd_d1[1] = 0.0f;

    ctrl->n = 2;
    ctrl->update = static_lin_update_empty;
    ctrl->reset = static_lin_reset_empty;
    ctrl->params = sl;
}
