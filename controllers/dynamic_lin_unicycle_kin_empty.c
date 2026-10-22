#include <math.h>

#include "dynamic_lin_unicycle_kin.h"
#include "ealgebra.h"

#ifndef __weak
#define __weak __attribute__((weak))
#endif

/* Publish-mode weak stub for dynamic_lin_unicycle_kin.c: update() outputs 0;
 * init()/reset() are complete. Overridden by the real dynamic_lin_unicycle_kin.c
 * when present (HIL_REAL_CONTROLLERS). */

static void dynamic_lin_update_empty(controller_t *c, const float *setpoint,
                                     const float *measured, float *control, float dt)
{
    (void)setpoint;
    (void)measured;
    (void)dt;

    /*
     * TODO: dynamic feedback-linearization for the kinematic unicycle -- write control[0..1] = [v, omega].
     *   dynamic_lin_unicycle_kin_t *d = c->params;
     *     offset d->e, gains d->kp/d->kd, integrator d->chi1 (= v - e*omega, keep != 0),
     *     setpoint history d->prev_hd[2]/d->prev_hd_d1[2], d->have_prev.
     *   measured[0..2] = [x, y, theta];  setpoint[0..1] = desired output point hd (delta = pi/2).
     *   steps:
     *     1. h(q) = [x - e*sin(theta), y + e*cos(theta)];  h' = d->chi1*[cos, sin];
     *     2. hd'/hd'' by finite-differencing setpoint[];
     *     3. eh = h-hd, eh' = h'-hd';  v = hd'' - d->kp*eh - d->kd*eh';
     *     4. w = Kd^-1*v, Kd = [[cth, -chi1*sth],[sth, chi1*cth]], det = chi1;
     *     5. omega = w[1];  vel = d->chi1 + e*omega;  d->chi1 += w[0]*dt;
     *     6. control[0] = vel;  control[1] = omega.
     * Placeholder below writes 0.
     */
    for (uint32_t i = 0; i < c->n; i++)
    {
        control[i] = 0.0f;
    }
}

static void dynamic_lin_reset_empty(controller_t *c)
{
    dynamic_lin_unicycle_kin_t *d = (dynamic_lin_unicycle_kin_t *)c->params;
    d->have_prev = 0;
    d->prev_hd[0] = d->prev_hd[1] = 0.0f;
    d->prev_hd_d1[0] = d->prev_hd_d1[1] = 0.0f;
}

__weak void dynamic_lin_unicycle_kin_init(controller_t *ctrl, dynamic_lin_unicycle_kin_t *dl,
                                          float e, float kp, float kd, float chi1_init)
{
    dl->e = e;
    dl->kp = kp;
    dl->kd = kd;
    dl->chi1 = chi1_init;
    dl->have_prev = 0;
    dl->prev_hd[0] = dl->prev_hd[1] = 0.0f;
    dl->prev_hd_d1[0] = dl->prev_hd_d1[1] = 0.0f;

    ctrl->n = 2;
    ctrl->update = dynamic_lin_update_empty;
    ctrl->reset = dynamic_lin_reset_empty;
    ctrl->params = dl;
}
