#include "static_lin_unicycle_kin.h"
#include "ealgebra.h"

#ifndef __weak
#define __weak __attribute__((weak))
#endif

/* Publish-mode weak stub for static_lin_unicycle_kin.c: update() outputs 0;
 * init()/reset() are complete. Overridden by the real static_lin_unicycle_kin.c
 * when present (HIL_REAL_CONTROLLERS). */

static void static_lin_kin_update_empty(controller_t *c, const float *setpoint,
                                        const float *measured, float *control, float dt)
{
    (void)setpoint;
    (void)measured;
    (void)dt;

    /*
     * TODO: static feedback-linearization for the kinematic unicycle -- write control[0..1] = [v, omega].
     *   static_lin_unicycle_kin_t *s = c->params;
     *     geometry s->geom (unicycle_params_t), gain s->kp,
     *     setpoint history s->prev_hd[2], s->have_prev.
     *   measured[0..2] = [x, y, theta];  setpoint[0..1] = desired output point hd.
     *   steps:
     *     1. h(q) and decoupling matrix R via unicycle_static_linearization(measured, &s->geom, &lin);
     *     2. hd' = (setpoint - s->prev_hd)/dt (single finite difference);
     *     3. eh = h - setpoint;  v = hd' - s->kp*eh;
     *     4. eta = R*v;  control[0] = eta0+eta1 (v);  control[1] = (eta0-eta1)/s->geom.axle_length (omega).
     * Placeholder below writes 0.
     */
    for (uint32_t i = 0; i < c->n; i++)
    {
        control[i] = 0.0f;
    }
}

static void static_lin_kin_reset_empty(controller_t *c)
{
    static_lin_unicycle_kin_t *s = (static_lin_unicycle_kin_t *)c->params;
    s->have_prev = 0;
    s->prev_hd[0] = 0.0f;
    s->prev_hd[1] = 0.0f;
}

__weak void static_lin_unicycle_kin_init(controller_t *ctrl, static_lin_unicycle_kin_t *sk,
                                         const unicycle_params_t *geom, float kp)
{
    sk->geom = *geom;
    sk->kp = kp;
    sk->have_prev = 0;
    sk->prev_hd[0] = 0.0f;
    sk->prev_hd[1] = 0.0f;

    ctrl->n = 2;
    ctrl->update = static_lin_kin_update_empty;
    ctrl->reset = static_lin_kin_reset_empty;
    ctrl->params = sk;
}
