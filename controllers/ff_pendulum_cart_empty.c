#include "ff_pendulum_cart.h"

#ifndef __weak
#define __weak __attribute__((weak))
#endif

/* Publish-mode weak stub for ff_pendulum_cart.c: update() outputs 0;
 * init()/reset() are complete. Overridden by the real ff_pendulum_cart.c
 * when present (HIL_REAL_CONTROLLERS). */

static void ff_pendulum_cart_update_empty(controller_t *c, const float *setpoint,
                                          const float *measured, float *control, float dt)
{
    (void)setpoint;
    (void)measured;
    (void)dt;

    /*
     * TODO: collocated partial-FBL + feedforward for the pendulum-on-a-cart --
     *       write control[0] = cart force F.
     *   ff_pendulum_cart_t *s = c->params;
     *     model s->model (m1,m2,l,g), gains s->kp, s->kd,
     *     history s->prev_x, s->prev_theta, s->prev_sp, s->prev_sp_d1, s->have_prev.
     *   measured[0..1] = [x, theta];  setpoint[0] = x_d.
     *   steps:
     *     1. x', theta' via finite difference of measured; x_d', x_d'' via double FD of setpoint;
     *     2. a = x_d'' - Kd (x'-x_d') - Kp (x-x_d);
     *     3. F = (m1+m2 - m2 cos^2 theta) a - m2 g sin(theta) cos(theta) - m2 l sin(theta) theta'^2.
     * Placeholder below writes 0.
     */
    for (uint32_t i = 0; i < c->n; i++)
    {
        control[i] = 0.0f;
    }
}

static void ff_pendulum_cart_reset_empty(controller_t *c)
{
    ff_pendulum_cart_t *s = (ff_pendulum_cart_t *)c->params;
    s->have_prev = 0;
    s->prev_x = s->prev_theta = 0.0f;
    s->prev_sp = s->prev_sp_d1 = 0.0f;
}

__weak void ff_pendulum_cart_init(controller_t *ctrl, ff_pendulum_cart_t *c,
                                  const pendulum_cart_params_t *model, float kp, float kd)
{
    c->model = *model;
    c->kp = kp;
    c->kd = kd;
    c->have_prev = 0;
    c->prev_x = c->prev_theta = 0.0f;
    c->prev_sp = c->prev_sp_d1 = 0.0f;

    ctrl->n = 1;
    ctrl->update = ff_pendulum_cart_update_empty;
    ctrl->reset = ff_pendulum_cart_reset_empty;
    ctrl->params = c;
}
