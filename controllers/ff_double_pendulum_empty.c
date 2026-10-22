#include "ff_double_pendulum.h"

#ifndef __weak
#define __weak __attribute__((weak))
#endif

/* Publish-mode weak stub for ff_double_pendulum.c: update() outputs 0;
 * init()/reset() are complete. Overridden by the real ff_double_pendulum.c
 * when present (HIL_REAL_CONTROLLERS). */

static void ff_double_pendulum_update_empty(controller_t *c, const float *setpoint,
                                            const float *measured, float *control, float dt)
{
    (void)setpoint;
    (void)measured;
    (void)dt;

    /*
     * TODO: computed-torque + feedforward for the double pendulum -- write
     *       control[0..1] = joint torques [tau1, tau2].
     *   ff_double_pendulum_t *s = c->params;
     *     model s->model (m1,m2,l1,l2,g), gains s->kp, s->kd,
     *     history s->prev_q[2], s->prev_sp[2], s->prev_sp_d1[2], s->have_prev.
     *   measured[0..1] = [theta1, theta2];  setpoint[0..1] = [theta1d, theta2d].
     *   steps:
     *     1. q' via finite difference of measured; q_d', q_d'' via double FD of setpoint;
     *     2. a = q_d'' - Kd (q'-q_d') - Kp (q-q_d);
     *     3. tau = M(q) a + C(q,q') q' + g(q)   (inverse dynamics, see double_pendulum.h).
     * Placeholder below writes 0.
     */
    for (uint32_t i = 0; i < c->n; i++)
    {
        control[i] = 0.0f;
    }
}

static void ff_double_pendulum_reset_empty(controller_t *c)
{
    ff_double_pendulum_t *s = (ff_double_pendulum_t *)c->params;
    s->have_prev = 0;
    s->prev_q[0] = s->prev_q[1] = 0.0f;
    s->prev_sp[0] = s->prev_sp[1] = 0.0f;
    s->prev_sp_d1[0] = s->prev_sp_d1[1] = 0.0f;
}

__weak void ff_double_pendulum_init(controller_t *ctrl, ff_double_pendulum_t *c,
                                    const double_pendulum_params_t *model,
                                    float kp, float kd)
{
    c->model = *model;
    c->kp = kp;
    c->kd = kd;
    c->have_prev = 0;
    c->prev_q[0] = c->prev_q[1] = 0.0f;
    c->prev_sp[0] = c->prev_sp[1] = 0.0f;
    c->prev_sp_d1[0] = c->prev_sp_d1[1] = 0.0f;

    ctrl->n = 2;
    ctrl->update = ff_double_pendulum_update_empty;
    ctrl->reset = ff_double_pendulum_reset_empty;
    ctrl->params = c;
}
