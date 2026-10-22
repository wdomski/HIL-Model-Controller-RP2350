#include "mds_robust.h"

#ifndef __weak
#define __weak __attribute__((weak))
#endif

/* Publish-mode weak stub for mds_robust.c: update() outputs 0; init()/reset()
 * are complete. Overridden by the real mds_robust.c (HIL_REAL_CONTROLLERS). */

static void mds_robust_update_empty(controller_t *c, const float *setpoint,
                                    const float *measured, float *control, float dt)
{
    (void)setpoint;
    (void)measured;
    (void)dt;

    /*
     * TODO: robust MDS control law (writes control[0]).
     *   mds_robust_t *s = c->params;  nominal m0/c0/k0 in s->nominal, bound s->delta.
    *   1. x' = measured[1]; x_d', x_d'' = double FD(setpoint[0]).
     *   2. e = x - x_d;  e' = x' - x_d';  v = x_d'' - kd e' - kp e.
     *   3. unom = m0 v + c0 x' + k0 x.
     *   4. phi by s->which:  MASS -> v,  DAMPING -> x',  STIFFNESS -> x.
     *   5. u_r = -delta * fabsf(phi) * tanhf(e' / epsilon);
     *   6. control[0] = unom + u_r.
     * Placeholder below writes 0.
     */
    for (uint32_t i = 0; i < c->n; i++)
    {
        control[i] = 0.0f;
    }
}

static void mds_robust_reset_empty(controller_t *c)
{
    mds_robust_t *s = (mds_robust_t *)c->params;
    s->traj_time = 0.0f;
    s->have_prev = 0;
    s->prev_sp = s->prev_sp_d1 = 0.0f;
}

static void mds_robust_setup_empty(controller_t *ctrl, mds_robust_t *s,
                                   const mds_params_t *nominal, mds_robust_param_t which,
                                   float delta, float kp, float kd, float epsilon)
{
    s->nominal = *nominal;
    s->which = which;
    s->delta = delta;
    s->kp = kp;
    s->kd = kd;
    s->epsilon = (epsilon > 0.0f) ? epsilon : 1e-3f;
    s->traj = NULL;
    s->traj_time = 0.0f;
    s->have_prev = 0;
    s->prev_sp = s->prev_sp_d1 = 0.0f;

    ctrl->n = 1;
    ctrl->update = mds_robust_update_empty;
    ctrl->reset = mds_robust_reset_empty;
    ctrl->params = s;
}

__weak void mds_robust_mass_init(controller_t *ctrl, mds_robust_t *s,
                                 const mds_params_t *nominal, float delta,
                                 float kp, float kd, float epsilon)
{
    mds_robust_setup_empty(ctrl, s, nominal, MDS_ROBUST_MASS, delta, kp, kd, epsilon);
}

__weak void mds_robust_damping_init(controller_t *ctrl, mds_robust_t *s,
                                    const mds_params_t *nominal, float delta,
                                    float kp, float kd, float epsilon)
{
    mds_robust_setup_empty(ctrl, s, nominal, MDS_ROBUST_DAMPING, delta, kp, kd, epsilon);
}

__weak void mds_robust_stiffness_init(controller_t *ctrl, mds_robust_t *s,
                                      const mds_params_t *nominal, float delta,
                                      float kp, float kd, float epsilon)
{
    mds_robust_setup_empty(ctrl, s, nominal, MDS_ROBUST_STIFFNESS, delta, kp, kd, epsilon);
}

__weak void mds_robust_set_trajectory(mds_robust_t *s, trajectory_t *traj)
{
    s->traj = traj;
    s->traj_time = 0.0f;
    s->have_prev = 0;
}
