#include "mds_adaptive.h"

#ifndef __weak
#define __weak __attribute__((weak))
#endif

/* Publish-mode weak stub for mds_adaptive.c: update() outputs 0; init()/reset()
 * are complete. Overridden by the real mds_adaptive.c (HIL_REAL_CONTROLLERS). */

static void mds_adaptive_update_empty(controller_t *c, const float *setpoint,
                                      const float *measured, float *control, float dt)
{
    (void)setpoint;
    (void)measured;
    (void)dt;

    /*
     * TODO: adaptive MDS control law (writes control[0]).
     *   mds_adaptive_t *s = c->params;  known m/c/k in s->known, estimate s->est.
    *   1. x' = measured[1]; x_d', x_d'' = double FD(setpoint[0]) (stage on
     *      s->have_prev 0->1->2 so the feedforward waits for 2 samples).
     *   2. e = x - x_d;  e' = x' - x_d';  v = x_d'' - kd e' - kp e.
     *   3. by s->which:
     *        MASS:      u = est v + c x' + k x;   est += dt*(-gamma * v * e');
     *        DAMPING:   u = m v + est x' + k x;   est += dt*(-gamma * x' * e');
     *        STIFFNESS: u = m v + c x' + est x;   est += dt*(-2*gamma * x * e');
     *   4. control[0] = u.
     * Placeholder below writes 0.
     */
    for (uint32_t i = 0; i < c->n; i++)
    {
        control[i] = 0.0f;
    }
}

static void mds_adaptive_reset_empty(controller_t *c)
{
    mds_adaptive_t *s = (mds_adaptive_t *)c->params;
    s->est = s->est0;
    s->traj_time = 0.0f;
    s->have_prev = 0;
    s->prev_sp = s->prev_sp_d1 = 0.0f;
}

static void mds_adaptive_setup_empty(controller_t *ctrl, mds_adaptive_t *s,
                                     const mds_params_t *known, mds_adapt_param_t which,
                                     float est0, float gamma, float kp, float kd)
{
    s->known = *known;
    s->which = which;
    s->est = s->est0 = est0;
    s->gamma = gamma;
    s->kp = kp;
    s->kd = kd;
    s->traj = NULL;
    s->traj_time = 0.0f;
    s->have_prev = 0;
    s->prev_sp = s->prev_sp_d1 = 0.0f;

    ctrl->n = 1;
    ctrl->update = mds_adaptive_update_empty;
    ctrl->reset = mds_adaptive_reset_empty;
    ctrl->params = s;
}

__weak void mds_adaptive_mass_init(controller_t *ctrl, mds_adaptive_t *s,
                                   const mds_params_t *known, float est0,
                                   float gamma, float kp, float kd)
{
    mds_adaptive_setup_empty(ctrl, s, known, MDS_ADAPT_MASS, est0, gamma, kp, kd);
}

__weak void mds_adaptive_damping_init(controller_t *ctrl, mds_adaptive_t *s,
                                      const mds_params_t *known, float est0,
                                      float gamma, float kp, float kd)
{
    mds_adaptive_setup_empty(ctrl, s, known, MDS_ADAPT_DAMPING, est0, gamma, kp, kd);
}

__weak void mds_adaptive_stiffness_init(controller_t *ctrl, mds_adaptive_t *s,
                                        const mds_params_t *known, float est0,
                                        float gamma, float kp, float kd)
{
    mds_adaptive_setup_empty(ctrl, s, known, MDS_ADAPT_STIFFNESS, est0, gamma, kp, kd);
}

__weak void mds_adaptive_set_trajectory(mds_adaptive_t *s, trajectory_t *traj)
{
    s->traj = traj;
    s->traj_time = 0.0f;
    s->have_prev = 0;
}
