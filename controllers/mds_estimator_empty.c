#include "mds_estimator.h"

#ifndef __weak
#define __weak __attribute__((weak))
#endif

/* Publish-mode weak stub for mds_estimator.c: update() outputs 0 (no excitation,
 * so no identification); init()/reset() are complete. Overridden by the real
 * mds_estimator.c when present (HIL_REAL_CONTROLLERS). */

static void mds_estimator_update_empty(controller_t *c, const float *setpoint,
                                       const float *measured, float *control, float dt)
{
    (void)setpoint;
    (void)measured;
    (void)dt;

    /*
     * TODO: parameter estimation with excitation (writes control[0]).
     *   mds_estimator_t *s = c->params;  known m/c/k in s->known, estimate s->est.
     *   1. Advance the 2nd-order x-filter (states xf1,xf2) and u-filter (uf1,uf2)
     *      by one Euler step: xi1' = xi2, xi2' = -lambda0 xi1 - lambda1 xi2 + w
     *      (w = measured x for the x-filter, s->last_u for the u-filter).
     *   2. x_f = xf1; v_f = xf2; a_f = x - lambda1 xf2 - lambda0 xf1; u_f = uf1.
     *   3. by s->which pick phi and y = u_f - (known terms):
     *        MASS:      phi = a_f, y = u_f - c v_f - k x_f
     *        DAMPING:   phi = v_f, y = u_f - m a_f - k x_f
     *        STIFFNESS: phi = x_f, y = u_f - m a_f - c v_f
     *   4. s->est += dt*gamma*phi*(y - est*phi)/(1+phi*phi);
     *   5. emit multisine u = sum amp[i]*sin(w[i]*t); s->last_u = u; s->t += dt;
     *      control[0] = u.
     * Placeholder below writes 0 (system is not excited).
     */
    for (uint32_t i = 0; i < c->n; i++)
    {
        control[i] = 0.0f;
    }
}

static void mds_estimator_reset_empty(controller_t *c)
{
    mds_estimator_t *s = (mds_estimator_t *)c->params;
    s->est = s->est0;
    s->xf1 = s->xf2 = 0.0f;
    s->uf1 = s->uf2 = 0.0f;
    s->last_u = 0.0f;
    s->t = 0.0f;
}

static void mds_estimator_setup_empty(controller_t *ctrl, mds_estimator_t *s,
                                      const mds_params_t *known, mds_est_param_t which,
                                      float est0, float gamma, float lambda0, float lambda1,
                                      const float *amp, const float *w)
{
    s->known = *known;
    s->which = which;
    s->est = s->est0 = est0;
    s->gamma = gamma;
    s->lambda0 = lambda0;
    s->lambda1 = lambda1;
    for (int i = 0; i < MDS_EST_NSINE; i++)
    {
        s->amp[i] = amp[i];
        s->w[i] = w[i];
    }
    s->xf1 = s->xf2 = 0.0f;
    s->uf1 = s->uf2 = 0.0f;
    s->last_u = 0.0f;
    s->t = 0.0f;

    ctrl->n = 1;
    ctrl->update = mds_estimator_update_empty;
    ctrl->reset = mds_estimator_reset_empty;
    ctrl->params = s;
}

__weak void mds_estimator_mass_init(controller_t *ctrl, mds_estimator_t *s,
                                    const mds_params_t *known, float est0,
                                    float gamma, float lambda0, float lambda1,
                                    const float *amp, const float *w)
{
    mds_estimator_setup_empty(ctrl, s, known, MDS_EST_MASS, est0, gamma, lambda0, lambda1, amp, w);
}

__weak void mds_estimator_damping_init(controller_t *ctrl, mds_estimator_t *s,
                                       const mds_params_t *known, float est0,
                                       float gamma, float lambda0, float lambda1,
                                       const float *amp, const float *w)
{
    mds_estimator_setup_empty(ctrl, s, known, MDS_EST_DAMPING, est0, gamma, lambda0, lambda1, amp, w);
}

__weak void mds_estimator_stiffness_init(controller_t *ctrl, mds_estimator_t *s,
                                         const mds_params_t *known, float est0,
                                         float gamma, float lambda0, float lambda1,
                                         const float *amp, const float *w)
{
    mds_estimator_setup_empty(ctrl, s, known, MDS_EST_STIFFNESS, est0, gamma, lambda0, lambda1, amp, w);
}
