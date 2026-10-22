#include <math.h>

#include "dynamic_lin_unicycle_dyn.h"
#include "ealgebra.h"

#ifndef __weak
#define __weak __attribute__((weak))
#endif

/* Publish-mode weak stub for dynamic_lin_unicycle_dyn.c: update() outputs 0;
 * init()/reset() are complete. Overridden by the real dynamic_lin_unicycle_dyn.c
 * when present (HIL_REAL_CONTROLLERS). */

static void dynamic_lin_dyn_update_empty(controller_t *c, const float *setpoint,
                                         const float *measured, float *control, float dt)
{
    (void)setpoint;
    (void)measured;
    (void)dt;

    /*
     * TODO: dynamic feedback-linearization for the dynamic (physics) unicycle at
     * delta = pi/2 -- write control[0..1] (wheel torques).
     *   dynamic_lin_unicycle_dyn_t *d = c->params;
     *     geometry d->geom, error-law gains d->k0/d->k1/d->k2, singularity guard d->chi1_min,
     *     compensator d->zeta, precomputed d->Einv[2][2],
     *     finite-difference buffers d->prev_hd/prev_hd1/prev_hd2/prev_hdot[2],
     *     flags d->have_prev_sp/d->have_prev_meas.
     *   measured[0..4] = [x, y, theta, v, omega];  setpoint[0..1] = hd.
     *   steps:
     *     1. chi1 = h' . [cos, sin];  guard |chi1| >= d->chi1_min (det Kd = chi1);
     *     2. hd'/hd''/hd''' by staged finite differences after 1/2/3 samples;
     *        h'' by finite-differencing measured h';
     *     3. e = h-hd, e' = h'-hd', e'' = h''-hd'';  f = Kd_dot*[zeta, omega];
     *     4. v = hd''' - d->k2*e'' - d->k1*e' - d->k0*e - f;  w = Kd^-1*v;
     *     5. tau = d->Einv * [zeta, w[1]];  d->zeta += w[0]*dt;  control[0..1] = tau.
     * Placeholder below writes 0.
     */
    for (uint32_t i = 0; i < c->n; i++)
    {
        control[i] = 0.0f;
    }
}

static void dynamic_lin_dyn_reset_empty(controller_t *c)
{
    dynamic_lin_unicycle_dyn_t *d = (dynamic_lin_unicycle_dyn_t *)c->params;
    d->zeta = 0.0f;
    d->traj_time = 0.0f;
    d->prev_hd[0]   = d->prev_hd[1]   = 0.0f;
    d->prev_hd1[0]  = d->prev_hd1[1]  = 0.0f;
    d->prev_hd2[0]  = d->prev_hd2[1]  = 0.0f;
    d->prev_hdot[0] = d->prev_hdot[1] = 0.0f;
    d->have_prev_sp = 0;
    d->have_prev_meas = 0;
}

__weak void dynamic_lin_unicycle_dyn_init(controller_t *ctrl, dynamic_lin_unicycle_dyn_t *d,
                                          const unicycle_params_t *geom,
                                          float k0, float k1, float k2)
{
    d->geom = *geom;
    d->k0 = k0; d->k1 = k1; d->k2 = k2;
    d->chi1_min = 1e-3f;
    d->traj[0] = d->traj[1] = NULL;

    /* E = M_vel * (M*^-1 B*); precompute E^-1 (constant, params-only). */
    float a, b;
    unicycle_reduced_mass(geom, &a, &b);
    float L  = geom->axle_length;
    float Rw = geom->wheel_radius;
    float d2 = a * a - b * b;
    float kD = (2.0f / Rw) / d2;               /* D = M*^-1 B* = kD [[a,-b],[-b,a]] */
    float D00 =  kD * a, D01 = -kD * b;        /* D symmetric: D = [[D00,D01],[D01,D00]] */
    float m00 = 1.0f - geom->e / L, m01 = 1.0f + geom->e / L;
    float m10 = 1.0f / L,            m11 = -1.0f / L;
    float E00 = m00 * D00 + m01 * D01;
    float E01 = m00 * D01 + m01 * D00;
    float E10 = m10 * D00 + m11 * D01;
    float E11 = m10 * D01 + m11 * D00;
    float detE = E00 * E11 - E01 * E10;
    d->Einv[0][0] =  E11 / detE;
    d->Einv[0][1] = -E01 / detE;
    d->Einv[1][0] = -E10 / detE;
    d->Einv[1][1] =  E00 / detE;

    ctrl->n = 2;
    ctrl->update = dynamic_lin_dyn_update_empty;
    ctrl->reset = dynamic_lin_dyn_reset_empty;
    ctrl->params = d;
    dynamic_lin_dyn_reset_empty(ctrl);
}

__weak void dynamic_lin_unicycle_dyn_set_trajectory(dynamic_lin_unicycle_dyn_t *d,
                                                     trajectory_t *traj_x,
                                                     trajectory_t *traj_y)
{
    d->traj[0] = traj_x;
    d->traj[1] = traj_y;
    d->traj_time = 0.0f;
    d->have_prev_sp = 0;
}
