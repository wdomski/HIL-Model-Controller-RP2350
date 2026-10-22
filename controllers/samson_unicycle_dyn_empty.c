#include "samson_unicycle_dyn.h"

#ifndef __weak
#define __weak __attribute__((weak))
#endif

/* Publish-mode weak stub for samson_unicycle_dyn.c: update() outputs 0;
 * init()/reset() are complete. Overridden by the real samson_unicycle_dyn.c
 * when present (HIL_REAL_CONTROLLERS). */

static void samson_dyn_update_empty(controller_t *c, const float *setpoint,
                                    const float *measured, float *control, float dt)
{
    (void)setpoint;
    (void)measured;
    (void)dt;

    /*
     * TODO: Samson tracker + inner computed-torque loop for the dynamic unicycle
     *       -- write control[0..1] = wheel torques [tau1, tau2].
     *   samson_unicycle_dyn_t *s = c->params;
     *     gains s->k1,s->k2,s->k3, inner gain s->kv; geometry s->geom.
     *   measured[0..4] = [x, y, theta, v, omega];  setpoint[0..1] = [xr, yr].
     *   steps:
     *     1. Samson outer loop (see samson_unicycle_kin) -> velocity command [vc, wc];
     *     2. eta_d = [(vc+L wc)/2, (vc-L wc)/2];  eta = [(v+L w)/2, (v-L w)/2];
     *     3. u = (Rw/2) M* kv (eta_d - eta), M* = [[a,b],[b,a]] via unicycle_reduced_mass().
     * Placeholder below writes 0.
     */
    for (uint32_t i = 0; i < c->n; i++)
    {
        control[i] = 0.0f;
    }
}

static void samson_dyn_reset_empty(controller_t *c)
{
    samson_unicycle_dyn_t *s = (samson_unicycle_dyn_t *)c->params;
    s->traj_time = 0.0f;
    s->have_prev = 0;
    s->prev_sp[0] = s->prev_sp[1] = 0.0f;
    s->prev_sp_d1[0] = s->prev_sp_d1[1] = 0.0f;
}

__weak void samson_unicycle_dyn_init(controller_t *ctrl, samson_unicycle_dyn_t *s,
                                     const unicycle_params_t *geom,
                                     float k1, float k2, float k3, float kv)
{
    s->geom = *geom;
    s->k1 = k1;
    s->k2 = k2;
    s->k3 = k3;
    s->kv = kv;
    s->traj[0] = s->traj[1] = NULL;
    s->traj_time = 0.0f;
    s->have_prev = 0;
    s->prev_sp[0] = s->prev_sp[1] = 0.0f;
    s->prev_sp_d1[0] = s->prev_sp_d1[1] = 0.0f;

    ctrl->n = 2;
    ctrl->update = samson_dyn_update_empty;
    ctrl->reset = samson_dyn_reset_empty;
    ctrl->params = s;
}

__weak void samson_unicycle_dyn_set_trajectory(samson_unicycle_dyn_t *s,
                                                trajectory_t *traj_x,
                                                trajectory_t *traj_y)
{
    s->traj[0] = traj_x;
    s->traj[1] = traj_y;
    s->traj_time = 0.0f;
    s->have_prev = 0;
}
