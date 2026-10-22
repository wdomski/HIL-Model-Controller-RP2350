#include "samson_unicycle_kin.h"

#ifndef __weak
#define __weak __attribute__((weak))
#endif

/* Publish-mode weak stub for samson_unicycle_kin.c: update() outputs 0;
 * init()/reset() are complete. Overridden by the real samson_unicycle_kin.c
 * when present (HIL_REAL_CONTROLLERS). */

static void samson_kin_update_empty(controller_t *c, const float *setpoint,
                                    const float *measured, float *control, float dt)
{
    (void)setpoint;
    (void)measured;
    (void)dt;

    /*
     * TODO: Samson kinematic tracker for the kinematic unicycle -- write control[0..1] = [v, omega].
     *   samson_unicycle_kin_t *s = c->params;
     *     gains s->k1,s->k2,s->k3; setpoint history s->prev_sp[2], s->prev_sp_d1[2], s->have_prev.
     *   measured[0..2] = [x, y, theta];  setpoint[0..1] = reference position [xr, yr].
     *   steps:
     *     1. xr',yr',xr'',yr'' via staged finite difference of setpoint;
     *     2. vr=hypot(xr',yr'); thr=atan2(yr',xr'); wr=(xr' yr'' - yr' xr'')/vr^2;
     *     3. robot-frame error e1,e2,e3=wrap(thr-theta);
     *     4. control[0]=vr cos(e3)+k1 e1;  control[1]=wr+k2 vr sinc(e3) e2+k3 e3.
     * Placeholder below writes 0.
     */
    for (uint32_t i = 0; i < c->n; i++)
    {
        control[i] = 0.0f;
    }
}

static void samson_kin_reset_empty(controller_t *c)
{
    samson_unicycle_kin_t *s = (samson_unicycle_kin_t *)c->params;
    s->traj_time = 0.0f;
    s->have_prev = 0;
    s->prev_sp[0] = s->prev_sp[1] = 0.0f;
    s->prev_sp_d1[0] = s->prev_sp_d1[1] = 0.0f;
}

__weak void samson_unicycle_kin_init(controller_t *ctrl, samson_unicycle_kin_t *s,
                                     const unicycle_params_t *geom,
                                     float k1, float k2, float k3)
{
    s->geom = *geom;
    s->k1 = k1;
    s->k2 = k2;
    s->k3 = k3;
    s->traj[0] = s->traj[1] = NULL;
    s->traj_time = 0.0f;
    s->have_prev = 0;
    s->prev_sp[0] = s->prev_sp[1] = 0.0f;
    s->prev_sp_d1[0] = s->prev_sp_d1[1] = 0.0f;

    ctrl->n = 2;
    ctrl->update = samson_kin_update_empty;
    ctrl->reset = samson_kin_reset_empty;
    ctrl->params = s;
}

__weak void samson_unicycle_kin_set_trajectory(samson_unicycle_kin_t *s,
                                                trajectory_t *traj_x,
                                                trajectory_t *traj_y)
{
    s->traj[0] = traj_x;
    s->traj[1] = traj_y;
    s->traj_time = 0.0f;
    s->have_prev = 0;
}
