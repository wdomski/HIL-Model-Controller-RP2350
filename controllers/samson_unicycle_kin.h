#ifndef SAMSON_UNICYCLE_KIN_H
#define SAMSON_UNICYCLE_KIN_H

#include "controller.h"
#include "trajectory.h"
#include "unicycle.h"

/*
 * Samson kinematic trajectory-tracking controller for the KINEMATIC unicycle
 * (models/unicycle.h: unicycle_model_init()), output u = [v, omega].
 *
 * Ported from Samson & Ait-Abderrahim (1991): the pose error is expressed in
 * the robot frame,
 *   e1 =  cos(th)(xr-x) + sin(th)(yr-y)
 *   e2 = -sin(th)(xr-x) + cos(th)(yr-y)
 *   e3 =  thr - th                         (wrapped to [-pi, pi])
 * and the control law is
 *   v     = vr cos(e3) + k1 e1
 *   omega = wr + k2 vr sinc(e3) e2 + k3 e3.
 * Unlike feedback linearisation, it tracks the FULL posture (x, y, theta) of
 * the robot centre and only requires vr != 0 (persistent excitation), not an
 * off-axle guidance point.
 *
 * Interface mapping (controller_t):
 *   measured[3] = kinematic-model state [x, y, theta]
 *   setpoint[2] = reference position [xr, yr]
 *   control[2]  = unicycle inputs [v, omega]
 * Exact trajectory derivatives are used when configured; otherwise they are
 * estimated by finite-differencing setpoint[].
 */
typedef struct
{
    unicycle_params_t geom;
    float k1, k2, k3;
     trajectory_t *traj[2]; /* optional exact reference derivatives */
     float traj_time;
    float prev_sp[2];      /* setpoint history      -> [xr', yr']  */
    float prev_sp_d1[2];   /* first-derivative hist -> [xr'', yr''] */
    int   have_prev;       /* staged: 0 -> 1 -> 2                   */
} samson_unicycle_kin_t;

void samson_unicycle_kin_init(controller_t *ctrl, samson_unicycle_kin_t *s,
                              const unicycle_params_t *geom,
                              float k1, float k2, float k3);

void samson_unicycle_kin_set_trajectory(samson_unicycle_kin_t *s,
                                         trajectory_t *traj_x,
                                         trajectory_t *traj_y);

#endif /* SAMSON_UNICYCLE_KIN_H */
