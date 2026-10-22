#ifndef SAMSON_UNICYCLE_DYN_H
#define SAMSON_UNICYCLE_DYN_H

#include "controller.h"
#include "trajectory.h"
#include "unicycle.h"

/*
 * Samson kinematic tracking law wrapped around the DYNAMIC unicycle
 * (models/unicycle.h: unicycle_dyn_model_init()), output = wheel torques.
 *
 * The outer loop is the same Samson posture tracker as samson_unicycle_kin
 * (produces a velocity command [vc, wc]); an inner computed-torque velocity
 * loop turns it into wheel torques. Desired / actual auxiliary velocities are
 *   eta_d = [(vc + L wc)/2, (vc - L wc)/2],  eta = [(v + L w)/2, (v - L w)/2],
 * and with M* eta' = B* u (C* == 0, B* = (2/Rw) I) a proportional velocity law
 *   u = (Rw/2) M* kv (eta_d - eta)
 * drives eta -> eta_d, where M* = [[a,b],[b,a]] from unicycle_reduced_mass().
 *
 * Interface mapping (controller_t):
 *   measured[5] = dynamic-model output [x, y, theta, v, omega]
 *   setpoint[2] = reference position [xr, yr]
 *   control[2]  = wheel torques [tau1, tau2]
 * Exact trajectory derivatives are used when configured; otherwise reference
 * thr/vr/wr are estimated by finite-differencing setpoint[].
 */
typedef struct
{
    unicycle_params_t geom;
    float k1, k2, k3;   /* Samson posture gains     */
    float kv;           /* inner velocity-loop gain */
    trajectory_t *traj[2]; /* optional exact reference derivatives */
    float traj_time;
    float prev_sp[2];
    float prev_sp_d1[2];
    int   have_prev;
} samson_unicycle_dyn_t;

void samson_unicycle_dyn_init(controller_t *ctrl, samson_unicycle_dyn_t *s,
                              const unicycle_params_t *geom,
                              float k1, float k2, float k3, float kv);

void samson_unicycle_dyn_set_trajectory(samson_unicycle_dyn_t *s,
                                         trajectory_t *traj_x,
                                         trajectory_t *traj_y);

#endif /* SAMSON_UNICYCLE_DYN_H */
