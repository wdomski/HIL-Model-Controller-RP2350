#ifndef DYNAMIC_LIN_UNICYCLE_DYN_H
#define DYNAMIC_LIN_UNICYCLE_DYN_H

#include "controller.h"
#include "unicycle.h"
#include "trajectory.h"

/*
 * Dynamic feedback-linearisation controller for the DYNAMIC unicycle
 * (models/unicycle.h: unicycle_dyn_model_init()), at delta = pi/2.
 *
 * Synthesises the two lectures: the auxiliary-velocity dynamics
 * (M* eta' = B* u, ARC-StaticLinearisation.tex) with the dynamic-decoupling
 * method (ARC-DynamicLinearisation.tex). At delta = pi/2 the static decoupling
 * is singular (det(dh/dq G) ~ cos(delta) = 0), so the state is extended with
 * zeta = chi1' (chi1 = v - e*omega); the output point
 * h(q) = [x - e sin(theta), y + e cos(theta)] then has vector relative degree 3:
 *
 *   h''' = f + Kd(q) w,   f = Kd_dot [zeta, omega],
 *   Kd = [[cos th, -chi1 sin th],[sin th, chi1 cos th]],  det Kd = chi1 != 0,
 *   w  = [chi1'', omega']^T.
 *
 * Control law (Hurwitz cubic error dynamics e''' + K2 e'' + K1 e' + K0 e = 0):
 *   w = Kd^-1 (h'''d - K2 e'' - K1 e' - K0 e - f),   e = h - h_d.
 * Real wheel torques are recovered from [chi1', omega'] via
 *   tau = E^-1 [zeta, w2],   E = M_vel * M*^-1 * B*   (precomputed at init).
 *
 * Interface mapping (controller_t):
 *   measured[5] = [x, y, theta, v, omega]
 *   setpoint[2] = desired output point [hxd, hyd]
 *   control[2]  = wheel torques [tau1, tau2]
 * Feedforward hd'/hd''/hd''' are estimated by finite-differencing setpoint[];
 * h'' by finite-differencing the measured h'. The system must stay on the move
 * (chi1 = v - e*omega != 0, else det Kd -> 0).
 */
typedef struct
{
    unicycle_params_t geom;
    float Einv[2][2];   /* precomputed inv(E), E = M_vel * M*^-1 * B* */
    float k0, k1, k2;   /* error-law gains (Hurwitz cubic)            */
    float chi1_min;     /* singularity guard: |det Kd| = |chi1|       */
    trajectory_t *traj[2]; /* optional exact hd through hd'''         */
    float traj_time;
    /* dynamic compensator: zeta = chi1' (zeta' = w1) */
    float zeta;
    /* finite-difference buffers */
    float prev_hd[2];     /* setpoint history -> hd'                    */
    float prev_hd1[2];    /*            history -> hd''                 */
    float prev_hd2[2];    /*            history -> hd'''                */
    float prev_hdot[2];   /* measured h' history -> h''                 */
    int   have_prev_sp;   /* setpoint samples available, saturated at 3 */
    int   have_prev_meas;
} dynamic_lin_unicycle_dyn_t;

void dynamic_lin_unicycle_dyn_init(controller_t *ctrl, dynamic_lin_unicycle_dyn_t *d,
                                   const unicycle_params_t *geom,
                                   float k0, float k1, float k2);

void dynamic_lin_unicycle_dyn_set_trajectory(dynamic_lin_unicycle_dyn_t *d,
                                              trajectory_t *traj_x,
                                              trajectory_t *traj_y);

#endif /* DYNAMIC_LIN_UNICYCLE_DYN_H */
