#ifndef STATIC_LIN_UNICYCLE_DYN_H
#define STATIC_LIN_UNICYCLE_DYN_H

#include "controller.h"
#include "unicycle.h"

/*
 * Static feedback-linearization controller for the DYNAMIC unicycle model
 * (models/unicycle.h: unicycle_dyn_model_init()).
 *
 * Ported from the reference Python `SimulatorDynamics` (staticLinFull.py) --
 * NOT `SimulatorKinematics`: the plant has mass/inertia, so tracking the
 * output point h(q) = [x + e*cos(theta+delta), y + e*sin(theta+delta)] needs
 * a PD law and the dynamics cancelled through Mh/Ch/Bh (see
 * unicycle_dynamic_linearization()), not just u = R*(hd' - Kp*(h-hd)).
 *
 * Interface mapping (controller_t):
 *   measured[] = dynamics-model output [x, y, theta, v, omega]
 *   setpoint[] = desired output point [hxd, hyd]
 *   control[]  = wheel torques [tau1, tau2]
 * Feedforward hd'/hd'' are estimated by double finite-differencing setpoint[].
 */
typedef struct
{
    unicycle_params_t geom;
    float kp;
    float kd;
    float prev_hd[2];
    float prev_hd_d1[2];
    int   have_prev;
} static_lin_unicycle_dyn_t;

void static_lin_unicycle_dyn_init(controller_t *ctrl, static_lin_unicycle_dyn_t *sl,
                                  const unicycle_params_t *geom, float kp, float kd);

#endif /* STATIC_LIN_UNICYCLE_DYN_H */
