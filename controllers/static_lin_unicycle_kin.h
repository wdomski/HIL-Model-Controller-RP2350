#ifndef STATIC_LIN_UNICYCLE_KIN_H
#define STATIC_LIN_UNICYCLE_KIN_H

#include "controller.h"
#include "unicycle.h"

/*
 * Static feedback-linearization controller for the KINEMATIC unicycle
 * (models/unicycle.h: unicycle_model_init()), input u = [v, omega].
 *
 * Ported from the reference Python `SimulatorKinematics` (staticLinFull.py):
 * the output point h(q) = [x + e*cos(theta+delta), y + e*sin(theta+delta)]
 * has vector relative degree 1 here, so a P law suffices -- the dynamics
 * cancel via u = R*(hd' - Kp*(h-hd)), with h(q)/R from the shared
 * unicycle_static_linearization() (no Mh/Ch/Bh, unlike static_lin_unicycle_dyn).
 *
 * Interface mapping (controller_t):
 *   measured[] = kinematic-model state [x, y, theta]
 *   setpoint[] = desired output point [hxd, hyd]
 *   control[]  = unicycle inputs [v, omega]
 * Feedforward hd' is estimated by single finite-differencing setpoint[].
 */
typedef struct
{
    unicycle_params_t geom;
    float kp;
    float prev_hd[2];
    int   have_prev;
} static_lin_unicycle_kin_t;

void static_lin_unicycle_kin_init(controller_t *ctrl, static_lin_unicycle_kin_t *sk,
                                  const unicycle_params_t *geom, float kp);

#endif /* STATIC_LIN_UNICYCLE_KIN_H */
