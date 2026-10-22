#ifndef DYNAMIC_LIN_UNICYCLE_KIN_H
#define DYNAMIC_LIN_UNICYCLE_KIN_H

#include "controller.h"

/*
 * Dynamic feedback-linearization controller for the kinematic unicycle.
 *
 * Ported from the reference Python `SimulatorDynamics` (dynamicLinFull.py) with
 * output point delta = pi/2: h(q) = [x - e*sin(theta), y + e*cos(theta)].
 * An integrator state chi1 = v - e*omega extends the system so that the output
 * has a well-defined vector relative degree; det(Kd) = chi1 (keep it nonzero).
 *
 * Interface mapping (controller_t):
 *   measured[] = full state [x, y, theta]
 *   setpoint[] = desired output point [hx, hy]
 *   control[]  = unicycle inputs [v, omega]
 * The feedforward hd' and hd'' are estimated by finite-differencing the setpoint.
 */
typedef struct
{
    float e;
    float kp;
    float kd;
    float chi1;          /* integrator state: chi1 = v - e*omega */
    float prev_hd[2];
    float prev_hd_d1[2];
    int   have_prev;
} dynamic_lin_unicycle_kin_t;

void dynamic_lin_unicycle_kin_init(controller_t *ctrl, dynamic_lin_unicycle_kin_t *dl,
                                   float e, float kp, float kd, float chi1_init);

#endif /* DYNAMIC_LIN_UNICYCLE_KIN_H */
