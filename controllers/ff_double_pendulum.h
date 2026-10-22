#ifndef FF_DOUBLE_PENDULUM_H
#define FF_DOUBLE_PENDULUM_H

#include "controller.h"
#include "double_pendulum.h"

/*
 * Computed-torque (inverse-dynamics) controller with feedforward for the fully
 * actuated double pendulum (models/double_pendulum.h). Tracks a joint-angle
 * reference q_d = [theta1d, theta2d] with
 *
 *   tau = M(q) (q_d'' - Kd e' - Kp e) + C(q,q') q' + g(q),   e = q - q_d,
 *
 * which cancels the nonlinear dynamics and leaves e'' + Kd e' + Kp e = 0.
 *
 * Interface mapping (controller_t):
 *   measured[2] = [theta1, theta2]
 *   setpoint[2] = [theta1d, theta2d]
 *   control[2]  = joint torques [tau1, tau2]
 * Joint velocities q' are estimated by finite-differencing the measured angles;
 * the feedforward q_d'/q_d'' by double finite-differencing the setpoint.
 */
typedef struct
{
    double_pendulum_params_t model;
    float kp, kd;
    float prev_q[2];       /* measured-angle history -> q'   */
    float prev_sp[2];      /* setpoint history       -> q_d' */
    float prev_sp_d1[2];   /* q_d' history           -> q_d''*/
    int   have_prev;       /* staged: 0 -> 1 -> 2            */
} ff_double_pendulum_t;

void ff_double_pendulum_init(controller_t *ctrl, ff_double_pendulum_t *c,
                             const double_pendulum_params_t *model,
                             float kp, float kd);

#endif /* FF_DOUBLE_PENDULUM_H */
