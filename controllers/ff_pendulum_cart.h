#ifndef FF_PENDULUM_CART_H
#define FF_PENDULUM_CART_H

#include "controller.h"
#include "pendulum_cart.h"

/*
 * Collocated partial-feedback-linearisation controller with feedforward for the
 * UNDERACTUATED pendulum-on-a-cart (models/pendulum_cart.h). Tracks the cart
 * position x -> x_d with the single force F; the pendulum is left as internal
 * (zero) dynamics, marginally stable about the hanging equilibrium in the
 * frictionless model.
 *
 * With desired cart acceleration a = x_d'' - Kd (x' - x_d') - Kp (x - x_d) and
 * the pendulum equation theta'' = -(cos(theta) x'' + g sin(theta)) / l
 * eliminated, the cart equation gives
 *
 *   F = (m1 + m2 - m2 cos^2 theta) a - m2 g sin(theta) cos(theta)
 *       - m2 l sin(theta) theta'^2.
 *
 * Interface mapping (controller_t):
 *   measured[2] = [x, theta]
 *   setpoint[1] = [x_d]
 *   control[1]  = cart force [F]
 * x' and theta' are estimated by finite-differencing the measured signals; the
 * feedforward x_d'/x_d'' by double finite-differencing the setpoint.
 */
typedef struct
{
    pendulum_cart_params_t model;
    float kp, kd;
    float prev_x, prev_theta;   /* measured history -> x', theta'  */
    float prev_sp, prev_sp_d1;  /* setpoint history -> x_d', x_d''  */
    int   have_prev;            /* staged: 0 -> 1 -> 2             */
} ff_pendulum_cart_t;

void ff_pendulum_cart_init(controller_t *ctrl, ff_pendulum_cart_t *c,
                           const pendulum_cart_params_t *model, float kp, float kd);

#endif /* FF_PENDULUM_CART_H */
