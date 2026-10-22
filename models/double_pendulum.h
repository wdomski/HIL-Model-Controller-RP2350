#ifndef DOUBLE_PENDULUM_H
#define DOUBLE_PENDULUM_H

#include "model.h"

#define DOUBLE_PENDULUM_N_STATES  4
#define DOUBLE_PENDULUM_N_INPUTS  2
#define DOUBLE_PENDULUM_N_OUTPUTS 2

/**
 * Double pendulum (fixed pivot) with point masses m1, m2 on massless rods
 * l1, l2. Coordinates q = [theta1, theta2], measured from the downward
 * vertical. Optional joint torques tau = [tau1, tau2] (0 => free pendulum).
 *
 * M(q) q'' + C(q,q') q' + g(q) = tau, with
 *   M11 = (m1+m2) l1^2,  M12 = m2 l1 l2 cos(theta1-theta2),  M22 = m2 l2^2
 *   [C q']_1 =  m2 l1 l2 sin(d) w2^2      (d = theta1 - theta2)
 *   [C q']_2 = -m2 l1 l2 sin(d) w1^2
 *   g1 = (m1+m2) g l1 sin(theta1),  g2 = m2 g l2 sin(theta2)
 */
typedef struct {
    float mass1;    /* m1 [kg]    */
    float mass2;    /* m2 [kg]    */
    float length1;  /* l1 [m]     */
    float length2;  /* l2 [m]     */
    float gravity;  /* g  [m/s^2] */
} double_pendulum_params_t;

void double_pendulum_model_init(model_t *model, const double_pendulum_params_t *params);

#endif /* DOUBLE_PENDULUM_H */
