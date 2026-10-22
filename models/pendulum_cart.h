#ifndef PENDULUM_CART_H
#define PENDULUM_CART_H

#include "model.h"

#define PENDULUM_CART_N_STATES  4
#define PENDULUM_CART_N_INPUTS  1
#define PENDULUM_CART_N_OUTPUTS 2

/**
 * Pendulum-on-a-cart parameters.
 *
 * Coordinates q = [x, theta]; bob at [x + l*sin(theta), -l*cos(theta)]
 * (theta = 0 hanging down, theta = pi inverted). Input is a horizontal force
 * on the cart.
 *
 * M(q) q'' + C(q,q') q' + G(q) = [F, 0]^T, with
 *   M = [[m1+m2,       m2*l*cos(theta)],
 *        [m2*l*cos(t), m2*l^2         ]]
 *   C q' = [-m2*l*sin(theta)*theta'^2, 0]^T
 *   G    = [0, m2*g*l*sin(theta)]^T
 */
typedef struct {
    float cart_mass;  /* m1 [kg]      */
    float bob_mass;   /* m2 [kg]      */
    float length;     /* l  [m]       */
    float gravity;    /* g  [m/s^2]   */
} pendulum_cart_params_t;

void pendulum_cart_model_init(model_t *model, const pendulum_cart_params_t *params);

#endif /* PENDULUM_CART_H */
