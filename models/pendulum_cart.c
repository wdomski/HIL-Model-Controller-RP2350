#include <math.h>

#include "pendulum_cart.h"

/**
 * Dynamics: dx/dt = f(x, u, params)
 *
 * State:  x[0]=x, x[1]=theta, x[2]=x', x[3]=theta'
 * Input:  u[0]=F (horizontal force on cart)
 *
 * Solve M(q) q'' = b, where
 *   b1 = F + m2*l*sin(theta)*theta'^2
 *   b2 = -m2*g*l*sin(theta)
 * det(M) = m2*l^2*(m1 + m2*sin^2(theta)) > 0.
 */
static void pendulum_cart_dynamics(const float *state,
                                   const float *input,
                                   float *dxdt,
                                   const void *params)
{
    const pendulum_cart_params_t *p = (const pendulum_cart_params_t *)params;

    float theta    = state[1];
    float xdot     = state[2];
    float thetadot = state[3];
    float F        = input[0];

    float m1 = p->cart_mass;
    float m2 = p->bob_mass;
    float l  = p->length;
    float g  = p->gravity;

    float s = sinf(theta);
    float c = cosf(theta);

    float M11 = m1 + m2;
    float M12 = m2 * l * c;
    float M22 = m2 * l * l;
    float det = M11 * M22 - M12 * M12;

    float b1 = F + m2 * l * s * thetadot * thetadot;
    float b2 = -m2 * g * l * s;

    dxdt[0] = xdot;
    dxdt[1] = thetadot;
    dxdt[2] = ( M22 * b1 - M12 * b2) / det;  /* x''     */
    dxdt[3] = (-M12 * b1 + M11 * b2) / det;  /* theta'' */
}

/**
 * Output: y = [cart position, pendulum angle]
 */
static void pendulum_cart_output(const float *state,
                                 const float *input,
                                 float *output,
                                 const void *params)
{
    (void)input;
    (void)params;

    output[0] = state[0];
    output[1] = atan2f(sinf(state[1]), cosf(state[1]));
}

void pendulum_cart_model_init(model_t *model, const pendulum_cart_params_t *params)
{
    model->n_states  = PENDULUM_CART_N_STATES;
    model->n_inputs  = PENDULUM_CART_N_INPUTS;
    model->n_outputs = PENDULUM_CART_N_OUTPUTS;
    model->dynamics  = pendulum_cart_dynamics;
    model->output    = pendulum_cart_output;
    model->params    = params;
}
