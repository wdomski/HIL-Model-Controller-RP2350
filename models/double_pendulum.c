#include <math.h>

#include "double_pendulum.h"

/**
 * Dynamics: dx/dt = f(x, u, params)
 *
 * State:  x[0]=theta1, x[1]=theta2, x[2]=w1, x[3]=w2
 * Input:  u[0]=tau1, u[1]=tau2  (joint torques)
 *
 * Solve M(q) q'' = b, b = tau - C(q,q') q' - g(q):
 *   b1 = tau1 - m2 l1 l2 sin(d) w2^2 - (m1+m2) g l1 sin(theta1)
 *   b2 = tau2 + m2 l1 l2 sin(d) w1^2 - m2 g l2 sin(theta2)
 * det(M) = M11*M22 - M12^2 > 0.
 */
static void double_pendulum_dynamics(const float *state,
                                     const float *input,
                                     float *dxdt,
                                     const void *params)
{
    const double_pendulum_params_t *p = (const double_pendulum_params_t *)params;

    float th1 = state[0];
    float th2 = state[1];
    float w1  = state[2];
    float w2  = state[3];
    float tau1 = input[0];
    float tau2 = input[1];

    float m1 = p->mass1;
    float m2 = p->mass2;
    float l1 = p->length1;
    float l2 = p->length2;
    float g  = p->gravity;

    float d  = th1 - th2;
    float sd = sinf(d);
    float cd = cosf(d);

    float M11 = (m1 + m2) * l1 * l1;
    float M12 = m2 * l1 * l2 * cd;
    float M22 = m2 * l2 * l2;
    float det = M11 * M22 - M12 * M12;

    float b1 = tau1 - m2 * l1 * l2 * sd * w2 * w2 - (m1 + m2) * g * l1 * sinf(th1);
    float b2 = tau2 + m2 * l1 * l2 * sd * w1 * w1 - m2 * g * l2 * sinf(th2);

    dxdt[0] = w1;
    dxdt[1] = w2;
    dxdt[2] = ( M22 * b1 - M12 * b2) / det;  /* theta1'' */
    dxdt[3] = (-M12 * b1 + M11 * b2) / det;  /* theta2'' */
}

/**
 * Output: y = [theta1, theta2]
 */
static void double_pendulum_output(const float *state,
                                   const float *input,
                                   float *output,
                                   const void *params)
{
    (void)input;
    (void)params;

    output[0] = state[0];  /* theta1 */
    output[1] = state[1];  /* theta2 */
}

void double_pendulum_model_init(model_t *model, const double_pendulum_params_t *params)
{
    model->n_states  = DOUBLE_PENDULUM_N_STATES;
    model->n_inputs  = DOUBLE_PENDULUM_N_INPUTS;
    model->n_outputs = DOUBLE_PENDULUM_N_OUTPUTS;
    model->dynamics  = double_pendulum_dynamics;
    model->output    = double_pendulum_output;
    model->params    = params;
}
