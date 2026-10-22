#include "mds.h"

/**
 * MDS dynamics: dx/dt = f(x, u, params)
 *
 * State:  x[0] = position, x[1] = velocity
 * Input:  u[0] = external force F
 *
 * dx[0]/dt = x[1]
 * dx[1]/dt = (F - c*x[1] - k*x[0]) / m
 */
static void mds_dynamics(const float *state,
                         const float *input,
                         float *dxdt,
                         const void *params)
{
    const mds_params_t *p = (const mds_params_t *)params;

    float x = state[0];
    float v = state[1];
    float F = input[0];

    dxdt[0] = v;
    dxdt[1] = (F - p->damping * v - p->stiffness * x) / p->mass;
}

/**
 * MDS output: y = g(x, u, params)
 *
 * y[0] = position, y[1] = velocity
 */
static void mds_output(const float *state,
                       const float *input,
                       float *output,
                       const void *params)
{
    (void)input;
    (void)params;

    output[0] = state[0];
    output[1] = state[1];
}

void mds_model_init(model_t *model, const mds_params_t *params)
{
    model->n_states  = MDS_N_STATES;
    model->n_inputs  = MDS_N_INPUTS;
    model->n_outputs = MDS_N_OUTPUTS;
    model->dynamics  = mds_dynamics;
    model->output    = mds_output;
    model->params    = params;
}
