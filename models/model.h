#ifndef MODEL_H
#define MODEL_H

#include <stdint.h>

/**
 * Function pointer type: compute state derivatives dx/dt = f(x, u, params).
 *
 * @param state   Current state vector [n_states]
 * @param input   Control input vector [n_inputs]
 * @param dxdt    Output derivative vector [n_states]
 * @param params  Model-specific parameters (cast internally)
 */
typedef void (*model_dynamics_fn)(const float *state,
                                  const float *input,
                                  float *dxdt,
                                  const void *params);

/**
 * Function pointer type: compute model output y = g(x, u, params).
 *
 * @param state   Current state vector [n_states]
 * @param input   Control input vector [n_inputs]
 * @param output  Output vector [n_outputs]
 * @param params  Model-specific parameters (cast internally)
 */
typedef void (*model_output_fn)(const float *state,
                                const float *input,
                                float *output,
                                const void *params);

/**
 * Model descriptor — virtual table for a dynamic model.
 */
typedef struct {
    uint32_t n_states;
    uint32_t n_inputs;
    uint32_t n_outputs;
    model_dynamics_fn dynamics;
    model_output_fn output;
    const void *params;
} model_t;

#endif /* MODEL_H */
