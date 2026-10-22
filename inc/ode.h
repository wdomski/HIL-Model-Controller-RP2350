#ifndef ODE_H
#define ODE_H

#include <stdint.h>
#include "model.h"

/**
 * ODE solver instance (fixed-step RK4).
 * All buffer pointers must remain valid for the solver's lifetime.
 */
typedef struct {
    const model_t *model;
    uint32_t dt_us;       /* time step in microseconds */
    float dt;             /* time step in seconds (precomputed) */
    float *state;         /* current state vector [n_states] */
    float *k1;            /* RK4 scratch [n_states] */
    float *k2;            /* RK4 scratch [n_states] */
    float *k3;            /* RK4 scratch [n_states] */
    float *k4;            /* RK4 scratch [n_states] */
    float *x_temp;        /* RK4 temporary state [n_states] */
    uint64_t time_us;     /* accumulated simulation time */
} ode_solver_t;

/**
 * Initialize the ODE solver.
 *
 * @param solver   Pointer to solver instance
 * @param model    Pointer to model descriptor (must remain valid)
 * @param dt_us    Time step in microseconds
 * @param state    State buffer [n_states] with initial conditions
 * @param scratch  Scratch buffer [5 * n_states] for RK4 intermediates
 */
void ode_init(ode_solver_t *solver,
              const model_t *model,
              uint32_t dt_us,
              float *state,
              float *scratch);

/**
 * Advance the solver by one time step (RK4).
 * Updates solver->state in-place.
 *
 * @param solver  Pointer to solver instance
 * @param input   Control input vector [n_inputs]
 */
void ode_step(ode_solver_t *solver, const float *input);

/**
 * Compute model output for the current state.
 *
 * @param solver  Pointer to solver instance
 * @param input   Control input vector [n_inputs]
 * @param output  Output buffer [n_outputs]
 */
void ode_output(ode_solver_t *solver, const float *input, float *output);

/**
 * Terminate/reset the solver. Zeroes internal pointers.
 *
 * @param solver  Pointer to solver instance
 */
void ode_terminate(ode_solver_t *solver);

#endif /* ODE_H */
