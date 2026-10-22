#include "ode.h"
#include <string.h>

void ode_init(ode_solver_t *solver,
              const model_t *model,
              uint32_t dt_us,
              float *state,
              float *scratch)
{
    solver->model   = model;
    solver->dt_us   = dt_us;
    solver->dt      = (float)dt_us * 1e-6f;
    solver->state   = state;
    solver->time_us = 0;

    uint32_t n = model->n_states;
    solver->k1     = scratch;
    solver->k2     = scratch + n;
    solver->k3     = scratch + 2 * n;
    solver->k4     = scratch + 3 * n;
    solver->x_temp = scratch + 4 * n;
}

void ode_step(ode_solver_t *solver, const float *input)
{
    const model_t *m = solver->model;
    const float dt = solver->dt;
    float *x = solver->state;
    const uint32_t n = m->n_states;

    /* k1 = f(x, u) */
    m->dynamics(x, input, solver->k1, m->params);

    /* x_temp = x + 0.5 * dt * k1 */
    for (uint32_t i = 0; i < n; i++)
        solver->x_temp[i] = x[i] + 0.5f * dt * solver->k1[i];

    /* k2 = f(x_temp, u) */
    m->dynamics(solver->x_temp, input, solver->k2, m->params);

    /* x_temp = x + 0.5 * dt * k2 */
    for (uint32_t i = 0; i < n; i++)
        solver->x_temp[i] = x[i] + 0.5f * dt * solver->k2[i];

    /* k3 = f(x_temp, u) */
    m->dynamics(solver->x_temp, input, solver->k3, m->params);

    /* x_temp = x + dt * k3 */
    for (uint32_t i = 0; i < n; i++)
        solver->x_temp[i] = x[i] + dt * solver->k3[i];

    /* k4 = f(x_temp, u) */
    m->dynamics(solver->x_temp, input, solver->k4, m->params);

    /* x = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4) */
    const float dt6 = dt / 6.0f;
    for (uint32_t i = 0; i < n; i++)
        x[i] += dt6 * (solver->k1[i]
                      + 2.0f * solver->k2[i]
                      + 2.0f * solver->k3[i]
                      + solver->k4[i]);

    solver->time_us += solver->dt_us;
}

void ode_output(ode_solver_t *solver, const float *input, float *output)
{
    const model_t *m = solver->model;
    m->output(solver->state, input, output, m->params);
}

void ode_terminate(ode_solver_t *solver)
{
    memset(solver, 0, sizeof(ode_solver_t));
}
