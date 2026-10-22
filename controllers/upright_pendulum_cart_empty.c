#include "upright_pendulum_cart.h"

#ifndef __weak
#define __weak __attribute__((weak))
#endif

static void upright_pendulum_cart_update_empty(
    controller_t *ctrl,
    const float *setpoint,
    const float *measured,
    float *control,
    float dt)
{
    (void)setpoint;
    (void)measured;
    (void)dt;
    for (uint32_t channel = 0; channel < ctrl->n; channel++)
    {
        control[channel] = 0.0f;
    }
}

static void upright_pendulum_cart_reset_empty(controller_t *ctrl)
{
    upright_pendulum_cart_t *state =
        (upright_pendulum_cart_t *)ctrl->params;
    state->mode = UPRIGHT_PCART_SWING_UP;
    state->elapsed_s = 0.0f;
    state->prev_x = 0.0f;
    state->prev_theta = 0.0f;
    state->xdot = 0.0f;
    state->thetadot = 0.0f;
    state->have_prev = 0;
}

__weak void upright_pendulum_cart_init(
    controller_t *ctrl,
    upright_pendulum_cart_t *state,
    const pendulum_cart_params_t *model,
    const upright_pendulum_cart_config_t *config)
{
    state->model = *model;
    state->config = *config;
    ctrl->n = 1;
    ctrl->update = upright_pendulum_cart_update_empty;
    ctrl->reset = upright_pendulum_cart_reset_empty;
    ctrl->params = state;
    upright_pendulum_cart_reset_empty(ctrl);
}