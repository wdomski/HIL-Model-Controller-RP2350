#ifndef UPRIGHT_PENDULUM_CART_H
#define UPRIGHT_PENDULUM_CART_H

#include "controller.h"
#include "pendulum_cart.h"

typedef enum
{
    UPRIGHT_PCART_SWING_UP = 0,
    UPRIGHT_PCART_STABILIZE = 1,
} upright_pendulum_cart_mode_t;

typedef struct
{
    float energy_gain;
    float energy_target_scale;
    float cart_kp;
    float cart_kd;
    float stabilize_k[4];
    float velocity_filter_tau;
    float kick_acceleration;
    float kick_duration;
    float catch_angle;
    float catch_rate;
    float catch_position;
    float release_angle;
    float release_rate;
    float release_position;
    float force_limit;
} upright_pendulum_cart_config_t;

typedef struct
{
    pendulum_cart_params_t model;
    upright_pendulum_cart_config_t config;
    upright_pendulum_cart_mode_t mode;
    float elapsed_s;
    float prev_x;
    float prev_theta;
    float xdot;
    float thetadot;
    int have_prev;
} upright_pendulum_cart_t;

void upright_pendulum_cart_init(
    controller_t *ctrl,
    upright_pendulum_cart_t *state,
    const pendulum_cart_params_t *model,
    const upright_pendulum_cart_config_t *config);

#endif /* UPRIGHT_PENDULUM_CART_H */