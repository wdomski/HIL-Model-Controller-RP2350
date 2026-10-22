#include <math.h>

#include "trajectory.h"

#ifndef TRAJ_PI
#define TRAJ_PI 3.14159265358979323846f
#endif

#ifndef __weak
#define __weak __attribute__((weak))
#endif

/* Publish-mode weak stub for trajectory.c: every eval() returns 0 and
 * trajectory_get() hands out stub descriptors. Overridden by the real
 * trajectory.c when present (HIL_REAL_CONTROLLERS). Implement each eval below. */

static float sin_eval_empty(const void *params, float t)
{
    (void)params;
    (void)t;

    return 0.0f;
}

static float pwm_eval_empty(const void *params, float t)
{
    (void)params;
    (void)t;

    return 0.0f;
}

static float triangle_eval_empty(const void *params, float t)
{
    (void)params;
    (void)t;

    return 0.0f;
}

static float saw_eval_empty(const void *params, float t)
{
    (void)params;
    (void)t;

    return 0.0f;
}

static float circle3d_eval_empty(const void *params, float t)
{
    (void)params;
    (void)t;

    return 0.0f;
}

/* 1st derivative (velocity) of circle3d_eval. */
static float circle3d_d1_eval_empty(const void *params, float t)
{
    (void)params;
    (void)t;

    return 0.0f;
}

/* 2nd derivative (acceleration) of circle3d_eval. */
static float circle3d_d2_eval_empty(const void *params, float t)
{
    (void)params;
    (void)t;

    return 0.0f;
}

/* Configurable predefined parameter sets (zeroed; real trajectory.c overrides). */
__weak traj_sin_params_t      traj_sin_params      = { 0 };
__weak traj_wave_params_t     traj_pwm_params      = { 0 };
__weak traj_wave_params_t     traj_triangle_params = { 0 };
__weak traj_wave_params_t     traj_saw_params      = { 0 };
__weak traj_circle3d_params_t traj_circle3d_params[3] = { 0 };

static trajectory_t predefined[TRAJ_COUNT] = {
    [TRAJ_SIN]         = { .eval = sin_eval_empty,         .params = &traj_sin_params },
    [TRAJ_PWM]         = { .eval = pwm_eval_empty,         .params = &traj_pwm_params },
    [TRAJ_TRIANGLE]    = { .eval = triangle_eval_empty,    .params = &traj_triangle_params },
    [TRAJ_SAW]         = { .eval = saw_eval_empty,         .params = &traj_saw_params },
    [TRAJ_CIRCLE3D_X]  = { .eval = circle3d_eval_empty, .d1 = circle3d_d1_eval_empty, .d2 = circle3d_d2_eval_empty, .params = &traj_circle3d_params[0] },
    [TRAJ_CIRCLE3D_Y]  = { .eval = circle3d_eval_empty, .d1 = circle3d_d1_eval_empty, .d2 = circle3d_d2_eval_empty, .params = &traj_circle3d_params[1] },
    [TRAJ_CIRCLE3D_Z]  = { .eval = circle3d_eval_empty, .d1 = circle3d_d1_eval_empty, .d2 = circle3d_d2_eval_empty, .params = &traj_circle3d_params[2] },
};

__weak trajectory_t *trajectory_get(trajectory_type_t type)
{
    if (type < 0 || type >= TRAJ_COUNT)
    {
        return NULL;
    }
    return &predefined[type];
}
