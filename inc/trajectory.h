#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stddef.h>

/*
 * Generic setpoint-trajectory interface.
 *
 * A trajectory maps time t [s] to a scalar value. Concrete trajectories fill in
 * an eval function pointer and a pointer to their parameter structure, so the
 * caller only sees this generic descriptor and can swap trajectories freely.
 *
 * `d1`/`d2`/`d3` are optional analytic derivatives (1st/2nd/3rd, e.g. velocity and
 * acceleration for a position trajectory). They are NULL when a trajectory
 * only defines a position, in which case trajectory_eval_d1()/_d2() return 0
 * and a caller needing feedforward derivatives must estimate them itself
 * (e.g. finite differences). This lets exact and position-only trajectories
 * share one interface.
 */

typedef enum
{
    TRAJ_SIN = 0,      /* A*sin(pi*omega*t) + b               */
    TRAJ_PWM = 1,      /* square wave (freq, duty, min, max)  */
    TRAJ_TRIANGLE = 2, /* triangle wave (freq, min, max)      */
    TRAJ_SAW = 3,      /* sawtooth wave (freq, min, max)      */
    TRAJ_CIRCLE3D_X = 4, /* 3D circle, x coordinate           */
    TRAJ_CIRCLE3D_Y = 5, /* 3D circle, y coordinate           */
    TRAJ_CIRCLE3D_Z = 6, /* 3D circle, z coordinate           */
    TRAJ_COUNT
} trajectory_type_t;

/* Evaluate a trajectory value (or derivative) at time t [s] from its parameter structure. */
typedef float (*trajectory_eval_fn)(const void *params, float t);

typedef struct
{
    trajectory_eval_fn eval; /* position (required)                      */
    trajectory_eval_fn d1;   /* 1st derivative, e.g. velocity (optional) */
    trajectory_eval_fn d2;   /* 2nd derivative, e.g. acceleration (optional) */
    trajectory_eval_fn d3;   /* 3rd derivative, e.g. jerk (optional)     */
    void *params;
} trajectory_t;

/* Parameters for TRAJ_SIN: A*sin(pi*omega*t) + b. */
typedef struct
{
    float amplitude; /* A     */
    float omega;     /* omega */
    float bias;      /* b     */
} traj_sin_params_t;

/* Parameters shared by TRAJ_PWM / TRAJ_TRIANGLE / TRAJ_SAW. */
typedef struct
{
    float frequency; /* Hz                         */
    float min;       /* low value                  */
    float max;       /* high value                 */
    float offset;    /* added to the output        */
    float duty;      /* duty cycle [0..1] (PWM only) */
} traj_wave_params_t;

/*
 * Parameters for a single coordinate of a 3D circular end-effector path:
 *   x = radius*cos(omega*t) + dx
 *   y = radius*sin(omega*t) + dy
 *   z = z_scale*radius*cos(omega*t) + dz
 * `axis` selects which coordinate this instance evaluates (0=x, 1=y, 2=z).
 */
typedef struct
{
    float radius;  /* circle radius k       */
    float omega;   /* angular rate [rad/s]  */
    float dx;      /* x centre offset       */
    float dy;      /* y centre offset       */
    float dz;      /* z centre offset       */
    float z_scale; /* z amplitude factor    */
    int   axis;    /* 0=x, 1=y, 2=z         */
} traj_circle3d_params_t;

/* Predefined, configurable parameter sets (exposed so callers can retune them). */
extern traj_sin_params_t  traj_sin_params;
extern traj_wave_params_t traj_pwm_params;
extern traj_wave_params_t traj_triangle_params;
extern traj_wave_params_t traj_saw_params;
extern traj_circle3d_params_t traj_circle3d_params[3]; /* [0]=x, [1]=y, [2]=z */

/* Evaluate the given trajectory at time t [s]. */
static inline float trajectory_eval(const trajectory_t *tr, float t)
{
    return tr->eval(tr->params, t);
}

/* Evaluate the trajectory's 1st derivative at time t [s], or 0 if none is defined. */
static inline float trajectory_eval_d1(const trajectory_t *tr, float t)
{
    return tr->d1 ? tr->d1(tr->params, t) : 0.0f;
}

/* Evaluate the trajectory's 2nd derivative at time t [s], or 0 if none is defined. */
static inline float trajectory_eval_d2(const trajectory_t *tr, float t)
{
    return tr->d2 ? tr->d2(tr->params, t) : 0.0f;
}

/* Evaluate the trajectory's 3rd derivative at time t [s], or 0 if none is defined. */
static inline float trajectory_eval_d3(const trajectory_t *tr, float t)
{
    return tr->d3 ? tr->d3(tr->params, t) : 0.0f;
}

/* True if the trajectory defines both analytic derivatives (exact feedforward). */
static inline int trajectory_has_derivs(const trajectory_t *tr)
{
    return tr->d1 != NULL && tr->d2 != NULL;
}

/* True if the trajectory defines all derivatives through third order. */
static inline int trajectory_has_derivs3(const trajectory_t *tr)
{
    return trajectory_has_derivs(tr) && tr->d3 != NULL;
}

/*
 * Return the predefined trajectory for `type`, or NULL if `type` is invalid.
 * The returned descriptor references the configurable parameter sets above.
 */
trajectory_t *trajectory_get(trajectory_type_t type);

#endif /* TRAJECTORY_H */
