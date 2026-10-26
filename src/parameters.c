#include "parameters.h"
#include "hil_registry.h"
#include "trajectory.h"
#include "pid_ctrl.h"
#include "io_decoupling_3r.h"
#include "static_lin_unicycle_kin.h"
#include "static_lin_unicycle_dyn.h"
#include "dynamic_lin_unicycle_kin.h"
#include "dynamic_lin_unicycle_dyn.h"
#include "samson_unicycle_kin.h"
#include "samson_unicycle_dyn.h"
#include "ff_double_pendulum.h"
#include "ff_pendulum_cart.h"
#include "upright_pendulum_cart.h"
#include "ff_mds.h"
#include "mds_adaptive.h"
#include "mds_robust.h"
#include "mds_estimator.h"
#include <math.h>

typedef enum { ANY, NONNEGATIVE, POSITIVE, DUTY } rule_t;
typedef struct { float *field; rule_t rule; float scale; } field_t;
#define F(x) { &(x), ANY, 1.0f }
#define N(x) { &(x), NONNEGATIVE, 1.0f }
#define P(x) { &(x), POSITIVE, 1.0f }
#define MDS(p) P((p).mass), N((p).damping), N((p).stiffness)
#define CART(p) P((p).cart_mass), P((p).bob_mass), P((p).length), N((p).gravity)
#define DPEND(p) P((p).mass1), P((p).mass2), P((p).length1), P((p).length2), N((p).gravity)
#define ARM(p) P((p).m1), P((p).m2), P((p).m3), P((p).l1), P((p).l2), P((p).l3), \
               F((p).a), F((p).b), F((p).c), N((p).g), N((p).Tv), N((p).Ts), N((p).Tk)
#define TRANSFER(...) do { field_t fields[] = { __VA_ARGS__ }; \
    return transfer(fields, sizeof(fields) / sizeof(fields[0]), values, count, write); } while (0)

static int transfer(const field_t *fields, size_t n, float *values, size_t count, int write)
{
    if (values == NULL || count < n || (write && count != n)) return -1;
    /* Validate the entire list before touching any live field. */
    if (write) {
        for (size_t i = 0; i < n; ++i) {
            float v = values[i];
            if (!isfinite(v) || !isfinite(v * fields[i].scale) ||
                (fields[i].rule == NONNEGATIVE && v < 0.0f) ||
                (fields[i].rule == POSITIVE && v <= 0.0f) ||
                (fields[i].rule == DUTY && (v < 0.0f || v > 1.0f))) return -1;
        }
    }
    for (size_t i = 0; i < n; ++i) {
        if (write) *fields[i].field = values[i] * fields[i].scale;
        else values[i] = *fields[i].field / fields[i].scale;
    }
    return (int)n;
}

/* The adapted/identified coefficient is state, not a known model setting.
 * The final three fields are always mass, damping, stiffness. */
static int transfer_known(field_t *fields, size_t n, size_t unknown,
                          float *values, size_t count, int write)
{
    size_t skip = n - 3u + unknown;
    for (size_t i = skip; i + 1u < n; ++i) fields[i] = fields[i + 1u];
    return transfer(fields, n - 1u, values, count, write);
}

int parameters_model(int id, void *params, float *values, size_t count, int write)
{
    /* Velocity-driven kinematics does not use the geometry parameter struct. */
    if (id == MODEL_OFF || id == MODEL_UNICYCLE) return count == 0 || !write ? 0 : -1;
    if (params == NULL) return -1;
    switch (id) {
    case MODEL_MASS_SPRING_DAMPER: {
        mds_params_t *p = params; TRANSFER(MDS(*p));
    }
    case MODEL_PENDULUM_CART: {
        pendulum_cart_params_t *p = params; TRANSFER(CART(*p));
    }
    case MODEL_DOUBLE_PENDULUM: {
        double_pendulum_params_t *p = params; TRANSFER(DPEND(*p));
    }
    case MODEL_MANIPULATOR_3R: {
        manipulator_3r_params_t *p = params; TRANSFER(ARM(*p));
    }
    case MODEL_UNICYCLE_DYN: {
        unicycle_params_t *p = params;
        TRANSFER(P(p->wheel_radius), P(p->axle_length), P(p->Iw), P(p->If), P(p->mw), P(p->mf));
    }
    default: return -1;
    }
}

int parameters_controller(int id, controller_t *ctrl, float dt,
                          float *values, size_t count, int write)
{
    if (id == CTRL_OFF) return write && count != 0 ? -1 : 0;
    if (ctrl == NULL || ctrl->params == NULL) return -1;
    void *state = ctrl->params;
    switch (id) {
    case CTRL_PID: {
        pid_ctrl_t *p = state;
        if (dt <= 0.0f || p->n != 1) return -1;
        TRANSFER(N(p->gains[0].kp),
                 { &p->gains[0].ki_dt, NONNEGATIVE, dt },
                 { &p->gains[0].kd_inv_dt, NONNEGATIVE, 1.0f / dt });
    }
    case CTRL_IO3R: {
        io_decoupling_3r_t *p = state;
        TRANSFER(N(p->kp[0]), N(p->kp[1]), N(p->kp[2]),
                 N(p->kd[0]), N(p->kd[1]), N(p->kd[2]), ARM(p->model));
    }
    case CTRL_SL_UNI_KIN: {
        static_lin_unicycle_kin_t *p = state; TRANSFER(N(p->kp));
    }
    case CTRL_SL_UNI_DYN: {
        static_lin_unicycle_dyn_t *p = state; TRANSFER(N(p->kp), N(p->kd));
    }
    case CTRL_DL_UNI_KIN: {
        dynamic_lin_unicycle_kin_t *p = state; TRANSFER(N(p->kp), N(p->kd));
    }
    case CTRL_DL_UNI_DYN: {
        dynamic_lin_unicycle_dyn_t *p = state;
        if (write && count == 3 && (double)values[1] * values[2] <= values[0]) return -1;
        TRANSFER(P(p->k0), P(p->k1), P(p->k2));
    }
    case CTRL_SAM_UNI_KIN: {
        samson_unicycle_kin_t *p = state; TRANSFER(N(p->k1), N(p->k2), N(p->k3));
    }
    case CTRL_SAM_UNI_DYN: {
        samson_unicycle_dyn_t *p = state; TRANSFER(N(p->k1), N(p->k2), N(p->k3), P(p->kv));
    }
    case CTRL_FF_DPEND: {
        ff_double_pendulum_t *p = state; TRANSFER(N(p->kp), N(p->kd), DPEND(p->model));
    }
    case CTRL_FF_PCART: {
        ff_pendulum_cart_t *p = state; TRANSFER(N(p->kp), N(p->kd), CART(p->model));
    }
    case CTRL_FF_MDS: {
        ff_mds_t *p = state; TRANSFER(N(p->kp), N(p->kd), MDS(p->model));
    }
    case CTRL_MDS_AD_M: case CTRL_MDS_AD_C: case CTRL_MDS_AD_K: {
        mds_adaptive_t *p = state;
        field_t fields[] = { N(p->kp), P(p->gamma), N(p->est0), MDS(p->known) };
        return transfer_known(fields, sizeof(fields) / sizeof(fields[0]),
                              (size_t)(id - CTRL_MDS_AD_M), values, count, write);
    }
    case CTRL_MDS_RB_M: case CTRL_MDS_RB_C: case CTRL_MDS_RB_K: {
        mds_robust_t *p = state;
        TRANSFER(N(p->kp), N(p->delta), P(p->epsilon), MDS(p->nominal));
    }
    case CTRL_MDS_ES_M: case CTRL_MDS_ES_C: case CTRL_MDS_ES_K: {
        mds_estimator_t *p = state;
        field_t fields[] = { P(p->gamma), P(p->lambda0), P(p->lambda1), N(p->est0),
                 F(p->amp[0]), F(p->amp[1]), F(p->amp[2]),
                 P(p->w[0]), P(p->w[1]), P(p->w[2]), MDS(p->known) };
        return transfer_known(fields, sizeof(fields) / sizeof(fields[0]),
                              (size_t)(id - CTRL_MDS_ES_M), values, count, write);
    }
    case CTRL_UPRIGHT_PCART: {
        upright_pendulum_cart_t *p = state;
        upright_pendulum_cart_config_t *c = &p->config;
        if (write && count == 22 &&
            (values[14] < values[11] || values[15] < values[12] || values[16] < values[13])) return -1;
        TRANSFER(N(c->energy_gain), P(c->energy_target_scale), N(c->cart_kp), N(c->cart_kd),
                 F(c->stabilize_k[0]), F(c->stabilize_k[1]), F(c->stabilize_k[2]), F(c->stabilize_k[3]),
                 P(c->velocity_filter_tau), F(c->kick_acceleration), N(c->kick_duration),
                 P(c->catch_angle), P(c->catch_rate), P(c->catch_position),
                 P(c->release_angle), P(c->release_rate), P(c->release_position),
                 P(c->force_limit), CART(p->model));
    }
    case CTRL_CUSTOM: return write && count != 0 ? -1 : 0;
    default: return -1;
    }
}

int parameters_trajectory(int id, float *values, size_t count, int write)
{
    trajectory_t *tr = trajectory_get((trajectory_type_t)id);
    if (tr == NULL) return -1;
    switch (id) {
    case TRAJ_SIN: {
        traj_sin_params_t *p = &traj_sin_params;
        TRANSFER(F(p->amplitude), N(p->omega), F(p->bias));
    }
    case TRAJ_PWM: case TRAJ_TRIANGLE: case TRAJ_SAW: {
        traj_wave_params_t *p = id == TRAJ_PWM ? &traj_pwm_params :
                               id == TRAJ_TRIANGLE ? &traj_triangle_params : &traj_saw_params;
        size_t expected = id == TRAJ_PWM ? 5u : 4u;
        if (write && count == expected && values[1] > values[2]) return -1;
        if (id == TRAJ_PWM)
            TRANSFER(P(p->frequency), F(p->min), F(p->max), F(p->offset), { &p->duty, DUTY, 1.0f });
        TRANSFER(P(p->frequency), F(p->min), F(p->max), F(p->offset));
    }
    case TRAJ_CIRCLE3D_X: case TRAJ_CIRCLE3D_Y: case TRAJ_CIRCLE3D_Z: {
        traj_circle3d_params_t *p = &traj_circle3d_params[id - TRAJ_CIRCLE3D_X];
        if (id == TRAJ_CIRCLE3D_X) TRANSFER(N(p->radius), N(p->omega), F(p->dx));
        if (id == TRAJ_CIRCLE3D_Y) TRANSFER(N(p->radius), N(p->omega), F(p->dy));
        TRANSFER(N(p->radius), N(p->omega), F(p->dz), F(p->z_scale));
    }
    default: return -1;
    }
}
