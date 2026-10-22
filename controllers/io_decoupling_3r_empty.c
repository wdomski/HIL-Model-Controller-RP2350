#include <math.h>

#include "io_decoupling_3r.h"
#include "ealgebra.h"

#ifndef __weak
#define __weak __attribute__((weak))
#endif

/* Publish-mode weak stub for io_decoupling_3r.c: update() outputs 0; init()/
 * reset()/set_trajectory() are complete. Overridden by the real
 * io_decoupling_3r.c when present (HIL_REAL_CONTROLLERS). */

static void io_decoupling_update_empty(controller_t *c, const float *setpoint,
                                       const float *measured, float *control, float dt)
{
    (void)setpoint;
    (void)measured;
    (void)dt;

    /*
     * TODO: 3R input/output decoupling (computed torque) -- write control[0..2] (joint torques).
     *   io_decoupling_3r_t *io = c->params;
     *     model io->model (manipulator_3r_params_t), gains io->kp[3]/io->kd[3],
    *     exact feedforward io->traj[3] with io->traj_scale/traj_offset
    *     (NULL trajectory -> finite-difference setpoint[]),
     *     own time base io->t, readback io->yd[3].
     *   measured[0..2] = q (joint angles), measured[3..5] = dq (joint rates);
     *   setpoint[0..2] = desired end-effector position yd.
     *   steps:
     *     1. inertia M(q), Coriolis C(q,q'), gravity D(q), friction T(q'), Jacobian J = dh/dq;
     *     2. G = J*M^-1;  F = J'*q' - J*M^-1*(C*q' + D + T);
    *     3. yd/yd'/yd'' from transformed io->traj[] on io->t or finite-diff setpoint[];
     *     4. v = yd'' - io->kd[]*(y'-yd') - io->kp[]*(y-yd);  u = G^-1*(v - F);
     *     5. control[0..2] = u.
     * Placeholder below writes 0.
     */
    for (uint32_t i = 0; i < c->n; i++)
    {
        control[i] = 0.0f;
    }
}

static void io_decoupling_reset_empty(controller_t *c)
{
    io_decoupling_3r_t *io = (io_decoupling_3r_t *)c->params;
    io->have_prev = 0;
    io->t = 0.0f;
    for (int i = 0; i < 3; i++)
    {
        io->prev_yd[i] = 0.0f;
        io->prev_yd_d1[i] = 0.0f;
        io->yd[i] = 0.0f;
    }
}

__weak void io_decoupling_3r_init(controller_t *ctrl, io_decoupling_3r_t *io,
                                  const manipulator_3r_params_t *model,
                                  const float kp[3], const float kd[3])
{
    io->model = *model;
    for (int i = 0; i < 3; i++)
    {
        io->kp[i] = kp[i];
        io->kd[i] = kd[i];
        io->prev_yd[i] = 0.0f;
        io->prev_yd_d1[i] = 0.0f;
        io->traj[i] = NULL;
        io->traj_scale[i] = 1.0f;
        io->traj_offset[i] = 0.0f;
        io->yd[i] = 0.0f;
    }
    io->have_prev = 0;
    io->t = 0.0f;

    ctrl->n = 3;
    ctrl->update = io_decoupling_update_empty;
    ctrl->reset = io_decoupling_reset_empty;
    ctrl->params = io;
}

__weak void io_decoupling_3r_set_trajectory(io_decoupling_3r_t *io,
                                            trajectory_t *traj_x,
                                            trajectory_t *traj_y,
                                            trajectory_t *traj_z,
                                            const float scale[3],
                                            const float offset[3])
{
    io->traj[0] = traj_x;
    io->traj[1] = traj_y;
    io->traj[2] = traj_z;
    for (int i = 0; i < 3; i++)
    {
        io->traj_scale[i] = scale ? scale[i] : 1.0f;
        io->traj_offset[i] = offset ? offset[i] : 0.0f;
    }
    io->t = 0.0f;
    io->have_prev = 0;
}
