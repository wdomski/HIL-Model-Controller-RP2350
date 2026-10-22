#ifndef IO_DECOUPLING_3R_H
#define IO_DECOUPLING_3R_H

#include "controller.h"
#include "manipulator_3r.h"
#include "trajectory.h"

/*
 * Input/output decoupling (computed-torque) controller for the 3R manipulator.
 *
 * Ported from the reference Python `model_ode` (inputOutputDecouplingFull.py).
 * The end-effector output y = h(q) is linearized and decoupled:
 *   y'' = F + G*u,  G = J*M^-1,  F = J'*q' - J*M^-1*(C*q' + D + T)
 *   v   = yd'' - Kd*(y' - yd') - Kp*(y - yd)
 *   u   = G^-1 * (v - F)      (joint torques)
 *
 * Robust Jacobian: near a kinematic singularity (det(J) -> 0, e.g. the arm
 * fully extended) G is non-invertible. A working copy of J used only for
 * F/G has its diagonal nudged by +-1e-5 (sign of det J) whenever
 * |det J| < 1e-5; the true J (unperturbed) still gives y' = J*q'.
 *
 * Interface mapping (controller_t):
 *   measured[] = full state [q1, q2, q3, dq1, dq2, dq3]
 *   setpoint[] = desired end-effector position [xd, yd, zd]
 *   control[]  = joint torques [tau1, tau2, tau3]
 *
 * Feedforward yd' and yd'' come from one of two sources:
 *   - default: finite-differencing setpoint[] between steps (no attached
 *     trajectory);
 *   - exact: a per-axis trajectory_t attached via io_decoupling_3r_set_trajectory(),
 *     evaluated (position + analytic d1/d2) on the controller's own time base
 *     with an optional affine scale/offset. setpoint[] is then ignored; the
 *     resolved position is readable back via io_decoupling_3r_t::yd.
 */
typedef struct
{
    manipulator_3r_params_t model;
    float kp[3];
    float kd[3];
    float prev_yd[3];
    float prev_yd_d1[3];
    int   have_prev;
    trajectory_t *traj[3]; /* optional exact feedforward source, NULL = finite differences */
    float traj_scale[3];   /* affine transform applied to attached trajectories */
    float traj_offset[3];
    float t;                /* internal clock for traj[], advanced by dt each update */
    float yd[3];            /* last commanded end-effector position (readback) */
} io_decoupling_3r_t;

void io_decoupling_3r_init(controller_t *ctrl, io_decoupling_3r_t *io,
                           const manipulator_3r_params_t *model,
                           const float kp[3], const float kd[3]);

/*
 * Attach exact per-axis end-effector trajectories as the feedforward source,
 * replacing the default finite-difference estimate. Resets the controller's
 * internal time base to 0. Optional scale/offset arrays apply the same slot
 * transform used for telemetry; the offset affects position only. Pass NULL
 * for all trajectories to detach (reverts to setpoint[] + finite differences).
 */
void io_decoupling_3r_set_trajectory(io_decoupling_3r_t *io, trajectory_t *traj_x,
                                     trajectory_t *traj_y, trajectory_t *traj_z,
                                     const float scale[3], const float offset[3]);

#endif /* IO_DECOUPLING_3R_H */
