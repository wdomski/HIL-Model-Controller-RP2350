#ifndef MDS_ROBUST_H
#define MDS_ROBUST_H

#include "controller.h"
#include "mds.h"
#include "trajectory.h"

/*
 * Robust control of the mass-spring-damper plant (models/mds.h):
 *   m x'' + c x' + k x = u.
 * Exactly ONE plant parameter is unknown but BOUNDED: it lies within delta of a
 * nominal value. Unlike the adaptive controllers, nothing is estimated on-line;
 * a discontinuous compensating term rejects the worst-case uncertainty and
 * guarantees the tracking error e = x - x_d stays bounded / converges (Lyapunov).
 * Derivation: wiki/mass-damper-spring/README.md.
 *
 * Common law (the derivation fixes Kd = 1):
 *   v  = x_d'' - Kd e' - Kp e
 *   u  = m0 v + c0 x' + k0 x + u_r        (nominal computed force + robust term)
 *   u_r = -delta |phi| sign(e')           (phi = the signal multiplying the
 *                                          uncertain parameter)
 *     mass unknown:      phi = v      (commanded acceleration proxy)
 *     damping unknown:   phi = x'
 *     stiffness unknown: phi = x
 * sign(.) is smoothed by tanh(e'/epsilon) to avoid chattering (ultimate
 * boundedness inside the boundary layer).
 *
 * Interface mapping (controller_t):
 *   measured[2] = [x, x'] setpoint[1] = [x_d]     control[1] = [F]
 * Exact trajectory derivatives are used when attached; otherwise x_d' and
 * x_d'' are finite-differenced from the setpoint.
 */

typedef enum
{
    MDS_ROBUST_MASS = 0,
    MDS_ROBUST_DAMPING,
    MDS_ROBUST_STIFFNESS
} mds_robust_param_t;

typedef struct
{
    mds_params_t nominal;      /* nominal m0, c0, k0 */
    mds_robust_param_t which;  /* which parameter is uncertain */
    float delta;               /* uncertainty bound on the unknown parameter */
    float kp, kd;              /* tracking gains (kd = 1 in the derivation) */
    float epsilon;             /* tanh boundary-layer width (> 0) */
    trajectory_t *traj;        /* optional exact reference derivatives */
    float traj_time;

    float prev_sp, prev_sp_d1; /* setpoint history -> x_d', x_d''  */
    int   have_prev;           /* staged: 0 -> 1 -> 2              */
} mds_robust_t;

void mds_robust_mass_init(controller_t *ctrl, mds_robust_t *s,
                          const mds_params_t *nominal, float delta,
                          float kp, float kd, float epsilon);
void mds_robust_damping_init(controller_t *ctrl, mds_robust_t *s,
                             const mds_params_t *nominal, float delta,
                             float kp, float kd, float epsilon);
void mds_robust_stiffness_init(controller_t *ctrl, mds_robust_t *s,
                               const mds_params_t *nominal, float delta,
                               float kp, float kd, float epsilon);
void mds_robust_set_trajectory(mds_robust_t *s, trajectory_t *traj);

#endif /* MDS_ROBUST_H */
