#ifndef MDS_ADAPTIVE_H
#define MDS_ADAPTIVE_H

#include "controller.h"
#include "mds.h"
#include "trajectory.h"

/*
 * Adaptive control of the mass-spring-damper plant (models/mds.h):
 *   m x'' + c x' + k x = u.
 * Exactly ONE plant parameter (mass, damping OR stiffness) is unknown; its
 * estimate is adapted on-line and used in a computed-force law that drives the
 * tracking error e = x - x_d to zero (Lyapunov / Barbalat). Derivation:
 * wiki/mass-damper-spring/README.md.
 *
 * IMPORTANT: the adaptive estimate makes the *tracking* work, but is NOT
 * guaranteed to converge to the true parameter (no persistent excitation).
 * For true parameter identification use the mds_estimator_* modules.
 *
 * Common law (the derivation fixes Kd = 1 to cancel the cross terms):
 *   v = x_d'' - Kd e' - Kp e
 *   mass unknown:      u = m_hat v + c x' + k x,   m_hat' = -gamma v e'
 *   damping unknown:   u = m v + c_hat x' + k x,   c_hat' = -gamma x' e'
 *   stiffness unknown: u = m v + c x' + k_hat x,   k_hat' = -2 gamma x e'
 *
 * Interface mapping (controller_t):
 *   measured[2] = [x, x'] setpoint[1] = [x_d]     control[1] = [F]
 * Exact trajectory derivatives are used when attached; otherwise x_d' and
 * x_d'' are finite-differenced from the setpoint.
 */

typedef enum
{
    MDS_ADAPT_MASS = 0,
    MDS_ADAPT_DAMPING,
    MDS_ADAPT_STIFFNESS
} mds_adapt_param_t;

typedef struct
{
    mds_params_t known;       /* the two known params (the unknown one is ignored) */
    mds_adapt_param_t which;  /* which parameter is unknown / adapted */
    float est;                /* running estimate of the unknown parameter */
    float est0;               /* initial estimate (restored on reset) */
    float gamma;              /* adaptation gain > 0 */
    float kp, kd;             /* tracking gains (kd = 1 in the derivation) */
    trajectory_t *traj;       /* optional exact reference derivatives */
    float traj_time;

    float prev_sp, prev_sp_d1;/* setpoint history -> x_d', x_d''  */
    int   have_prev;          /* staged: 0 -> 1 -> 2              */
} mds_adaptive_t;

void mds_adaptive_mass_init(controller_t *ctrl, mds_adaptive_t *s,
                            const mds_params_t *known, float est0,
                            float gamma, float kp, float kd);
void mds_adaptive_damping_init(controller_t *ctrl, mds_adaptive_t *s,
                               const mds_params_t *known, float est0,
                               float gamma, float kp, float kd);
void mds_adaptive_stiffness_init(controller_t *ctrl, mds_adaptive_t *s,
                                 const mds_params_t *known, float est0,
                                 float gamma, float kp, float kd);
void mds_adaptive_set_trajectory(mds_adaptive_t *s, trajectory_t *traj);

#endif /* MDS_ADAPTIVE_H */
