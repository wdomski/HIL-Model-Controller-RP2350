#ifndef MDS_ESTIMATOR_H
#define MDS_ESTIMATOR_H

#include "controller.h"
#include "mds.h"

/*
 * On-line parameter ESTIMATION for the mass-spring-damper plant (models/mds.h):
 *   m x'' + c x' + k x = u.
 * Exactly ONE physical parameter (mass, damping OR stiffness) is unknown and is
 * identified to its TRUE value (contrast with the adaptive controllers, whose
 * estimate only makes the tracking work). This is NOT a tracking controller: it
 * emits a rich EXCITATION force (a fixed multisine) so the regressor stays
 * persistently exciting, which is the condition for true convergence. The
 * estimate lives in s->est. Derivation: wiki/mass-damper-spring/README.md.
 *
 * To avoid measuring acceleration, both x and u are passed through a stable
 * 2nd-order filter 1/Lambda(s), Lambda(s) = s^2 + lambda1 s + lambda0, giving
 * the filtered regression   m a_f + c v_f + k x_f = u_f   with
 *   x_f = x/Lambda,  v_f = s x/Lambda,  a_f = s^2 x/Lambda,  u_f = u/Lambda.
 * A normalised gradient (LMS) law then updates the single unknown:
 *   theta_hat' = gamma phi (y - theta_hat phi) / (1 + phi^2),
 * with phi and y assembled from the two known parameters.
 *
 * Interface mapping (controller_t):
 *   measured[1] = [x]     setpoint[1] = [unused]     control[1] = [F excitation]
 */

typedef enum
{
    MDS_EST_MASS = 0,
    MDS_EST_DAMPING,
    MDS_EST_STIFFNESS
} mds_est_param_t;

#define MDS_EST_NSINE 3   /* number of excitation harmonics */

typedef struct
{
    mds_params_t known;       /* the two known params (the unknown one is ignored) */
    mds_est_param_t which;    /* which parameter is being identified */
    float est, est0;          /* estimate + initial value (restored on reset) */
    float gamma;              /* learning rate > 0 */
    float lambda0, lambda1;   /* filter Lambda(s) = s^2 + lambda1 s + lambda0 */

    float amp[MDS_EST_NSINE]; /* excitation amplitudes [N] */
    float w[MDS_EST_NSINE];   /* excitation frequencies [rad/s] */

    float xf1, xf2;           /* x-filter states: xf1 = x/Lambda, xf2 = s x/Lambda */
    float uf1, uf2;           /* u-filter states: uf1 = u/Lambda */
    float last_u;             /* excitation emitted last step (drives the u-filter) */
    float t;                  /* internal time [s] */
} mds_estimator_t;

void mds_estimator_mass_init(controller_t *ctrl, mds_estimator_t *s,
                             const mds_params_t *known, float est0,
                             float gamma, float lambda0, float lambda1,
                             const float *amp, const float *w);
void mds_estimator_damping_init(controller_t *ctrl, mds_estimator_t *s,
                                const mds_params_t *known, float est0,
                                float gamma, float lambda0, float lambda1,
                                const float *amp, const float *w);
void mds_estimator_stiffness_init(controller_t *ctrl, mds_estimator_t *s,
                                  const mds_params_t *known, float est0,
                                  float gamma, float lambda0, float lambda1,
                                  const float *amp, const float *w);

#endif /* MDS_ESTIMATOR_H */
