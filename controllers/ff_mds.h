#ifndef FF_MDS_H
#define FF_MDS_H

#include "controller.h"
#include "mds.h"

/*
 * Full-knowledge computed-force controller for the mass-spring-damper plant
 * (models/mds.h): m x'' + c x' + k x = u. ALL three parameters are known, so
 * the plant is exactly cancelled and replaced by tunable linear error dynamics
 * (contrast with the adaptive/robust controllers, where one parameter is
 * unknown). This is the baseline the adaptive/robust laws reduce to.
 *
 * Control law (computed force + PD, derivation fixes nothing on Kd/Kp):
 *   v = x_d'' - Kd (x' - x_d') - Kp (x - x_d)
 *   u = m v + c x' + k x
 * Closing the loop gives  e'' + Kd e' + Kp e = 0,  e = x - x_d,
 * exponentially stable for Kp, Kd > 0 (choose (Kp,Kd) for the desired w_n, zeta).
 *
 * Interface mapping (controller_t):
 *   measured[1] = [x]     setpoint[1] = [x_d]     control[1] = [F]
 * x' is estimated by finite-differencing the measured position, and the
 * feedforward x_d', x_d'' by double finite-differencing the setpoint (staged
 * over the first two samples), exactly like the other FF controllers.
 */
typedef struct
{
    mds_params_t model;         /* known plant parameters (m, c, k) */
    float kp, kd;               /* error-dynamics gains */
    float prev_x;               /* measured history -> x'          */
    float prev_sp, prev_sp_d1;  /* setpoint history -> x_d', x_d''  */
    int   have_prev;            /* staged: 0 -> 1 -> 2             */
} ff_mds_t;

void ff_mds_init(controller_t *ctrl, ff_mds_t *c,
                 const mds_params_t *model, float kp, float kd);

#endif /* FF_MDS_H */
