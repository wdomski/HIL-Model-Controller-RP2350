#include <math.h>

#include "unicycle.h"
#include "ealgebra.h"

/*
 * Kinematics: dx/dt = f(x, u)
 *
 * State:  x[0]=x, x[1]=y, x[2]=theta
 * Input:  u[0]=v (linear velocity), u[1]=omega (angular velocity)
 *
 *   x'     = v * cos(theta)
 *   y'     = v * sin(theta)
 *   theta' = omega
 */
static void unicycle_kinematics(const float *state,
                                const float *input,
                                float *dxdt,
                                const void *params)
{
    (void)params;

    float theta = state[2];
    float v = input[0];
    float omega = input[1];

    dxdt[0] = v * cosf(theta);
    dxdt[1] = v * sinf(theta);
    dxdt[2] = omega;
}

/*
 * Output: y = [x, y, theta] (vehicle pose).
 */
static void unicycle_output(const float *state,
                            const float *input,
                            float *output,
                            const void *params)
{
    (void)input;
    (void)params;

    output[0] = state[0];
    output[1] = state[1];
    output[2] = remainderf(state[2], 2.0f * 3.14159265358979323846f);
}

void unicycle_model_init(model_t *model, const unicycle_params_t *params)
{
    model->n_states  = UNICYCLE_N_STATES;
    model->n_inputs  = UNICYCLE_N_INPUTS;
    model->n_outputs = UNICYCLE_N_OUTPUTS;
    model->dynamics  = unicycle_kinematics;
    model->output    = unicycle_output;
    model->params    = params;
}

void unicycle_static_linearization(const float *state,
                                   const unicycle_params_t *params,
                                   unicycle_static_lin_t *out)
{
    float x  = state[0];
    float y  = state[1];
    float th = state[2];

    float e = params->e;
    float delta = params->delta;
    float L = params->axle_length;

    float cth = cosf(th);
    float sth = sinf(th);
    float ctd = cosf(th + delta);
    float std = sinf(th + delta);

    /* Linearizing output point h(q). */
    out->h[0] = x + e * ctd;
    out->h[1] = y + e * std;

    /* Rinv = dh/dq * G[0:3], with
     *   dh/dq = [[1, 0, -e*sin(th+delta)], [0, 1,  e*cos(th+delta)]]
     *   G[0:3] = [[cos th, cos th], [sin th, sin th], [1/L, -1/L]]. */
    float Rinv00 = cth - e * std / L;
    float Rinv01 = cth + e * std / L;
    float Rinv10 = sth + e * ctd / L;
    float Rinv11 = sth - e * ctd / L;

    out->Rinv[0][0] = Rinv00;
    out->Rinv[0][1] = Rinv01;
    out->Rinv[1][0] = Rinv10;
    out->Rinv[1][1] = Rinv11;

    /* R = inv(Rinv), det(Rinv) via ealgebra (out->Rinv already populated). */
    ematrix Rinv_m = emat_make((float *)out->Rinv, 2, 2);
    ematrix R_m    = emat_make((float *)out->R, 2, 2);
    out->det_Rinv  = edet(&Rinv_m);
    einverse(&Rinv_m, &R_m);
}

/*
 * Dynamic unicycle: full second-order model (Ported from the reference
 * Python `UnicycleModel`/`SimulatorDynamics`, staticLinFull.py).
 *
 * Generalized coordinates (5): [x, y, theta, phi1, phi2] (phi1/phi2 = wheel
 * rotation angles). Mass matrix M = diag(mp, mp, Ip, Iw, Iw) with
 * mp = mf + 2*mw, Ip = If + 2*Iw; input matrix B actuates phi1', phi2'
 * directly (wheel torques); G maps the auxiliary velocity eta (2) to q':
 *
 *   G = [[cos, cos], [sin, sin], [1/L, -1/L], [2/Rw, 0], [0, 2/Rw]]   (5x2)
 *   B = [[0,0], [0,0], [0,0], [1,0], [0,1]]                          (5x2)
 *
 * phi1/phi2 don't feed back into anything else, so they are dropped from the
 * integrated state (only x, y, theta are kept -- see unicycle_dyn_model_init()).
 *
 * With this G/B, the reduced matrices Ms = G^T*M*G, Cs = G^T*M*G' and
 * Bs = G^T*B collapse to CONSTANTS (independent of theta): the cos/sin terms
 * cancel exactly, and Cs == 0 identically. So this port skips G' entirely and
 * only needs R_d1 (dR/dtheta, from the identity dR = -R (dR^-1/dtheta) R) to
 * build Ch.
 */
void unicycle_reduced_mass(const unicycle_params_t *params, float *a, float *b)
{
    float mp = params->mf + 2.0f * params->mw;
    float Ip = params->If + 2.0f * params->Iw;
    float L = params->axle_length;
    float Rw = params->wheel_radius;

    *a = mp + Ip / (L * L) + 4.0f * params->Iw / (Rw * Rw); /* Ms[0][0]=Ms[1][1] */
    *b = mp - Ip / (L * L);                                 /* Ms[0][1]=Ms[1][0] */
}

void unicycle_dynamic_linearization(const float *state,
                                    const unicycle_params_t *params,
                                    unicycle_dyn_lin_t *out)
{
    float h_d1_0 = state[2], h_d1_1 = state[3];
    float theta = state[6];

    unicycle_static_lin_t slin;
    unicycle_static_linearization(&state[4], params, &slin);
    out->h[0] = slin.h[0];
    out->h[1] = slin.h[1];
    out->det_Rinv = slin.det_Rinv;
    out->R[0][0] = slin.R[0][0]; out->R[0][1] = slin.R[0][1];
    out->R[1][0] = slin.R[1][0]; out->R[1][1] = slin.R[1][1];
    ematrix R_m = emat_make((float *)out->R, 2, 2);

    /* Auxiliary velocity eta = R*h_d1 (via ealgebra). */
    float hd1[2] = { h_d1_0, h_d1_1 };
    float eta[2];
    evector hd1_v = evec_make(hd1, 2), eta_v = evec_make(eta, 2);
    emat_vec_mul(&R_m, &hd1_v, &eta_v);

    /* Base velocity q' = G[0:3]*eta (3x2 * 2 -> 3): x'=cos*(eta0+eta1),
       y'=sin*(eta0+eta1), theta'=(eta0-eta1)/L. */
    float L = params->axle_length;
    float cth = cosf(theta), sth = sinf(theta);
    float Gd[6]; ematrix G_m = emat_make(Gd, 3, 2);
    emat_set(&G_m, 0, 0, cth);        emat_set(&G_m, 0, 1, cth);
    emat_set(&G_m, 1, 0, sth);        emat_set(&G_m, 1, 1, sth);
    emat_set(&G_m, 2, 0, 1.0f / L);   emat_set(&G_m, 2, 1, -1.0f / L);
    evector qd1_v = evec_make(out->q_d1, 3);
    emat_vec_mul(&G_m, &eta_v, &qd1_v);
    float theta_d1 = out->q_d1[2];

    /* R_d1 = dR/dtheta * theta_d1, via the identity dR = -R (dR^-1/dtheta) R.
       R here is the auxiliary-velocity decoupling (R^-1 = dh/dq * G has e/L
       terms), so its own derivative must be used -- not the kinematic-R one. */
    float e = params->e, delta = params->delta;
    float p_eL = e / L;
    float cd = cosf(theta + delta), sd = sinf(theta + delta);
    float dRinvd[4]; ematrix dRinv_m = emat_make(dRinvd, 2, 2);
    emat_set(&dRinv_m, 0, 0, -sth - p_eL * cd);  emat_set(&dRinv_m, 0, 1, -sth + p_eL * cd);
    emat_set(&dRinv_m, 1, 0,  cth - p_eL * sd);  emat_set(&dRinv_m, 1, 1,  cth + p_eL * sd);
    float Td[4]; ematrix T_m = emat_make(Td, 2, 2);
    emat_mul(&dRinv_m, &R_m, &T_m);              /* T = (dR^-1/dtheta) R */
    float Rdd[4]; ematrix Rd_m = emat_make(Rdd, 2, 2);
    emat_mul(&R_m, &T_m, &Rd_m);                 /* Rd = R T             */
    emat_scale(&Rd_m, -theta_d1, &Rd_m);         /* Rd = -theta_d1 R T   */

    /* Ms = [[a,b],[b,a]]; Mh = R^T*Ms*R; Ch = R^T*Ms*R_d1 (Cs == 0). */
    float a, b;
    unicycle_reduced_mass(params, &a, &b);
    float Msd[4]; ematrix Ms_m = emat_make(Msd, 2, 2);
    emat_set(&Ms_m, 0, 0, a);  emat_set(&Ms_m, 0, 1, b);
    emat_set(&Ms_m, 1, 0, b);  emat_set(&Ms_m, 1, 1, a);

    float Rtd[4]; ematrix Rt_m = emat_make(Rtd, 2, 2);
    etranspose(&R_m, &Rt_m);
    float tmpd[4]; ematrix tmp_m = emat_make(tmpd, 2, 2);
    emat_mul(&Rt_m, &Ms_m, &tmp_m);

    ematrix Mh_m = emat_make((float *)out->Mh, 2, 2);
    emat_mul(&tmp_m, &R_m, &Mh_m);
    ematrix Ch_m = emat_make((float *)out->Ch, 2, 2);
    emat_mul(&tmp_m, &Rd_m, &Ch_m);

    /* Bh = R^T*Bs, Bs = (2/Rw)*I. */
    float bscale = 2.0f / params->wheel_radius;
    float Bsd[4]; ematrix Bs_m = emat_make(Bsd, 2, 2);
    emat_set(&Bs_m, 0, 0, bscale);  emat_set(&Bs_m, 0, 1, 0.0f);
    emat_set(&Bs_m, 1, 0, 0.0f);    emat_set(&Bs_m, 1, 1, bscale);
    ematrix Bh_m = emat_make((float *)out->Bh, 2, 2);
    emat_mul(&Rt_m, &Bs_m, &Bh_m);
}

/*
 * Dynamic (physics) unicycle: dx/dt = f(x, u), state [x, y, theta, eta1, eta2],
 * input = wheel torques [tau1, tau2].
 *   x'     = (eta1+eta2) cos(theta)
 *   y'     = (eta1+eta2) sin(theta)
 *   theta' = (eta1-eta2)/L
 *   eta'   = M*^-1 B* u = D u,  D = (2/Rw)/(a^2-b^2) [[a,-b],[-b,a]]
 * with a,b from unicycle_reduced_mass() (M* = [[a,b],[b,a]], B* = (2/Rw) I).
 */
static void unicycle_dyn_dynamics(const float *state,
                                   const float *input,
                                   float *dxdt,
                                   const void *params)
{
    const unicycle_params_t *p = (const unicycle_params_t *)params;

    float eta1 = state[3], eta2 = state[4];
    float theta = state[2];

    float a, b;
    unicycle_reduced_mass(p, &a, &b);

    float L  = p->axle_length;
    float Rw = p->wheel_radius;
    float d2 = a * a - b * b;
    if (fabsf(d2) < 1e-12f)
    {
        d2 = (d2 < 0.0f) ? -1e-12f : 1e-12f;
    }
    float k = (2.0f / Rw) / d2;
    float D00 =  k * a;   /* M*^-1 B* diagonal     */
    float D01 = -k * b;   /* M*^-1 B* off-diagonal */

    float tau1 = input[0], tau2 = input[1];

    dxdt[0] = (eta1 + eta2) * cosf(theta);
    dxdt[1] = (eta1 + eta2) * sinf(theta);
    dxdt[2] = (eta1 - eta2) / L;
    dxdt[3] = D00 * tau1 + D01 * tau2;
    dxdt[4] = D01 * tau1 + D00 * tau2;
}

/* Output: [x, y, theta, v, omega] (pose + body velocities); each dynamics
 * controller forms its own output point h(q) from this. */
static void unicycle_dyn_output(const float *state,
                                const float *input,
                                float *output,
                                const void *params)
{
    (void)input;
    const unicycle_params_t *p = (const unicycle_params_t *)params;

    float eta1 = state[3], eta2 = state[4];
    float L = p->axle_length;

    output[0] = state[0];          /* x     */
    output[1] = state[1];          /* y     */
    output[2] = remainderf(state[2], 2.0f * 3.14159265358979323846f); /* theta */
    output[3] = eta1 + eta2;       /* v     */
    output[4] = (eta1 - eta2) / L; /* omega */
}

void unicycle_dyn_model_init(model_t *model, const unicycle_params_t *params)
{
    model->n_states  = UNICYCLE_DYN_N_STATES;
    model->n_inputs  = UNICYCLE_DYN_N_INPUTS;
    model->n_outputs = UNICYCLE_DYN_N_OUTPUTS;
    model->dynamics  = unicycle_dyn_dynamics;
    model->output    = unicycle_dyn_output;
    model->params    = params;
}
