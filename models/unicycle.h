#ifndef UNICYCLE_H
#define UNICYCLE_H

#include "model.h"

#define UNICYCLE_N_STATES  3  /* [x, y, theta]  */
#define UNICYCLE_N_INPUTS  2  /* [v, omega]     */
#define UNICYCLE_N_OUTPUTS 3  /* [x, y, theta]  */

/**
 * Kinematic unicycle (differential-drive vehicle).
 *
 * State q = [x, y, theta]; control u = [v, omega] (linear/angular velocity):
 *   x'     = v * cos(theta)
 *   y'     = v * sin(theta)
 *   theta' = omega
 *
 * The inertial parameters (Iw, If, mw, mf) are not used by this kinematic
 * model; they are kept in the same descriptor so the dynamic model/helpers
 * below can reuse it.
 */
typedef struct {
    float wheel_radius; /* Rw [m]                                   */
    float axle_length;  /* L  [m] (half-track used in G: 1/L, -1/L) */
    float e;            /* output-point distance from the axle [m]  */
    float delta;        /* output-point angular offset [rad]        */
    float Iw;           /* wheel inertia [kg m^2]                   */
    float If;           /* frame (chassis) inertia [kg m^2]         */
    float mw;           /* wheel mass [kg]                          */
    float mf;           /* frame (chassis) mass [kg]                */
} unicycle_params_t;

void unicycle_model_init(model_t *model, const unicycle_params_t *params);

/*
 * Static feedback-linearization variables (NOT part of the model dynamics).
 *
 * For the output point h(q) = [x + e*cos(theta+delta), y + e*sin(theta+delta)]
 * this computes the decoupling matrix Rinv = dh/dq * G, its inverse R, and the
 * point h. A controller can then pick auxiliary inputs w and map them to the
 * unicycle inputs via u = R * w (see the reference Python `SimulatorKinematics`).
 */
typedef struct {
    float h[2];       /* linearizing output point (x_h, y_h) */
    float Rinv[2][2]; /* dh/dq * G                           */
    float R[2][2];    /* inv(Rinv): auxiliary -> [v, omega]  */
    float det_Rinv;   /* det(Rinv): ~0 near a singularity    */
} unicycle_static_lin_t;

void unicycle_static_linearization(const float *state,
                                   const unicycle_params_t *params,
                                   unicycle_static_lin_t *out);

/*
 * Reduced auxiliary-velocity mass matrix M* = G^T M G = [[a,b],[b,a]]
 * (constant for the unicycle; B* = (2/Rw) I, C* = 0). Shared by the
 * physics-based dynamics model and the dynamic-linearisation controller so the
 * inertial terms are defined exactly once.
 */
void unicycle_reduced_mass(const unicycle_params_t *params, float *a, float *b);

/*
 * Dynamic feedback-linearization intermediates, used by the static-lin
 * dynamics controller (controllers/static_lin_unicycle_dyn.c) to cancel the
 * plant dynamics. Kept here so the linear algebra lives in one place.
 *
 * `state` is a 7-element vector [h0, h1, h_d1_0, h_d1_1, x, y, theta] the
 * caller assembles from the plant state (only h_d1 and x,y,theta are used).
 */
typedef struct {
    float h[2];       /* linearizing output point (x_h, y_h) */
    float R[2][2];    /* inv(dh/dq * G): auxiliary -> base velocity map */
    float det_Rinv;   /* singularity indicator (~0 near det(Rinv)=0)    */
    float Mh[2][2];   /* linearized-space mass matrix   R^T Ms R        */
    float Ch[2][2];   /* linearized-space "Coriolis"    R^T Ms R_d1     */
    float Bh[2][2];   /* linearized-space input matrix  R^T Bs          */
    float q_d1[3];    /* base velocity [x', y', theta'], from h_d1      */
} unicycle_dyn_lin_t;

void unicycle_dynamic_linearization(const float *state,
                                    const unicycle_params_t *params,
                                    unicycle_dyn_lin_t *out);

/*
 * Dynamic (physics-based) unicycle: the second unicycle model, with mass and
 * inertia. Integrated in auxiliary velocities eta = [eta1, eta2] via q' = G eta
 * and M* eta' = B* u (C* == 0), so the plant is independent of any output point
 * h(q) and serves BOTH dynamics controllers (static- and dynamic-linearisation).
 *
 * State  [x, y, theta, eta1, eta2] (5); input = wheel torques [tau1, tau2] (2);
 * output [x, y, theta, v, omega]   (5), v = eta1+eta2, omega = (eta1-eta2)/L.
 * Each controller forms its own output point h(q) from this pose+velocity.
 */
#define UNICYCLE_DYN_N_STATES  5
#define UNICYCLE_DYN_N_INPUTS  2
#define UNICYCLE_DYN_N_OUTPUTS 5

void unicycle_dyn_model_init(model_t *model, const unicycle_params_t *params);

#endif /* UNICYCLE_H */
