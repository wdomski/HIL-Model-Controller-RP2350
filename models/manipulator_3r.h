#ifndef MANIPULATOR_3R_H
#define MANIPULATOR_3R_H

#include "model.h"

#define MANIP3R_N_STATES  6  /* [q1, q2, q3, dq1, dq2, dq3] */
#define MANIP3R_N_INPUTS  3  /* joint torques [tau1, tau2, tau3] */
#define MANIP3R_N_OUTPUTS 6  /* full joint state [q1, q2, q3, dq1, dq2, dq3] */

/**
 * Rigid 3R (spatial) manipulator.
 *
 * Rigid-body dynamics (B = I, so the torques enter directly):
 *   M(q) q'' + C(q, q') q' + D(q) + T(q') = tau
 * where M is the inertia matrix, C the Coriolis/centrifugal matrix, D the
 * gravity vector and T a static/viscous joint-friction torque
 *   T_i = Ts * atan(2*Tk*q'_i / pi) + Tv * q'_i.
 *
 * The output is the Cartesian end-effector position obtained from the forward
 * kinematics (base offset a, b, c):
 *   x = a + l1 c1 + (l2 + l3 c3) c12
 *   y = b + l1 s1 + (l2 + l3 c3) s12
 *   z = c + l3 s3            (ci = cos(qi), si = sin(qi), c12 = cos(q1+q2), ...)
 *
 * Ported from the reference Python model (dynamics only, no controller).
 */
typedef struct {
    float m1, m2, m3;   /* link masses [kg]              */
    float l1, l2, l3;   /* link lengths [m]              */
    float a, b, c;      /* base position offset [m]      */
    float g;            /* gravity [m/s^2]               */
    float Tv;           /* viscous friction coefficient  */
    float Ts;           /* static (Coulomb) friction     */
    float Tk;           /* friction atan steepness       */
} manipulator_3r_params_t;

void manipulator_3r_model_init(model_t *model, const manipulator_3r_params_t *params);

/*
 * Forward kinematics: Cartesian end-effector position from the joint angles.
 *   x = a + l1*cos(q1) + (l2 + l3*cos(q3))*cos(q1+q2)
 *   y = b + l1*sin(q1) + (l2 + l3*cos(q3))*sin(q1+q2)
 *   z = c + l3*sin(q3)
 *
 * @param q       Joint angles [q1, q2, q3]
 * @param params  Manipulator parameters
 * @param pos     Output end-effector position [x, y, z]
 */
void manipulator_3r_forward_kinematics(const float *q,
                                       const manipulator_3r_params_t *params,
                                       float *pos);

/*
 * Inverse and differential kinematics for a Cartesian reference.
 * Uses the principal q3 solution and selects q2's sign with elbow_sign.
 * If velocity is non-NULL, also solves J(q)*dq = velocity.
 * Returns 0 on success or -1 when the pose is unreachable or J is singular.
 */
int manipulator_3r_inverse_kinematics(const float position[3],
                                      const float velocity[3],
                                      int elbow_sign,
                                      const manipulator_3r_params_t *params,
                                      float state[MANIP3R_N_STATES]);

#endif /* MANIPULATOR_3R_H */
