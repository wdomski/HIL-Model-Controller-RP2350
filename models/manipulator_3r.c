#include <math.h>

#include "manipulator_3r.h"
#include "ealgebra.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
 * Dynamics: dx/dt = f(x, u, params)
 *
 * State:  x[0..2] = q  (joint angles), x[3..5] = q' (joint velocities)
 * Input:  u[0..2] = tau (joint torques)
 *
 * Solve M(q) q'' = tau - C(q,q') q' - D(q) - T(q'):
 *   rhs = tau - C q' - D - T,   q'' = M^-1 rhs.
 * The M, C, D and friction terms are transcribed from the reference model.
 */
static void manipulator_3r_dynamics(const float *state,
                                    const float *input,
                                    float *dxdt,
                                    const void *params)
{
    const manipulator_3r_params_t *p = (const manipulator_3r_params_t *)params;

    float q2 = state[1];
    float q3 = state[2];
    float dq1 = state[3];
    float dq2 = state[4];
    float dq3 = state[5];

    float m1 = p->m1, m2 = p->m2, m3 = p->m3;
    float l1 = p->l1, l2 = p->l2, l3 = p->l3;
    float g  = p->g;

    float c2 = cosf(q2), s2 = sinf(q2);
    float c3 = cosf(q3), s3 = sinf(q3);

    /* Inertia matrix M (symmetric). */
    float M11 = (l1 * l1 * m1) / 3.0f + l1 * l1 * m2 + l1 * l1 * m3
              + (l2 * l2 * m2) / 3.0f + l2 * l2 * m3
              + (l3 * l3 * m3 * c3 * c3) / 3.0f
              + l1 * l2 * m2 * c2 + 2.0f * l1 * l2 * m3 * c2
              + l2 * l3 * m3 * c3 + l1 * l3 * m3 * c2 * c3;
    float M12 = (l2 * l2 * m2) / 3.0f + l2 * l2 * m3
              + (l3 * l3 * m3 * c3 * c3) / 3.0f
              + (l1 * l2 * m2 * c2) / 2.0f + l1 * l2 * m3 * c2
              + l2 * l3 * m3 * c3 + (l1 * l3 * m3 * c2 * c3) / 2.0f;
    float M13 = -(l1 * l3 * m3 * s2 * s3) / 2.0f;
    float M22 = (l2 * l2 * m2) / 3.0f + l2 * l2 * m3
              + (l3 * l3 * m3 * c3 * c3) / 3.0f + l2 * l3 * m3 * c3;
    float M23 = 0.0f;
    float M33 = (l3 * l3 * m3) / 3.0f;

    /* Coriolis/centrifugal matrix C, shared sub-expressions. */
    float A1  = (l1 * l2 * m2 * s2) / 2.0f + l1 * l2 * m3 * s2
              + (l1 * l3 * m3 * c3 * s2) / 2.0f;
    float A3  = (l3 * l3 * m3 * c3 * s3) / 3.0f + (l2 * l3 * m3 * s3) / 2.0f;
    float A2  = A3 + (l1 * l3 * m3 * c2 * s3) / 2.0f;
    float A2b = (l1 * l3 * m3 * c3 * s2) / 2.0f;

    float C11 = -dq2 * A1 - dq3 * A2;
    float C12 = -dq1 * A1 - dq2 * A1 - dq3 * A2;
    float C13 = -dq1 * A2 - dq2 * A2 - dq3 * A2b;
    float C21 =  dq1 * A1 - dq3 * A3;
    float C22 = -dq3 * A3;
    float C23 = -dq1 * A3 - dq2 * A3;
    float C31 =  dq2 * A3 + dq1 * A2;
    float C32 =  dq1 * A3 + dq2 * A3;
    float C33 = 0.0f;

    /* Gravity vector D. */
    float D3 = -(g * l3 * m3 * c3) / 2.0f;

    /* Joint friction T. */
    float T1 = p->Ts * atanf((2.0f * p->Tk * dq1) / (float)M_PI) + p->Tv * dq1;
    float T2 = p->Ts * atanf((2.0f * p->Tk * dq2) / (float)M_PI) + p->Tv * dq2;
    float T3 = p->Ts * atanf((2.0f * p->Tk * dq3) / (float)M_PI) + p->Tv * dq3;

    /* Inertia matrix M (symmetric) — populated from the Mij via ealgebra. */
    float Md[9]; ematrix M = emat_make(Md, 3, 3);
    emat_set(&M, 0, 0, M11);  emat_set(&M, 0, 1, M12);  emat_set(&M, 0, 2, M13);
    emat_set(&M, 1, 0, M12);  emat_set(&M, 1, 1, M22);  emat_set(&M, 1, 2, M23);
    emat_set(&M, 2, 0, M13);  emat_set(&M, 2, 1, M23);  emat_set(&M, 2, 2, M33);

    /* Coriolis matrix C — populated from the Cij via ealgebra. */
    float Cd[9]; ematrix Cm = emat_make(Cd, 3, 3);
    emat_set(&Cm, 0, 0, C11);  emat_set(&Cm, 0, 1, C12);  emat_set(&Cm, 0, 2, C13);
    emat_set(&Cm, 1, 0, C21);  emat_set(&Cm, 1, 1, C22);  emat_set(&Cm, 1, 2, C23);
    emat_set(&Cm, 2, 0, C31);  emat_set(&Cm, 2, 1, C32);  emat_set(&Cm, 2, 2, C33);

    /* rhs = tau - C q' - D - T  (D = [0,0,D3]). */
    float qd[3] = { dq1, dq2, dq3 };
    float Cqd[3];
    evector qd_v = evec_make(qd, 3), Cqd_v = evec_make(Cqd, 3);
    emat_vec_mul(&Cm, &qd_v, &Cqd_v);

    float rhs[3] = {
        input[0] - Cqd[0]            - T1,
        input[1] - Cqd[1]            - T2,
        input[2] - Cqd[2] - D3       - T3
    };

    /* q'' = M^-1 rhs via ealgebra (cofactor inverse). */
    float Mind[9]; ematrix Minv = emat_make(Mind, 3, 3);
    einverse(&M, &Minv);
    float qdd[3];
    evector rhs_v = evec_make(rhs, 3), qdd_v = evec_make(qdd, 3);
    emat_vec_mul(&Minv, &rhs_v, &qdd_v);

    dxdt[0] = dq1;
    dxdt[1] = dq2;
    dxdt[2] = dq3;
    dxdt[3] = qdd[0];
    dxdt[4] = qdd[1];
    dxdt[5] = qdd[2];
}

/*
 * Output: full joint state [q1, q2, q3, dq1, dq2, dq3] so the controller
 * (input/output decoupling) has complete state feedback. The end-effector
 * position is recovered by the controller via manipulator_3r_forward_kinematics.
 */
static void manipulator_3r_output(const float *state,
                                  const float *input,
                                  float *output,
                                  const void *params)
{
    (void)input;
    (void)params;
    for (int i = 0; i < MANIP3R_N_STATES; i++)
    {
        output[i] = state[i];
    }
}

void manipulator_3r_model_init(model_t *model, const manipulator_3r_params_t *params)
{
    model->n_states  = MANIP3R_N_STATES;
    model->n_inputs  = MANIP3R_N_INPUTS;
    model->n_outputs = MANIP3R_N_OUTPUTS;
    model->dynamics  = manipulator_3r_dynamics;
    model->output    = manipulator_3r_output;
    model->params    = params;
}

void manipulator_3r_forward_kinematics(const float *q,
                                       const manipulator_3r_params_t *params,
                                       float *pos)
{
    float q1 = q[0];
    float q2 = q[1];
    float q3 = q[2];

    float reach = params->l2 + params->l3 * cosf(q3);

    pos[0] = params->a + params->l1 * cosf(q1) + reach * cosf(q1 + q2);
    pos[1] = params->b + params->l1 * sinf(q1) + reach * sinf(q1 + q2);
    pos[2] = params->c + params->l3 * sinf(q3);
}

int manipulator_3r_inverse_kinematics(const float position[3],
                                      const float velocity[3],
                                      int elbow_sign,
                                      const manipulator_3r_params_t *params,
                                      float state[MANIP3R_N_STATES])
{
    const float tolerance = 1e-6f;
    if (fabsf(params->l1) < tolerance || fabsf(params->l3) < tolerance)
    {
        return -1;
    }

    float sin_q3 = (position[2] - params->c) / params->l3;
    if (sin_q3 < -1.0f - tolerance || sin_q3 > 1.0f + tolerance)
    {
        return -1;
    }
    sin_q3 = fmaxf(-1.0f, fminf(1.0f, sin_q3));
    float q3 = asinf(sin_q3);
    float reach = params->l2 + params->l3 * cosf(q3);
    float denominator = 2.0f * params->l1 * reach;
    if (fabsf(denominator) < tolerance)
    {
        return -1;
    }

    float x = position[0] - params->a;
    float y = position[1] - params->b;
    float cos_q2 = (x * x + y * y - params->l1 * params->l1 - reach * reach)
                   / denominator;
    if (cos_q2 < -1.0f - tolerance || cos_q2 > 1.0f + tolerance)
    {
        return -1;
    }
    cos_q2 = fmaxf(-1.0f, fminf(1.0f, cos_q2));
    float q2 = acosf(cos_q2);
    if (elbow_sign < 0)
    {
        q2 = -q2;
    }
    float q1 = atan2f(y, x)
               - atan2f(reach * sinf(q2), params->l1 + reach * cosf(q2));

    state[0] = q1;
    state[1] = q2;
    state[2] = q3;
    state[3] = 0.0f;
    state[4] = 0.0f;
    state[5] = 0.0f;
    if (velocity == NULL)
    {
        return 0;
    }

    float s1 = sinf(q1), c1 = cosf(q1);
    float s12 = sinf(q1 + q2), c12 = cosf(q1 + q2);
    float s3 = sinf(q3), c3 = cosf(q3);
    float Jd[9] = {
        -params->l1 * s1 - reach * s12, -reach * s12, -params->l3 * s3 * c12,
         params->l1 * c1 + reach * c12,  reach * c12, -params->l3 * s3 * s12,
         0.0f,                              0.0f,       params->l3 * c3,
    };
    float velocity_data[3] = { velocity[0], velocity[1], velocity[2] };
    ematrix J = emat_make(Jd, 3, 3);
    evector cartesian_velocity = evec_make(velocity_data, 3);
    evector joint_velocity = evec_make(&state[3], 3);
    return emat_solve(&J, &cartesian_velocity, &joint_velocity);
}
