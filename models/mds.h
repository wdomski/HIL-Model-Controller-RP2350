#ifndef MDS_H
#define MDS_H

#include "model.h"

#define MDS_N_STATES  2
#define MDS_N_INPUTS  1
#define MDS_N_OUTPUTS 2

/**
 * Mass-Damper-Spring system parameters.
 *
 * Equation: m*x'' + c*x' + k*x = F
 */
typedef struct {
    float mass;       /* m [kg]     */
    float damping;    /* c [N*s/m]  */
    float stiffness;  /* k [N/m]    */
} mds_params_t;

/**
 * Initialize the MDS model descriptor.
 *
 * @param model   Pointer to model_t to populate
 * @param params  Pointer to MDS parameters (must remain valid)
 */
void mds_model_init(model_t *model, const mds_params_t *params);

#endif /* MDS_H */
