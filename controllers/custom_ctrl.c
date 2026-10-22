#include "custom_ctrl.h"

/*
 * No-op control law: writes 0 to every control channel. Replace the body with
 * your own control law (it receives the setpoint, the measured state and the
 * timestep dt; it writes control[]).
 */
static void custom_ctrl_update(controller_t *c, const float *setpoint,
                               const float *measured, float *control, float dt)
{
    (void)setpoint;
    (void)measured;
    (void)dt;

    for (uint32_t i = 0; i < c->n; i++)
    {
        control[i] = 0.0f;
    }
    /* TODO: implement your control law here */
}

/* Reset internal state. Add your reset logic here (may be a no-op). */
static void custom_ctrl_reset(controller_t *c)
{
    (void)c;
}

void custom_ctrl_init(controller_t *ctrl, custom_ctrl_t *cc, uint32_t n)
{
    cc->n = n;

    ctrl->n      = n;
    ctrl->update = custom_ctrl_update;
    ctrl->reset  = custom_ctrl_reset;
    ctrl->params = cc;
}
