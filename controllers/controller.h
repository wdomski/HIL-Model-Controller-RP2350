#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdint.h>

/*
 * Abstract controller interface.
 *
 * Concrete controllers (PID, feedforward, model-based, ...) fill in the
 * function pointers. The control loop only sees this generic vtable, so the
 * strategy can be swapped without touching the loop.
 */
typedef struct controller_t controller_t;

struct controller_t
{
    uint32_t n;  /* number of control channels */
    /* Compute control[] from setpoint[] and measured[] over timestep dt [s]. */
    void (*update)(controller_t *c, const float *setpoint,
                   const float *measured, float *control, float dt);
    /* Reset internal state (integrators, history). May be NULL. */
    void (*reset)(controller_t *c);
    void *params;
};

static inline void controller_update(controller_t *c, const float *setpoint,
                                     const float *measured, float *control, float dt)
{
    c->update(c, setpoint, measured, control, dt);
}

static inline void controller_reset(controller_t *c)
{
    if (c->reset)
    {
        c->reset(c);
    }
}

#endif /* CONTROLLER_H */
