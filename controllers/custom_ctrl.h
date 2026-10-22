#ifndef CUSTOM_CTRL_H
#define CUSTOM_CTRL_H

#include "controller.h"

/*
 * Custom controller template.
 *
 * A ready-to-use skeleton implementing the controller_t interface with a no-op
 * update (writes 0 to every control channel) and an empty reset. It is exposed
 * as the last entry of the controller switch (slot id 4 / "$CTRL,4").
 *
 * To implement your own control law:
 *   1. add your state/parameters to custom_ctrl_t below;
 *   2. fill in custom_ctrl_update() in custom_ctrl.c (and reset(), if needed);
 *   3. adjust the slot dimensions (n_meas/n_sp/n_ctrl) and PWM scales in
 *      src/main_ctrl.c ctrl_slots[] to match your model.
 * Everything else (selection over $CTRL, the 1 kHz loop, PWM I/O, telemetry) is
 * already wired and does not need changes.
 */
typedef struct
{
    uint32_t n; /* number of channels (informational) */
    /* TODO: add your controller state / parameters here */
} custom_ctrl_t;

void custom_ctrl_init(controller_t *ctrl, custom_ctrl_t *cc, uint32_t n);

#endif /* CUSTOM_CTRL_H */
