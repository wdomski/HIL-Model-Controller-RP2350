#ifndef HIL_PARAMETERS_H
#define HIL_PARAMETERS_H

#include <stddef.h>
typedef struct controller_t controller_t;

#define PARAMETERS_MAX 32u
#define PARAMETERS_VERSION 1

/* Explicit ordered float lists; return count, or -1 without changing fields.
 * On read count is output capacity; on write it must match the layout exactly.
 * Call only between steps on the owner of the supplied instance. */
int parameters_model(int id, void *params, float *values, size_t count, int write);
int parameters_controller(int id, controller_t *ctrl, float dt,
                          float *values, size_t count, int write);
int parameters_trajectory(int id, float *values, size_t count, int write);

#endif
