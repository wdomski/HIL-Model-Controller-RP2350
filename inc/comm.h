#ifndef COMM_HEADER
#define COMM_HEADER

#include <stddef.h>
#include <stdint.h>

typedef enum {
    COMM_PARSE_OK,
    COMM_PARSE_ERROR
} comm_parse_status_t;

typedef struct {
    void (*model_select)(int index);
    int (*setpoint_const)(int index, float value);
    int (*setpoint_traj)(int index, int type);
    int (*controller_select)(int id);
    void (*reset)(void);
    void (*reset_remote)(void);
    /* Return the ordered value count, or -1. Writes must be all-or-nothing. */
    int (*parameters)(char kind, int id, float *values, size_t count, int write);
} comm_config_t;

int comm_init(const comm_config_t *config);
comm_parse_status_t comm_parse(const uint8_t *buffer,
                               size_t buffer_length,
                               char *response,
                               size_t response_capacity,
                               size_t *response_length);

int comm_build_model_select(char *output, size_t capacity, int index);
int comm_build_rst(char *output, size_t capacity);
uint32_t comm_log_period_us(void);
int comm_utilization_enabled(void);

#endif
