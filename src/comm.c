#include "comm.h"

#include "aproto/aproto.h"
#include "hil_registry.h"
#include "trajectory.h"

#define LOG_PERIOD_DEFAULT_MS 200u
#define LOG_PERIOD_MIN_MS 10u
#define LOG_PERIOD_MAX_MS 5000u

static const char *const model_names[] = {
#define X(sym, name) name,
    HIL_MODEL_LIST(X)
#undef X
};

static const char *const controller_names[] = {
#define X(sym, name) name,
    HIL_CTRL_LIST(X)
#undef X
};

static const char *const trajectory_names[TRAJ_COUNT] = {
    [TRAJ_SIN] = "Sin",
    [TRAJ_PWM] = "PWM",
    [TRAJ_TRIANGLE] = "Triangle",
    [TRAJ_SAW] = "Saw",
    [TRAJ_CIRCLE3D_X] = "Circle-X",
    [TRAJ_CIRCLE3D_Y] = "Circle-Y",
    [TRAJ_CIRCLE3D_Z] = "Circle-Z",
};

static aproto_t protocol;
static comm_config_t application;
static uint32_t log_period_us = LOG_PERIOD_DEFAULT_MS * 1000u;
static int utilization_enabled;

static aproto_command_result_t command_list(
    const aproto_request_t *request,
    aproto_response_t *response,
    const char *heading,
    const char *const *names,
    size_t count)
{
    size_t index;

    if (request->query || aproto_argument_count(request) != 0u)
    {
        return APROTO_COMMAND_ERROR;
    }
    aproto_response_append(response, heading);
    for (index = 0u; index < count; ++index)
    {
        aproto_response_append(response, ",");
        aproto_response_append_int(response, (int)index);
        aproto_response_append(response, ":");
        aproto_response_append(response, names[index]);
    }
    return APROTO_COMMAND_DATA;
}

static aproto_command_result_t handle_models(
    const aproto_request_t *request, aproto_response_t *response, void *context)
{
    (void)context;
    return command_list(request, response, "MODELS", model_names,
                        HIL_MODEL_COUNT);
}

static aproto_command_result_t handle_model(
    const aproto_request_t *request, aproto_response_t *response, void *context)
{
    int index;
    (void)context;

    if (request->query || aproto_argument_count(request) != 1u ||
        aproto_argument_int(request, 0u, &index) != 0 || index < 0 ||
        (unsigned int)index >= HIL_MODEL_COUNT || application.model_select == NULL)
    {
        return APROTO_COMMAND_ERROR;
    }
    application.model_select(index);
    aproto_response_append(response, ",");
    aproto_response_append_int(response, index);
    aproto_response_append(response, ":");
    aproto_response_append(response, model_names[index]);
    return APROTO_COMMAND_OK;
}

static aproto_command_result_t handle_ctrls(
    const aproto_request_t *request, aproto_response_t *response, void *context)
{
    (void)context;
    return command_list(request, response, "CTRLS", controller_names,
                        HIL_CTRL_COUNT);
}

static aproto_command_result_t handle_ctrl(
    const aproto_request_t *request, aproto_response_t *response, void *context)
{
    int index;
    (void)context;

    if (request->query || aproto_argument_count(request) != 1u ||
        aproto_argument_int(request, 0u, &index) != 0 || index < 0 ||
        (unsigned int)index >= HIL_CTRL_COUNT ||
        application.controller_select == NULL ||
        application.controller_select(index) != 0)
    {
        return APROTO_COMMAND_ERROR;
    }
    aproto_response_append(response, ",");
    aproto_response_append_int(response, index);
    aproto_response_append(response, ":");
    aproto_response_append(response, controller_names[index]);
    return APROTO_COMMAND_OK;
}

static aproto_command_result_t handle_trajs(
    const aproto_request_t *request, aproto_response_t *response, void *context)
{
    (void)context;
    return command_list(request, response, "TRAJS", trajectory_names, TRAJ_COUNT);
}

static aproto_command_result_t handle_sp(
    const aproto_request_t *request, aproto_response_t *response, void *context)
{
    int index;
    float value;
    (void)context;

    if (request->query || aproto_argument_count(request) != 2u ||
        aproto_argument_int(request, 0u, &index) != 0 ||
        aproto_argument_float(request, 1u, &value) != 0 ||
        application.setpoint_const == NULL ||
        application.setpoint_const(index, value) != 0)
    {
        return APROTO_COMMAND_ERROR;
    }
    aproto_response_append(response, ",");
    aproto_response_append_int(response, index);
    aproto_response_append(response, ",");
    aproto_response_append_float(response, value, 3u);
    return APROTO_COMMAND_OK;
}

static aproto_command_result_t handle_sptraj(
    const aproto_request_t *request, aproto_response_t *response, void *context)
{
    int index;
    int type;
    (void)context;

    if (request->query || aproto_argument_count(request) != 2u ||
        aproto_argument_int(request, 0u, &index) != 0 ||
        aproto_argument_int(request, 1u, &type) != 0 ||
        application.setpoint_traj == NULL ||
        application.setpoint_traj(index, type) != 0)
    {
        return APROTO_COMMAND_ERROR;
    }
    aproto_response_append(response, ",");
    aproto_response_append_int(response, index);
    aproto_response_append(response, ",");
    aproto_response_append_int(response, type);
    return APROTO_COMMAND_OK;
}

static aproto_command_result_t handle_log(
    const aproto_request_t *request, aproto_response_t *response, void *context)
{
    int milliseconds;
    (void)context;

    if (request->query)
    {
        if (aproto_argument_count(request) != 0u)
        {
            return APROTO_COMMAND_ERROR;
        }
        milliseconds = (int)(log_period_us / 1000u);
        aproto_response_append(response, "OK,LOG,");
        aproto_response_append_int(response, milliseconds);
        return APROTO_COMMAND_DATA;
    }
    if (aproto_argument_count(request) != 1u ||
        aproto_argument_int(request, 0u, &milliseconds) != 0 ||
        (milliseconds != 0 &&
         (milliseconds < (int)LOG_PERIOD_MIN_MS ||
          milliseconds > (int)LOG_PERIOD_MAX_MS)))
    {
        return APROTO_COMMAND_ERROR;
    }
    log_period_us = (uint32_t)milliseconds * 1000u;
    aproto_response_append(response, ",");
    aproto_response_append_int(response, milliseconds);
    return APROTO_COMMAND_OK;
}

static aproto_command_result_t handle_util(
    const aproto_request_t *request, aproto_response_t *response, void *context)
{
    int enabled;
    (void)context;

    if (request->query)
    {
        if (aproto_argument_count(request) != 0u)
        {
            return APROTO_COMMAND_ERROR;
        }
        aproto_response_append(response, "OK,UTIL,");
        aproto_response_append_int(response, utilization_enabled);
        return APROTO_COMMAND_DATA;
    }
    if (aproto_argument_count(request) != 1u ||
        aproto_argument_int(request, 0u, &enabled) != 0 ||
        (enabled != 0 && enabled != 1))
    {
        return APROTO_COMMAND_ERROR;
    }
    utilization_enabled = enabled;
    aproto_response_append(response, ",");
    aproto_response_append_int(response, enabled);
    return APROTO_COMMAND_OK;
}

static aproto_command_result_t handle_rst(
    const aproto_request_t *request, aproto_response_t *response, void *context)
{
    (void)response;
    (void)context;
    if (request->query || aproto_argument_count(request) != 0u ||
        application.reset == NULL)
    {
        return APROTO_COMMAND_ERROR;
    }
    application.reset();
    return APROTO_COMMAND_OK;
}

static aproto_command_result_t handle_rstall(
    const aproto_request_t *request, aproto_response_t *response, void *context)
{
    (void)response;
    (void)context;
    if (request->query || aproto_argument_count(request) != 0u ||
        application.reset == NULL || application.reset_remote == NULL)
    {
        return APROTO_COMMAND_ERROR;
    }
    application.reset();
    application.reset_remote();
    return APROTO_COMMAND_OK;
}

static const aproto_command_t commands[] = {
    { "MODELS", handle_models, NULL },
    { "MODEL", handle_model, NULL },
    { "CTRLS", handle_ctrls, NULL },
    { "CTRL", handle_ctrl, NULL },
    { "TRAJS", handle_trajs, NULL },
    { "SP", handle_sp, NULL },
    { "SPTRAJ", handle_sptraj, NULL },
    { "LOG", handle_log, NULL },
    { "UTIL", handle_util, NULL },
    { "RST", handle_rst, NULL },
    { "RSTALL", handle_rstall, NULL },
};

int comm_init(const comm_config_t *config)
{
    if (config == NULL)
    {
        return -1;
    }
    application = *config;
    log_period_us = LOG_PERIOD_DEFAULT_MS * 1000u;
    utilization_enabled = 0;
    return aproto_init(&protocol, commands, sizeof(commands) / sizeof(commands[0])) ==
                   APROTO_STATUS_OK ? 0 : -1;
}

comm_parse_status_t comm_parse(const uint8_t *buffer,
                               size_t buffer_length,
                               char *response,
                               size_t response_capacity,
                               size_t *response_length)
{
    aproto_status_t status = aproto_parse(&protocol, buffer, buffer_length,
                                          response, response_capacity,
                                          response_length);
    return status == APROTO_STATUS_OK ? COMM_PARSE_OK : COMM_PARSE_ERROR;
}

int comm_build_model_select(char *output, size_t capacity, int index)
{
    char body[24];
    size_t length;
    aproto_response_t response = { body, sizeof(body), 0u, 0 };

    aproto_response_append(&response, "MODEL,");
    aproto_response_append_int(&response, index);
    if (response.overflow ||
        aproto_build_frame(output, capacity, body, response.length, &length) !=
            APROTO_STATUS_OK)
    {
        return -1;
    }
    return (int)length;
}

int comm_build_rst(char *output, size_t capacity)
{
    size_t length;
    if (aproto_build_frame(output, capacity, "RST", 3u, &length) !=
        APROTO_STATUS_OK)
    {
        return -1;
    }
    return (int)length;
}

uint32_t comm_log_period_us(void)
{
    return log_period_us;
}

int comm_utilization_enabled(void)
{
    return utilization_enabled;
}