#include <stdio.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "parameters.h"

#include "hardware/clocks.h"
#include "hardware/dma.h"

#include "pwm_input.h"
#include "pwm_output.h"
#include "led.h"
#include "hw_test.h"
#include "serial.h"
#include "config_hw.h"
#include "main.h"
#include "comm.h"
#include "hil_registry.h"
#include "eprintfs/eprintfs.h"
#include "utils.h"

#include "mds.h"
#include "pendulum_cart.h"
#include "double_pendulum.h"
#include "manipulator_3r.h"
#include "unicycle.h"
#include "ode.h"
#include "fifo/fifo.h"
#include "trajectory.h"

#include "circ_buff/circ_buff_hw.h"
#include "circ_buff/circ_buff.h"

uint8_t pwm_input_pins[] = {7, 9, 11, 13, 15, 17, 19, 21};
uint8_t pwm_output_pins[] = {6, 8, 10, 12, 14, 16, 18, 20};
uint8_t led_pins[] = {22, 26, 27, 28};

int ret;

#define TESTING 0

#define N_PWM_CHANNELS   8   /* physical PWM input/output channels */
#define MODEL_MAX_STATES 8   /* upper bound on states across all models */
#define MODEL_STEP_US    1000 /* 1 kHz model rate */

/* Model slot ids and their order come from the CMake-generated registry
 * (hil_registry.h); the ids are stable symbols, not hand-maintained numbers. */
typedef enum hil_model_id selected_model_t;

/* Model-selection command passed from core 0 (comm) to core 1 (simulation). */
typedef struct
{
  uint32_t model_id;
} model_cmd_t;

#define MODEL_CMD_QUEUE_DEPTH 4

static fifo_t model_cmd_queue;
static model_cmd_t model_cmd_storage[MODEL_CMD_QUEUE_DEPTH];

/* Only core 1 reads/writes parameter blocks. Core 0 waits for the committed
 * result; the simulation loop consumes requests without blocking. */
typedef struct {
  char kind;
  int id, write, result;
  size_t count;
  float values[PARAMETERS_MAX];
} parameter_request_t;
static fifo_t parameter_requests, parameter_results;
static parameter_request_t parameter_request_storage[1], parameter_result_storage[1];

static int comm_parameters(char kind, int id, float *values, size_t count, int write)
{
  if ((kind != 'M' && kind != 'T') || count > PARAMETERS_MAX) return -1;
  parameter_request_t request = { .kind = kind, .id = id, .write = write, .count = count };
  if (write) memcpy(request.values, values, count * sizeof(float));
  if (!fifo_try_add(&parameter_requests, &request)) return -1;
  while (!fifo_try_remove(&parameter_results, &request)) sleep_us(50);
  if (!write && request.result >= 0)
    memcpy(values, request.values, (size_t)request.result * sizeof(float));
  return request.result;
}

absolute_time_t current_time_core1;
absolute_time_t last_time_model_run;

/* Model-step execution time [us] (debug, updated every step). */
volatile int64_t model_exec_time_us;

/*
 * Per-channel PWM I/O scaling. A duty cycle of 50% maps to `offset`;
 * `scale` sets the physical span covered by the full 0..100% duty range.
 * These are (re)configured by the selected model.
 */
typedef struct
{
  float offset;
  float scale;
} pwm_io_cfg_t;

pwm_io_cfg_t pwm_in_cfg[N_PWM_CHANNELS];
pwm_io_cfg_t pwm_out_cfg[N_PWM_CHANNELS];

/* Mass-spring-damper interface scaling. */
#define MDS_FORCE_SCALE 50.0f /* input ch0: force spans +/-25 N */
#define MDS_POS_SCALE    2.4f /* output ch0: position spans +/-1.2 m */
#define MDS_VEL_SCALE    4.0f /* output ch1: velocity spans +/-2 m/s */

mds_params_t mds_params = {
    .mass = 1.0f,
    .damping = 0.1f,
    .stiffness = 10.0f
};

model_t mds_model;

/* Pendulum-cart interface scaling. */
#define PEND_FORCE_SCALE 170.0f    /* input ch0: force spans +/-85 N   */
#define PEND_POS_SCALE     8.0f    /* output ch0: cart pos spans +/-4 m */
#define PEND_ANGLE_SCALE  8.0f     /* output ch1: angle spans +/-4 rad */

pendulum_cart_params_t pendulum_cart_params = {
    .cart_mass = 1.0f,
    .bob_mass  = 0.5f,
    .length    = 1.0f,
    .gravity   = 9.81f
};

model_t pendulum_cart_model;

/* Double-pendulum interface scaling. */
#define DPEND_TORQUE_SCALE 110.0f    /* inputs ch0,ch1: torque spans +/-55 Nm */
#define DPEND_ANGLE_SCALE  6.2831853f /* outputs ch0,ch1: angle spans +/-pi rad */

double_pendulum_params_t double_pendulum_params = {
    .mass1   = 1.0f,
    .mass2   = 0.5f,
    .length1 = 1.0f,
    .length2 = 0.8f,
    .gravity = 9.81f
};

model_t double_pendulum_model;

/* 3R-manipulator interface scaling. */
#define MANIP3R_TORQUE_SCALE 20.0f       /* inputs ch0..2: torque spans +/-10 Nm       */
#define MANIP3R_ANGLE_SCALE  6.2831853f  /* outputs ch0..2: joint angle spans +/-pi rad  */
#define MANIP3R_VEL_SCALE    6.2831853f  /* outputs ch3..5: joint vel spans +/-pi rad/s  */

manipulator_3r_params_t manipulator_3r_params = {
    .m1 = 1.0f, .m2 = 1.0f, .m3 = 1.0f,
    .l1 = 1.0f, .l2 = 1.0f, .l3 = 1.0f,
    .a  = 0.0f, .b  = 0.0f, .c  = 0.0f,
    .g  = 9.81f,
    .Tv = 0.1f, .Ts = 0.05f, .Tk = 100.0f
};

model_t manipulator_3r_model;

/* Unicycle interface scaling. */
#define UNI_VEL_SCALE    26.0f      /* input ch0: linear velocity spans +/-13 m/s   */
#define UNI_OMEGA_SCALE  26.0f      /* input ch1: angular velocity spans +/-13 rad/s */
#define UNI_POS_SCALE    8.0f       /* outputs ch0,ch1: position spans +/-4 m       */
#define UNI_ANGLE_SCALE  7.0f       /* output ch2: heading spans +/-3.5 rad         */

unicycle_params_t unicycle_params = {
    .wheel_radius = 0.05f,
    .axle_length  = 0.15f,
    .e            = 0.1f,
    .delta        = 0.05f
};

model_t unicycle_model;

/* Dynamic (physics) unicycle: the second unicycle model (mass/inertia,
   wheel-torque input); shared by both dynamics controllers (src/main_ctrl.c).
   Output [x, y, theta, v, omega]; delta/e are unused by the plant (each
   controller applies its own output point). */
#define UNI_DYN_TORQUE_SCALE 40.0f  /* inputs ch0,ch1: torque spans +/-20 Nm */
#define UNI_DYN_VEL_SCALE     40.0f /* output ch3: velocity spans +/-20 m/s   */
#define UNI_DYN_OMEGA_SCALE   40.0f /* output ch4: yaw rate spans +/-20 rad/s */

unicycle_params_t unicycle_dyn_params = {
    .wheel_radius = 0.05f, .axle_length = 0.15f, .e = 0.1f, .delta = 0.05f,
    .Iw = 0.000125f, .If = 0.01125f, .mw = 0.1f, .mf = 1.0f
};

model_t unicycle_dyn_model;

static int arm_start_valid(const manipulator_3r_params_t *params)
{
  float position[3], velocity[3], state[MANIP3R_N_STATES];
  for (int i = 0; i < 3; ++i) {
    trajectory_t *tr = trajectory_get((trajectory_type_t)(TRAJ_CIRCLE3D_X + i));
    position[i] = trajectory_eval(tr, 0.0f);
    velocity[i] = trajectory_eval_d1(tr, 0.0f);
    if (!isfinite(position[i]) || !isfinite(velocity[i])) return 0;
  }
  position[0] -= 0.12f;
  if (manipulator_3r_inverse_kinematics(position, velocity, 1, params, state) != 0) return 0;
  for (int i = 0; i < MANIP3R_N_STATES; ++i)
    if (!isfinite(state[i])) return 0;
  return 1;
}

static int model_parameters(parameter_request_t *request)
{
  int id = request->id;
  float *v = request->values;
  if (request->kind == 'T') {
    float previous[PARAMETERS_MAX];
    int n = parameters_trajectory(id, previous, PARAMETERS_MAX, 0);
    if (n < 0) return -1;
    n = parameters_trajectory(id, v, request->count, request->write);
    if (n >= 0 && request->write && id >= TRAJ_CIRCLE3D_X &&
        !arm_start_valid(&manipulator_3r_params)) {
      parameters_trajectory(id, previous, (size_t)n, 1);
      return -1;
    }
    return n;
  }
  void *sets[HIL_MODEL_COUNT] = {
    [MODEL_MASS_SPRING_DAMPER] = &mds_params,
    [MODEL_PENDULUM_CART] = &pendulum_cart_params,
    [MODEL_DOUBLE_PENDULUM] = &double_pendulum_params,
    [MODEL_MANIPULATOR_3R] = &manipulator_3r_params,
    [MODEL_UNICYCLE] = &unicycle_params,
    [MODEL_UNICYCLE_DYN] = &unicycle_dyn_params,
  };
  if (id < 0 || id >= HIL_MODEL_COUNT) return -1;
  if (id == MODEL_MANIPULATOR_3R && request->write) {
    manipulator_3r_params_t candidate = manipulator_3r_params;
    int n = parameters_model(id, &candidate, v, request->count, 1);
    if (n < 0 || !arm_start_valid(&candidate)) return -1;
    manipulator_3r_params = candidate;
    return n;
  }
  return parameters_model(id, sets[id], v, request->count, request->write);
}

/* Shared ODE solver and buffers (one model runs at a time). */
ode_solver_t solver;
float model_state[MODEL_MAX_STATES];
float model_scratch[5 * MODEL_MAX_STATES];

const model_t *active_model = NULL;
selected_model_t active_selection = MODEL_OFF;

/* Unified I/O buffers for all channels. */
float model_inputs[N_PWM_CHANNELS];
float model_outputs[N_PWM_CHANNELS];

/* Telemetry buffer (core 0). Period set via $LOG (default 200 ms, 0 = off). */
static uint8_t tlm_buf[192];

static char *tlm_append(char *p, const char *label, const float *v, uint32_t n)
{
  p += esprintf(p, "%s", label);
  for (uint32_t i = 0; i < n; i++)
  {
    p += esprintf(p, "%.3f%s", v[i], (i + 1 < n) ? "," : "");
  }
  return p;
}

/*
 * Apply I/O scaling and (re)initialize the solver for a model selection.
 * Extend the switch to register additional models.
 */
static void model_apply_selection(selected_model_t sel)
{
  /* default: passthrough scaling on every channel */
  for (uint8_t i = 0; i < N_PWM_CHANNELS; i++)
  {
    pwm_in_cfg[i]  = (pwm_io_cfg_t){ .offset = 0.0f, .scale = 1.0f };
    pwm_out_cfg[i] = (pwm_io_cfg_t){ .offset = 0.0f, .scale = 1.0f };
  }

  switch (sel)
  {
    case MODEL_MASS_SPRING_DAMPER:
      pwm_in_cfg[0]  = (pwm_io_cfg_t){ .offset = 0.0f, .scale = MDS_FORCE_SCALE };
      pwm_out_cfg[0] = (pwm_io_cfg_t){ .offset = 0.0f, .scale = MDS_POS_SCALE };
      pwm_out_cfg[1] = (pwm_io_cfg_t){ .offset = 0.0f, .scale = MDS_VEL_SCALE };
      memset(model_state, 0, sizeof(model_state));
      ode_init(&solver, &mds_model, MODEL_STEP_US, model_state, model_scratch);
      active_model = &mds_model;
      break;

    case MODEL_PENDULUM_CART:
      pwm_in_cfg[0]  = (pwm_io_cfg_t){ .offset = 0.0f, .scale = PEND_FORCE_SCALE };
      pwm_out_cfg[0] = (pwm_io_cfg_t){ .offset = 0.0f, .scale = PEND_POS_SCALE };
      pwm_out_cfg[1] = (pwm_io_cfg_t){ .offset = 0.0f, .scale = PEND_ANGLE_SCALE };
      memset(model_state, 0, sizeof(model_state));
      ode_init(&solver, &pendulum_cart_model, MODEL_STEP_US, model_state, model_scratch);
      active_model = &pendulum_cart_model;
      break;

    case MODEL_DOUBLE_PENDULUM:
      pwm_in_cfg[0]  = (pwm_io_cfg_t){ .offset = 0.0f, .scale = DPEND_TORQUE_SCALE };
      pwm_in_cfg[1]  = (pwm_io_cfg_t){ .offset = 0.0f, .scale = DPEND_TORQUE_SCALE };
      pwm_out_cfg[0] = (pwm_io_cfg_t){ .offset = 0.0f, .scale = DPEND_ANGLE_SCALE };
      pwm_out_cfg[1] = (pwm_io_cfg_t){ .offset = 0.0f, .scale = DPEND_ANGLE_SCALE };
      memset(model_state, 0, sizeof(model_state));
      ode_init(&solver, &double_pendulum_model, MODEL_STEP_US, model_state, model_scratch);
      active_model = &double_pendulum_model;
      break;

    case MODEL_MANIPULATOR_3R:
      pwm_in_cfg[0]  = (pwm_io_cfg_t){ .offset = 0.0f, .scale = MANIP3R_TORQUE_SCALE };
      pwm_in_cfg[1]  = (pwm_io_cfg_t){ .offset = 0.0f, .scale = MANIP3R_TORQUE_SCALE };
      pwm_in_cfg[2]  = (pwm_io_cfg_t){ .offset = 0.0f, .scale = MANIP3R_TORQUE_SCALE };
      pwm_out_cfg[0] = (pwm_io_cfg_t){ .offset = 0.0f, .scale = MANIP3R_ANGLE_SCALE };
      pwm_out_cfg[1] = (pwm_io_cfg_t){ .offset = 0.0f, .scale = MANIP3R_ANGLE_SCALE };
      pwm_out_cfg[2] = (pwm_io_cfg_t){ .offset = 0.0f, .scale = MANIP3R_ANGLE_SCALE };
      pwm_out_cfg[3] = (pwm_io_cfg_t){ .offset = 0.0f, .scale = MANIP3R_VEL_SCALE };
      pwm_out_cfg[4] = (pwm_io_cfg_t){ .offset = 0.0f, .scale = MANIP3R_VEL_SCALE };
      pwm_out_cfg[5] = (pwm_io_cfg_t){ .offset = 0.0f, .scale = MANIP3R_VEL_SCALE };
      memset(model_state, 0, sizeof(model_state));
      trajectory_t *trajectory[3] = {
        trajectory_get(TRAJ_CIRCLE3D_X),
        trajectory_get(TRAJ_CIRCLE3D_Y),
        trajectory_get(TRAJ_CIRCLE3D_Z),
      };
      float position[3], velocity[3];
      for (int i = 0; i < 3; i++)
      {
        position[i] = trajectory_eval(trajectory[i], 0.0f);
        velocity[i] = trajectory_eval_d1(trajectory[i], 0.0f);
      }
      position[0] -= 0.12f;
      hard_assert(manipulator_3r_inverse_kinematics(
                    position, velocity, 1, &manipulator_3r_params, model_state) == 0);
      ode_init(&solver, &manipulator_3r_model, MODEL_STEP_US, model_state, model_scratch);
      active_model = &manipulator_3r_model;
      break;

    case MODEL_UNICYCLE:
      pwm_in_cfg[0]  = (pwm_io_cfg_t){ .offset = 0.0f, .scale = UNI_VEL_SCALE };
      pwm_in_cfg[1]  = (pwm_io_cfg_t){ .offset = 0.0f, .scale = UNI_OMEGA_SCALE };
      pwm_out_cfg[0] = (pwm_io_cfg_t){ .offset = 0.0f, .scale = UNI_POS_SCALE };
      pwm_out_cfg[1] = (pwm_io_cfg_t){ .offset = 0.0f, .scale = UNI_POS_SCALE };
      pwm_out_cfg[2] = (pwm_io_cfg_t){ .offset = 0.0f, .scale = UNI_ANGLE_SCALE };
      memset(model_state, 0, sizeof(model_state));
      ode_init(&solver, &unicycle_model, MODEL_STEP_US, model_state, model_scratch);
      active_model = &unicycle_model;
      break;

    case MODEL_UNICYCLE_DYN:
      pwm_in_cfg[0]  = (pwm_io_cfg_t){ .offset = 0.0f, .scale = UNI_DYN_TORQUE_SCALE };
      pwm_in_cfg[1]  = (pwm_io_cfg_t){ .offset = 0.0f, .scale = UNI_DYN_TORQUE_SCALE };
      pwm_out_cfg[0] = (pwm_io_cfg_t){ .offset = 0.0f, .scale = UNI_POS_SCALE };   /* x     */
      pwm_out_cfg[1] = (pwm_io_cfg_t){ .offset = 0.0f, .scale = UNI_POS_SCALE };   /* y     */
      pwm_out_cfg[2] = (pwm_io_cfg_t){ .offset = 0.0f, .scale = UNI_ANGLE_SCALE }; /* theta */
      pwm_out_cfg[3] = (pwm_io_cfg_t){ .offset = 0.0f, .scale = UNI_DYN_VEL_SCALE };   /* v     */
      pwm_out_cfg[4] = (pwm_io_cfg_t){ .offset = 0.0f, .scale = UNI_DYN_OMEGA_SCALE }; /* omega */
      memset(model_state, 0, sizeof(model_state));
      /* eta1=eta2=0.5 gives v=1, omega=0 and keeps dynamic linearization
        away from its chi1=v-e*omega=0 startup singularity. */
      model_state[3] = 0.5f;
      model_state[4] = 0.5f;
      ode_init(&solver, &unicycle_dyn_model, MODEL_STEP_US, model_state, model_scratch);
      active_model = &unicycle_dyn_model;
      break;

    case MODEL_OFF:
    default:
      active_model = NULL;
      break;
  }

  active_selection = sel;
}

/* Stage 1: read all channels into physical values (model-independent). */
static void model_read_inputs(float *inputs)
{
  for (uint8_t i = 0; i < N_PWM_CHANNELS; i++)
  {
    inputs[i] = pwm_input_get_value(pwm_input_pins, i,
                                    pwm_in_cfg[i].offset, pwm_in_cfg[i].scale);
  }
}

/* Stage 3: write all channels from physical values (model-independent). */
static void model_write_outputs(const float *outputs)
{
  for (uint8_t i = 0; i < N_PWM_CHANNELS; i++)
  {
    pwm_output_set_value(pwm_output_pins, i, outputs[i],
                         pwm_out_cfg[i].offset, pwm_out_cfg[i].scale);
  }
}

/* Runs on core 0 (comm): forward the selection to core 1 via the queue. */
void comm_model_select(int index)
{
  model_cmd_t cmd = { .model_id = (uint32_t)index };
  fifo_try_add(&model_cmd_queue, &cmd);
}

/*
 * Reset the active model through the callback registered with comm_init().
 * Runs on core 0 and
 * re-applies the current selection by pushing it back into the queue; core 1
 * then calls model_apply_selection(), which restores the model-specific reset
 * state and re-initializes the solver. Harmless when MODEL_OFF is selected.
 */
void comm_reset(void)
{
  model_cmd_t cmd = { .model_id = (uint32_t)active_selection };
  fifo_try_add(&model_cmd_queue, &cmd);
}

/*
 * Forward a "$RST" to the controller board over the uart1 link. Uses a local
 * buffer, not buffer_uart_tx, which holds the "$OK,RSTALL" parser response.
 */
void comm_reset_remote(void)
{
  char fwd[16];
  int len = comm_build_rst(fwd, sizeof(fwd));
  send_with_repeat(&serial_link, (uint8_t *)fwd, (uint32_t)len);
}

void main_core1(void){

  int64_t time_diff_us;
  model_cmd_t cmd;

  //initialize all available models
  mds_model_init(&mds_model, &mds_params);
  pendulum_cart_model_init(&pendulum_cart_model, &pendulum_cart_params);
  double_pendulum_model_init(&double_pendulum_model, &double_pendulum_params);
  manipulator_3r_model_init(&manipulator_3r_model, &manipulator_3r_params);
  unicycle_model_init(&unicycle_model, &unicycle_params);
  unicycle_dyn_model_init(&unicycle_dyn_model, &unicycle_dyn_params);
  model_apply_selection(MODEL_OFF);

  for(;;){
    // Run the selected model at 1 kHz
    current_time_core1 = get_absolute_time();
    time_diff_us = absolute_time_diff_us(last_time_model_run, current_time_core1);
    if (time_diff_us < MODEL_STEP_US) // 1 kHz
    {
      continue;
    }
    last_time_model_run = current_time_core1;

    // apply a pending model selection from core 0 (non-blocking)
    if (fifo_try_remove(&model_cmd_queue, &cmd))
    {
      model_apply_selection((selected_model_t)cmd.model_id);
    }

    // 1) read inputs: unified over all 8 PWM channels
    parameter_request_t request;
    if (fifo_try_remove(&parameter_requests, &request)) {
      request.result = model_parameters(&request);
      bool added = fifo_try_add(&parameter_results, &request);
      hard_assert(added);
    }

    model_read_inputs(model_inputs);
    // re-arm DMA so fresh samples are captured before the next cycle
    pwm_input_rearm();

    // 2) execute a single 1 ms model step
    memset(model_outputs, 0, sizeof(model_outputs));
    if (active_model != NULL)
    {
      absolute_time_t model_t0 = get_absolute_time();
      ode_step(&solver, model_inputs);
      ode_output(&solver, model_inputs, model_outputs);
      model_exec_time_us = absolute_time_diff_us(model_t0, get_absolute_time());
    }

    // 3) write outputs: unified over all 8 PWM channels
    model_write_outputs(model_outputs);
    led_heartbeat(led_pins[0], to_ms_since_boot(current_time_core1), active_model != NULL);
  }

}

int main(void)
{
  const comm_config_t comm_config = {
    .model_select = comm_model_select,
    .reset = comm_reset,
    .reset_remote = comm_reset_remote,
    .parameters = comm_parameters,
  };

  // set clock speed to 150 MHz
  set_clock_speed_khz(150000);

  stdio_init_all();

  init_serial(&serial_console); // uart0: telemetry to the outside world
  init_serial(&serial_link);    // uart1: controller <-> model link
  comm_init(&comm_config);

  // initialize all peripherals
  pwm_input_init(pwm_input_pins, sizeof(pwm_input_pins) / sizeof(pwm_input_pins[0]));
  pwm_output_init(pwm_output_pins, sizeof(pwm_output_pins) / sizeof(pwm_output_pins[0]));
  led_init(led_pins, sizeof(led_pins) / sizeof(led_pins[0]));

  #if TESTING == 1
  // Test code for the model
  hw_test_routine((uint8_t *)"Model");
  #endif

  // model-selection queue must be ready before core 1 starts consuming it
  fifo_init(&model_cmd_queue, sizeof(model_cmd_t), MODEL_CMD_QUEUE_DEPTH, model_cmd_storage);
  fifo_init(&parameter_requests, sizeof(parameter_request_t), 1, parameter_request_storage);
  fifo_init(&parameter_results, sizeof(parameter_request_t), 1, parameter_result_storage);

  // enable model loop on separate core
  multicore_launch_core1(main_core1);

  absolute_time_t last_tlm = get_absolute_time();

  while (1)
  {
    uint32_t transfer_count_uart = circ_buff_new_data_amount(&serial_link.cb_rx);

    if (transfer_count_uart > 0)
    {
      uint32_t transfer_count;
      // handle UART
      if (transfer_count_uart > 0)
      {
          // reset number of transfers for UART RX in DMA circular mode
          circ_buff_hw_reset_transfer_count(serial_link.cb_rx.hw);
          circ_buff_read(&serial_link.cb_rx, buffer, transfer_count_uart);
          transfer_count = transfer_count_uart;
      }

      // parse the received message
      size_t len_out = 0;
      comm_parse_status_t ret = comm_parse(buffer, transfer_count,
                   (char *)buffer_uart_tx,
                   UART_BUFFER_TX_SIZE, &len_out);
      (void)ret;

      // check if response was generated and send it
      if (len_out > 0)
      {
          if (transfer_count > 0)
          {
              send_with_repeat(&serial_link, buffer_uart_tx, len_out);
          }
      }
    }

    transfer_count_uart = circ_buff_new_data_amount(&serial_console.cb_rx);

    if (transfer_count_uart > 0)
    {
      uint32_t transfer_count;
      // handle UART
      if (transfer_count_uart > 0)
      {
          // reset number of transfers for UART RX in DMA circular mode
          circ_buff_hw_reset_transfer_count(serial_console.cb_rx.hw);
          circ_buff_read(&serial_console.cb_rx, buffer, transfer_count_uart);
          transfer_count = transfer_count_uart;
      }

      // parse the received message
      size_t len_out = 0;
      comm_parse_status_t ret = comm_parse(buffer, transfer_count,
                   (char *)buffer_uart_tx,
                   UART_BUFFER_TX_SIZE, &len_out);
      (void)ret;

      // check if response was generated and send it
      if (len_out > 0)
      {
          if (transfer_count > 0)
          {
              send_with_repeat(&serial_console, buffer_uart_tx, len_out);
          }
      }
    }

    // telemetry: period set via $LOG (default 200 ms, 0 = off)
    absolute_time_t now = get_absolute_time();
    uint32_t tlm_period_us = comm_log_period_us();
    if (tlm_period_us != 0 && absolute_time_diff_us(last_tlm, now) >= (int64_t)tlm_period_us)
    {
      last_tlm = now;
      char *p = (char *)tlm_buf;
      p += esprintf(p, "[MODEL] t=%d ", (int)to_ms_since_boot(now));
      if (active_model != NULL)
      {
        p = tlm_append(p, "x=", model_state, active_model->n_states);
        p = tlm_append(p, " u=", model_inputs, active_model->n_inputs);
        if (comm_utilization_enabled())
        {
          float utilization = 100.0f * (float)model_exec_time_us / (float)MODEL_STEP_US;
          p = tlm_append(p, " util=", &utilization, 1);
        }

        if(active_selection == MODEL_MANIPULATOR_3R)
        {
          // end-effector position
          float ef[3];
          manipulator_3r_forward_kinematics(model_state, &manipulator_3r_params, ef);
          p = tlm_append(p, " ef=", ef, 3);
        }
      }
      else
      {
        p += esprintf(p, "OFF");
      }
      p += esprintf(p, "\r\n");
      send_with_repeat(&serial_console, tlm_buf, (uint32_t)((uint8_t *)p - tlm_buf));
    }
  }
}
