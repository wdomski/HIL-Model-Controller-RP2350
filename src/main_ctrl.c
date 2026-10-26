#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

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
#include "parameters.h"
#include "hil_registry.h"
#include "controller.h"
#include "pid_ctrl.h"
#include "io_decoupling_3r.h"
#include "static_lin_unicycle_kin.h"
#include "static_lin_unicycle_dyn.h"
#include "dynamic_lin_unicycle_kin.h"
#include "dynamic_lin_unicycle_dyn.h"
#include "samson_unicycle_kin.h"
#include "samson_unicycle_dyn.h"
#include "ff_double_pendulum.h"
#include "ff_pendulum_cart.h"
#include "upright_pendulum_cart.h"
#include "ff_mds.h"
#include "mds_adaptive.h"
#include "mds_robust.h"
#include "mds_estimator.h"
#include "custom_ctrl.h"
#include "manipulator_3r.h"
#include "unicycle.h"
#include "trajectory.h"
#include "eprintfs/eprintfs.h"
#include "utils.h"

#include "circ_buff/circ_buff_hw.h"
#include "circ_buff/circ_buff.h"

uint8_t pwm_input_pins[] = {7, 9, 11, 13, 15, 17, 19, 21};
uint8_t pwm_output_pins[] = {6, 8, 10, 12, 14, 16, 18, 20};
uint8_t led_pins[] = {22, 26, 27, 28};

int ret;

#define TESTING 0

/*
 * Controller switch: all controllers share the controller_t interface, so the
 * 1 kHz loop only ever touches the active slot. "$CTRL,<id>" (uart0) selects it.
 *
 * PWM scales MUST mirror the model application (see *_SCALE in src/main_model.c):
 *   measured  <- model outputs (in_scale),  control -> model inputs (out_scale).
 */
#define CTRL_DEFAULT    CTRL_OFF

#define CTRL_STEP_US      1000   /* 1 kHz control loop */
/* Telemetry period is configured at runtime via "$LOG,<ms>" (see comm.c). */

/* Upper bound on measured/setpoint/control channels across all controllers. */
#define CTRL_MAX          8

/* ---- controller backing state (one instance per algorithm) ---- */
static pid_ctrl_t                  pid;
static io_decoupling_3r_t          io3r;
static static_lin_unicycle_kin_t   sl_unic_kin;
static static_lin_unicycle_dyn_t   sl_unic_dyn;
static dynamic_lin_unicycle_kin_t  dl_unic_kin;
static dynamic_lin_unicycle_dyn_t  dl_unic_dyn;
static samson_unicycle_kin_t       sm_unic_kin;
static samson_unicycle_dyn_t       sm_unic_dyn;
static ff_double_pendulum_t        ff_dpend;
static ff_pendulum_cart_t          ff_pcart;
static upright_pendulum_cart_t     upright_pcart;
static ff_mds_t                    ff_mds_st;
static mds_adaptive_t              mds_ad_m, mds_ad_c, mds_ad_k;
static mds_robust_t               mds_rb_m, mds_rb_c, mds_rb_k;
static mds_estimator_t            mds_es_m, mds_es_c, mds_es_k;
static custom_ctrl_t               custom;
static controller_t ctrl_pid, ctrl_io3r, ctrl_sl_kin, ctrl_sl_dyn, ctrl_dl_kin, ctrl_dl_dyn,
                    ctrl_sm_kin, ctrl_sm_dyn, ctrl_ff_dpend, ctrl_ff_pcart, ctrl_ff_mds,
                    ctrl_mds_ad_m, ctrl_mds_ad_c, ctrl_mds_ad_k,
                    ctrl_mds_rb_m, ctrl_mds_rb_c, ctrl_mds_rb_k,
                    ctrl_mds_es_m, ctrl_mds_es_c, ctrl_mds_es_k,
                    ctrl_upright_pcart, ctrl_custom;

/* Per-controller model parameters / gains (mirror src/main_model.c). */
static const pid_gains_t pid_gains_mds[1] = {
    { .kp = 40.0f, .ki = 20.0f, .kd = 8.0f, .out_min = -10.0f, .out_max = 10.0f,
      .integ_min = -1.0f, .integ_max = 1.0f }
};
static const unicycle_params_t unicycle_geom = {
    .wheel_radius = 0.05f, .axle_length = 0.15f, .e = 0.1f, .delta = 0.05f,
    .Iw = 0.000125f, .If = 0.01125f, .mw = 0.1f, .mf = 1.0f
};
static const manipulator_3r_params_t manip3r_params = {
    .m1 = 1.0f, .m2 = 1.0f, .m3 = 1.0f,
    .l1 = 1.0f, .l2 = 1.0f, .l3 = 1.0f,
    .a = 0.0f, .b = 0.0f, .c = 0.0f,
    .g = 9.81f, .Tv = 0.1f, .Ts = 0.05f, .Tk = 100.0f
};
/* Feedforward-controller plant parameters (mirror src/main_model.c). */
static const double_pendulum_params_t dpend_params = {
    .mass1 = 1.0f, .mass2 = 0.5f, .length1 = 1.0f, .length2 = 0.8f, .gravity = 9.81f
};
static const pendulum_cart_params_t pcart_params = {
    .cart_mass = 1.0f, .bob_mass = 0.5f, .length = 1.0f, .gravity = 9.81f
};
static const upright_pendulum_cart_config_t upright_pcart_config = {
  .energy_gain = 0.35f,
  .energy_target_scale = 1.1f,
  .cart_kp = 0.35f,
  .cart_kd = 0.8f,
  .stabilize_k = { -3.0f, 60.0f, -6.0f, 12.0f },
  .velocity_filter_tau = 0.05f,
  .kick_acceleration = 4.0f,
  .kick_duration = 0.25f,
  .catch_angle = 0.35f,
  .catch_rate = 3.0f,
  .catch_position = 1.5f,
  .release_angle = 0.65f,
  .release_rate = 6.0f,
  .release_position = 1.8f,
  .force_limit = 10.0f,
};
/* MDS plant parameters (mirror src/main_model.c mds_params). Each MDS
 * controller/estimator treats exactly one of these as unknown. */
static const mds_params_t mds_plant = { .mass = 1.0f, .damping = 0.1f, .stiffness = 10.0f };
static const mds_params_t mds_robust_nom_m = { .mass = 0.5f, .damping = 0.1f, .stiffness = 10.0f };
static const mds_params_t mds_robust_nom_c = { .mass = 1.0f, .damping = 0.2f, .stiffness = 10.0f };
static const mds_params_t mds_robust_nom_k = { .mass = 1.0f, .damping = 0.1f, .stiffness = 15.0f };
#define MDS_KP        4.0f   /* baseline tracking gain; derivation fixes Kd = 1 */
#define MDS_KD        1.0f
#define MDS_ADAPT_KP_M 16.0f /* mass profile needs tighter error dynamics */
#define MDS_ROBUST_KP 12.0f
#define FF_MDS_KP     25.0f  /* full-knowledge FF: critically damped (wn=5, zeta=1) */
#define FF_MDS_KD     10.0f
#define MDS_ADAPT_GM  80.0f  /* adaptive gain gamma (mass)      */
#define MDS_ADAPT_GC  240.0f /* adaptive gain gamma (damping)   */
#define MDS_ADAPT_GK  120.0f /* adaptive gain gamma (stiffness) */
#define MDS_EST_G     8000.0f/* estimator learning rate gamma */
#define MDS_EST_L0    25.0f  /* filter Lambda(s)=s^2+lambda1 s+lambda0 (wn=5, zeta=1) */
#define MDS_EST_L1    10.0f
#define MDS_DELTA_M   0.5f   /* robust uncertainty bounds (|param - nominal|) */
#define MDS_DELTA_C   0.2f
#define MDS_DELTA_K   7.0f
#define MDS_ROBUST_EPS 0.01f /* tanh boundary-layer width */
/* Multisine excitation for the estimators (persistent excitation). */
static const float mds_est_amp[MDS_EST_NSINE] = { 2.0f, 1.0f, 0.5f };
static const float mds_est_w[MDS_EST_NSINE]   = { 1.885f, 4.398f, 8.168f };
/* tunable controller gains (kp/kd) */
#define UNI_SL_KP    2.0f
#define UNI_SL_KD    1.0f
#define UNI_DL_KP    1.0f
#define UNI_DL_KD    3.0f
#define UNI_DL_CHI1  1.0f   /* keep nonzero: det(Kd) = chi1 */
#define UNI_DL_DYN_P 6.0f   /* dyn-lin (dynamics) Hurwitz bandwidth: (s+p)^3 */
#define UNI_SAM_K1   10.0f  /* Samson posture gains [k1, k2, k3] */
#define UNI_SAM_K2   64.0f
#define UNI_SAM_K3   10.0f
#define UNI_SAM_KV   200.0f /* Samson (dynamics) inner velocity-loop gain */
#define DPEND_KP     100.0f /* double-pendulum computed-torque PD gains */
#define DPEND_KD     20.0f
#define PCART_KP     25.0f  /* pendulum-cart partial-FBL PD gains */
#define PCART_KD     10.0f
static const float io3r_kp[3] = { 5.0f, 5.0f, 5.0f };
static const float io3r_kd[3] = { 5.0f, 5.0f, 5.0f };

/* Per-channel PWM scales (mirror src/main_model.c). */
#define TWO_PI_F 6.2831853f
static const float sc_meas_pid[1]   = { 2.4f };
static const float sc_ctrl_pid[1]   = { 50.0f };
static const float sc_meas_io3r[6]  = { TWO_PI_F, TWO_PI_F, TWO_PI_F,
                                        TWO_PI_F, TWO_PI_F, TWO_PI_F };
static const float sc_ctrl_io3r[3]  = { 20.0f, 20.0f, 20.0f };
static const float offset_traj_io3r[3] = { -0.12f, 0.0f, 0.0f };
static const float sc_meas_unic[3]   = { 8.0f, 8.0f, 7.0f };
static const float sc_ctrl_unic[2]   = { 26.0f, 26.0f };
static const float offset_traj_unic[2] = { -2.3f, -0.1f };
/* Both dynamics controllers drive the DYNAMIC unicycle model
   ([x, y, theta, v, omega], torque inputs); scales MUST mirror
   MODEL_UNICYCLE_DYN in src/main_model.c. */
static const float sc_meas_unic_dyn[5] = { 8.0f, 8.0f, 7.0f, 40.0f, 40.0f };
static const float sc_ctrl_unic_dyn[2] = { 40.0f, 40.0f };
/* FF-Double-Pendulum: measured [theta1,theta2], control torques;
   scales MUST mirror DPEND_ANGLE_SCALE / DPEND_TORQUE_SCALE in src/main_model.c. */
static const float sc_meas_dpend[2] = { TWO_PI_F, TWO_PI_F };
static const float sc_ctrl_dpend[2] = { 110.0f, 110.0f };
static const float sc_sp_dpend[2] = { 0.4f, 0.3f };
/* FF-Pendulum-Cart: measured [x, theta], control force;
   scales MUST mirror PEND_POS_SCALE / PEND_ANGLE_SCALE / PEND_FORCE_SCALE. */
static const float sc_meas_pcart[2] = { 8.0f, 8.0f };
static const float sc_ctrl_pcart[1] = { 170.0f };
static const float sc_meas_custom[1] = { 1.0f };
static const float sc_ctrl_custom[1] = { 1.0f };
/* Adaptive/robust MDS controllers read [x, xdot]; other MDS slots read [x]. */
static const float sc_meas_mds[2] = { 2.4f, 4.0f };
static const float sc_ctrl_mds[1] = { 50.0f };

/*
 * One ready-to-run controller with its PWM I/O dimensions. n_meas/n_sp/n_ctrl
 * may differ (e.g. the unicycle reads 3, targets 2, drives 2). Slot ids, display
 * names and the slot count come from the CMake-generated registry
 * (hil_registry.h); a registry slot without a wiring row below stays all-zero
 * and is rejected by comm_controller_select(), except for CTRL_OFF.
 */
typedef struct
{
    controller_t *ctrl;
    uint32_t n_meas;         /* measured channels read from PWM inputs   */
    uint32_t n_sp;           /* setpoint channels                        */
    uint32_t n_ctrl;         /* control channels written to PWM outputs  */
    const float *in_scale;   /* [n_meas] measured PWM scales */
    const float *out_scale;  /* [n_ctrl]  control PWM scales  */
    const float *traj_scale; /* optional [n_sp] trajectory multiplier */
    const float *traj_offset;/* optional [n_sp] trajectory offset     */
} ctrl_slot_t;

static ctrl_slot_t ctrl_slots[HIL_CTRL_COUNT] = {
    [CTRL_OFF]         = { 0 },
    [CTRL_PID]         = { &ctrl_pid,      1, 1, 1, sc_meas_pid,      sc_ctrl_pid      },
    [CTRL_IO3R]        = { &ctrl_io3r,     6, 3, 3, sc_meas_io3r,     sc_ctrl_io3r,
           NULL, offset_traj_io3r },
    [CTRL_SL_UNI_KIN]  = { &ctrl_sl_kin,   3, 2, 2, sc_meas_unic,     sc_ctrl_unic     },
    [CTRL_SL_UNI_DYN]  = { &ctrl_sl_dyn,   5, 2, 2, sc_meas_unic_dyn, sc_ctrl_unic_dyn },
    [CTRL_DL_UNI_KIN]  = { &ctrl_dl_kin,   3, 2, 2, sc_meas_unic,     sc_ctrl_unic     },
    [CTRL_DL_UNI_DYN]  = { &ctrl_dl_dyn,   5, 2, 2, sc_meas_unic_dyn, sc_ctrl_unic_dyn },
    [CTRL_SAM_UNI_KIN] = { &ctrl_sm_kin,   3, 2, 2, sc_meas_unic,     sc_ctrl_unic,
                 NULL, offset_traj_unic },
    [CTRL_SAM_UNI_DYN] = { &ctrl_sm_dyn,   5, 2, 2, sc_meas_unic_dyn, sc_ctrl_unic_dyn,
                 NULL, offset_traj_unic },
    [CTRL_FF_DPEND]    = { &ctrl_ff_dpend, 2, 2, 2, sc_meas_dpend,    sc_ctrl_dpend, sc_sp_dpend },
    [CTRL_FF_PCART]    = { &ctrl_ff_pcart, 2, 1, 1, sc_meas_pcart,    sc_ctrl_pcart    },
    [CTRL_FF_MDS]      = { &ctrl_ff_mds,   1, 1, 1, sc_meas_mds,      sc_ctrl_mds      },
    [CTRL_MDS_AD_M]    = { &ctrl_mds_ad_m, 2, 1, 1, sc_meas_mds,      sc_ctrl_mds      },
    [CTRL_MDS_AD_C]    = { &ctrl_mds_ad_c, 2, 1, 1, sc_meas_mds,      sc_ctrl_mds      },
    [CTRL_MDS_AD_K]    = { &ctrl_mds_ad_k, 2, 1, 1, sc_meas_mds,      sc_ctrl_mds      },
    [CTRL_MDS_RB_M]    = { &ctrl_mds_rb_m, 2, 1, 1, sc_meas_mds,      sc_ctrl_mds      },
    [CTRL_MDS_RB_C]    = { &ctrl_mds_rb_c, 2, 1, 1, sc_meas_mds,      sc_ctrl_mds      },
    [CTRL_MDS_RB_K]    = { &ctrl_mds_rb_k, 2, 1, 1, sc_meas_mds,      sc_ctrl_mds      },
    [CTRL_MDS_ES_M]    = { &ctrl_mds_es_m, 1, 1, 1, sc_meas_mds,      sc_ctrl_mds      },
    [CTRL_MDS_ES_C]    = { &ctrl_mds_es_c, 1, 1, 1, sc_meas_mds,      sc_ctrl_mds      },
    [CTRL_MDS_ES_K]    = { &ctrl_mds_es_k, 1, 1, 1, sc_meas_mds,      sc_ctrl_mds      },
    [CTRL_UPRIGHT_PCART] = { &ctrl_upright_pcart, 2, 1, 1, sc_meas_pcart, sc_ctrl_pcart },
    [CTRL_CUSTOM]      = { &ctrl_custom,   1, 1, 1, sc_meas_custom,   sc_ctrl_custom   },
};

static ctrl_slot_t *active;
static int selected_ctrl = CTRL_DEFAULT;

static float measured[CTRL_MAX];
static float setpoint[CTRL_MAX];
static float control[CTRL_MAX];

/*
 * Per-channel setpoint source: either a constant value or a predefined
 * trajectory. Selected at runtime via "$SP,..." / "$SPTRAJ,..." on uart0.
 */
typedef enum
{
  SP_SOURCE_CONST = 0,
  SP_SOURCE_TRAJ = 1
} sp_source_t;

typedef struct
{
  sp_source_t source;
  float constant;
  trajectory_t *traj;
  float epoch_s;
} setpoint_src_t;

static setpoint_src_t sp_src[CTRL_MAX] = {
  { .source = SP_SOURCE_CONST, .constant = 0.5f, .traj = NULL }
};

static float now_s(void)
{
  return (float)(to_us_since_boot(get_absolute_time()) * 1e-6);
}

static void sync_controller_trajectories(void)
{
  trajectory_t *trajectory[3] = { NULL, NULL, NULL };
  for (int i = 0; i < 3; i++)
  {
    if (sp_src[i].source == SP_SOURCE_TRAJ)
    {
      trajectory[i] = sp_src[i].traj;
    }
  }
    io_decoupling_3r_set_trajectory(
      &io3r,
      selected_ctrl == CTRL_IO3R ? trajectory[0] : NULL,
      selected_ctrl == CTRL_IO3R ? trajectory[1] : NULL,
      selected_ctrl == CTRL_IO3R ? trajectory[2] : NULL,
      selected_ctrl == CTRL_IO3R ? active->traj_scale : NULL,
      selected_ctrl == CTRL_IO3R ? active->traj_offset : NULL);
    dynamic_lin_unicycle_dyn_set_trajectory(
      &dl_unic_dyn,
      selected_ctrl == CTRL_DL_UNI_DYN ? trajectory[0] : NULL,
      selected_ctrl == CTRL_DL_UNI_DYN ? trajectory[1] : NULL);
    samson_unicycle_kin_set_trajectory(
      &sm_unic_kin,
      selected_ctrl == CTRL_SAM_UNI_KIN ? trajectory[0] : NULL,
      selected_ctrl == CTRL_SAM_UNI_KIN ? trajectory[1] : NULL);
    samson_unicycle_dyn_set_trajectory(
      &sm_unic_dyn,
      selected_ctrl == CTRL_SAM_UNI_DYN ? trajectory[0] : NULL,
      selected_ctrl == CTRL_SAM_UNI_DYN ? trajectory[1] : NULL);
    mds_adaptive_set_trajectory(&mds_ad_m, selected_ctrl == CTRL_MDS_AD_M ? trajectory[0] : NULL);
    mds_adaptive_set_trajectory(&mds_ad_c, selected_ctrl == CTRL_MDS_AD_C ? trajectory[0] : NULL);
    mds_adaptive_set_trajectory(&mds_ad_k, selected_ctrl == CTRL_MDS_AD_K ? trajectory[0] : NULL);
    mds_robust_set_trajectory(&mds_rb_m, selected_ctrl == CTRL_MDS_RB_M ? trajectory[0] : NULL);
    mds_robust_set_trajectory(&mds_rb_c, selected_ctrl == CTRL_MDS_RB_C ? trajectory[0] : NULL);
    mds_robust_set_trajectory(&mds_rb_k, selected_ctrl == CTRL_MDS_RB_K ? trajectory[0] : NULL);
}

/* Controller-algorithm execution time [us] (debug, updated every step). */
volatile int64_t ctrl_exec_time_us;

manipulator_3r_params_t manipulator_3r_params = {
    .m1 = 1.0f, .m2 = 1.0f, .m3 = 1.0f,
    .l1 = 1.0f, .l2 = 1.0f, .l3 = 1.0f,
    .a  = 0.0f, .b  = 0.0f, .c  = 0.0f,
    .g  = 9.81f,
    .Tv = 0.1f, .Ts = 0.05f, .Tk = 100.0f
};

/* UART parsing and controller updates share this core, so a complete block
 * commits between updates. Reading a slot does not select/reset it. */
static int comm_parameters(char kind, int id, float *values, size_t count, int write)
{
  if (kind == 'T') return parameters_trajectory(id, values, count, write);
  if (kind != 'C' || id < 0 || id >= HIL_CTRL_COUNT) return -1;
  int result = parameters_controller(id, ctrl_slots[id].ctrl, CTRL_STEP_US * 1e-6f,
                                     values, count, write);
  if (result >= 0 && write && id == CTRL_IO3R)
    manipulator_3r_params = io3r.model;
  return result;
}

/* Set a constant setpoint for the callback registered with comm_init(). */
int comm_setpoint_const(int index, float value)
{
  if (index < 0 || index >= (int)active->n_sp)
  {
    return -1;
  }
  sp_src[index].source = SP_SOURCE_CONST;
  sp_src[index].constant = value;
  sync_controller_trajectories();
  return 0;
}

/* Select a predefined trajectory as setpoint for channel `index`. */
int comm_setpoint_traj(int index, int type)
{
  if (index < 0 || index >= (int)active->n_sp)
  {
    return -1;
  }
  trajectory_t *tr = trajectory_get((trajectory_type_t)type);
  if (tr == NULL)
  {
    return -1;
  }
  sp_src[index].source = SP_SOURCE_TRAJ;
  sp_src[index].traj = tr;
  sp_src[index].epoch_s = now_s();
  sync_controller_trajectories();
  return 0;
}

/* Select the active controller for the callback registered with comm_init(). */
int comm_controller_select(int id)
{
  if (id < 0 || (uint32_t)id >= HIL_CTRL_COUNT ||
      (id != CTRL_OFF && ctrl_slots[id].ctrl == NULL))
  {
    return -1;
  }
  active = &ctrl_slots[id];
  if (active->ctrl != NULL)
  {
    controller_reset(active->ctrl);
  }
  else
  {
    memset(setpoint, 0, sizeof(setpoint));
    memset(control, 0, sizeof(control));
    for (uint32_t i = 0; i < CTRL_MAX; i++)
    {
      pwm_output_set_value(pwm_output_pins, i, 0.0f, 0.0f, 1.0f);
    }
  }
  selected_ctrl = id;
  sync_controller_trajectories();
  return 0;
}

/* Reset the active controller for the callback registered with comm_init(). */
void comm_reset(void)
{
  if (active != NULL && active->ctrl != NULL)
  {
    controller_reset(active->ctrl);
    float epoch_s = now_s();
    for (uint32_t i = 0; i < active->n_sp; i++)
    {
      sp_src[i].epoch_s = epoch_s;
    }
  }
}

/*
 * Forward a "$RST" to the model board over the uart1 link. Uses a local buffer,
 * not buffer_uart_tx, which holds the "$OK,RSTALL" parser response.
 */
void comm_reset_remote(void)
{
  char fwd[16];
  int len = comm_build_rst(fwd, sizeof(fwd));
  send_with_repeat(&serial_link, (uint8_t *)fwd, (uint32_t)len);
}

static uint8_t tlm_buf[192];

// append "label=v0,v1,...," to p, return advanced pointer
static char *tlm_append(char *p, const char *label, const float *v, uint32_t n)
{
  p += esprintf(p, "%s", label);
  for (uint32_t i = 0; i < n; i++)
  {
    p += esprintf(p, "%.3f%s", v[i], (i + 1 < n) ? "," : "");
  }
  return p;
}

static int controller_estimate(float *estimate)
{
  switch (selected_ctrl)
  {
    case CTRL_MDS_AD_M: *estimate = mds_ad_m.est; return 1;
    case CTRL_MDS_AD_C: *estimate = mds_ad_c.est; return 1;
    case CTRL_MDS_AD_K: *estimate = mds_ad_k.est; return 1;
    case CTRL_MDS_ES_M: *estimate = mds_es_m.est; return 1;
    case CTRL_MDS_ES_C: *estimate = mds_es_c.est; return 1;
    case CTRL_MDS_ES_K: *estimate = mds_es_k.est; return 1;
    default: return 0;
  }
}

int main(void)
{
  const comm_config_t comm_config = {
    .setpoint_const = comm_setpoint_const,
    .setpoint_traj = comm_setpoint_traj,
    .controller_select = comm_controller_select,
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
  // Test code for the controller
  hw_test_routine((uint8_t *)"Ctrl");
  #endif

  // initialize all controllers (each fills its controller_t vtable)
  pid_controller_init(&ctrl_pid, &pid, pid_gains_mds, 1, CTRL_STEP_US * 1e-6f);
  io_decoupling_3r_init(&ctrl_io3r, &io3r, &manip3r_params, io3r_kp, io3r_kd);
  static_lin_unicycle_kin_init(&ctrl_sl_kin, &sl_unic_kin, &unicycle_geom, UNI_SL_KP);
  static_lin_unicycle_dyn_init(&ctrl_sl_dyn, &sl_unic_dyn, &unicycle_geom, UNI_SL_KP, UNI_SL_KD);
  dynamic_lin_unicycle_kin_init(&ctrl_dl_kin, &dl_unic_kin, unicycle_geom.e, UNI_DL_KP,
                                UNI_DL_KD, UNI_DL_CHI1);
  {
      float p = UNI_DL_DYN_P;  /* Hurwitz cubic (s+p)^3: K0=p^3, K1=3p^2, K2=3p */
      dynamic_lin_unicycle_dyn_init(&ctrl_dl_dyn, &dl_unic_dyn, &unicycle_geom,
                                    p * p * p, 3.0f * p * p, 3.0f * p);
  }
  samson_unicycle_kin_init(&ctrl_sm_kin, &sm_unic_kin, &unicycle_geom,
                           UNI_SAM_K1, UNI_SAM_K2, UNI_SAM_K3);
  samson_unicycle_dyn_init(&ctrl_sm_dyn, &sm_unic_dyn, &unicycle_geom,
                           UNI_SAM_K1, UNI_SAM_K2, UNI_SAM_K3, UNI_SAM_KV);
  ff_double_pendulum_init(&ctrl_ff_dpend, &ff_dpend, &dpend_params, DPEND_KP, DPEND_KD);
  ff_pendulum_cart_init(&ctrl_ff_pcart, &ff_pcart, &pcart_params, PCART_KP, PCART_KD);
  upright_pendulum_cart_init(&ctrl_upright_pcart, &upright_pcart,
                             &pcart_params, &upright_pcart_config);
  ff_mds_init(&ctrl_ff_mds, &ff_mds_st, &mds_plant, FF_MDS_KP, FF_MDS_KD);
  /* MDS adaptive controllers: one unknown parameter, estimate starts off-true. */
  mds_adaptive_mass_init(&ctrl_mds_ad_m, &mds_ad_m, &mds_plant, 0.5f, MDS_ADAPT_GM, MDS_ADAPT_KP_M, MDS_KD);
  mds_adaptive_damping_init(&ctrl_mds_ad_c, &mds_ad_c, &mds_plant, 0.0f, MDS_ADAPT_GC, MDS_KP, MDS_KD);
  mds_adaptive_stiffness_init(&ctrl_mds_ad_k, &mds_ad_k, &mds_plant, 0.0f, MDS_ADAPT_GK, MDS_KP, MDS_KD);
  /* MDS robust controllers: bounded uncertainty on one parameter. */
  mds_robust_mass_init(&ctrl_mds_rb_m, &mds_rb_m, &mds_robust_nom_m, MDS_DELTA_M, MDS_ROBUST_KP, MDS_KD, MDS_ROBUST_EPS);
  mds_robust_damping_init(&ctrl_mds_rb_c, &mds_rb_c, &mds_robust_nom_c, MDS_DELTA_C, MDS_ROBUST_KP, MDS_KD, MDS_ROBUST_EPS);
  mds_robust_stiffness_init(&ctrl_mds_rb_k, &mds_rb_k, &mds_robust_nom_k, MDS_DELTA_K, MDS_ROBUST_KP, MDS_KD, MDS_ROBUST_EPS);
  /* MDS estimators: identify the true parameter via excitation. */
  mds_estimator_mass_init(&ctrl_mds_es_m, &mds_es_m, &mds_plant, 0.5f, MDS_EST_G, MDS_EST_L0, MDS_EST_L1, mds_est_amp, mds_est_w);
  mds_estimator_damping_init(&ctrl_mds_es_c, &mds_es_c, &mds_plant, 0.0f, MDS_EST_G, MDS_EST_L0, MDS_EST_L1, mds_est_amp, mds_est_w);
  mds_estimator_stiffness_init(&ctrl_mds_es_k, &mds_es_k, &mds_plant, 0.0f, MDS_EST_G, MDS_EST_L0, MDS_EST_L1, mds_est_amp, mds_est_w);
  custom_ctrl_init(&ctrl_custom, &custom, 1);
  // select the default controller slot
  comm_controller_select(CTRL_DEFAULT);

  absolute_time_t now;
  absolute_time_t last_ctrl = get_absolute_time();
  absolute_time_t last_tlm = last_ctrl;

  while (1)
  {
    now = get_absolute_time();

    // control loop at 1 kHz
    if (absolute_time_diff_us(last_ctrl, now) >= CTRL_STEP_US)
    {
      last_ctrl = now;

      // read model outputs (measured state); zero unused channels so a
      // controller never reads stale data beyond its dimension
      memset(measured, 0, sizeof(measured));
      for (uint32_t i = 0; i < active->n_meas; i++)
      {
        measured[i] = pwm_input_get_value(pwm_input_pins, i, 0.0f, active->in_scale[i]);
      }
      pwm_input_rearm();

      // resolve the active setpoint source (constant or trajectory)
      float t = (float)(to_us_since_boot(now) * 1e-6);
      for (uint32_t i = 0; i < active->n_sp; i++)
      {
        if (sp_src[i].source == SP_SOURCE_TRAJ && sp_src[i].traj != NULL)
        {
          float scale = (active->traj_scale != NULL) ? active->traj_scale[i] : 1.0f;
          float offset = (active->traj_offset != NULL) ? active->traj_offset[i] : 0.0f;
          setpoint[i] = scale * trajectory_eval(sp_src[i].traj, t - sp_src[i].epoch_s) + offset;
        }
        else
        {
          setpoint[i] = sp_src[i].constant;
        }
      }

      // compute the control signal (timed for debug)
      absolute_time_t ctrl_t0 = get_absolute_time();
      if (active->ctrl != NULL)
      {
        controller_update(active->ctrl, setpoint, measured, control, CTRL_STEP_US * 1e-6f);
      }
      ctrl_exec_time_us = absolute_time_diff_us(ctrl_t0, get_absolute_time());

      // write the control signal to the model inputs
      for (uint32_t i = 0; i < active->n_ctrl; i++)
      {
        pwm_output_set_value(pwm_output_pins, i, control[i], 0.0f, active->out_scale[i]);
      }
      led_heartbeat(led_pins[0], to_ms_since_boot(now), active->ctrl != NULL);
    }

    // handle user commands from uart0 (setpoint / trajectory selection)
    uint32_t rx_count = circ_buff_new_data_amount(&serial_console.cb_rx);
    if (rx_count > 0)
    {
      circ_buff_hw_reset_transfer_count(serial_console.cb_rx.hw);
      circ_buff_read(&serial_console.cb_rx, buffer, rx_count);

      size_t len_out = 0;
      comm_parse(buffer, rx_count, (char *)buffer_uart_tx,
             UART_BUFFER_TX_SIZE, &len_out);
      if (len_out > 0)
      {
        send_with_repeat(&serial_console, buffer_uart_tx, len_out);
      }
    }

    // telemetry: period set via $LOG (default 200 ms, 0 = off)
    uint32_t tlm_period_us = comm_log_period_us();
    if (tlm_period_us != 0 && absolute_time_diff_us(last_tlm, now) >= (int64_t)tlm_period_us)
    {
      last_tlm = now;
      char *p = (char *)tlm_buf;
      p += esprintf(p, "[CTRL] t=%d ", (int)to_ms_since_boot(now));
      if (selected_ctrl == CTRL_OFF)
      {
        p += esprintf(p, "OFF");
      }
      else
      {
        p = tlm_append(p, "y=", measured, active->n_meas);
        p = tlm_append(p, " sp=", setpoint, active->n_sp);
        p = tlm_append(p, " u=", control, active->n_ctrl);
      }
      if (comm_utilization_enabled())
      {
        float utilization = 100.0f * (float)ctrl_exec_time_us / (float)CTRL_STEP_US;
        p = tlm_append(p, " util=", &utilization, 1);
      }
      float estimate;
      if (controller_estimate(&estimate))
      {
        p = tlm_append(p, " est=", &estimate, 1);
      }
      if(selected_ctrl == CTRL_IO3R)
      {
        // end-effector position
        float ef[3];
        manipulator_3r_forward_kinematics(measured, &manipulator_3r_params, ef);
        p = tlm_append(p, " ef=", ef, 3);
      }
      p += esprintf(p, "\r\n");
      send_with_repeat(&serial_console, tlm_buf, (uint32_t)((uint8_t *)p - tlm_buf));
    }
  }
}

