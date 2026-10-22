# =============================================================================
# Model / controller registry -- the single file owning the slot tables.
#
# Included by the root CMakeLists.txt. Responsibilities:
#   1. HIL_MODELS / HIL_CONTROLLERS slot lists (order = runtime slot ids):
#        "<SLOT_SYMBOL>|<Display Name>"
#      SLOT_SYMBOL becomes a C enum constant, Display Name is reported by the
#      "$MODELS" / "$CTRLS" replies (src/comm.c). MODEL_OFF stays first,
#      Custom stays LAST.
#   2. Explicit per-variant source lists (NO discovery, NO suffix construction):
#        HIL_MODEL_SRC   -- model sources (models have no stubs)
#        HIL_CTRL_REAL   -- real controller implementations
#        HIL_CTRL_EMPTY  -- *_empty.c publish stubs (custom_ctrl.c has no stub
#                           and appears in BOTH controller lists)
#      Depending on the HIL_REAL_CONTROLLERS definition, ONE controller variant
#      is preselected into HIL_CTRL_SRC (and HIL_TRAJ_SRC for the trajectory).
#   3. Generates build/gen/hil_registry.h (X-macro lists + enums + counts) from
#      the slot lists -- consumed by src/comm.c, src/main_model.c and
#      src/main_ctrl.c.
# =============================================================================

set(HIL_MODELS
    "MODEL_OFF|Off"
    "MODEL_MASS_SPRING_DAMPER|Mass-Spring-Damper"
    "MODEL_PENDULUM_CART|Pendulum-Cart"
    "MODEL_DOUBLE_PENDULUM|Double-Pendulum"
    "MODEL_MANIPULATOR_3R|3R-Manipulator"
    "MODEL_UNICYCLE|Unicycle"
    "MODEL_UNICYCLE_DYN|Unicycle-Dynamic"
)

set(HIL_CONTROLLERS
    "CTRL_PID|PID"
    "CTRL_IO3R|IO-Decoupling-3R"
    "CTRL_SL_UNI_KIN|Static-Lin-Unicycle-Kin"
    "CTRL_SL_UNI_DYN|Static-Lin-Unicycle-Dyn"
    "CTRL_DL_UNI_KIN|Dynamic-Lin-Unicycle-Kin"
    "CTRL_DL_UNI_DYN|Dynamic-Lin-Unicycle-Dyn"
    "CTRL_SAM_UNI_KIN|Samson-Unicycle-Kin"
    "CTRL_SAM_UNI_DYN|Samson-Unicycle-Dyn"
    "CTRL_FF_DPEND|FF-Double-Pendulum"
    "CTRL_FF_PCART|FF-Pendulum-Cart"
    "CTRL_FF_MDS|FF-MDS"
    "CTRL_MDS_AD_M|MDS-Adaptive-Mass"
    "CTRL_MDS_AD_C|MDS-Adaptive-Damping"
    "CTRL_MDS_AD_K|MDS-Adaptive-Stiffness"
    "CTRL_MDS_RB_M|MDS-Robust-Mass"
    "CTRL_MDS_RB_C|MDS-Robust-Damping"
    "CTRL_MDS_RB_K|MDS-Robust-Stiffness"
    "CTRL_MDS_ES_M|MDS-Estimator-Mass"
    "CTRL_MDS_ES_C|MDS-Estimator-Damping"
    "CTRL_MDS_ES_K|MDS-Estimator-Stiffness"
    "CTRL_UPRIGHT_PCART|Upright-Pendulum-Cart"
    "CTRL_CUSTOM|Custom"
)

# Model sources (unicycle.c backs two slots -> listed once).
set(HIL_MODEL_SRC
    models/mds.c
    models/pendulum_cart.c
    models/double_pendulum.c
    models/manipulator_3r.c
    models/unicycle.c
)

# Real controller implementations (one file per algorithm; the MDS files each
# back 3 slots).
set(HIL_CTRL_REAL
    controllers/pid_ctrl.c
    controllers/io_decoupling_3r.c
    controllers/static_lin_unicycle_kin.c
    controllers/static_lin_unicycle_dyn.c
    controllers/dynamic_lin_unicycle_kin.c
    controllers/dynamic_lin_unicycle_dyn.c
    controllers/samson_unicycle_kin.c
    controllers/samson_unicycle_dyn.c
    controllers/ff_double_pendulum.c
    controllers/ff_pendulum_cart.c
    controllers/ff_mds.c
    controllers/mds_adaptive.c
    controllers/mds_robust.c
    controllers/mds_estimator.c
    controllers/upright_pendulum_cart.c
    controllers/custom_ctrl.c
)

# Publish stubs (same public symbols, update()/eval() output 0). custom_ctrl.c
# is already a no-op template without a stub -- used in both variants.
set(HIL_CTRL_EMPTY
    controllers/pid_ctrl_empty.c
    controllers/io_decoupling_3r_empty.c
    controllers/static_lin_unicycle_kin_empty.c
    controllers/static_lin_unicycle_dyn_empty.c
    controllers/dynamic_lin_unicycle_kin_empty.c
    controllers/dynamic_lin_unicycle_dyn_empty.c
    controllers/samson_unicycle_kin_empty.c
    controllers/samson_unicycle_dyn_empty.c
    controllers/ff_double_pendulum_empty.c
    controllers/ff_pendulum_cart_empty.c
    controllers/ff_mds_empty.c
    controllers/mds_adaptive_empty.c
    controllers/mds_robust_empty.c
    controllers/mds_estimator_empty.c
    controllers/upright_pendulum_cart_empty.c
    controllers/custom_ctrl.c
)

# Preselect the controller/trajectory variant from the build-mode definition.
if(HIL_REAL_CONTROLLERS)
    set(HIL_CTRL_SRC ${HIL_CTRL_REAL})
    set(HIL_TRAJ_SRC src/trajectory.c)
else()
    set(HIL_CTRL_SRC ${HIL_CTRL_EMPTY})
    set(HIL_TRAJ_SRC src/trajectory_empty.c)
endif()

# Expand the slot lists into the X-macro bodies and counts for hil_registry.h.
set(HIL_MODEL_X)
foreach(e IN LISTS HIL_MODELS)
    string(REPLACE "|" ";" f "${e}")
    list(GET f 0 sym)
    list(GET f 1 name)
    list(APPEND HIL_MODEL_X "    X(${sym}, \"${name}\")")
endforeach()
list(LENGTH HIL_MODELS HIL_MODEL_COUNT)

set(HIL_CTRL_X)
foreach(e IN LISTS HIL_CONTROLLERS)
    string(REPLACE "|" ";" f "${e}")
    list(GET f 0 sym)
    list(GET f 1 name)
    list(APPEND HIL_CTRL_X "    X(${sym}, \"${name}\")")
endforeach()
list(LENGTH HIL_CONTROLLERS HIL_CTRL_COUNT)

# Join the X-macro entries with " \<newline>" into multi-line macro bodies.
string(REPLACE ";" " \\\n" HIL_MODEL_LIST "${HIL_MODEL_X}")
string(REPLACE ";" " \\\n" HIL_CTRL_LIST "${HIL_CTRL_X}")

# Registry header consumed by src/comm.c, src/main_model.c and src/main_ctrl.c.
set(HIL_REGISTRY_DIR ${CMAKE_CURRENT_BINARY_DIR}/gen)
configure_file(${CMAKE_CURRENT_LIST_DIR}/hil_registry.h.in ${HIL_REGISTRY_DIR}/hil_registry.h @ONLY)
