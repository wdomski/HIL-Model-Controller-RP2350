# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

set(PICO_PLATFORM "rp2350")
# set(PICO_BOARD "pico2")

include(pico_sdk_import.cmake)

project(HIL_Platform C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

## controller/trajectory implementation switch
# When ON (default) the REAL controller and trajectory sources are compiled;
# when OFF the *_empty.c stubs are compiled instead, so the firmware still
# builds but every controller/trajectory outputs 0. Exactly ONE variant is ever
# compiled: a real source missing under ON is a configure error, not a silent
# fallback (publish builds use OFF).
option(HIL_REAL_CONTROLLERS "Compile real controller/trajectory sources (empty stubs otherwise)" OFF)

## Model / controller registry: slot lists, explicit real/stub source lists,
# the HIL_REAL_CONTROLLERS preselection and the generated hil_registry.h all
# live in one dedicated file.
include(cmake/hil_registry.cmake)

## static external libraries

add_library(ext-libs-stm32 STATIC
    ext/libs-stm32/circ_buff/circ_buff_hw_rp2040.c
    ext/libs-stm32/circ_buff/circ_buff_hw.c
    ext/libs-stm32/circ_buff/circ_buff.c
    ext/libs-stm32/fifo/fifo.c
    ext/libs-stm32/fifo/fifo_hw_rp2040.c
    ext/libs-stm32/utils/time_math.c
    ext/libs-stm32/eprintfs/eprintfs.c
    ext/libs-stm32/aproto/aproto.c
    ext/libs-stm32/ealgebra/ealgebra.c
)

target_include_directories(ext-libs-stm32
PUBLIC
    ext/libs-stm32
    ext/libs-stm32/utils
    ext/libs-stm32/ealgebra
)

target_link_libraries(ext-libs-stm32
PUBLIC
    pico_stdlib hardware_timer hardware_dma
)

## Model 

add_executable(HIL_Model
    src/main_model.c
    src/pwm_input.c
    src/pwm_output.c
    src/led.c
    src/hw_test.c
    src/serial.c
    src/comm.c
    src/utils.c

    models/model.c
    ${HIL_MODEL_SRC}
    ${HIL_TRAJ_SRC}
    src/ode.c
)

pico_generate_pio_header(HIL_Model ${CMAKE_CURRENT_LIST_DIR}/src/pwm_in.pio)

target_include_directories(HIL_Model 
PUBLIC
inc
models
${HIL_REGISTRY_DIR})

target_compile_options(HIL_Model 
PUBLIC 
-O0 -Wall
)

# pico_enable_stdio_usb(HIL_Model 1)
pico_enable_stdio_uart(HIL_Model 1)
pico_add_extra_outputs(HIL_Model)
target_link_libraries(HIL_Model 
    pico_stdlib pico_unique_id hardware_pio hardware_pwm pico_multicore
    ext-libs-stm32
)

## Controller 

# Controller and trajectory sources come from the registry: the real or the stub
# variant preselected in cmake/hil_registry.cmake.
add_executable(HIL_Ctrl
    src/main_ctrl.c
    src/pwm_input.c
    src/pwm_output.c
    src/led.c
    src/hw_test.c
    src/serial.c
    src/comm.c
    src/utils.c

    ${HIL_CTRL_SRC}
    ${HIL_TRAJ_SRC}

    # model sources used by the controllers and the telemetry
    models/manipulator_3r.c
    models/unicycle.c
)

pico_generate_pio_header(HIL_Ctrl ${CMAKE_CURRENT_LIST_DIR}/src/pwm_in.pio)

target_include_directories(HIL_Ctrl 
PUBLIC
inc
models
controllers
${HIL_REGISTRY_DIR})

target_compile_options(HIL_Ctrl 
PUBLIC 
-O0 -Wall
)

# pico_enable_stdio_usb(HIL_Ctrl 1)
pico_enable_stdio_uart(HIL_Ctrl 1)
pico_add_extra_outputs(HIL_Ctrl)
target_link_libraries(HIL_Ctrl 
    pico_stdlib pico_unique_id hardware_pio hardware_pwm pico_multicore
    ext-libs-stm32
)
