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
# fallback (publish builds use OFF). External libraries independently override
# controller or trajectory source selection.
option(HIL_REAL_CONTROLLERS "Compile real controller/trajectory sources (empty stubs otherwise)" OFF)
option(HIL_USE_EXTERNAL_CONTROLLERS "Import a controller library instead of compiling controller sources" OFF)
set(HIL_CONTROLLERS_LIBRARY "" CACHE FILEPATH "Controller static library used when HIL_USE_EXTERNAL_CONTROLLERS is ON")
option(HIL_USE_EXTERNAL_TRAJECTORIES "Import a trajectory library instead of compiling trajectory sources" OFF)
set(HIL_TRAJECTORIES_LIBRARY "" CACHE FILEPATH "Trajectory static library used when HIL_USE_EXTERNAL_TRAJECTORIES is ON")

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

## Trajectories shared by both firmware targets

if(HIL_USE_EXTERNAL_TRAJECTORIES)
    get_filename_component(HIL_TRAJECTORIES_LIBRARY_ABS "${HIL_TRAJECTORIES_LIBRARY}"
        ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
    if(NOT HIL_TRAJECTORIES_LIBRARY OR NOT EXISTS "${HIL_TRAJECTORIES_LIBRARY_ABS}"
       OR IS_DIRECTORY "${HIL_TRAJECTORIES_LIBRARY_ABS}")
        message(FATAL_ERROR "HIL_USE_EXTERNAL_TRAJECTORIES requires HIL_TRAJECTORIES_LIBRARY to name an existing static library")
    endif()
    add_library(HIL_Trajectories STATIC IMPORTED)
    set_target_properties(HIL_Trajectories PROPERTIES
        IMPORTED_LOCATION "${HIL_TRAJECTORIES_LIBRARY_ABS}")
else()
    add_library(HIL_Trajectories STATIC ${HIL_TRAJ_SRC})
    target_include_directories(HIL_Trajectories PUBLIC inc)
    target_compile_options(HIL_Trajectories PRIVATE -O0 -Wall)
endif()

## Model 

add_executable(HIL_Model
    src/main_model.c
    src/pwm_input.c
    src/pwm_output.c
    src/led.c
    src/hw_test.c
    src/serial.c
    src/comm.c
    src/parameters.c
    src/utils.c

    models/model.c
    ${HIL_MODEL_SRC}
    src/ode.c
)

pico_generate_pio_header(HIL_Model ${CMAKE_CURRENT_LIST_DIR}/src/pwm_in.pio)

target_include_directories(HIL_Model 
PUBLIC
inc
models
controllers
${HIL_REGISTRY_DIR})

target_compile_options(HIL_Model 
PUBLIC 
-O0 -Wall
)

# pico_enable_stdio_usb(HIL_Model 1)
pico_enable_stdio_uart(HIL_Model 1)
pico_add_extra_outputs(HIL_Model)
target_link_libraries(HIL_Model 
    HIL_Trajectories
    pico_stdlib pico_unique_id hardware_pio hardware_pwm pico_multicore
    ext-libs-stm32
)

## Controller 

# External controllers take precedence over both registry source variants.
if(HIL_USE_EXTERNAL_CONTROLLERS)
    get_filename_component(HIL_CONTROLLERS_LIBRARY_ABS "${HIL_CONTROLLERS_LIBRARY}"
        ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
    if(NOT HIL_CONTROLLERS_LIBRARY OR NOT EXISTS "${HIL_CONTROLLERS_LIBRARY_ABS}"
       OR IS_DIRECTORY "${HIL_CONTROLLERS_LIBRARY_ABS}")
        message(FATAL_ERROR "HIL_USE_EXTERNAL_CONTROLLERS requires HIL_CONTROLLERS_LIBRARY to name an existing static library")
    endif()
    add_library(HIL_Controllers STATIC IMPORTED)
    set_target_properties(HIL_Controllers PROPERTIES
        IMPORTED_LOCATION "${HIL_CONTROLLERS_LIBRARY_ABS}")
else()
    add_library(HIL_Controllers STATIC ${HIL_CTRL_SRC})

    target_include_directories(HIL_Controllers
    PUBLIC
        inc
        models
        controllers
    PRIVATE
        ext/libs-stm32/ealgebra
    )

    target_compile_options(HIL_Controllers PRIVATE -O0 -Wall)
endif()

add_executable(HIL_Ctrl
    src/main_ctrl.c
    src/pwm_input.c
    src/pwm_output.c
    src/led.c
    src/hw_test.c
    src/serial.c
    src/comm.c
    src/parameters.c
    src/utils.c

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
    HIL_Controllers HIL_Trajectories
    pico_stdlib pico_unique_id hardware_pio hardware_pwm pico_multicore
    ext-libs-stm32
)
