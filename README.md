# HIL Model — RP2350

Fixed-step ODE solver with pluggable dynamic models for Hardware-in-the-Loop simulation on RP2350.

## Overview

The project allows to run HIL testing using Raspberry Pi RP2350 boards. One of the boards serves as the "model" board, running a fixed-step ODE solver with a pluggable dynamic model (mass-spring-damper, pendulum-on-cart, double-pendulum, 3R manipulator, unicycle). The other board serves as the "controller" board, running a control law (PID, decoupling, or unicycle linearization) that reads the model outputs and drives the model inputs.

The communication between the model <-> controller boards is done in two ways:
- **UART1**: for high level commands, e.g. model selection, controller selection, setpoint updates, telemetry/logging.
- **PWM**: for real-time model I/O (8 channels each).

The task is to implement a controller for a given model. Models live in `models/` directory and implement various physical system. Since the implementation is done in C, a polymorphism-like implementation was derived using a common interface from `models/model.h`. Similarly, controllers live in `controllers/` directory and implement a common interface from `inc/controller.h`. The controller is selected at runtime using the `$CTRL,<id>` command.

## Communication

For manual testing of the controllers, you can derive e.g. a simple Python script that will oversee the communication. By default CRC checking is enabled for the communication protocol `src/comm.c`. To turn off the CRC checking send following command to the model/controller board:
`$CRC,0*4E` to disable CRC check.
This is a required step if you want to manually communicate with the boards and not calculate CRC for each command. Then for every command you can append whatever CRC value you want, e.g. `$SP,0,1.0*00` to set channel 0 to 1.0.

Both model and controller utilize two UARTs.

### Model Board

UART0 is used for communication and telemetry via PC. It is connected to a Raspberry Pi Pico acting as Debug Probe, thus available via USB. The default baud rate is 115200, 8N1. The commands are sent in the form of `$<CMD>,<args>*<XOR>\r\n` and the responses are sent back in the same format. The commands are parsed by `src/comm.c` and handled by the appropriate functions.
UART1 is used for communication with the controller board.

### Controller Board

UART0 is used for communication and telemetry via PC. It is connected to a Raspberry Pi Pico acting as Debug Probe, thus available via USB. The default baud rate is 115200, 8N1. The commands are sent in the form of `$<CMD>,<args>*<XOR>\r\n` and the responses are sent back in the same format. The commands are parsed by `src/comm.c` and handled by the appropriate functions.
UART1 is used for communication with the model board.

## Board Schematics

The schematic for the board can be found in [schematics](HIL_Testing_Platform_RP2350.pdf). The board has 8 PWM inputs and 8 PWM outputs, which are used to read the model outputs and drive the model inputs. The board also has two UARTs. Both are configures as 115200 baud 8N1.

## Project Structure

```
├── inc/
│   ├── ode.h              # ODE solver API
│   ├── controller.h       # Abstract controller interface
│   └── trajectory.h       # Setpoint-trajectory interface
├── controllers/
│   ├── controller.h       # (re-exported) abstract controller interface
│   ├── pid_ctrl.h/.c                 # Per-channel PID
│   ├── io_decoupling_3r.h/.c         # 3R input/output decoupling
│   ├── static_lin_unicycle_kin.h/.c  # Unicycle static lin. (kinematic model)
│   ├── static_lin_unicycle_dyn.h/.c  # Unicycle static lin. (dynamic model)
│   ├── dynamic_lin_unicycle_kin.h/.c # Unicycle dynamic lin. (kinematic model)
│   ├── dynamic_lin_unicycle_dyn.h/.c # Unicycle dynamic lin. (dynamic model, δ=π/2)
│   └── *_empty.c                     # Templates for controller implementations
├── models/
│   ├── model.h            # Model interface (function pointers)
│   ├── model.c
│   ├── mds.h              # Mass-Damper-Spring model
│   ├── mds.c
│   ├── pendulum_cart.h    # Pendulum-on-a-cart model
│   ├── pendulum_cart.c
│   ├── double_pendulum.h  # Double-pendulum model
│   ├── double_pendulum.c
│   ├── manipulator_3r.h   # 3R manipulator model
│   ├── manipulator_3r.c
│   ├── unicycle.h         # Kinematic unicycle model
│   └── unicycle.c
├── src/
│   ├── ode.c              # RK4 solver implementation
│   ├── trajectory.c       # Predefined setpoint trajectories
│   ├── main_model.c       # Embedded entry point (HIL Model)
│   ├── main_ctrl.c        # Embedded entry point (HIL Controller)
│   └── trajectory_empty.c # Weak publish-mode trajectory stub (see Publish mode)
├── ext/
│   └── libs-stm32/                 # External static lib (circ_buff, eprintfs, ealgebra)
└── CMakeLists.txt          # Embedded build (Pico SDK)
```
