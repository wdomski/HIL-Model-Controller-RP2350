"""Version-1 ordered parameter layouts and UART operations.

Keep these tuples in the same order as the explicit fields in src/parameters.c.
Only configuration is exposed, never integrators, estimates, or history.
"""

from __future__ import annotations

import math
from contextlib import ExitStack

from .protocol import BoardConnection, HilError, ProtocolError

VERSION = 1
Key = tuple[str, int]
MDS = ("mass [kg]", "damping [Ns/m]", "stiffness [N/m]")
CART = ("cart_mass [kg]", "bob_mass [kg]", "length [m]", "gravity [m/s²]")
DPEND = ("mass1 [kg]", "mass2 [kg]", "length1 [m]", "length2 [m]", "gravity [m/s²]")
ARM = ("m1 [kg]", "m2 [kg]", "m3 [kg]", "l1 [m]", "l2 [m]", "l3 [m]",
       "a [m]", "b [m]", "c [m]", "g [m/s²]", "Tv", "Ts", "Tk")


def nominal(names: tuple[str, ...]) -> tuple[str, ...]:
    return tuple(f"model.{name}" for name in names)


def known_mds(unknown: int) -> tuple[str, ...]:
    return nominal(MDS[:unknown] + MDS[unknown + 1:])


MODEL_PARAMETERS = {
    0: (), 1: MDS, 2: CART, 3: DPEND, 4: ARM,
    5: (),  # The velocity-driven kinematic plant does not use geometry.
    6: ("wheel_radius [m]", "axle_length [m]", "Iw [kg m²]", "If [kg m²]", "mw [kg]", "mf [kg]"),
}
CONTROLLER_PARAMETERS = {
    0: ("P", "I", "D"),
    1: ("P_x", "P_y", "P_z", "D_x", "D_y", "D_z") + nominal(ARM),
    2: ("P",), 3: ("P", "D"), 4: ("P", "D"),
    5: ("K0", "K1", "K2"),
    6: ("K1", "K2", "K3"), 7: ("K1", "K2", "K3", "Kv"),
    8: ("P", "D") + nominal(DPEND),
    9: ("P", "D") + nominal(CART),
    10: ("P", "D") + nominal(MDS),
    **{i: ("P", "Gamma", "Initial estimate") + known_mds(i - 11)
       for i in (11, 12, 13)},
    **{i: ("P", "Delta", "Epsilon") + nominal(MDS) for i in (14, 15, 16)},
    **{i: ("Gamma", "Lambda0", "Lambda1", "Initial estimate",
           "Amplitude1 [N]", "Amplitude2 [N]", "Amplitude3 [N]",
           "Omega1 [rad/s]", "Omega2 [rad/s]", "Omega3 [rad/s]")
          + known_mds(i - 17)
       for i in (17, 18, 19)},
    20: ("Energy gain", "Energy target scale", "Cart P", "Cart D",
         "Stabilize K0", "Stabilize K1", "Stabilize K2", "Stabilize K3",
         "Velocity filter tau [s]", "Kick acceleration [m/s²]", "Kick duration [s]",
         "Catch angle [rad]", "Catch rate [rad/s]", "Catch position [m]",
         "Release angle [rad]", "Release rate [rad/s]", "Release position [m]",
         "Force limit [N]") + nominal(CART),
    21: (),
}
TRAJECTORY_PARAMETERS = {
    0: ("Amplitude", "Omega (sin(pi*omega*t))", "Bias"),
    1: ("Frequency [Hz]", "Min", "Max", "Offset", "Duty [0..1]"),
    2: ("Frequency [Hz]", "Min", "Max", "Offset"),
    3: ("Frequency [Hz]", "Min", "Max", "Offset"),
    4: ("Radius [m]", "Omega [rad/s]", "Centre X [m]"),
    5: ("Radius [m]", "Omega [rad/s]", "Centre Y [m]"),
    6: ("Radius [m]", "Omega [rad/s]", "Centre Z [m]", "Z scale"),
}
LAYOUTS = {"M": MODEL_PARAMETERS, "C": CONTROLLER_PARAMETERS, "T": TRAJECTORY_PARAMETERS}


def parameter_names(kind: str, id: int) -> tuple[str, ...]:
    try:
        return LAYOUTS[kind][id]
    except KeyError as exc:
        raise ProtocolError(f"Unknown parameter layout {kind},{id}") from exc


def read_parameters(board: BoardConnection, kind: str, id: int) -> tuple[float, ...]:
    names = parameter_names(kind, id)
    reply = board.command(f"PGET,{kind},{id}").split(",")
    if reply[:5] != ["OK", "PGET", kind, str(id), str(VERSION)] or len(reply) != 5 + len(names):
        raise ProtocolError(f"Unexpected parameter layout for {kind},{id}: {','.join(reply)}")
    try:
        values = tuple(float(value) for value in reply[5:])
    except ValueError as exc:
        raise ProtocolError("Invalid numeric parameter response") from exc
    if not all(math.isfinite(value) for value in values):
        raise ProtocolError("Non-finite parameter response")
    return values


def set_parameters(board: BoardConnection, kind: str, id: int, values: tuple[float, ...]) -> None:
    if len(values) != len(parameter_names(kind, id)) or not all(math.isfinite(v) for v in values):
        raise HilError(f"Invalid parameter list for {kind},{id}")
    suffix = "".join(f",{value:.9g}" for value in values)
    if board.command(f"PSET,{kind},{id}{suffix}") != f"OK,PSET,{kind},{id}":
        raise ProtocolError(f"Unexpected parameter write reply for {kind},{id}")


def configuration_parameters(
    model_port: str, controller_port: str, keys: tuple[Key, ...],
    changes: dict[Key, tuple[float, ...]] | None = None,
) -> dict[Key, tuple[float, ...]]:
    """Read, or apply and read back UI-selected blocks without selecting slots.

    Each PSET is atomic on its board. A collection of writes across boards is
    not a transaction; failures identify the board and require a fresh read.
    For 3R, the model must use the same reference at reset as the controller.
    """
    if not model_port or not controller_port or model_port == controller_port:
        raise HilError("Select different model and controller ports")
    is_arm = ("M", 4) in keys
    values = {}
    with ExitStack() as stack:
        model = stack.enter_context(BoardConnection(model_port, "model"))
        controller = stack.enter_context(BoardConnection(controller_port, "controller"))
        for kind, id in keys:
            board = model if kind == "M" else controller
            if changes is not None:
                block = changes[(kind, id)]
                if kind == "T" and is_arm:
                    set_parameters(model, kind, id, block)
                set_parameters(board, kind, id, block)
            values[(kind, id)] = read_parameters(board, kind, id)
    return values
