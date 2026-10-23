"""Firmware catalogs and supported HIL model/controller pair metadata."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping


@dataclass(frozen=True)
class CatalogItem:
    id: int
    name: str


@dataclass(frozen=True)
class PairSpec:
    model: str
    model_id: int
    controller: str
    controller_id: int
    default_trajectories: tuple[str, ...]
    default_setpoints: tuple[float, ...] = ()
    tracking_mode: str = "direct"
    quality_mode: str = "tracking"
    tracking_labels: tuple[str, ...] = ("position",)
    tracking_unit: str = ""
    control_unit: str = ""
    control_limit: float | None = None
    max_rmse: float = 0.05
    requires_excitation: bool = True
    default_duration_s: float = 20.0
    minimum_quality_duration_s: float = 0.0
    estimate_target: float | None = None
    estimate_tolerance: float | None = None
    max_pendulum_angle: float | None = None
    pendulum_target_angle: float = 0.0
    model_state_labels: tuple[str, ...] = ()
    cartesian_planes: tuple[tuple[str, int, int], ...] = ()

    @property
    def setpoint_channels(self) -> tuple[int, ...]:
        count = len(self.default_trajectories) or len(self.default_setpoints)
        return tuple(range(count))

    def tracking_values(
        self, signals: Mapping[str, tuple[float, ...]]
    ) -> tuple[float, ...] | None:
        if self.tracking_mode == "end-effector":
            return signals.get("ef")
        if self.tracking_mode == "estimate":
            return signals.get("est")

        measured = signals.get("y")
        if measured is None:
            return None
        if self.tracking_mode == "static-unicycle":
            if len(measured) < 3:
                return None
            theta = measured[2]
            return (
                measured[0] + 0.1 * math.cos(theta + 0.05),
                measured[1] + 0.1 * math.sin(theta + 0.05),
            )
        if self.tracking_mode == "dynamic-unicycle":
            if len(measured) < 3:
                return None
            theta = measured[2]
            return (
                measured[0] - 0.1 * math.sin(theta),
                measured[1] + 0.1 * math.cos(theta),
            )
        return measured[: len(self.setpoint_channels)]


MODELS = (
    CatalogItem(0, "Off"),
    CatalogItem(1, "Mass-Spring-Damper"),
    CatalogItem(2, "Pendulum-Cart"),
    CatalogItem(3, "Double-Pendulum"),
    CatalogItem(4, "3R-Manipulator"),
    CatalogItem(5, "Unicycle"),
    CatalogItem(6, "Unicycle-Dynamic"),
)

CONTROLLERS = (
    CatalogItem(0, "PID"),
    CatalogItem(1, "IO-Decoupling-3R"),
    CatalogItem(2, "Static-Lin-Unicycle-Kin"),
    CatalogItem(3, "Static-Lin-Unicycle-Dyn"),
    CatalogItem(4, "Dynamic-Lin-Unicycle-Kin"),
    CatalogItem(5, "Dynamic-Lin-Unicycle-Dyn"),
    CatalogItem(6, "Samson-Unicycle-Kin"),
    CatalogItem(7, "Samson-Unicycle-Dyn"),
    CatalogItem(8, "FF-Double-Pendulum"),
    CatalogItem(9, "FF-Pendulum-Cart"),
    CatalogItem(10, "FF-MDS"),
    CatalogItem(11, "MDS-Adaptive-Mass"),
    CatalogItem(12, "MDS-Adaptive-Damping"),
    CatalogItem(13, "MDS-Adaptive-Stiffness"),
    CatalogItem(14, "MDS-Robust-Mass"),
    CatalogItem(15, "MDS-Robust-Damping"),
    CatalogItem(16, "MDS-Robust-Stiffness"),
    CatalogItem(17, "MDS-Estimator-Mass"),
    CatalogItem(18, "MDS-Estimator-Damping"),
    CatalogItem(19, "MDS-Estimator-Stiffness"),
    CatalogItem(20, "Upright-Pendulum-Cart"),
    CatalogItem(21, "Custom"),
)

TRAJECTORIES = {
    "sin": 0,
    "pwm": 1,
    "triangle": 2,
    "saw": 3,
    "circle-x": 4,
    "circle-y": 5,
    "circle-z": 6,
}

TRAJECTORY_CATALOG = (
    CatalogItem(0, "Sin"),
    CatalogItem(1, "PWM"),
    CatalogItem(2, "Triangle"),
    CatalogItem(3, "Saw"),
    CatalogItem(4, "Circle-X"),
    CatalogItem(5, "Circle-Y"),
    CatalogItem(6, "Circle-Z"),
)


SUPPORTED_PAIRS = {
    ("Mass-Spring-Damper", "PID"): PairSpec(
        "Mass-Spring-Damper", 1, "PID", 0, ("sin",),
        tracking_labels=("position",), tracking_unit="m", control_unit="N",
        control_limit=25.0, max_rmse=0.1,
    ),
    ("Mass-Spring-Damper", "FF-MDS"): PairSpec(
        "Mass-Spring-Damper", 1, "FF-MDS", 10, ("sin",),
        tracking_labels=("position",), tracking_unit="m", control_unit="N",
        control_limit=25.0, max_rmse=0.01,
    ),
    ("Mass-Spring-Damper", "MDS-Adaptive-Mass"): PairSpec(
        "Mass-Spring-Damper", 1, "MDS-Adaptive-Mass", 11, ("sin",),
        tracking_labels=("position",), tracking_unit="m", control_unit="N",
        control_limit=25.0, max_rmse=0.03,
        model_state_labels=("position", "velocity"),
    ),
    ("Mass-Spring-Damper", "MDS-Adaptive-Damping"): PairSpec(
        "Mass-Spring-Damper", 1, "MDS-Adaptive-Damping", 12, ("sin",),
        tracking_labels=("position",), tracking_unit="m", control_unit="N",
        control_limit=25.0, max_rmse=0.03,
        model_state_labels=("position", "velocity"),
    ),
    ("Mass-Spring-Damper", "MDS-Adaptive-Stiffness"): PairSpec(
        "Mass-Spring-Damper", 1, "MDS-Adaptive-Stiffness", 13, ("sin",),
        tracking_labels=("position",), tracking_unit="m", control_unit="N",
        control_limit=25.0, max_rmse=0.03,
        model_state_labels=("position", "velocity"),
    ),
    ("Mass-Spring-Damper", "MDS-Robust-Mass"): PairSpec(
        "Mass-Spring-Damper", 1, "MDS-Robust-Mass", 14, ("sin",),
        tracking_labels=("position",), tracking_unit="m", control_unit="N",
        control_limit=25.0, max_rmse=0.03,
        model_state_labels=("position", "velocity"),
    ),
    ("Mass-Spring-Damper", "MDS-Robust-Damping"): PairSpec(
        "Mass-Spring-Damper", 1, "MDS-Robust-Damping", 15, ("sin",),
        tracking_labels=("position",), tracking_unit="m", control_unit="N",
        control_limit=25.0, max_rmse=0.03,
        model_state_labels=("position", "velocity"),
    ),
    ("Mass-Spring-Damper", "MDS-Robust-Stiffness"): PairSpec(
        "Mass-Spring-Damper", 1, "MDS-Robust-Stiffness", 16, ("sin",),
        tracking_labels=("position",), tracking_unit="m", control_unit="N",
        control_limit=25.0, max_rmse=0.03,
        model_state_labels=("position", "velocity"),
    ),
    ("Mass-Spring-Damper", "MDS-Estimator-Mass"): PairSpec(
        "Mass-Spring-Damper", 1, "MDS-Estimator-Mass", 17, (),
        default_setpoints=(0.0,), tracking_mode="estimate",
        quality_mode="estimate", tracking_labels=("mass estimate",),
        tracking_unit="kg", control_unit="N", control_limit=25.0,
        requires_excitation=False, default_duration_s=40.0,
        minimum_quality_duration_s=39.0, estimate_target=1.0,
        estimate_tolerance=0.05, model_state_labels=("position", "velocity"),
    ),
    ("Mass-Spring-Damper", "MDS-Estimator-Damping"): PairSpec(
        "Mass-Spring-Damper", 1, "MDS-Estimator-Damping", 18, (),
        default_setpoints=(0.0,), tracking_mode="estimate",
        quality_mode="estimate", tracking_labels=("damping estimate",),
        tracking_unit="N s/m", control_unit="N", control_limit=25.0,
        requires_excitation=False, default_duration_s=40.0,
        minimum_quality_duration_s=39.0, estimate_target=0.1,
        estimate_tolerance=0.03, model_state_labels=("position", "velocity"),
    ),
    ("Mass-Spring-Damper", "MDS-Estimator-Stiffness"): PairSpec(
        "Mass-Spring-Damper", 1, "MDS-Estimator-Stiffness", 19, (),
        default_setpoints=(0.0,), tracking_mode="estimate",
        quality_mode="estimate", tracking_labels=("stiffness estimate",),
        tracking_unit="N/m", control_unit="N", control_limit=25.0,
        requires_excitation=False, default_duration_s=40.0,
        minimum_quality_duration_s=39.0, estimate_target=10.0,
        estimate_tolerance=0.3, model_state_labels=("position", "velocity"),
    ),
    ("3R-Manipulator", "IO-Decoupling-3R"): PairSpec(
        "3R-Manipulator", 4, "IO-Decoupling-3R", 1,
        ("circle-x", "circle-y", "circle-z"), tracking_mode="end-effector",
        tracking_labels=("x", "y", "z"), tracking_unit="m", control_unit="Nm",
        control_limit=10.0, max_rmse=0.05,
        model_state_labels=("q1", "q2", "q3", "dq1", "dq2", "dq3"),
        cartesian_planes=(("XY", 0, 1), ("XZ", 0, 2), ("YZ", 1, 2)),
    ),
    ("Unicycle", "Static-Lin-Unicycle-Kin"): PairSpec(
        "Unicycle", 5, "Static-Lin-Unicycle-Kin", 2,
        ("circle-x", "circle-y"), tracking_mode="static-unicycle",
        tracking_labels=("h_x", "h_y"), tracking_unit="m",
        control_unit="m/s, rad/s", control_limit=13.0, max_rmse=0.05,
        model_state_labels=("x", "y", "theta"),
        cartesian_planes=(("XY", 0, 1),),
    ),
    ("Unicycle", "Dynamic-Lin-Unicycle-Kin"): PairSpec(
        "Unicycle", 5, "Dynamic-Lin-Unicycle-Kin", 4,
        ("circle-x", "circle-y"), tracking_mode="dynamic-unicycle",
        tracking_labels=("h_x", "h_y"), tracking_unit="m",
        control_unit="m/s, rad/s", control_limit=13.0, max_rmse=0.05,
        model_state_labels=("x", "y", "theta"),
        cartesian_planes=(("XY", 0, 1),),
    ),
    ("Unicycle", "Samson-Unicycle-Kin"): PairSpec(
        "Unicycle", 5, "Samson-Unicycle-Kin", 6,
        ("circle-x", "circle-y"), tracking_labels=("x", "y"),
        tracking_unit="m", control_unit="m/s, rad/s", control_limit=13.0,
        max_rmse=0.05, model_state_labels=("x", "y", "theta"),
        cartesian_planes=(("XY", 0, 1),),
    ),
    ("Unicycle-Dynamic", "Static-Lin-Unicycle-Dyn"): PairSpec(
        "Unicycle-Dynamic", 6, "Static-Lin-Unicycle-Dyn", 3,
        ("circle-x", "circle-y"), tracking_mode="static-unicycle",
        tracking_labels=("h_x", "h_y"), tracking_unit="m", control_unit="Nm",
        control_limit=20.0, max_rmse=0.05,
        model_state_labels=("x", "y", "theta", "eta1", "eta2"),
        cartesian_planes=(("XY", 0, 1),),
    ),
    ("Unicycle-Dynamic", "Dynamic-Lin-Unicycle-Dyn"): PairSpec(
        "Unicycle-Dynamic", 6, "Dynamic-Lin-Unicycle-Dyn", 5,
        ("circle-x", "circle-y"), tracking_mode="dynamic-unicycle",
        tracking_labels=("h_x", "h_y"), tracking_unit="m", control_unit="Nm",
        control_limit=20.0, max_rmse=0.05,
        model_state_labels=("x", "y", "theta", "eta1", "eta2"),
        cartesian_planes=(("XY", 0, 1),),
    ),
    ("Unicycle-Dynamic", "Samson-Unicycle-Dyn"): PairSpec(
        "Unicycle-Dynamic", 6, "Samson-Unicycle-Dyn", 7,
        ("circle-x", "circle-y"), tracking_labels=("x", "y"),
        tracking_unit="m", control_unit="Nm", control_limit=20.0,
        max_rmse=0.05,
        model_state_labels=("x", "y", "theta", "eta1", "eta2"),
        cartesian_planes=(("XY", 0, 1),),
    ),
    ("Pendulum-Cart", "FF-Pendulum-Cart"): PairSpec(
        "Pendulum-Cart", 2, "FF-Pendulum-Cart", 9, ("sin",),
        tracking_labels=("cart position",), tracking_unit="m", control_unit="N",
        control_limit=85.0, max_rmse=0.1, max_pendulum_angle=0.5,
        model_state_labels=("cart x", "theta", "cart velocity", "theta rate"),
    ),
    ("Pendulum-Cart", "Upright-Pendulum-Cart"): PairSpec(
        "Pendulum-Cart", 2, "Upright-Pendulum-Cart", 20, (),
        default_setpoints=(0.0,), tracking_labels=("cart position",),
        tracking_unit="m", control_unit="N", control_limit=85.0,
        max_rmse=0.05, requires_excitation=False, max_pendulum_angle=0.05,
        pendulum_target_angle=math.pi,
        model_state_labels=("cart x", "theta", "cart velocity", "theta rate"),
    ),
    ("Double-Pendulum", "FF-Double-Pendulum"): PairSpec(
        "Double-Pendulum", 3, "FF-Double-Pendulum", 8, ("sin", "sin"),
        tracking_labels=("theta1", "theta2"), tracking_unit="rad", control_unit="Nm",
        control_limit=55.0, max_rmse=0.05,
    ),
}


def find_pair(model: str, controller: str) -> PairSpec | None:
    return SUPPORTED_PAIRS.get((model, controller))
