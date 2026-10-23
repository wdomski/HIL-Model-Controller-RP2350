"""Tkinter frontend for the HIL experiment runner."""

from __future__ import annotations

import math
import operator
import queue
import threading
import tkinter as tk
from bisect import bisect_left, bisect_right
from datetime import datetime, timezone
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Callable

import serial
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d import Axes3D
from serial.tools import list_ports

if __package__:
    from .utils.protocol import detect_board
    from .hil_runner import (
        SUPPORTED_PAIRS,
        ExperimentConfig,
        ExperimentResult,
        HilError,
        TelemetrySample,
        default_output_path,
        pair_for,
        plot_result,
        run_experiment,
        write_csv,
    )
else:
    from utils.protocol import detect_board
    from hil_runner import (
        SUPPORTED_PAIRS,
        ExperimentConfig,
        ExperimentResult,
        HilError,
        TelemetrySample,
        default_output_path,
        pair_for,
        plot_result,
        run_experiment,
        write_csv,
    )

_OPERATIONS: dict[str, Callable[[float, float], float]] = {
    "A - B": operator.sub,
    "A + B": operator.add,
    "A * B": operator.mul,
    "A / B": operator.truediv,
}


class HilApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("RP2350 HIL Runner")
        self.root.minsize(1200, 760)
        self.root.protocol("WM_DELETE_WINDOW", self._close)

        first_model, first_controller = next(iter(SUPPORTED_PAIRS))
        self.model_port = tk.StringVar(value="COM6")
        self.controller_port = tk.StringVar(value="COM8")
        self.model = tk.StringVar(value=first_model)
        self.controller = tk.StringVar(value=first_controller)
        first_pair = SUPPORTED_PAIRS[(first_model, first_controller)]
        self.trajectory = tk.StringVar(value=",".join(first_pair.default_trajectories))
        self.duration = tk.StringVar(value=f"{first_pair.default_duration_s:g}")
        self.log_period = tk.StringVar(value="50")
        self.output = tk.StringVar(value=str(default_output_path()))
        self.status = tk.StringVar(value="Ready")
        self.indefinite = tk.BooleanVar(value=False)
        self.export_csv = tk.BooleanVar(value=False)
        self.export_png = tk.BooleanVar(value=False)
        self.follow_live = tk.BooleanVar(value=True)
        self.window_seconds = tk.StringVar(value="10")
        self.xy_enabled = tk.BooleanVar(value=False)
        self.xyz_enabled = tk.BooleanVar(value=False)
        self.xy_values = [tk.StringVar(value=f"MODEL.x[{i}]") for i in range(2)]
        self.xyz_values = [tk.StringVar(value=f"MODEL.x[{i}]") for i in range(3)]
        self.signal_a = tk.StringVar(value="CTRL.sp[0]")
        self.signal_b = tk.StringVar(value="CTRL.y[0]")
        self.signal_operation = tk.StringVar(value="A")
        self.signal_status = tk.StringVar(value="Add signals or A - B to plot errors.")

        self._messages: queue.Queue[tuple[str, object]] = queue.Queue()
        self._stop_event = threading.Event()
        self._worker: threading.Thread | None = None
        self._port_devices: dict[str, str] = {}
        self._port_boxes: list[ttk.Combobox] = []
        self._plot_data: dict[
            tuple[int, str, int], tuple[list[float], list[float], Line2D]
        ] = {}
        self._signals: dict[str, tuple[list[float], list[float]]] = {}
        self._selectors: list[ttk.Combobox] = []
        self._curves: dict[str, tuple[str, str, str | float, Line2D]] = {}
        self._auto_png = False
        self._result: ExperimentResult | None = None
        self._window_s = 10.0
        self._view_start = 0.0
        self._latest_time = 0.0
        self._detached = False
        self._closing = False

        self._build()
        self._refresh_ports()
        self.root.after(100, self._poll_messages)

    def _build(self) -> None:
        content = ttk.Frame(self.root, padding=16)
        content.grid(sticky="nsew")
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)
        content.columnconfigure(1, weight=1)

        fields = (
            ("Model port", self.model_port),
            ("Controller port", self.controller_port),
            ("Duration [s]", self.duration),
            ("Log period [ms]", self.log_period),
        )
        for row, (label, variable) in enumerate(fields):
            ttk.Label(content, text=label).grid(row=row, column=0, sticky="w", pady=4)
            if row < 2:
                field = ttk.Combobox(
                    content, textvariable=variable, width=38,
                    postcommand=self._refresh_ports,
                )
                self._port_boxes.append(field)
            else:
                field = ttk.Entry(content, textvariable=variable)
                if variable is self.duration:
                    self.duration_entry = field
            field.grid(
                row=row, column=1, columnspan=2, sticky="ew", padx=(12, 0), pady=4
            )

        row = len(fields)
        self.detect_button = ttk.Button(
            content, text="Autodetect ports", command=self._autodetect
        )
        self.detect_button.grid(row=row, column=0, sticky="ew")
        ttk.Checkbutton(
            content, text="Run indefinitely", variable=self.indefinite,
            command=lambda: self.duration_entry.configure(
                state="disabled" if self.indefinite.get() else "normal"
            ),
        ).grid(row=row, column=1, columnspan=2, sticky="w", padx=(12, 0))
        row += 1
        ttk.Label(content, text="Model").grid(row=row, column=0, sticky="w", pady=4)
        model_values = sorted({pair[0] for pair in SUPPORTED_PAIRS})
        model_box = ttk.Combobox(
            content, textvariable=self.model, values=model_values, state="readonly"
        )
        model_box.grid(row=row, column=1, columnspan=2, sticky="ew", padx=(12, 0), pady=4)
        model_box.bind("<<ComboboxSelected>>", self._model_changed)

        row += 1
        ttk.Label(content, text="Controller").grid(row=row, column=0, sticky="w", pady=4)
        self.controller_box = ttk.Combobox(
            content, textvariable=self.controller, state="readonly"
        )
        self.controller_box.grid(
            row=row, column=1, columnspan=2, sticky="ew", padx=(12, 0), pady=4
        )
        self.controller_box.bind("<<ComboboxSelected>>", self._pair_changed)
        self._model_changed()

        row += 1
        ttk.Label(content, text="Trajectories").grid(row=row, column=0, sticky="w", pady=4)
        ttk.Entry(content, textvariable=self.trajectory).grid(
            row=row, column=1, columnspan=2, sticky="ew", padx=(12, 0), pady=4
        )

        row += 1
        ttk.Label(content, text="CSV output").grid(row=row, column=0, sticky="w", pady=4)
        ttk.Entry(content, textvariable=self.output).grid(
            row=row, column=1, sticky="ew", padx=(12, 6), pady=4
        )
        ttk.Button(content, text="Browse...", command=self._browse).grid(
            row=row, column=2, sticky="ew", pady=4
        )

        row += 1
        ttk.Checkbutton(
            content, text="Automatically export CSV when stopped",
            variable=self.export_csv,
        ).grid(row=row, column=0, columnspan=3, sticky="w", pady=4)

        row += 1
        ttk.Checkbutton(
            content, text="Automatically export PNG when stopped",
            variable=self.export_png,
        ).grid(row=row, column=0, columnspan=3, sticky="w", pady=4)

        row += 1
        controls = ttk.Frame(content)
        controls.grid(row=row, column=0, columnspan=3, sticky="ew", pady=(16, 8))
        controls.columnconfigure(0, weight=1)
        self.run_button = ttk.Button(controls, text="Run", command=self._start)
        self.run_button.grid(row=0, column=0, sticky="e", padx=4)
        self.stop_button = ttk.Button(
            controls, text="Stop", command=self._stop, state="disabled"
        )
        self.stop_button.grid(row=0, column=1, sticky="e")
        self.export_button = ttk.Button(
            controls, text="Export logs", command=self._export_logs, state="disabled"
        )
        self.export_button.grid(row=0, column=2, padx=(8, 0))

        row += 1
        ttk.Separator(content).grid(row=row, column=0, columnspan=3, sticky="ew", pady=8)
        row += 1
        ttk.Label(content, textvariable=self.status, anchor="w", wraplength=420).grid(
            row=row, column=0, columnspan=3, sticky="ew"
        )

        row += 1
        readings = ttk.LabelFrame(content, text="Instantaneous values", padding=4)
        readings.grid(row=row, column=0, columnspan=3, sticky="nsew", pady=(12, 0))
        content.rowconfigure(row, weight=1)
        readings.columnconfigure(0, weight=1)
        readings.rowconfigure(0, weight=1)
        self._readings = ttk.Treeview(
            readings, columns=("value", "time"), height=7
        )
        self._readings.heading("#0", text="Signal")
        self._readings.heading("value", text="Value")
        self._readings.heading("time", text="Time [s]")
        for column, width in (("#0", 220), ("value", 90), ("time", 80)):
            self._readings.column(column, width=width, minwidth=60)
        self._readings.grid(row=0, column=0, sticky="nsew")
        values_scroll = ttk.Scrollbar(readings, command=self._readings.yview)
        values_scroll.grid(row=0, column=1, sticky="ns")
        self._readings.configure(yscrollcommand=values_scroll.set)

        self._plots = plots = tk.Frame(self.root)
        plots.grid(row=0, column=1, sticky="nsew", padx=(0, 16), pady=16)
        plots.columnconfigure(0, weight=1)
        plots.rowconfigure(1, weight=1)
        tools = ttk.Frame(plots)
        tools.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        ttk.Label(tools, text="Window [s]").pack(side="left")
        ttk.Entry(tools, textvariable=self.window_seconds, width=6).pack(side="left", padx=4)
        ttk.Button(tools, text="Apply", command=self._apply_window).pack(side="left")
        ttk.Checkbutton(
            tools, text="Follow live", variable=self.follow_live, command=self._redraw
        ).pack(side="left", padx=6)
        for index, (label, enabled) in enumerate(
            (("XY", self.xy_enabled), ("3D", self.xyz_enabled)), start=1
        ):
            ttk.Checkbutton(
                tools, text=label, variable=enabled,
                command=lambda i=index, flag=enabled: self._toggle_plot(i, flag.get()),
            ).pack(side="left")
        self._detach_button = ttk.Button(
            tools, text="Detach plots", command=self._toggle_preview
        )
        self._detach_button.pack(side="right")
        self._export_plot_button = ttk.Button(
            tools, text="Export PNG", command=self._export_plot, state="disabled"
        )
        self._export_plot_button.pack(side="right", padx=4)
        self._dock_command = plots.register(self._toggle_preview)

        self._notebook = ttk.Notebook(plots)
        self._notebook.grid(row=1, column=0, sticky="nsew")
        time_tab = ttk.Frame(self._notebook)
        time_tab.columnconfigure(0, weight=1)
        time_tab.rowconfigure(0, weight=1)
        self._notebook.add(time_tab, text="Time plots")
        figure = Figure(figsize=(6, 6), tight_layout=True)
        tracking_axis = figure.add_subplot(311)
        self._axes = (
            tracking_axis,
            figure.add_subplot(312, sharex=tracking_axis),
            figure.add_subplot(313, sharex=tracking_axis),
        )
        self._canvas = FigureCanvasTkAgg(figure, master=time_tab)
        self._canvas.get_tk_widget().grid(sticky="nsew")
        self._spatial_canvases: list[FigureCanvasTkAgg] = []
        for title, variables in (("XY", self.xy_values), ("3D", self.xyz_values)):
            tab = ttk.Frame(self._notebook)
            tab.columnconfigure(0, weight=1)
            tab.rowconfigure(1, weight=1)
            self._notebook.add(tab, text=title, state="disabled")
            selectors = ttk.Frame(tab)
            selectors.grid(row=0, column=0, sticky="ew", pady=6)
            for column, (name, variable) in enumerate(zip("XYZ", variables)):
                selectors.columnconfigure(column, weight=1)
                group = ttk.Frame(selectors)
                group.grid(row=0, column=column, sticky="ew", padx=4)
                ttk.Label(group, text=name).pack(anchor="w")
                box = ttk.Combobox(group, textvariable=variable, state="readonly", width=18)
                box.pack(fill="x")
                box.bind("<<ComboboxSelected>>", lambda _event: self._redraw())
                self._selectors.append(box)
            spatial_figure = Figure(figsize=(6, 6), tight_layout=True)
            if title == "3D":
                axis = spatial_figure.add_subplot(111, projection="3d")
                if not isinstance(axis, Axes3D):
                    raise HilError("Could not create the 3D preview")
                self._xyz_axis = axis
                self._xyz_line, = self._xyz_axis.plot([], [], [])
            else:
                self._xy_axis = spatial_figure.add_subplot(111)
                self._xy_line, = self._xy_axis.plot([], [])
            canvas = FigureCanvasTkAgg(spatial_figure, master=tab)
            canvas.get_tk_widget().grid(row=1, column=0, sticky="nsew")
            self._spatial_canvases.append(canvas)
        signals_tab = ttk.Frame(self._notebook)
        signals_tab.columnconfigure(0, weight=1)
        signals_tab.rowconfigure(2, weight=1)
        self._notebook.add(signals_tab, text="Signals")
        inputs = ttk.Frame(signals_tab)
        inputs.grid(row=0, column=0, sticky="ew", pady=6)
        for column, (label, variable, readonly) in enumerate((
            ("A (signal)", self.signal_a, True),
            ("Operation", self.signal_operation, True),
            ("B (signal or number)", self.signal_b, False),
        )):
            inputs.columnconfigure(column, weight=1)
            group = ttk.Frame(inputs)
            group.grid(row=0, column=column, sticky="ew", padx=4)
            ttk.Label(group, text=label).pack(anchor="w")
            box = ttk.Combobox(
                group, textvariable=variable, width=18,
                state="readonly" if readonly else "normal",
            )
            box.pack(fill="x")
            if variable is self.signal_operation:
                box.configure(values=("A", *_OPERATIONS))
            else:
                self._selectors.append(box)
        curves = ttk.Frame(signals_tab)
        curves.grid(row=1, column=0, sticky="ew", padx=4)
        curves.columnconfigure(0, weight=1)
        self._curve_list = tk.Listbox(
            curves, height=3, selectmode="extended", exportselection=False
        )
        self._curve_list.grid(row=0, column=0, rowspan=3, sticky="ew", padx=(0, 6))
        for row, (label, command) in enumerate((
            ("Add curve", self._add_curve),
            ("Add A-B error", self._add_difference),
            ("Remove selected", self._remove_curves),
        )):
            ttk.Button(curves, text=label, command=command).grid(
                row=row, column=1, sticky="ew"
            )
        signal_figure = Figure(figsize=(6, 6), tight_layout=True)
        self._signal_axis = signal_figure.add_subplot(111)
        self._signal_axis.set_xlabel("Capture time [s]")
        self._signal_axis.set_ylabel("Value")
        self._signal_axis.grid(True, alpha=0.3)
        self._signal_canvas = FigureCanvasTkAgg(signal_figure, master=signals_tab)
        self._signal_canvas.get_tk_widget().grid(row=2, column=0, sticky="nsew")
        ttk.Label(
            signals_tab, textvariable=self.signal_status, wraplength=550
        ).grid(row=3, column=0, sticky="w", padx=4)
        self._canvases = (self._canvas, *self._spatial_canvases, self._signal_canvas)
        self._time_scroll = ttk.Scrollbar(
            plots, orient="horizontal", command=self._scroll_time
        )
        self._time_scroll.grid(row=2, column=0, sticky="ew", pady=(6, 0))
        self._notebook.bind("<<NotebookTabChanged>>", lambda _event: self._redraw())
        self._reset_plots()

    def _apply_window(self) -> None:
        try:
            seconds = float(self.window_seconds.get())
            if not math.isfinite(seconds) or seconds <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Invalid window", "Window must be finite and greater than zero.")
            return
        self._window_s = seconds
        self._redraw()

    def _scroll_time(self, action: str, value: str, units: str = "") -> None:
        total = max(self._latest_time, self._window_s)
        start = (
            float(value) * total if action == "moveto"
            else self._view_start + int(value) * self._window_s * (
                0.9 if units == "pages" else 0.1
            )
        )
        maximum = max(0.0, total - self._window_s)
        self._view_start = max(0.0, min(start, maximum))
        self.follow_live.set(self._view_start >= maximum)
        self._redraw()

    def _toggle_preview(self) -> None:
        if self._detached:
            self.root.tk.call("wm", "forget", self._plots)
            self._plots.grid(row=0, column=1, sticky="nsew", padx=(0, 16), pady=16)
        else:
            self._plots.grid_forget()
            self.root.tk.call("wm", "manage", self._plots)
            self.root.tk.call("wm", "title", self._plots, "HIL plot preview")
            self.root.tk.call("wm", "geometry", self._plots, "850x700")
            self.root.tk.call("wm", "protocol", self._plots, "WM_DELETE_WINDOW", self._dock_command)
        self._detached = not self._detached
        self._detach_button.configure(text="Dock plots" if self._detached else "Detach plots")

    def _toggle_plot(self, index: int, enabled: bool) -> None:
        self._notebook.tab(index, state="normal" if enabled else "disabled")
        self._notebook.select(index if enabled else 0)
        self._redraw()

    def _aligned_series(self, keys: list[str]) -> tuple[list[float], list[list[float]]]:
        stamps: list[float] = []
        values: list[list[float]] = [[] for _ in keys]
        if not all(key in self._signals for key in keys):
            return stamps, values
        series = [self._signals[key] for key in keys]
        times = series[0][0]
        left = bisect_left(times, self._view_start)
        right = bisect_right(times, self._view_start + self._window_s)
        source = keys[0].split(".", 1)[0]
        for row in range(left, right):
            time = times[row]
            indices = [
                row if key.split(".", 1)[0] == source
                and row < len(stamps) and stamps[row] == time
                else bisect_right(stamps, time) - 1
                for key, (stamps, _) in zip(keys, series)
            ]
            if min(indices) >= 0:
                stamps.append(time)
                for output, (_, points), index in zip(values, series, indices):
                    output.append(points[index])
        return stamps, values

    def _spatial_values(self, variables: list[tk.StringVar]) -> list[list[float]]:
        return self._aligned_series([variable.get() for variable in variables])[1]

    def _add_curve(self) -> None:
        a, b, operation = (
            self.signal_a.get().strip(), self.signal_b.get().strip(),
            self.signal_operation.get(),
        )
        operand: str | float = b
        try:
            if a not in self._signals:
                raise ValueError("Choose an available signal for A.")
            if operation != "A":
                if operation not in _OPERATIONS:
                    raise ValueError("Choose a supported operation.")
                if b not in self._signals:
                    operand = float(b)
                    if not math.isfinite(operand):
                        raise ValueError("B must be an available signal or a finite number.")
        except ValueError as exc:
            messagebox.showerror("Invalid curve", str(exc))
            return
        label = a if operation == "A" else f"{a} {operation.split()[1]} {b}"
        if label not in self._curves:
            line, = self._signal_axis.plot([], [], label=label)
            self._curves[label] = (a, operation, operand, line)
            self._curve_list.insert("end", label)
            self._signal_axis.legend(loc="upper right", fontsize="small")
        self._curve_list.selection_clear(0, "end")
        self._curve_list.selection_set(tuple(self._curves).index(label))
        self._notebook.select(3)
        self._redraw()

    def _add_difference(self) -> None:
        self.signal_operation.set("A - B")
        self._add_curve()

    def _remove_curves(self) -> None:
        selected = self._curve_list.curselection()
        if not selected:
            messagebox.showinfo("Remove curves", "Select one or more curves to remove.")
            return
        for index in reversed(selected):
            *_, line = self._curves.pop(self._curve_list.get(index))
            line.remove()
            self._curve_list.delete(index)
        legend = self._signal_axis.get_legend()
        if self._curves:
            self._signal_axis.legend(loc="upper right", fontsize="small")
        elif legend is not None:
            legend.remove()
        self._redraw()

    def _update_signal_plot(self) -> None:
        messages = []
        for label, (a, operation, b, line) in self._curves.items():
            keys = [a, b] if operation != "A" and isinstance(b, str) else [a]
            times, columns = self._aligned_series(keys)
            if any(key not in self._signals for key in keys):
                line.set_data([], [])
                messages.append(f"Waiting for signals: {label}")
                continue
            points = columns[0]
            if operation != "A":
                other = columns[1] if isinstance(b, str) else [b] * len(points)
                computed = []
                for left, right in zip(points, other):
                    try:
                        computed.append(_OPERATIONS[operation](left, right))
                    except (ZeroDivisionError, OverflowError):
                        computed.append(math.nan)
                points = computed
            invalid = sum(not math.isfinite(value) for value in points)
            if invalid:
                messages.append(f"{label}: {invalid} undefined/non-finite samples shown as gaps")
                points = [value if math.isfinite(value) else math.nan for value in points]
            line.set_data(times, points)
        self.signal_status.set("; ".join(messages) or (
            "Add signals or A - B to plot errors." if not self._curves
            else "Signals aligned at A's sample times using the latest available B."
        ))
        self._signal_axis.relim()
        self._signal_axis.autoscale_view(scalex=False)
        self._signal_axis.set_xlim(self._view_start, self._view_start + self._window_s)

    def _redraw(self) -> None:
        maximum = max(0.0, self._latest_time - self._window_s)
        self._view_start = maximum if self.follow_live.get() else min(self._view_start, maximum)
        end = self._view_start + self._window_s
        for times, points, line in self._plot_data.values():
            left, right = bisect_left(times, self._view_start), bisect_right(times, end)
            line.set_data(times[left:right], points[left:right])
        for axis in self._axes:
            axis.relim()
            axis.autoscale_view(scalex=False)
        self._axes[0].set_xlim(self._view_start, end)
        total = max(self._latest_time, self._window_s)
        self._time_scroll.set(self._view_start / total, min(1.0, end / total))
        available = tuple(sorted(self._signals))
        for box in self._selectors:
            box.configure(values=available)
        if self.xy_enabled.get():
            x, y = self._spatial_values(self.xy_values)
            self._xy_line.set_data(x, y)
            self._xy_axis.set_xlabel(self.xy_values[0].get())
            self._xy_axis.set_ylabel(self.xy_values[1].get())
            self._xy_axis.relim()
            self._xy_axis.autoscale_view()
        if self.xyz_enabled.get():
            x, y, z = self._spatial_values(self.xyz_values)
            self._xyz_line.set_data_3d(x, y, z)
            self._xyz_axis.set_xlabel(self.xyz_values[0].get())
            self._xyz_axis.set_ylabel(self.xyz_values[1].get())
            self._xyz_axis.set_zlabel(self.xyz_values[2].get())
            if x:
                self._xyz_axis.auto_scale_xyz(x, y, z)
        self._update_signal_plot()
        selected = self._notebook.index(self._notebook.select())
        self._canvases[selected].draw_idle()

    def _refresh_ports(self) -> None:
        try:
            ports = sorted(list_ports.comports())
        except (OSError, serial.SerialException) as exc:
            messagebox.showerror("Serial ports", str(exc))
            return
        selected = [
            self._port_devices.get(variable.get(), variable.get())
            for variable in (self.model_port, self.controller_port)
        ]
        self._port_devices = {
            f"{port.device} - {port.description}"
            + (f" [S/N: {port.serial_number}]" if port.serial_number else ""): port.device
            for port in ports
        }
        labels = {device: label for label, device in self._port_devices.items()}
        for box, device in zip(self._port_boxes, selected):
            box.configure(values=tuple(self._port_devices))
            box.set(labels.get(device, device))

    def _autodetect(self) -> None:
        self._stop_event.clear()
        self.run_button.configure(state="disabled")
        self.detect_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self.status.set("Scanning serial ports...")
        self._worker = threading.Thread(target=self._detect_worker, daemon=True)
        self._worker.start()

    def _detect_worker(self) -> None:
        detected: dict[str, list[str]] = {"model": [], "controller": []}
        errors: list[str] = []
        try:
            for port in sorted(list_ports.comports()):
                if self._stop_event.is_set():
                    break
                self._messages.put(("status", f"Scanning {port.device}..."))
                try:
                    role = detect_board(port.device, timeout_s=2.0)
                    detected[role].append(port.device)
                except (HilError, OSError, serial.SerialException) as exc:
                    errors.append(f"{port.device}: {exc}")
            self._messages.put(("ports", (detected, errors)))
        except (OSError, serial.SerialException) as exc:
            self._messages.put(("error", exc))

    def _reset_plots(self) -> None:
        pair = SUPPORTED_PAIRS[(self.model.get(), self.controller.get())]
        if pair != getattr(self, "_plot_pair", None):
            source = "CTRL.ef" if pair.tracking_mode == "end-effector" else "MODEL.x"
            for variables in (self.xy_values, self.xyz_values):
                for channel, variable in enumerate(variables):
                    variable.set(f"{source}[{channel}]")
        self._plot_pair = pair
        self._plot_data.clear()
        self._signals.clear()
        self._view_start = self._latest_time = 0.0
        self.follow_live.set(True)
        self._readings.delete(*self._readings.get_children())
        self.export_button.configure(state="disabled")
        self._export_plot_button.configure(state="disabled")
        for axis, label in zip(self._axes, (
            f"Tracking [{pair.tracking_unit}]" if pair.tracking_unit else "Tracking",
            f"Control [{pair.control_unit}]" if pair.control_unit else "Control",
            "Model states",
        )):
            axis.clear()
            axis.set_ylabel(label)
            axis.grid(True, alpha=0.3)
        self._axes[-1].set_xlabel("Capture time [s]")
        self._redraw()

    def _plot_sample(self, sample: TelemetrySample) -> None:
        pair = self._plot_pair
        self._latest_time = max(self._latest_time, sample.host_time_s)
        if sample.signals:
            self._export_plot_button.configure(state="normal")
        for signal, values in sample.signals.items():
            for channel, value in enumerate(values):
                key = f"{sample.source}.{signal}[{channel}]"
                if key not in self._signals:
                    self._signals[key] = ([], [])
                    label = (
                        f" ({pair.model_state_labels[channel]})"
                        if sample.source == "MODEL" and signal == "x"
                        and channel < len(pair.model_state_labels) else ""
                    )
                    self._readings.insert("", "end", iid=key, text=key + label)
                times, points = self._signals[key]
                times.append(sample.host_time_s)
                points.append(value)
                self._readings.item(key, values=(f"{value:.6g}", f"{sample.host_time_s:.3f}"))
        if sample.source == "CTRL":
            reference = (
                (pair.estimate_target,)
                if pair.quality_mode == "estimate" and pair.estimate_target is not None
                else sample.signals.get("sp", ())
            )
            groups = (
                (0, "sp", reference, pair.tracking_labels),
                (0, "actual", pair.tracking_values(sample.signals) or (), pair.tracking_labels),
                (1, "u", sample.signals.get("u", ()), ()),
            )
        else:
            groups = ((2, "x", sample.signals.get("x", ()), pair.model_state_labels),)
        for axis_index, signal, values, labels in groups:
            axis = self._axes[axis_index]
            for channel, value in enumerate(values):
                key = (axis_index, signal, channel)
                if key not in self._plot_data:
                    label = labels[channel] if channel < len(labels) else f"{signal}[{channel}]"
                    if signal == "sp":
                        label = f"sp {label}"
                    line, = axis.plot(
                        [], [], "--" if signal == "sp" else "-",
                        color=f"C{channel % 10}", label=label,
                    )
                    self._plot_data[key] = ([], [], line)
                    axis.legend(loc="upper right", fontsize="small")
                times, points, line = self._plot_data[key]
                times.append(sample.host_time_s)
                points.append(value)

    def _model_changed(self, _event: object | None = None) -> None:
        controllers = sorted(
            controller
            for model, controller in SUPPORTED_PAIRS
            if model == self.model.get()
        )
        self.controller_box.configure(values=controllers)
        if self.controller.get() not in controllers and controllers:
            self.controller.set(controllers[0])
        self._pair_changed()

    def _pair_changed(self, _event: object | None = None) -> None:
        pair = SUPPORTED_PAIRS.get((self.model.get(), self.controller.get()))
        if pair is not None:
            self.trajectory.set(",".join(pair.default_trajectories))
            self.duration.set(f"{pair.default_duration_s:g}")

    def _browse(self) -> None:
        selected = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=(("CSV logs", "*.csv"), ("All files", "*.*")),
            initialfile=Path(self.output.get()).name,
        )
        if selected:
            self.output.set(selected)

    def _export_logs(self) -> None:
        if self._result is None or not self._result.samples:
            messagebox.showinfo("Export logs", "No telemetry has been captured yet.")
            return
        output = Path(self.output.get().strip() or default_output_path())
        try:
            write_csv(self._result, output)
        except (OSError, HilError) as exc:
            messagebox.showerror("Export logs", str(exc))
            return
        self.status.set(f"Exported {len(self._result.samples)} samples to {output}")

    def _export_plot(self) -> None:
        if not self._signals:
            messagebox.showinfo("Export PNG", "No telemetry has been captured yet.")
            return
        index = self._notebook.index(self._notebook.select())
        tab = self._notebook.tab(index, "text").lower().replace(" ", "_")
        output = Path(self.output.get().strip() or default_output_path())
        selected = filedialog.asksaveasfilename(
            title="Export current plot", defaultextension=".png",
            filetypes=(("PNG images", "*.png"),),
            initialdir=str(output.parent), initialfile=f"{output.stem}_{tab}.png",
        )
        if selected:
            try:
                self._redraw()
                self._canvases[index].figure.savefig(selected, dpi=150)
            except (OSError, ValueError) as exc:
                messagebox.showerror("Export PNG", str(exc))
                return
            self.status.set(f"Exported plot to {selected}")

    def _start(self) -> None:
        try:
            config = ExperimentConfig(
                model_port=self._port_devices.get(
                    self.model_port.get(), self.model_port.get()
                ).strip(),
                controller_port=self._port_devices.get(
                    self.controller_port.get(), self.controller_port.get()
                ).strip(),
                model=self.model.get(),
                controller=self.controller.get(),
                trajectories=tuple(
                    item.strip().lower()
                    for item in self.trajectory.get().split(",")
                    if item.strip()
                ),
                duration_s=None if self.indefinite.get() else float(self.duration.get()),
                log_period_ms=int(self.log_period.get()),
            )
            pair_for(config)
        except ValueError:
            messagebox.showerror("Invalid settings", "Duration and log period must be numbers.")
            return
        except HilError as exc:
            messagebox.showerror("Invalid settings", str(exc))
            return

        output = Path(self.output.get().strip() or default_output_path())
        self._reset_plots()
        self._result = ExperimentResult(config, datetime.now(timezone.utc), [])
        self._auto_png = self.export_png.get()
        self._stop_event.clear()
        self.run_button.configure(state="disabled")
        self.detect_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self.status.set("Connecting...")
        self._worker = threading.Thread(
            target=self._run_worker, args=(config, output, self.export_csv.get()), daemon=True
        )
        self._worker.start()

    def _run_worker(
        self, config: ExperimentConfig, output: Path, export_csv: bool
    ) -> None:
        try:
            result = run_experiment(
                config,
                status=lambda message: self._messages.put(("status", message)),
                stop_event=self._stop_event,
                on_sample=lambda sample: self._messages.put(("sample", sample)),
            )
            if export_csv:
                write_csv(result, output)
            self._messages.put(("done", (result, output, export_csv)))
        except Exception as exc:
            self._messages.put(("error", exc))

    def _stop(self) -> None:
        self._stop_event.set()
        self.status.set("Stopping...")

    def _close(self) -> None:
        if self._worker is not None:
            self._closing = True
            self._stop()
        else:
            self.root.destroy()

    def _poll_messages(self) -> None:
        plot_changed = False
        try:
            while True:
                kind, payload = self._messages.get_nowait()
                if kind == "status":
                    self.status.set(str(payload))
                elif kind == "ports":
                    if not isinstance(payload, tuple) or len(payload) != 2:
                        raise HilError("Invalid port scan result")
                    detected, errors = payload
                    self._set_idle()
                    if self._stop_event.is_set():
                        self.status.set("Port scan stopped")
                        continue
                    summary = []
                    incomplete = False
                    for role, variable in (
                        ("model", self.model_port), ("controller", self.controller_port)
                    ):
                        matches = detected[role]
                        if len(matches) == 1:
                            variable.set(matches[0])
                            summary.append(f"{role.title()}: {matches[0]}")
                        else:
                            incomplete = True
                            summary.append(
                                f"{role.title()}: multiple matches ({', '.join(matches)})"
                                if matches else f"{role.title()}: not found"
                            )
                    self._refresh_ports()
                    message = "; ".join(summary + errors)
                    self.status.set(message)
                    if incomplete:
                        messagebox.showwarning("Autodetect ports", message)
                elif kind == "sample":
                    if not isinstance(payload, TelemetrySample):
                        raise HilError("Invalid telemetry sample")
                    if self._result is not None:
                        self._result.samples.append(payload)
                        self.export_button.configure(state="normal")
                    self._plot_sample(payload)
                    plot_changed = True
                elif kind == "done":
                    if not isinstance(payload, tuple) or len(payload) != 3:
                        raise HilError("Invalid worker result")
                    result, output, exported_csv = payload
                    if not isinstance(result, ExperimentResult) or not isinstance(output, Path):
                        raise HilError("Invalid worker result")
                    self._result = result
                    self._set_idle()
                    saved = [str(output)] if exported_csv else []
                    if self._auto_png:
                        plot_path = output.with_suffix(".png")
                        plot_result(result, plot_path, show=False)
                        saved.append(str(plot_path))
                    self.status.set(
                        f"Capture complete: {len(result.samples)} samples"
                        + (f"; saved {', '.join(saved)}" if saved else "")
                    )
                elif kind == "error":
                    self._set_idle()
                    messagebox.showerror("HIL error", str(payload))
        except queue.Empty:
            if plot_changed:
                self._redraw()
        except (OSError, HilError) as exc:
            self._set_idle()
            messagebox.showerror("HIL error", str(exc))
        finally:
            if self._closing and self._worker is None:
                self.root.destroy()
            else:
                self.root.after(100, self._poll_messages)

    def _set_idle(self) -> None:
        self.run_button.configure(state="normal")
        self.detect_button.configure(state="normal")
        self.stop_button.configure(state="disabled")
        self._worker = None


def main() -> None:
    root = tk.Tk()
    HilApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()