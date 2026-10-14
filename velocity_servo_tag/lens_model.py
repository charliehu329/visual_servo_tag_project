"""Dependency-free camera-0 zoom/focus/intrinsics surface model."""

from __future__ import annotations

import bisect
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence


@dataclass(frozen=True)
class ControlPrediction:
    zoom_steps: float
    distance_m: float
    focus_steps: float
    fx: float
    fy: float


@dataclass(frozen=True)
class CameraCalibration:
    zoom_steps: float
    distance_m: float
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: tuple[float, float, float, float, float]


@dataclass(frozen=True)
class ZoomSolution:
    z_from_fx: float
    z_from_fy: float
    zoom_steps: float
    disagreement_steps: float
    control: ControlPrediction


def _pchip_slopes(x: Sequence[float], y: Sequence[float]) -> tuple[float, ...]:
    """Fritsch-Carlson slopes used by shape-preserving cubic interpolation."""
    n = len(x)
    if n == 2:
        slope = (y[1] - y[0]) / (x[1] - x[0])
        return (slope, slope)
    h = [x[i + 1] - x[i] for i in range(n - 1)]
    delta = [(y[i + 1] - y[i]) / h[i] for i in range(n - 1)]
    slopes = [0.0] * n
    for i in range(1, n - 1):
        if delta[i - 1] * delta[i] > 0.0:
            w1 = 2.0 * h[i] + h[i - 1]
            w2 = h[i] + 2.0 * h[i - 1]
            slopes[i] = (w1 + w2) / (
                w1 / delta[i - 1] + w2 / delta[i]
            )

    def endpoint(h0, h1, d0, d1):
        value = ((2.0 * h0 + h1) * d0 - h0 * d1) / (h0 + h1)
        if value * d0 <= 0.0:
            return 0.0
        if d0 * d1 < 0.0 and abs(value) > 3.0 * abs(d0):
            return 3.0 * d0
        return value

    slopes[0] = endpoint(h[0], h[1], delta[0], delta[1])
    slopes[-1] = endpoint(h[-1], h[-2], delta[-1], delta[-2])
    return tuple(slopes)


def _pchip(x: Sequence[float], y: Sequence[float], query: float) -> float:
    if not x[0] <= query <= x[-1]:
        raise ValueError(f"query {query:g} outside [{x[0]:g}, {x[-1]:g}]")
    index = len(x) - 2 if query == x[-1] else bisect.bisect_right(x, query) - 1
    slopes = _pchip_slopes(x, y)
    h = x[index + 1] - x[index]
    t = (query - x[index]) / h
    return (
        (2*t**3 - 3*t**2 + 1) * y[index]
        + (t**3 - 2*t**2 + t) * h * slopes[index]
        + (-2*t**3 + 3*t**2) * y[index + 1]
        + (t**3 - t**2) * h * slopes[index + 1]
    )


class ZoomLensModel:
    """PCHIP over Z followed by piecewise-linear interpolation over distance."""

    def __init__(self, calibration: Mapping) -> None:
        self.model_id = str(calibration["model_id"])
        self.resolution = tuple(int(v) for v in calibration["resolution"])
        self.distances = tuple(float(v) for v in calibration["distance_anchors_m"])
        self.zooms = tuple(float(v) for v in calibration["zoom_anchors"])
        self.control_grids = self._read_grids(calibration["control_grids"])
        self.camera_grids = self._read_grids(calibration["camera_info_grids"])
        if len(self.distances) < 2 or len(self.zooms) < 2:
            raise ValueError("calibration needs at least two distance and zoom anchors")
        if any(b <= a for a, b in zip(self.distances, self.distances[1:])):
            raise ValueError("distance anchors must be strictly increasing")
        if any(b <= a for a, b in zip(self.zooms, self.zooms[1:])):
            raise ValueError("zoom anchors must be strictly increasing")

    @classmethod
    def from_json(cls, path: str | Path) -> "ZoomLensModel":
        with Path(path).open("r", encoding="utf-8") as stream:
            return cls(json.load(stream))

    def _read_grids(self, raw: Mapping) -> dict[str, tuple[tuple[float, ...], ...]]:
        grids = {}
        for name, rows in raw.items():
            grid = tuple(tuple(float(value) for value in row) for row in rows)
            if len(grid) != len(self.distances) or any(
                len(row) != len(self.zooms) for row in grid
            ):
                raise ValueError(f"invalid grid shape for {name}")
            grids[str(name)] = grid
        return grids

    @property
    def zoom_range(self) -> tuple[float, float]:
        return self.zooms[0], self.zooms[-1]

    @property
    def distance_range(self) -> tuple[float, float]:
        return self.distances[0], self.distances[-1]

    def _check_domain(self, zoom: float, distance: float) -> None:
        if not self.zooms[0] <= zoom <= self.zooms[-1]:
            raise ValueError(
                f"zoom {zoom:g} outside calibrated range "
                f"[{self.zooms[0]:g}, {self.zooms[-1]:g}]"
            )
        if not self.distances[0] <= distance <= self.distances[-1]:
            raise ValueError(
                f"distance {distance:g} outside calibrated range "
                f"[{self.distances[0]:g}, {self.distances[-1]:g}] m"
            )

    def _surface(self, grid, zoom: float, distance: float) -> float:
        self._check_domain(zoom, distance)
        values = [_pchip(self.zooms, row, zoom) for row in grid]
        if distance == self.distances[-1]:
            index = len(self.distances) - 2
        else:
            index = bisect.bisect_right(self.distances, distance) - 1
        weight = (
            (distance - self.distances[index])
            / (self.distances[index + 1] - self.distances[index])
        )
        return values[index] + weight * (values[index + 1] - values[index])

    def predict_control(self, zoom_steps: float, distance_m: float) -> ControlPrediction:
        zoom, distance = float(zoom_steps), float(distance_m)
        return ControlPrediction(
            zoom_steps=zoom,
            distance_m=distance,
            focus_steps=self._surface(self.control_grids["focus_steps"], zoom, distance),
            fx=self._surface(self.control_grids["fx_px"], zoom, distance),
            fy=self._surface(self.control_grids["fy_px"], zoom, distance),
        )

    def predict_camera(self, zoom_steps: float, distance_m: float) -> CameraCalibration:
        zoom, distance = float(zoom_steps), float(distance_m)
        value = lambda name: self._surface(self.camera_grids[name], zoom, distance)
        return CameraCalibration(
            zoom_steps=zoom,
            distance_m=distance,
            fx=value("fx_px"),
            fy=value("fy_px"),
            cx=value("cx_px"),
            cy=value("cy_px"),
            distortion=(value("k1"), value("k2"), value("p1"), value("p2"), value("k3")),
        )

    def _inverse(self, target: float, distance: float, field: str) -> float:
        getter = lambda z: getattr(self.predict_control(z, distance), field)
        low, high = self.zooms[0], self.zooms[-1]
        minimum, maximum = getter(low), getter(high)
        if not minimum <= target <= maximum:
            raise ValueError(
                f"target {field}={target:.3f} outside calibrated range "
                f"[{minimum:.3f}, {maximum:.3f}] px at d={distance:.3f} m"
            )
        for _ in range(50):
            middle = 0.5 * (low + high)
            if getter(middle) < target:
                low = middle
            else:
                high = middle
        return 0.5 * (low + high)

    def solve_fx_fy(
        self,
        fx_target: float,
        fy_target: float,
        distance_m: float,
        max_disagreement_steps: float,
    ) -> ZoomSolution:
        distance = float(distance_m)
        self._check_domain(self.zooms[0], distance)
        z_fx = self._inverse(float(fx_target), distance, "fx")
        z_fy = self._inverse(float(fy_target), distance, "fy")
        disagreement = abs(z_fx - z_fy)
        if disagreement > max_disagreement_steps:
            raise ValueError(
                f"inconsistent fx/fy target: z_fx={z_fx:.2f}, z_fy={z_fy:.2f}, "
                f"difference={disagreement:.2f} steps"
            )
        zoom = 0.5 * (z_fx + z_fy)
        return ZoomSolution(
            z_from_fx=z_fx,
            z_from_fy=z_fy,
            zoom_steps=zoom,
            disagreement_steps=disagreement,
            control=self.predict_control(zoom, distance),
        )
