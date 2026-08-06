"""Dependency-free one-DOF zoom/intrinsics model."""

from __future__ import annotations

import bisect
from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class Intrinsics:
    zoom_steps: float
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
    predicted: Intrinsics


class ZoomIntrinsicsModel:
    def __init__(
        self,
        zoom_steps: Sequence[float],
        fx: Sequence[float],
        fy: Sequence[float],
        cx: Sequence[float],
        cy: Sequence[float],
        distortion_flat: Sequence[float],
    ) -> None:
        self.z = tuple(float(value) for value in zoom_steps)
        self.fx = tuple(float(value) for value in fx)
        self.fy = tuple(float(value) for value in fy)
        self.cx = tuple(float(value) for value in cx)
        self.cy = tuple(float(value) for value in cy)
        if len(self.z) < 2 or any(
            len(values) != len(self.z)
            for values in (self.fx, self.fy, self.cx, self.cy)
        ):
            raise ValueError("all calibration arrays must have the same length >= 2")
        if any(self.z[i + 1] <= self.z[i] for i in range(len(self.z) - 1)):
            raise ValueError("zoom samples must be strictly increasing")
        if any(self.fx[i + 1] <= self.fx[i] for i in range(len(self.z) - 1)):
            raise ValueError("fx samples must be strictly increasing")
        if any(self.fy[i + 1] <= self.fy[i] for i in range(len(self.z) - 1)):
            raise ValueError("fy samples must be strictly increasing")
        if len(distortion_flat) != 5 * len(self.z):
            raise ValueError("distortion_flat must contain five values per zoom sample")
        self.distortion = tuple(
            tuple(float(distortion_flat[5*i+j]) for j in range(5))
            for i in range(len(self.z))
        )
        self.fx_slopes = self._pchip_slopes(self.fx)
        self.fy_slopes = self._pchip_slopes(self.fy)

    def _pchip_slopes(self, values: Sequence[float]) -> tuple[float, ...]:
        n = len(self.z)
        h = [self.z[i + 1] - self.z[i] for i in range(n - 1)]
        delta = [(values[i + 1] - values[i]) / h[i] for i in range(n - 1)]
        slopes = [0.0] * n
        for i in range(1, n - 1):
            if delta[i - 1] * delta[i] > 0.0:
                w1 = 2.0 * h[i] + h[i - 1]
                w2 = h[i] + 2.0 * h[i - 1]
                slopes[i] = (w1 + w2) / (
                    w1 / delta[i - 1] + w2 / delta[i]
                )
        left = ((2*h[0]+h[1])*delta[0]-h[0]*delta[1])/(h[0]+h[1])
        right = ((2*h[-1]+h[-2])*delta[-1]-h[-1]*delta[-2])/(h[-1]+h[-2])
        slopes[0] = max(0.0, min(left, 3.0 * delta[0]))
        slopes[-1] = max(0.0, min(right, 3.0 * delta[-1]))
        return tuple(slopes)

    def _bounded(self, value: float, limits: tuple[float, float]) -> float:
        if limits[0] <= value <= limits[1]:
            return value
        raise ValueError(
            f"value {value:g} outside calibrated range [{limits[0]:g}, {limits[1]:g}]"
        )

    def _interval(self, zoom: float) -> int:
        if zoom == self.z[-1]:
            return len(self.z) - 2
        return bisect.bisect_right(self.z, zoom) - 1

    def _linear(self, zoom: float, values: Sequence[float]) -> float:
        i = self._interval(zoom)
        t = (zoom - self.z[i]) / (self.z[i + 1] - self.z[i])
        return values[i] + t * (values[i + 1] - values[i])

    def _pchip(
        self, zoom: float, values: Sequence[float], slopes: Sequence[float]
    ) -> float:
        i = self._interval(zoom)
        h = self.z[i + 1] - self.z[i]
        t = (zoom - self.z[i]) / h
        return (
            (2*t**3 - 3*t**2 + 1) * values[i]
            + (t**3 - 2*t**2 + t) * h * slopes[i]
            + (-2*t**3 + 3*t**2) * values[i + 1]
            + (t**3 - t**2) * h * slopes[i + 1]
        )

    def intrinsics(self, zoom_steps: float) -> Intrinsics:
        zoom = self._bounded(float(zoom_steps), (self.z[0], self.z[-1]))
        distortion = tuple(
            self._linear(zoom, tuple(row[j] for row in self.distortion))
            for j in range(5)
        )
        return Intrinsics(
            zoom_steps=zoom,
            fx=self._pchip(zoom, self.fx, self.fx_slopes),
            fy=self._pchip(zoom, self.fy, self.fy_slopes),
            cx=self._linear(zoom, self.cx),
            cy=self._linear(zoom, self.cy),
            distortion=distortion,
        )

    def _inverse(
        self, target: float, values: Sequence[float], slopes: Sequence[float]
    ) -> float:
        target = self._bounded(float(target), (values[0], values[-1]))
        low, high = self.z[0], self.z[-1]
        for _ in range(60):
            middle = 0.5 * (low + high)
            if self._pchip(middle, values, slopes) < target:
                low = middle
            else:
                high = middle
        return 0.5 * (low + high)

    def solve_fx_fy(
        self, fx_target: float, fy_target: float, max_disagreement_steps: float
    ) -> ZoomSolution:
        z_fx = self._inverse(fx_target, self.fx, self.fx_slopes)
        z_fy = self._inverse(fy_target, self.fy, self.fy_slopes)
        disagreement = abs(z_fx - z_fy)
        if disagreement > max_disagreement_steps:
            raise ValueError(
                f"inconsistent fx/fy request: z_fx={z_fx:.3f}, "
                f"z_fy={z_fy:.3f}, difference={disagreement:.3f} steps"
            )
        zoom = 0.5 * (z_fx + z_fy)
        return ZoomSolution(
            z_from_fx=z_fx,
            z_from_fy=z_fy,
            zoom_steps=zoom,
            disagreement_steps=disagreement,
            predicted=self.intrinsics(zoom),
        )
