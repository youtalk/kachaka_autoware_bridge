# Copyright 2026 Yutaka Kondo
# Licensed under the Apache License, Version 2.0 (the "License").

"""Pure localization-divergence logic (no ROS imports).

Shared by scripts/localization_monitor (publishes the divergence feed) and
scripts/run_endurance (decides when the divergence means RELOCALIZING).
Divergence is measured between the trusted reference (Kachaka SLAM) and the
estimate under test (shadow or real NDT+EKF output).
"""

from __future__ import annotations

import math
from dataclasses import dataclass


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class PoseSample:
    x: float
    y: float
    yaw: float


def divergence(reference: PoseSample, estimate: PoseSample) -> tuple[float, float]:
    """(planar translation error [m], signed yaw error [rad])."""
    trans = math.hypot(estimate.x - reference.x, estimate.y - reference.y)
    return trans, wrap_pi(estimate.yaw - reference.yaw)


@dataclass(frozen=True)
class HealthConfig:
    """Thresholds for declaring the estimate unhealthy. Consecutive-sample
    debounce on BOTH edges: a single noisy divergence sample must not stop the
    robot, and a single lucky sample must not declare recovery."""

    translation_threshold_m: float = 0.5
    yaw_threshold_rad: float = 0.5
    stale_sec: float = 3.0
    unhealthy_consecutive: int = 3
    recovered_consecutive: int = 2


class HealthTracker:
    def __init__(self, config: HealthConfig) -> None:
        self._cfg = config
        self._bad_streak = 0
        self._good_streak = 0
        self._unhealthy = False

    def update(self, translation_m: float, yaw_rad: float, estimate_age_s: float) -> bool:
        cfg = self._cfg
        # A non-finite sample (NaN/inf) means there is no valid measurement --
        # a missing NDT estimate yields NaN divergence. Treat it as bad so it
        # can neither clear an unhealthy state (a false recovery) nor mask a
        # real divergence.
        finite = (
            math.isfinite(translation_m)
            and math.isfinite(yaw_rad)
            and math.isfinite(estimate_age_s)
        )
        bad = not finite or (
            translation_m > cfg.translation_threshold_m
            or abs(yaw_rad) > cfg.yaw_threshold_rad
            or estimate_age_s > cfg.stale_sec
        )
        if bad:
            self._bad_streak += 1
            self._good_streak = 0
        else:
            self._good_streak += 1
            self._bad_streak = 0
        if not self._unhealthy and self._bad_streak >= cfg.unhealthy_consecutive:
            self._unhealthy = True
        elif self._unhealthy and self._good_streak >= cfg.recovered_consecutive:
            self._unhealthy = False
        return self._unhealthy

    @property
    def unhealthy(self) -> bool:
        return self._unhealthy
