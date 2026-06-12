# Copyright 2026 Yutaka Kondo
# Licensed under the Apache License, Version 2.0 (the "License").

import math

import pytest

from kachaka_autoware_bridge.locmon import (
    HealthConfig,
    HealthTracker,
    PoseSample,
    divergence,
)


def test_divergence_translation_and_yaw() -> None:
    ref = PoseSample(x=1.0, y=2.0, yaw=0.0)
    est = PoseSample(x=4.0, y=6.0, yaw=0.5)
    trans, yaw = divergence(ref, est)
    assert trans == pytest.approx(5.0)
    assert yaw == pytest.approx(0.5)


def test_divergence_yaw_wraps() -> None:
    ref = PoseSample(x=0.0, y=0.0, yaw=math.pi - 0.1)
    est = PoseSample(x=0.0, y=0.0, yaw=-math.pi + 0.1)
    _, yaw = divergence(ref, est)
    assert yaw == pytest.approx(0.2, abs=1e-9)


def test_tracker_debounces_single_bad_sample() -> None:
    tr = HealthTracker(HealthConfig(unhealthy_consecutive=3))
    assert tr.update(translation_m=10.0, yaw_rad=0.0, estimate_age_s=0.0) is False
    assert tr.update(translation_m=0.0, yaw_rad=0.0, estimate_age_s=0.0) is False
    assert tr.unhealthy is False


def test_tracker_trips_after_consecutive_bad_samples() -> None:
    tr = HealthTracker(HealthConfig(translation_threshold_m=0.5, unhealthy_consecutive=3))
    for _ in range(2):
        assert tr.update(translation_m=1.0, yaw_rad=0.0, estimate_age_s=0.0) is False
    assert tr.update(translation_m=1.0, yaw_rad=0.0, estimate_age_s=0.0) is True
    assert tr.unhealthy is True


def test_tracker_recovers_after_consecutive_good_samples() -> None:
    cfg = HealthConfig(unhealthy_consecutive=1, recovered_consecutive=2)
    tr = HealthTracker(cfg)
    assert tr.update(translation_m=99.0, yaw_rad=0.0, estimate_age_s=0.0) is True
    assert tr.update(translation_m=0.0, yaw_rad=0.0, estimate_age_s=0.0) is True  # 1 good
    assert tr.update(translation_m=0.0, yaw_rad=0.0, estimate_age_s=0.0) is False  # 2 good


def test_tracker_yaw_and_staleness_also_trip() -> None:
    cfg = HealthConfig(yaw_threshold_rad=0.5, stale_sec=2.0, unhealthy_consecutive=1)
    assert HealthTracker(cfg).update(0.0, yaw_rad=1.0, estimate_age_s=0.0) is True
    assert HealthTracker(cfg).update(0.0, yaw_rad=-1.0, estimate_age_s=0.0) is True
    assert HealthTracker(cfg).update(0.0, yaw_rad=0.0, estimate_age_s=5.0) is True


def test_config_defaults() -> None:
    cfg = HealthConfig()
    assert cfg.translation_threshold_m == pytest.approx(0.5)
    assert cfg.yaw_threshold_rad == pytest.approx(0.5)
    assert cfg.unhealthy_consecutive >= 2  # never trip on one noisy sample


def test_tracker_treats_nan_as_bad() -> None:
    # A NaN divergence (a missing NDT estimate) must count as bad, and must
    # never clear an unhealthy state (no false recovery on missing data).
    cfg = HealthConfig(unhealthy_consecutive=1, recovered_consecutive=2)
    tr = HealthTracker(cfg)
    assert tr.update(float("nan"), 0.0, 0.0) is True
    assert tr.update(float("nan"), 0.0, 0.0) is True
    assert tr.update(float("nan"), 0.0, 0.0) is True


def test_tracker_good_interrupt_resets_bad_streak() -> None:
    # A single good sample mid-bad-streak resets the streak: the trip edge
    # debounce must re-accumulate from zero.
    tr = HealthTracker(HealthConfig(translation_threshold_m=0.5, unhealthy_consecutive=3))
    tr.update(1.0, 0.0, 0.0)  # bad[1]
    tr.update(1.0, 0.0, 0.0)  # bad[2]
    tr.update(0.0, 0.0, 0.0)  # good -> resets bad streak to 0
    assert tr.update(1.0, 0.0, 0.0) is False  # bad[1] again, NOT bad[3]
    assert tr.update(1.0, 0.0, 0.0) is False  # bad[2]
    assert tr.update(1.0, 0.0, 0.0) is True   # bad[3], trips
