# Copyright 2026 Yutaka Kondo
# Licensed under the Apache License, Version 2.0 (the "License").

"""Pure-logic core of the endurance orchestrator (no ROS imports).

The rclpy shell in scripts/run_endurance builds an Observation each tick, calls
decide_transition to advance the top-level state machine, and executes the
returned Actions; within RUNNING it pulls ScenarioSteps from ScenarioScheduler
and uses the carrot/lap geometry in loop_route.py. Keeping all decisions here
makes them deterministic and unit-testable without a robot.
"""

from __future__ import annotations

import json
import random
from dataclasses import asdict, dataclass
from enum import Enum


class State(Enum):
    INIT = "init"
    ONTO_RING = "onto_ring"
    RUNNING = "running"
    RECOVERING = "recovering"
    STOPPING = "stopping"
    FAULT = "fault"
    DONE = "done"


class StopReason(Enum):
    NONE = "none"
    BATTERY = "battery"
    DURATION = "duration"
    OPERATOR = "operator"
    FAULT = "fault"


class StepMode(Enum):
    CRUISE = "cruise"
    DWELL = "dwell"
    ARRIVE = "arrive"


class Action(Enum):
    DRIVE_ONTO_RING = "drive_onto_ring"
    START_RUNNING = "start_running"
    REENGAGE = "reengage"
    SAFE_STOP = "safe_stop"
    CLEAR_ROUTE = "clear_route"
    RETURN_HOME = "return_home"
    CAPTURE_DIAGNOSTICS = "capture_diagnostics"
    WRITE_FAULT_REPORT = "write_fault_report"
    WRITE_SESSION_RECORD = "write_session_record"


@dataclass(frozen=True)
class ScenarioConfig:
    """Bounds + menu for the scenario scheduler. dwell_long_sec must stay below
    Kachaka's ~15 s idle-return threshold."""

    speeds_mps: tuple[float, ...] = (0.1, 0.2, 0.3)
    cruise_arc_fractions: tuple[float, ...] = (0.5, 1.0, 2.0)
    dwell_short_sec: float = 3.0
    dwell_long_sec: float = 8.0
    dwell_weights: tuple[float, float, float] = (0.6, 0.3, 0.1)  # none, short, long
    arrive_every_n_steps: int = 8
    arrive_fraction: float = 0.5

    def __post_init__(self) -> None:
        if self.arrive_every_n_steps < 1:
            raise ValueError(
                f"arrive_every_n_steps must be >= 1, got {self.arrive_every_n_steps}"
            )


@dataclass(frozen=True)
class ScenarioStep:
    """One scheduled step. ``extent`` is a lap fraction for CRUISE/ARRIVE and a
    duration in seconds for DWELL."""

    mode: StepMode
    target_speed_mps: float
    extent: float


class ScenarioScheduler:
    """Deterministic, bounded scenario generator. Same (config, seed) -> same
    step sequence, so endurance runs are reproducible (the seed is recorded in
    the SessionRecord) while still injecting variety. Every
    ``arrive_every_n_steps``-th step is an ARRIVE (to cover Autoware's
    ARRIVED -> goal-stop path); other steps are a CRUISE or a DWELL chosen by
    ``dwell_weights``.
    """

    def __init__(self, config: ScenarioConfig, seed: int) -> None:
        self._cfg = config
        self._rng = random.Random(seed)
        self._index = 0

    def next_step(self) -> ScenarioStep:
        cfg = self._cfg
        self._index += 1
        if self._index % cfg.arrive_every_n_steps == 0:
            speed = self._rng.choice(cfg.speeds_mps)
            return ScenarioStep(StepMode.ARRIVE, speed, cfg.arrive_fraction)
        category = self._rng.choices(
            ("none", "short", "long"), weights=cfg.dwell_weights, k=1
        )[0]
        if category == "none":
            speed = self._rng.choice(cfg.speeds_mps)
            arc = self._rng.choice(cfg.cruise_arc_fractions)
            return ScenarioStep(StepMode.CRUISE, speed, arc)
        dwell = cfg.dwell_short_sec if category == "short" else cfg.dwell_long_sec
        return ScenarioStep(StepMode.DWELL, 0.0, dwell)


@dataclass(frozen=True)
class Observation:
    """Snapshot of the inputs the state machine reacts to, built by the shell
    from subscriptions/timers each tick."""

    bringup_ready: bool = False
    bringup_timed_out: bool = False
    on_ring: bool = False
    off_ring: bool = False
    onto_ring_failed: bool = False
    onto_ring_exhausted: bool = False
    hard_fault: bool = False
    transient_fault: bool = False
    recovered: bool = False
    retry_exhausted: bool = False
    stop_request: StopReason = StopReason.NONE


@dataclass(frozen=True)
class Transition:
    next_state: State
    actions: tuple[Action, ...] = ()
    stop_reason: StopReason = StopReason.NONE


_FAULT_ACTIONS = (Action.SAFE_STOP, Action.CAPTURE_DIAGNOSTICS, Action.WRITE_FAULT_REPORT)
_STOP_ACTIONS = (
    Action.SAFE_STOP,
    Action.RETURN_HOME,
    Action.CLEAR_ROUTE,
    Action.WRITE_SESSION_RECORD,
)


def decide_transition(state: State, obs: Observation) -> Transition:
    """Pure top-level state-machine step: (state, observation) -> transition.

    Priority inside the running states is hard_fault > stop_request >
    transient_fault. A hard fault always halts-and-captures and never restarts a
    node (preserving the failure and the memory baseline for analysis).
    """
    running_states = (State.ONTO_RING, State.RUNNING, State.RECOVERING)
    if state in running_states and obs.hard_fault:
        return Transition(State.FAULT, _FAULT_ACTIONS, StopReason.FAULT)

    # An operator/SP4/duration stop is honored from ANY non-terminal state, not
    # just RUNNING, so a stop requested during bring-up, drive-on, or recovery
    # still halts cleanly and writes a session record. A hard fault (above) still
    # preempts it; transient/off-ring handling (below) is lower priority.
    non_terminal = (State.INIT, State.ONTO_RING, State.RUNNING, State.RECOVERING)
    if state in non_terminal and obs.stop_request is not StopReason.NONE:
        return Transition(State.STOPPING, _STOP_ACTIONS, obs.stop_request)

    if state is State.INIT:
        if obs.bringup_timed_out:
            return Transition(
                State.FAULT,
                (Action.CAPTURE_DIAGNOSTICS, Action.WRITE_FAULT_REPORT),
                StopReason.FAULT,
            )
        if obs.bringup_ready:
            return Transition(State.ONTO_RING, (Action.DRIVE_ONTO_RING,))
        return Transition(State.INIT)

    if state is State.ONTO_RING:
        # Onto-ring is a TRANSIENT tier: a Kachaka MOVE_TO_POSE abort (conservative
        # near walls / a transient nav bad-state) must not instantly kill an
        # unattended run. The shell retries the drive-on with backoff; only after
        # the bounded retries are exhausted does a still-failing onto-ring become a
        # hard fault. A bare onto_ring_failed (retries remaining) stays in ONTO_RING
        # so the shell re-attempts.
        if obs.onto_ring_failed and obs.onto_ring_exhausted:
            return Transition(State.FAULT, _FAULT_ACTIONS, StopReason.FAULT)
        if obs.on_ring:
            return Transition(State.RUNNING, (Action.START_RUNNING,))
        return Transition(State.ONTO_RING)

    if state is State.RUNNING:
        if obs.transient_fault:
            return Transition(State.RECOVERING)
        # Re-centering: the robot has drifted off the loop centerline (the sub-1 m
        # loop's outward control-tracking drift). Re-enter the tested ONTO_RING
        # path to reposition onto the centerline, bounding the accumulating drift.
        # Lowest-priority RUNNING corrective: a hard fault (handled above), an
        # operator/duration stop, and a transient mode-drop all preempt it.
        if obs.off_ring:
            return Transition(State.ONTO_RING, (Action.DRIVE_ONTO_RING,))
        return Transition(State.RUNNING)

    if state is State.RECOVERING:
        if obs.retry_exhausted:
            return Transition(State.FAULT, _FAULT_ACTIONS, StopReason.FAULT)
        if obs.recovered:
            return Transition(State.RUNNING, (Action.REENGAGE,))
        return Transition(State.RECOVERING)

    # STOPPING, FAULT, DONE are terminal w.r.t. the machine: settle to DONE.
    return Transition(State.DONE)


def exit_stop_reason(shutdown_via_signal: bool) -> StopReason:
    """StopReason to stamp on the SessionRecord the shell writes from its finally
    block when a run ends OUTSIDE the FSM's STOPPING/FAULT path -- i.e. a SIGINT/
    SIGTERM (or an unexpected exception) broke the tick loop before a terminal
    transition could run WRITE_SESSION_RECORD. A signal is an operator-driven
    stop; anything else is treated as a fault so the unexpected exit is still
    analysable. Keeping the rule here (pure, unit-testable) leaves the shell to
    only do the I/O (best-effort safe stop, write the record)."""
    return StopReason.OPERATOR if shutdown_via_signal else StopReason.FAULT


@dataclass
class SessionRecord:
    """Minimal self-contained ledger written by the orchestrator on exit. SP5
    later augments runs with memory/perf sampling and rich reports; this record
    is always written so an SP3-only run is still analysable."""

    start_iso: str
    end_iso: str
    stop_reason: StopReason
    lap_count: float
    step_histogram: dict[str, int]
    seed: int
    fault_info: str | None = None
    recenter_count: int = 0   # mid-RUNNING drift re-centerings; recenters/lap = loop-tightness health
    stop_count: int = 0  # planned/observed halts during RUNNING (e.g. stop lines)

    def to_json(self) -> str:
        def _default(obj: object) -> object:
            if isinstance(obj, Enum):
                return obj.value
            raise TypeError(f"not JSON-serializable: {type(obj)}")
        return json.dumps(asdict(self), indent=2, sort_keys=True, default=_default)
