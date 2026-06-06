# Copyright 2026 Yutaka Kondo
# Licensed under the Apache License, Version 2.0 (the "License").

import json

from kachaka_autoware_bridge.endurance import (
    Action,
    Observation,
    ScenarioConfig,
    ScenarioScheduler,
    ScenarioStep,
    SessionRecord,
    State,
    StepMode,
    StopReason,
    decide_transition,
)


def test_enums_have_expected_members() -> None:
    assert {s.value for s in State} == {
        "init", "onto_ring", "running", "recovering", "stopping", "fault", "done",
    }
    assert {r.value for r in StopReason} == {"none", "battery", "duration", "operator", "fault"}
    assert {m.value for m in StepMode} == {"cruise", "dwell", "arrive"}
    assert {a.value for a in Action} == {
        "drive_onto_ring", "start_running", "reengage", "safe_stop", "clear_route",
        "return_home", "capture_diagnostics", "write_fault_report", "write_session_record",
    }


def test_scenario_config_defaults() -> None:
    cfg = ScenarioConfig()
    assert cfg.speeds_mps == (0.1, 0.2, 0.3)
    assert cfg.arrive_every_n_steps == 8
    assert cfg.dwell_long_sec <= 10.0  # must stay under Kachaka's idle-return threshold


def test_scenario_step_is_frozen() -> None:
    step = ScenarioStep(StepMode.CRUISE, 0.2, 1.0)
    assert step.mode is StepMode.CRUISE
    import dataclasses
    import pytest
    with pytest.raises(dataclasses.FrozenInstanceError):
        step.target_speed_mps = 0.3  # type: ignore[misc]


def test_scheduler_is_deterministic_for_a_seed() -> None:
    a = ScenarioScheduler(ScenarioConfig(), seed=42)
    b = ScenarioScheduler(ScenarioConfig(), seed=42)
    seq_a = [a.next_step() for _ in range(50)]
    seq_b = [b.next_step() for _ in range(50)]
    assert seq_a == seq_b


def test_scheduler_emits_arrive_on_the_cadence() -> None:
    cfg = ScenarioConfig(arrive_every_n_steps=8)
    s = ScenarioScheduler(cfg, seed=1)
    steps = [s.next_step() for _ in range(24)]
    for i, step in enumerate(steps, start=1):
        if i % 8 == 0:
            assert step.mode is StepMode.ARRIVE
            assert step.extent == cfg.arrive_fraction


def test_scheduler_values_stay_in_bounds() -> None:
    cfg = ScenarioConfig()
    s = ScenarioScheduler(cfg, seed=7)
    for step in (s.next_step() for _ in range(200)):
        assert step.target_speed_mps in cfg.speeds_mps or step.mode is StepMode.DWELL
        if step.mode is StepMode.CRUISE:
            assert step.extent in cfg.cruise_arc_fractions
            assert step.target_speed_mps in cfg.speeds_mps
        elif step.mode is StepMode.DWELL:
            assert step.extent in (cfg.dwell_short_sec, cfg.dwell_long_sec)
            assert step.extent <= 10.0
            assert step.target_speed_mps == 0.0
        elif step.mode is StepMode.ARRIVE:
            assert step.extent == cfg.arrive_fraction
            assert step.target_speed_mps in cfg.speeds_mps


def test_scheduler_produces_a_mix() -> None:
    s = ScenarioScheduler(ScenarioConfig(), seed=3)
    modes = {s.next_step().mode for _ in range(200)}
    assert modes == {StepMode.CRUISE, StepMode.DWELL, StepMode.ARRIVE}


def test_init_waits_then_advances_to_onto_ring() -> None:
    assert decide_transition(State.INIT, Observation()).next_state is State.INIT
    t = decide_transition(State.INIT, Observation(bringup_ready=True))
    assert t.next_state is State.ONTO_RING
    assert Action.DRIVE_ONTO_RING in t.actions


def test_init_timeout_faults() -> None:
    t = decide_transition(State.INIT, Observation(bringup_timed_out=True))
    assert t.next_state is State.FAULT
    assert t.stop_reason is StopReason.FAULT


def test_onto_ring_success_and_failure() -> None:
    t_ok = decide_transition(State.ONTO_RING, Observation(on_ring=True))
    assert t_ok.next_state is State.RUNNING
    assert Action.START_RUNNING in t_ok.actions
    # A failed drive-on with retries remaining is transient: stay in ONTO_RING so
    # the shell re-attempts with backoff (one MOVE_TO_POSE abort must not kill the run).
    t_retry = decide_transition(State.ONTO_RING, Observation(onto_ring_failed=True))
    assert t_retry.next_state is State.ONTO_RING
    assert t_retry.actions == ()
    # Only once the bounded retries are exhausted does it become a hard fault.
    t_bad = decide_transition(
        State.ONTO_RING, Observation(onto_ring_failed=True, onto_ring_exhausted=True)
    )
    assert t_bad.next_state is State.FAULT
    assert t_bad.stop_reason is StopReason.FAULT
    # Exhausted but not (yet) failed on the latest attempt -> keep trying, no fault.
    t_exh_only = decide_transition(State.ONTO_RING, Observation(onto_ring_exhausted=True))
    assert t_exh_only.next_state is State.ONTO_RING


def test_running_stop_request_goes_to_stopping_with_reason() -> None:
    t = decide_transition(State.RUNNING, Observation(stop_request=StopReason.BATTERY))
    assert t.next_state is State.STOPPING
    assert t.stop_reason is StopReason.BATTERY
    assert Action.SAFE_STOP in t.actions
    assert Action.WRITE_SESSION_RECORD in t.actions


def test_running_transient_fault_goes_to_recovering() -> None:
    t = decide_transition(State.RUNNING, Observation(transient_fault=True))
    assert t.next_state is State.RECOVERING


def test_running_off_ring_recenters_via_onto_ring() -> None:
    # Drifting beyond the radial trigger during RUNNING re-centers by re-entering
    # the (already tested) ONTO_RING path; ONTO_RING -> RUNNING on on_ring then
    # resumes cruising, so the accumulating outward drift is bounded.
    t = decide_transition(State.RUNNING, Observation(off_ring=True))
    assert t.next_state is State.ONTO_RING
    assert Action.DRIVE_ONTO_RING in t.actions


def test_off_ring_is_the_lowest_priority_running_corrective() -> None:
    # Re-centering must not preempt a hard fault, an operator/duration stop, or a
    # transient mode-drop recovery (hard_fault > stop > transient > re-center).
    assert (
        decide_transition(
            State.RUNNING, Observation(off_ring=True, hard_fault=True)
        ).next_state
        is State.FAULT
    )
    assert (
        decide_transition(
            State.RUNNING, Observation(off_ring=True, stop_request=StopReason.OPERATOR)
        ).next_state
        is State.STOPPING
    )
    assert (
        decide_transition(
            State.RUNNING, Observation(off_ring=True, transient_fault=True)
        ).next_state
        is State.RECOVERING
    )


def test_hard_fault_preempts_from_any_running_state() -> None:
    for st in (State.ONTO_RING, State.RUNNING, State.RECOVERING):
        t = decide_transition(st, Observation(hard_fault=True))
        assert t.next_state is State.FAULT
        assert Action.CAPTURE_DIAGNOSTICS in t.actions
        assert Action.WRITE_FAULT_REPORT in t.actions


def test_hard_fault_preempts_a_concurrent_stop_request() -> None:
    t = decide_transition(
        State.RUNNING, Observation(hard_fault=True, stop_request=StopReason.DURATION)
    )
    assert t.next_state is State.FAULT


def test_recovering_recovers_or_exhausts() -> None:
    t_ok = decide_transition(State.RECOVERING, Observation(recovered=True))
    assert t_ok.next_state is State.RUNNING
    assert Action.REENGAGE in t_ok.actions
    t_bad = decide_transition(State.RECOVERING, Observation(retry_exhausted=True))
    assert t_bad.next_state is State.FAULT


def test_terminal_states_go_to_done() -> None:
    assert decide_transition(State.STOPPING, Observation()).next_state is State.DONE
    assert decide_transition(State.FAULT, Observation()).next_state is State.DONE
    assert decide_transition(State.DONE, Observation()).next_state is State.DONE


def test_running_idle_stays_running() -> None:
    t = decide_transition(State.RUNNING, Observation())
    assert t.next_state is State.RUNNING
    assert t.actions == ()


def test_onto_ring_idle_stays_onto_ring() -> None:
    t = decide_transition(State.ONTO_RING, Observation())
    assert t.next_state is State.ONTO_RING


def test_recovering_idle_stays_recovering() -> None:
    t = decide_transition(State.RECOVERING, Observation())
    assert t.next_state is State.RECOVERING


def test_session_record_serializes_to_json() -> None:
    rec = SessionRecord(
        start_iso="2026-06-05T10:00:00",
        end_iso="2026-06-05T10:45:00",
        stop_reason=StopReason.DURATION,
        lap_count=12.5,
        step_histogram={"cruise": 30, "dwell": 10, "arrive": 5},
        seed=42,
    )
    data = json.loads(rec.to_json())
    assert data["stop_reason"] == "duration"   # enum serialized as its value
    assert data["lap_count"] == 12.5
    assert data["step_histogram"]["cruise"] == 30
    assert data["seed"] == 42
    assert data["fault_info"] is None
    assert data["recenter_count"] == 0   # defaults to 0 when no re-centering happened


def test_session_record_serializes_recenter_count() -> None:
    rec = SessionRecord(
        start_iso="t0", end_iso="t1", stop_reason=StopReason.DURATION,
        lap_count=5.0, step_histogram={}, seed=1, recenter_count=7,
    )
    assert json.loads(rec.to_json())["recenter_count"] == 7


def test_session_record_carries_fault_info() -> None:
    rec = SessionRecord(
        start_iso="t0", end_iso="t1", stop_reason=StopReason.FAULT,
        lap_count=3.0, step_histogram={}, seed=1, fault_info="mission_planner died",
    )
    assert json.loads(rec.to_json())["fault_info"] == "mission_planner died"


def test_scenario_config_rejects_zero_arrive_cadence() -> None:
    import pytest
    with pytest.raises(ValueError):
        ScenarioConfig(arrive_every_n_steps=0)
