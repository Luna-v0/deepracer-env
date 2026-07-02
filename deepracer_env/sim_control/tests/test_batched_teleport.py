#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""Tests for the batched multi-entity teleport (B7 §1.1 reset-storm lever)."""
from deepracer_env.sim_control.interface import NullSimControl
from deepracer_env.sim_control.backends.ros_gz_backend import RosGzBackend
from deepracer_env.sim_control.types import EntityState


def test_set_entity_states_default_loops_per_entity():
    """The interface default applies each state via set_entity_state (correct,
    unbatched) — so any backend gets a working batched call for free."""
    sc = NullSimControl()
    ok = sc.set_entity_states([("car0", EntityState()), ("car1", EntityState())])
    assert ok is True
    set_names = [c[1] for c in sc.calls if c[0] == "set_entity_state"]
    assert set_names == ["car0", "car1"]


def test_ros_gz_batches_into_one_pose_v_call():
    """RosGzBackend collapses N teleports into ONE set_pose_vector (gz.msgs.Pose_V)
    call carrying every entity, instead of N per-entity round-trips."""
    be = RosGzBackend.__new__(RosGzBackend)  # bypass gz-connecting __init__
    be._prefix = "/world/test"
    captured = {}

    def fake_service(service, reqtype, reptype, req):
        captured.update(service=service, reqtype=reqtype, reptype=reptype, req=req)
        return "data: true"

    be._service = fake_service
    be._gz_alive = lambda: True

    ok = be.set_entity_states([("car0", EntityState()), ("car1", EntityState())])
    assert ok is True
    assert captured["service"] == "set_pose_vector"
    assert captured["reqtype"] == "gz.msgs.Pose_V"
    # ONE request, TWO named pose entries (a Pose_V), not two requests.
    assert captured["req"].count("name:") == 2
    assert 'name: "car0"' in captured["req"]
    assert 'name: "car1"' in captured["req"]


def test_ros_gz_empty_batch_is_noop():
    be = RosGzBackend.__new__(RosGzBackend)
    be._prefix = "/world/test"
    called = []
    be._service = lambda *a, **k: called.append(a) or "data: true"
    assert be.set_entity_states([]) is True
    assert called == []  # no service call for an empty batch
