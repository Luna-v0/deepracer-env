#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""Host-runnable tests for the ROS-free core of the simulator seam.

These exercise the parts that must be correct independent of any simulator:
the value-type math, the multi-arena geometry/decoupling, the action→joint
mapping (which trained policies depend on), the gz ``pose/info`` parser, and the
Null backend. They import no ``rclpy`` and run on a plain Python install.
"""
import math

import pytest

from deepracer_env.sim_control import (
    Arena,
    ArenaLayout,
    ColorRGBA,
    EntityState,
    NullSimControl,
    Pose,
    Quaternion,
    Twist,
    Vec3,
)
from deepracer_env.agent_ctrl.drive import (
    action_to_joint_commands,
    get_wheel_radius,
    zero_commands,
    WHEEL_JOINTS,
    STEERING_JOINTS,
)


# --------------------------------------------------------------------------- #
# value types
# --------------------------------------------------------------------------- #

def test_vec3_arithmetic():
    assert Vec3(1, 2, 3) + Vec3(10, 20, 30) == Vec3(11, 22, 33)
    assert Vec3(11, 22, 33) - Vec3(1, 2, 3) == Vec3(10, 20, 30)
    assert Vec3(1, 2, 3).as_tuple() == (1, 2, 3)


@pytest.mark.parametrize("yaw", [0.0, 0.5, 1.5707963, 3.0, -2.0])
def test_quaternion_yaw_roundtrip(yaw):
    q = Quaternion.from_yaw(yaw)
    assert math.isclose(q.yaw, yaw, abs_tol=1e-9)
    # unit norm
    n = q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2
    assert math.isclose(n, 1.0, abs_tol=1e-9)


def test_pose_at():
    p = Pose.at(3.0, 4.0, yaw=math.pi / 2)
    assert p.position == Vec3(3.0, 4.0, 0.0)
    assert math.isclose(p.orientation.yaw, math.pi / 2, abs_tol=1e-9)


# --------------------------------------------------------------------------- #
# multi-arena layout (the decoupling core)
# --------------------------------------------------------------------------- #

def test_grid_offsets_arena0_at_origin_and_compact():
    offs = ArenaLayout.grid_offsets(5, 300.0)
    assert offs[0] == (0.0, 0.0)
    assert len(offs) == 5
    # 5 arenas -> ceil(sqrt(5))=3 columns
    assert offs[3] == (0.0, 300.0)   # row 1, col 0
    assert offs[4] == (300.0, 300.0)


def test_single_track_broadcasts_to_all_arenas():
    layout = ArenaLayout(4, ["reinvent_base"])
    assert len(layout) == 4
    assert [a.track_name for a in layout] == ["reinvent_base"] * 4
    # but each arena is still decoupled: distinct car, entity, origin, seed
    assert [a.car_name for a in layout] == ["car_0", "car_1", "car_2", "car_3"]
    assert [a.track_entity_name for a in layout] == [
        "racetrack_0", "racetrack_1", "racetrack_2", "racetrack_3"]
    assert len({a.dr_seed for a in layout}) == 4


def test_different_track_per_arena():
    layout = ArenaLayout(3, ["a", "b", "c"], base_seed=100)
    assert [a.track_name for a in layout] == ["a", "b", "c"]
    assert [a.dr_seed for a in layout] == [100, 101, 102]


def test_track_count_mismatch_raises():
    with pytest.raises(ValueError):
        ArenaLayout(3, ["a", "b"])  # 2 != 3 and != 1
    with pytest.raises(ValueError):
        ArenaLayout(0, ["a"])


def test_local_world_roundtrip_subtracts_origin():
    layout = ArenaLayout(2, ["t"], spacing=300.0)
    arena = layout.get(1)
    assert arena.origin == Vec3(300.0, 0.0, 0.0)
    world_pose = Pose.at(305.0, 2.0, yaw=0.3)
    local = layout.to_local(arena, world_pose)
    # local frame: arena origin subtracted, heading preserved
    assert math.isclose(local.position.x, 5.0)
    assert math.isclose(local.position.y, 2.0)
    assert math.isclose(local.orientation.yaw, 0.3, abs_tol=1e-9)
    # round trips back to world
    back = layout.to_world(arena, local)
    assert math.isclose(back.position.x, 305.0)
    assert math.isclose(back.position.y, 2.0)


# --------------------------------------------------------------------------- #
# action -> joint command mapping (policy-critical arithmetic)
# --------------------------------------------------------------------------- #

def test_wheel_radius_version_locked():
    assert get_wheel_radius(5.0) == 0.035
    assert get_wheel_radius(3.0) == 0.035
    assert get_wheel_radius(2.0) == 0.0277
    assert get_wheel_radius(1.0) == 0.1


def test_action_mapping_exact_math():
    # 30 deg, 4 m/s, v5 (radius 0.035)
    wheels, steer = action_to_joint_commands(30.0, 4.0, simapp_version=5.0)
    assert len(wheels) == len(WHEEL_JOINTS) == 4
    assert len(steer) == len(STEERING_JOINTS) == 2
    # same value to all four wheels / both hinges (no Ackermann decomposition)
    assert wheels == [pytest.approx(4.0 / 0.035)] * 4
    assert steer == [pytest.approx(math.radians(30.0))] * 2


def test_action_mapping_clamps_before_conversion():
    # over-range inputs clamp to [-30,30] deg and [0.1,4.0] m/s
    wheels, steer = action_to_joint_commands(90.0, 99.0, simapp_version=5.0)
    assert steer[0] == pytest.approx(math.radians(30.0))
    assert wheels[0] == pytest.approx(4.0 / 0.035)
    w2, s2 = action_to_joint_commands(-90.0, 0.0, simapp_version=5.0)
    assert s2[0] == pytest.approx(math.radians(-30.0))
    assert w2[0] == pytest.approx(0.1 / 0.035)  # speed clamped up to MIN_SPEED


def test_zero_commands():
    wheels, steer = zero_commands()
    assert wheels == [0.0, 0.0, 0.0, 0.0]
    assert steer == [0.0, 0.0]


# --------------------------------------------------------------------------- #
# gz pose/info parser
# --------------------------------------------------------------------------- #

POSE_V_DUMP = """\
header {
  stamp {
    sec: 12
  }
}
pose {
  name: "ground_plane"
  id: 8
  position {
    x: 0
    y: 0
    z: 0
  }
  orientation {
    x: 0
    y: 0
    z: 0
    w: 1
  }
}
pose {
  name: "car_0"
  id: 10
  position {
    x: 5.5
    y: -2.25
    z: 0.05
  }
  orientation {
    x: 0
    y: 0
    z: 0.70710678
    w: 0.70710678
  }
}
"""


def test_parse_pose_v():
    from deepracer_env.sim_control.backends.ros_gz_backend import RosGzBackend
    poses = RosGzBackend._parse_pose_v(POSE_V_DUMP)
    assert set(poses) == {"ground_plane", "car_0"}
    car = poses["car_0"]
    assert car.position == Vec3(5.5, -2.25, 0.05)
    assert math.isclose(car.orientation.yaw, math.pi / 2, abs_tol=1e-6)
    # the header sub-block (sec: 12) must not leak into any pose's position
    assert poses["ground_plane"].position == Vec3(0.0, 0.0, 0.0)


# --------------------------------------------------------------------------- #
# Null backend
# --------------------------------------------------------------------------- #

def test_null_backend_records_and_roundtrips():
    sc = NullSimControl()
    sc.spawn_entity("racetrack_0", "<sdf/>", Pose.at(1, 2))
    assert "racetrack_0" in sc.list_entities()
    assert isinstance(sc.get_entity_state("racetrack_0"), EntityState)
    sc.set_entity_state("car_0", EntityState(Pose.at(0, 0), Twist()))
    sc.step(3)
    sc.delete_entity("racetrack_0")
    assert "racetrack_0" not in sc.list_entities()
    kinds = [c[0] for c in sc.calls]
    assert kinds == [
        "spawn_entity", "get_entity_state", "set_entity_state", "step", "delete_entity"]
    assert sc.supports("anything") is True
