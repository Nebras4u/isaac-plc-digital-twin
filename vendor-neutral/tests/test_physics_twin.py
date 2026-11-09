"""Tests for joint_mapper.JointMapper — no Isaac needed."""
import numpy as np
import pytest

from joint_mapper import JointMapper


def test_joint_mapper_identity():
    names = ["j1", "j2", "j3", "j4"]
    m = JointMapper(names, names)
    assert m.n == 4
    assert m.yaml_to_isaac.tolist() == [0, 1, 2, 3]


def test_joint_mapper_reordered():
    isaac = ["j1", "j2", "j3", "j4", "j5"]
    yaml  = ["j1", "j2", "j3", "j5", "j4"]
    m = JointMapper(isaac, yaml)
    assert m.yaml_to_isaac.tolist() == [0, 1, 2, 4, 3]


def test_joint_mapper_rejects_mismatch():
    with pytest.raises(RuntimeError, match="DOF set mismatch"):
        JointMapper(["j1", "j2"], ["j1", "j3"])


def test_joint_mapper_roundtrip():
    isaac = ["a", "b", "c"]
    yaml  = ["c", "a", "b"]
    m = JointMapper(isaac, yaml)
    isaac_vals = np.array([10.0, 20.0, 30.0], dtype=np.float32)
    yaml_vals  = isaac_vals[m.yaml_to_isaac]
    assert yaml_vals.tolist() == [30.0, 10.0, 20.0]

    target_yaml  = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    target_isaac = np.zeros(3, dtype=np.float32)
    target_isaac[m.yaml_to_isaac] = target_yaml
    assert target_isaac.tolist() == [2.0, 3.0, 1.0]