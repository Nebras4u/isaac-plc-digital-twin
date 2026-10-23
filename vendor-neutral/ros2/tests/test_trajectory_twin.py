import pytest
from trajectory_twin import RuckigPlanner


@pytest.mark.parametrize("bad", [
    {"control_hz": 0}, {"control_hz": 2000},
    {"max_vel": 0}, {"max_vel": -1},
    {"max_acc": 0}, {"max_jerk": 0},
    {"jump_threshold": 0},
])
def test_ruckig_rejects_bad_params(bad):
    args = dict(
        dof=4, control_hz=100, max_vel=1.0,
        max_acc=2.0, max_jerk=10.0, jump_threshold=0.1,
    )
    args.update(bad)
    with pytest.raises(ValueError):
        RuckigPlanner(**args)