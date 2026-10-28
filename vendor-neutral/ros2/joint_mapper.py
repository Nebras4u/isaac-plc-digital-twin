# joint_mapper.py — importable without Isaac Sim
import numpy as np


class JointMapper:
    """
    Bidirectional mapping between robot.yaml 'all_joints' order (operator /
    PLC order) and Isaac's dof_names order.

        yaml_to_isaac[j] = isaac index of the joint named dof_order[j]

        Apply:  positions_isaac[yaml_to_isaac] = target_yaml
        Read:   current_yaml               = current_isaac[yaml_to_isaac]

    Fails hard if the two joint sets differ — no silent axis swaps.
    """

    def __init__(self, isaac_names, dof_order):
        isaac_names = list(isaac_names)
        dof_order   = list(dof_order)

        if set(isaac_names) != set(dof_order):
            missing_in_isaac = sorted(set(dof_order) - set(isaac_names))
            missing_in_yaml  = sorted(set(isaac_names) - set(dof_order))
            raise RuntimeError(
                "DOF set mismatch — refusing to run.\n"
                f"  Isaac dof_names: {isaac_names}\n"
                f"  YAML all_joints: {dof_order}\n"
                f"  Missing in Isaac: {missing_in_isaac}\n"
                f"  Missing in YAML : {missing_in_yaml}"
            )

        self.isaac_names   = isaac_names
        self.dof_order     = dof_order
        self.n             = len(dof_order)
        self.yaml_to_isaac = np.array(
            [isaac_names.index(nm) for nm in dof_order], dtype=np.int32
        )