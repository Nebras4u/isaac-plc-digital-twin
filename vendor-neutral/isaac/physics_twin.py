# physics_twin.py
# Isaac Sim physics executor — vendor-neutral.
# Reads robot definition from robot.yaml.
#
# Gravity: kept ENABLED (real physics). Compensation is done implicitly
# by high drive stiffness in robot.yaml. Isaac Sim 5.1 does not expose a
# gravity compensation API on Articulation, so this is the only way to
# preserve real physics while still tracking position targets accurately.

import asyncio
import sys
import time
import numpy as np
import yaml
import omni.kit.app
import omni.usd
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState
from isaacsim.core.prims import Articulation
from pxr import UsdPhysics


# -------------------------------------------------------------------------
# Load robot definition
# -------------------------------------------------------------------------
ROBOT_CFG_FILE = "/workspace/robot.yaml"
try:
    with open(ROBOT_CFG_FILE) as f:
        _loaded = yaml.safe_load(f)
    if _loaded is None:
        raise ValueError(f"robot.yaml is empty: {ROBOT_CFG_FILE}")
    if "robot" not in _loaded:
        raise KeyError(f"robot.yaml missing 'robot:' key")
    _cfg = _loaded["robot"]
except FileNotFoundError:
    raise FileNotFoundError(f"robot.yaml not found at {ROBOT_CFG_FILE}")

ROBOT_PATH  = _cfg["isaac_path"]
ROBOT_SCOPE = _cfg["isaac_scope"]
DOF_ORDER   = _cfg["all_joints"]
DRIVES      = _cfg["drives"]
JOINT_PATHS = {
    k: v.replace("{scope}", ROBOT_SCOPE)
    for k, v in _cfg["joint_paths"].items()
}
PUBLISH_HZ = 60.0


# -------------------------------------------------------------------------
# Helpers
# -------------------------------------------------------------------------
def inspect_limits():
    stage = omni.usd.get_context().get_stage()
    print("=" * 60)
    print("Joint limits:")
    for name, path in JOINT_PATHS.items():
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            print(f"  {name}: NOT FOUND at {path}")
            continue
        joint = UsdPhysics.RevoluteJoint(prim)
        if joint:
            lo = joint.GetLowerLimitAttr().Get()
            hi = joint.GetUpperLimitAttr().Get()
            print(f"  {name}: [{lo}, {hi}]")
        else:
            print(f"  {name}: no RevoluteJoint API")
    print("=" * 60)


def configure_drives():
    stage = omni.usd.get_context().get_stage()
    print("Configuring drives...")
    for joint_name, params in DRIVES.items():
        path = JOINT_PATHS[joint_name]
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            print(f"  [skip] {joint_name}: prim not found")
            continue
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drive:
            print(f"  [skip] {joint_name}: no angular drive API")
            continue
        drive.GetStiffnessAttr().Set(float(params["stiffness"]))
        drive.GetDampingAttr().Set(float(params["damping"]))
        drive.GetMaxForceAttr().Set(float(params["max_force"]))
        print(f"  [ok] {joint_name}: K={params['stiffness']:.0f} "
              f"D={params['damping']:.0f} F={params['max_force']:.0f}")


# -------------------------------------------------------------------------
# ROS node
# -------------------------------------------------------------------------
class PhysicsTwin(Node):
    def __init__(self, robot):
        super().__init__("physics_twin")
        self.robot = robot
        self.latest_setpoint = None
        self._last_sp_t = 0.0
        self._tag = ""
        self._applied_tag = ""
        self._pub_count = 0
        self._needs_reinit = False

        sp_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            JointState, "/servo_setpoints", self.on_setpoint, sp_qos
        )
        self.js_pub = self.create_publisher(JointState, "/joint_states", 1)

        self.get_logger().info(
            f"PhysicsTwin ready. DOF={DOF_ORDER}. gravity=ENABLED"
        )

    def on_setpoint(self, msg: JointState):
        self.latest_setpoint = dict(zip(msg.name, msg.position))
        self._tag = msg.header.frame_id
        self._last_sp_t = time.monotonic()

    def step(self):
        if self.latest_setpoint is None:
            return
        if (time.monotonic() - self._last_sp_t) > 0.5:
            return

        try:
            current = self.robot.get_joint_positions()
        except Exception as e:
            print(f"[physics_twin] Articulation invalid: {e}")
            self._needs_reinit = True
            return
        if current is None:
            return
        current = current[0]

        target = np.zeros(len(DOF_ORDER), dtype=np.float32)
        for i, jname in enumerate(DOF_ORDER):
            if jname in self.latest_setpoint:
                target[i] = float(self.latest_setpoint[jname])
            else:
                target[i] = float(current[i])

        try:
            self.robot.set_joint_position_targets(
                positions=target.reshape(1, -1),
                joint_indices=list(range(len(DOF_ORDER))),
            )
        except Exception as e:
            print(f"[physics_twin] set_joint_position_targets failed: {e}")
            self._needs_reinit = True
            return

        self._applied_tag = self._tag

    def publish_states(self):
        try:
            pos = self.robot.get_joint_positions()
        except Exception as e:
            print(f"[physics_twin] Articulation invalid in publish_states: {e}")
            self._needs_reinit = True
            return
        if pos is None:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(DOF_ORDER)
        msg.position = [float(pos[0][i]) for i in range(len(DOF_ORDER))]
        tag = self._applied_tag
        msg.header.frame_id = f"{tag};{time.monotonic_ns()}" if tag else ""
        self.js_pub.publish(msg)

        self._pub_count += 1
        if self._pub_count % 600 == 0:
            print(f"[physics_twin] gravity=ENABLED reinit_pending={self._needs_reinit}")


# -------------------------------------------------------------------------
# Main
# -------------------------------------------------------------------------
def _make_robot():
    robot = Articulation(ROBOT_PATH)
    robot.initialize()
    if list(robot.dof_names) != DOF_ORDER:
        print(f"!!! WARNING: DOF order mismatch: {robot.dof_names}")
    return robot


async def main():
    if not rclpy.ok():
        try:
            rclpy.init()
        except RuntimeError:
            pass

    print("=" * 60)
    print("Initializing physics twin (gravity ENABLED)...")
    print("=" * 60)

    inspect_limits()
    configure_drives()

    print("Initializing articulation...")
    robot = _make_robot()
    print(f"Articulation DOF names: {robot.dof_names}")
    print(f"Expected DOF_ORDER  : {DOF_ORDER}")

    node = PhysicsTwin(robot)
    print("Entering main loop. Publishing /joint_states @ 60 Hz.")

    try:
        while True:
            if node._needs_reinit:
                print("[physics_twin] Reinitializing Articulation...")
                try:
                    node.robot = _make_robot()
                    node._needs_reinit = False
                    print("[physics_twin] Reinit OK")
                except Exception as e:
                    print(f"[physics_twin] Reinit failed: {e}")
                    await omni.kit.app.get_app().next_update_async()
                    continue

            # Receive FIRST, then apply, then publish: a setpoint that
            # arrived since the last frame is applied in THIS frame instead
            # of waiting one extra frame.
            try:
                rclpy.spin_once(node, timeout_sec=0.0)
            except Exception:
                pass
            node.step()
            node.publish_states()
            await omni.kit.app.get_app().next_update_async()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass


_prev = getattr(sys, "_physics_twin_task", None)
if _prev is not None and not _prev.done():
    _prev.cancel()
sys._physics_twin_task = asyncio.ensure_future(main())