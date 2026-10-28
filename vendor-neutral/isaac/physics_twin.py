# physics_twin.py
# Isaac Sim physics executor — vendor-neutral.
# Reads robot definition from robot.yaml (ROS parameter "robot_cfg").
#
# Gravity: kept ENABLED. Compensation via high drive stiffness in robot.yaml.
#
# Safety model:
#   * Setpoint older than `setpoint_timeout_s` → HOLD last valid target.
#   * /plc/emergency_stop = True              → HOLD current position.
#
# DOF ordering:
#   JointMapper fails hard if robot.dof_names != robot.yaml all_joints set.
#
# QoS:
#   /servo_setpoints    sub: BEST_EFFORT depth=1
#   /plc/emergency_stop sub: TRANSIENT_LOCAL
#   /joint_states       pub: BEST_EFFORT depth=1
#
# v3.6 changes:
#   * [G] _applied_tag is NOT cleared on every publish. It is kept until
#     either (a) the setpoint ages past TAG_VALID_S, or (b) a newer
#     setpoint arrives. This lets several /joint_states messages carry the
#     same tag chain, so the latency probe has a chance to actually reach
#     the bridge (previously 50% of /joint_states had no frame_id because
#     the tag was consumed by the very first publish after each setpoint).
#   * [H] Only the tag chain's ORIGINAL command origin is preserved. We
#     never reattach a stale chain after TAG_VALID_S has elapsed.

import asyncio
import sys
import time
import numpy as np
import yaml

import omni.kit.app
import omni.usd

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from isaacsim.core.prims import Articulation
from pxr import UsdPhysics

# --- Ensure joint_mapper is importable inside Isaac Sim's Script Editor ---
# Isaac copies the script to /tmp/carb.*/script_*.py, so __file__ does not
# point to /workspace. Add both candidate directories to sys.path.
import os as _os
_SCRIPT_DIR = _os.path.dirname(_os.path.abspath(__file__))
for _p in (_SCRIPT_DIR, '/workspace'):
    if _os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)
del _os, _SCRIPT_DIR, _p

from joint_mapper import JointMapper


DEFAULT_ROBOT_CFG    = "/workspace/robot.yaml"
DEFAULT_PUBLISH_HZ   = 120.0
DEFAULT_TIMEOUT_S    = 0.5
TAG_VALID_S          = 0.5
REINIT_BACKOFF_S     = 1.0
STATUS_LOG_PERIOD_S  = 10.0
MAX_MSG_PER_FRAME    = 8


def load_robot_cfg(path: str) -> dict:
    with open(path) as f:
        loaded = yaml.safe_load(f)
    if loaded is None:
        raise ValueError(f"robot.yaml is empty: {path}")
    if "robot" not in loaded:
        raise KeyError(f"robot.yaml missing 'robot:' key")
    cfg = loaded["robot"]
    for k in ("isaac_path", "isaac_scope", "all_joints", "drives", "joint_paths"):
        if k not in cfg:
            raise KeyError(f"robot.yaml missing robot.{k}")
    return cfg


def inspect_limits(joint_paths: dict):
    stage = omni.usd.get_context().get_stage()
    print("=" * 60)
    print("Joint limits:")
    for name, path in joint_paths.items():
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


def configure_drives(drives: dict, joint_paths: dict):
    stage = omni.usd.get_context().get_stage()
    print("Configuring drives...")
    for joint_name, params in drives.items():
        path = joint_paths[joint_name]
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
class PhysicsTwin(Node):
    def __init__(self, robot, mapper, cfg, params):
        super().__init__("physics_twin")
        self.robot  = robot
        self.mapper = mapper
        self.cfg    = cfg

        self.publish_hz = max(1.0, float(params["publish_hz"]))
        self.timeout_s  = max(0.0, float(params["setpoint_timeout_s"]))

        n = mapper.n

        self._latest_setpoint   = None
        self._last_sp_t         = 0.0
        self._tag               = ""       # tag from most recent setpoint
        self._tag_t             = 0.0      # monotonic when it arrived
        self._applied_tag       = ""       # tag actually applied by step()
        self._applied_tag_t     = 0.0
        self._fresh_this_frame  = False
        self._estop_active      = False
        self._needs_reinit      = False
        self._last_reinit_t     = 0.0
        self._last_pub_t        = 0.0
        self._last_status_log_t = 0.0
        self._seed_done         = False

        self._target_yaml      = np.zeros(n, dtype=np.float32)
        self._target_isaac     = np.zeros(n, dtype=np.float32)
        self._last_valid_yaml  = np.zeros(n, dtype=np.float32)
        self._last_valid_isaac = np.zeros(n, dtype=np.float32)
        self._current_isaac    = np.zeros(n, dtype=np.float32)
        self._velocity_isaac   = np.zeros(n, dtype=np.float32)
        self._joint_indices    = np.arange(n, dtype=np.int32)
        self._target_isaac_2d  = self._target_isaac.reshape(1, -1)

        self._seed_from_current()

        sp_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            JointState, "/servo_setpoints", self.on_setpoint, sp_qos
        )
        estop_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool, "/plc/emergency_stop", self.on_estop, estop_qos
        )

        js_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.js_pub = self.create_publisher(JointState, "/joint_states", js_qos)

        self.get_logger().info(
            f"PhysicsTwin ready. n={n} pub_hz={self.publish_hz} "
            f"timeout={self.timeout_s}s gravity=ENABLED"
        )
        self.get_logger().info(f"DOF order (YAML) : {mapper.dof_order}")
        self.get_logger().info(f"DOF order (Isaac): {mapper.isaac_names}")

    def on_setpoint(self, msg: JointState):
        if self._estop_active:
            return
        sp = {}
        for name, pos in zip(msg.name, msg.position):
            sp[name] = float(pos)
        self._latest_setpoint = sp
        self._tag              = msg.header.frame_id
        self._tag_t            = time.monotonic()
        self._last_sp_t        = self._tag_t

    def on_estop(self, msg: Bool):
        was = self._estop_active
        self._estop_active = bool(msg.data)
        if self._estop_active and not was:
            self.get_logger().warn("E-Stop ACTIVE — holding current position")
            self._latest_setpoint = None
            self._last_sp_t       = 0.0
            self._tag             = ""
            self._tag_t           = 0.0
            self._applied_tag     = ""
            self._applied_tag_t   = 0.0
        elif not self._estop_active and was:
            self.get_logger().info("E-Stop CLEARED — awaiting fresh setpoint")

    def _seed_from_current(self) -> bool:
        try:
            cur = self.robot.get_joint_positions()
        except Exception as e:
            print(f"[physics_twin] seed: get_joint_positions failed: {e}")
            return False
        if cur is None:
            return False
        self._last_valid_isaac[:] = cur[0]
        self._last_valid_yaml[:]  = cur[0][self.mapper.yaml_to_isaac]
        self._seed_done           = True
        return True

    def _refresh_current(self) -> bool:
        try:
            cur = self.robot.get_joint_positions()
        except Exception as e:
            print(f"[physics_twin] Articulation invalid: {e}")
            self._needs_reinit = True
            return False
        if cur is None:
            return False
        self._current_isaac[:] = cur[0]
        return True

    def _build_target(self) -> bool:
        self._fresh_this_frame = False

        if self._estop_active:
            if not self._refresh_current():
                return False
            self._target_isaac[:] = self._current_isaac
            return True

        if self._latest_setpoint is None:
            if not self._seed_done:
                if not self._refresh_current():
                    return False
                self._last_valid_isaac[:] = self._current_isaac
                self._last_valid_yaml[:]  = self._current_isaac[self.mapper.yaml_to_isaac]
                self._seed_done           = True
            self._target_isaac[:] = self._last_valid_isaac
            return True

        age = time.monotonic() - self._last_sp_t
        if age > self.timeout_s:
            self._target_isaac[:] = self._last_valid_isaac
            return True

        sp = self._latest_setpoint
        for j, jname in enumerate(self.mapper.dof_order):
            v = sp.get(jname)
            if v is not None:
                self._target_yaml[j] = v
            else:
                self._target_yaml[j] = self._last_valid_yaml[j]

        self._target_isaac[self.mapper.yaml_to_isaac] = self._target_yaml
        self._last_valid_yaml[:]  = self._target_yaml
        self._last_valid_isaac[:] = self._target_isaac
        self._fresh_this_frame    = True
        return True

    def step(self):
        if not self._build_target():
            return

        try:
            self.robot.set_joint_position_targets(
                positions=self._target_isaac_2d,
                joint_indices=self._joint_indices,
            )
        except Exception as e:
            print(f"[physics_twin] set_joint_position_targets failed: {e}")
            self._needs_reinit = True
            return

        if self._fresh_this_frame:
            self._applied_tag   = self._tag
            self._applied_tag_t = self._tag_t

    def publish_states(self):
        now = time.monotonic()
        if (now - self._last_pub_t) < (1.0 / self.publish_hz):
            return
        self._last_pub_t = now

        try:
            pos = self.robot.get_joint_positions()
            vel = self.robot.get_joint_velocities()
        except Exception as e:
            print(f"[physics_twin] Articulation invalid in publish_states: {e}")
            self._needs_reinit = True
            return
        if pos is None:
            return

        self._current_isaac[:] = pos[0]
        if vel is not None:
            self._velocity_isaac[:] = vel[0]
        else:
            self._velocity_isaac[:] = 0.0

        pos_yaml = self._current_isaac[self.mapper.yaml_to_isaac]
        vel_yaml = self._velocity_isaac[self.mapper.yaml_to_isaac]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name         = self.mapper.dof_order
        msg.position     = pos_yaml.tolist()
        msg.velocity     = vel_yaml.tolist()
        msg.effort       = []

        # v3.6 [G]: keep the tag attached as long as it's fresh. Multiple
        # /joint_states messages may carry the same tag chain while it's
        # valid; the bridge dedupes on the origin timestamp inside the tag
        # (see _LatencyProbe.record → nums[0] == self._last check).
        tag = self._applied_tag
        fresh = (
            tag
            and not self._estop_active
            and (now - self._applied_tag_t) <= TAG_VALID_S
        )
        if fresh:
            msg.header.frame_id = f"{tag};{time.monotonic_ns()}"
        else:
            msg.header.frame_id = ""
            # The tag chain is now definitively dead. Drop it so we do not
            # keep re-checking the age on every publish.
            self._applied_tag   = ""
            self._applied_tag_t = 0.0
            self._tag           = ""
            self._tag_t         = 0.0

        self.js_pub.publish(msg)

        if (now - self._last_status_log_t) >= STATUS_LOG_PERIOD_S:
            self._last_status_log_t = now
            sp_age = (now - self._last_sp_t) if self._last_sp_t else -1.0
            self.get_logger().info(
                f"gravity=ENABLED estop={self._estop_active} "
                f"reinit_pending={self._needs_reinit} sp_age={sp_age:.2f}s"
            )


def make_robot(robot_path: str) -> Articulation:
    robot = Articulation(robot_path)
    robot.initialize()
    return robot


async def main():
    if not rclpy.ok():
        try:
            rclpy.init()
        except RuntimeError:
            pass

    tmp = Node("physics_twin_params")
    tmp.declare_parameter("robot_cfg",           DEFAULT_ROBOT_CFG)
    tmp.declare_parameter("publish_hz",          DEFAULT_PUBLISH_HZ)
    tmp.declare_parameter("setpoint_timeout_s",  DEFAULT_TIMEOUT_S)
    cfg_path = tmp.get_parameter("robot_cfg").value
    params   = {
        "publish_hz":         tmp.get_parameter("publish_hz").value,
        "setpoint_timeout_s": tmp.get_parameter("setpoint_timeout_s").value,
    }
    tmp.destroy_node()

    cfg         = load_robot_cfg(cfg_path)
    robot_path  = cfg["isaac_path"]
    robot_scope = cfg["isaac_scope"]
    dof_order   = list(cfg["all_joints"])
    joint_paths = {
        k: v.replace("{scope}", robot_scope)
        for k, v in cfg["joint_paths"].items()
    }

    print("=" * 60)
    print("Initializing physics twin (gravity ENABLED)...")
    print("=" * 60)

    inspect_limits(joint_paths)
    configure_drives(cfg["drives"], joint_paths)

    print("Initializing articulation...")
    robot = make_robot(robot_path)
    try:
        mapper = JointMapper(robot.dof_names, dof_order)
    except RuntimeError as e:
        print(f"[FATAL] {e}")
        raise

    node = PhysicsTwin(robot, mapper, cfg, params)

    print(f"Entering main loop. Publishing /joint_states @ ≤{node.publish_hz} Hz.")

    try:
        while True:
            if node._needs_reinit:
                now = time.monotonic()
                if (now - node._last_reinit_t) < REINIT_BACKOFF_S:
                    await omni.kit.app.get_app().next_update_async()
                    continue
                node._last_reinit_t = now
                print("[physics_twin] Reinitializing Articulation...")
                try:
                    node.robot = make_robot(robot_path)
                    JointMapper(node.robot.dof_names, dof_order)
                    node._seed_from_current()
                    node._needs_reinit = False
                    print("[physics_twin] Reinit OK")
                except Exception as e:
                    print(f"[physics_twin] Reinit failed: {e}")
                    await omni.kit.app.get_app().next_update_async()
                    continue

            # Bounded spin_all: drain up to MAX_MSG_PER_FRAME per frame so
            # E-Stop and setpoint from the same DDS burst are both handled
            # before step().
            for _ in range(MAX_MSG_PER_FRAME):
                try:
                    rclpy.spin_once(node, timeout_sec=0.0)
                except Exception:
                    break

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