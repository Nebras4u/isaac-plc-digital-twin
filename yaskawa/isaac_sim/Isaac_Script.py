"""
Isaac Sim bridge — minimal terminal output (1 Hz, single tick).
  • Subscribe /joint_commands -> Articulation
  • Publish   /joint_states   (from Articulation)
  • Publish   /tf             (manual, from USD XformCache)
  • Print once per second:
        received (deg): [('joint_1', X.X), ('joint_2', X.X), ('joint_3', X.X), ('joint_4', X.X)]
        [WCS]     tool  pos(mm): (X.XX, Y.YY, Z.ZZ)
"""

import time
import numpy as np
import asyncio
import omni.kit.app
import omni.usd
from pxr import UsdGeom

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped

from isaacsim.core.prims import Articulation


ROBOT_PATH  = "/World/Main_Robot_L_URDF/base_link"
ROBOT_SCOPE = "/World/Main_Robot_L_URDF"

DOF_ORDER  = ["joint_1", "joint_2", "joint_3", "joint_5", "joint_4"]
TRACKED_FRAMES = ["base_link", "link_1", "link_2", "link_3", "link_4", "tool"]
WORLD_FRAME = "base_link"

PRINT_JOINT_ORDER = ["joint_1", "joint_2", "joint_3", "joint_4"]

PUBLISH_HZ = 30
TF_HZ      = 30

PRINT_PERIOD_SEC = 1.0

RAD2DEG = 180.0 / np.pi
DEG2RAD = np.pi / 180.0


def get_prim_world_pose(stage, prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return None, None
    m = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
    t = m.ExtractTranslation()
    q = m.ExtractRotationQuat()
    imag = q.GetImaginary()
    pos = (float(t[0]), float(t[1]), float(t[2]))
    quat = (float(q.GetReal()), float(imag[0]), float(imag[1]), float(imag[2]))
    return pos, quat


class IsaacBridge(Node):
    def __init__(self, robot):
        super().__init__("isaac_bridge")
        self.robot = robot
        self.latest_cmd = None

        self._last_print_t = 0.0

        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.tf_pub = self.create_publisher(TransformStamped, "/tf", 10)
        self.create_subscription(JointState, "/joint_commands", self.on_cmd, 10)

        self.create_timer(1.0 / PUBLISH_HZ, self.publish_joint_states)
        self.create_timer(1.0 / TF_HZ, self.publish_tf)
        # single combined print timer
        self.create_timer(1.0 / TF_HZ, self.print_tick)

        self.get_logger().info("IsaacBridge up (no tf2_ros).")

    def on_cmd(self, msg: JointState):
        # silent; only stored
        self.latest_cmd = dict(zip(msg.name, msg.position))

    def apply_command_if_any(self):
        if self.latest_cmd is None:
            return
        target = [float(self.latest_cmd.get(j, 0.0)) for j in DOF_ORDER]
        self.robot.set_joint_position_targets(
            positions=np.array([target]),
            joint_indices=list(range(len(DOF_ORDER))),
        )

    def publish_joint_states(self):
        pos = self.robot.get_joint_positions()
        if pos is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(DOF_ORDER)
        msg.position = [float(pos[0][i]) for i in range(len(DOF_ORDER))]
        self.js_pub.publish(msg)

    def publish_tf(self):
        stage = omni.usd.get_context().get_stage()
        now = self.get_clock().now().to_msg()
        for frame in TRACKED_FRAMES:
            if frame == WORLD_FRAME:
                continue
            pos, quat = get_prim_world_pose(stage, f"{ROBOT_SCOPE}/{frame}")
            if pos is None:
                continue
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = WORLD_FRAME
            t.child_frame_id = frame
            t.transform.translation.x = pos[0]
            t.transform.translation.y = pos[1]
            t.transform.translation.z = pos[2]
            t.transform.rotation.x = quat[1]
            t.transform.rotation.y = quat[2]
            t.transform.rotation.z = quat[3]
            t.transform.rotation.w = quat[0]
            self.tf_pub.publish(t)

    def print_tick(self):
        """Single 1 Hz tick: prints joint line + WCS line together (or nothing)."""
        now = time.monotonic()
        if now - self._last_print_t < PRINT_PERIOD_SEC:
            return
        self._last_print_t = now

        # ---- joint line (only if a command has ever been received) ----
        if self.latest_cmd is not None:
            printed = [
                (j, round(self.latest_cmd.get(j, 0.0) * RAD2DEG, 1))
                for j in PRINT_JOINT_ORDER
            ]
            print(f"received (deg): {printed}")

        # ---- WCS line (only if the stage has the tool) ----
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return
        cache = UsdGeom.XformCache()
        base_prim = stage.GetPrimAtPath(f"{ROBOT_SCOPE}/{WORLD_FRAME}")
        tool_prim = stage.GetPrimAtPath(f"{ROBOT_SCOPE}/tool")
        if not base_prim or not base_prim.IsValid():
            return
        if not tool_prim or not tool_prim.IsValid():
            return
        Mb_inv = cache.GetLocalToWorldTransform(base_prim).GetInverse()
        Mrel = cache.GetLocalToWorldTransform(tool_prim) * Mb_inv
        t = Mrel.ExtractTranslation()
        print(f"[WCS]     tool  pos(mm): "
              f"({t[0]*1000.0: .2f}, {t[1]*1000.0: .2f}, {t[2]*1000.0: .2f})")


async def main():
    if not rclpy.ok():
        rclpy.init()

    robot = Articulation(ROBOT_PATH)
    robot.initialize()
    print(f"[IsaacBridge] DOF: {robot.dof_names}")

    node = IsaacBridge(robot)
    print("[IsaacBridge] Running. Waiting for /joint_commands...")

    while True:
        node.apply_command_if_any()
        rclpy.spin_once(node, timeout_sec=0.0)
        await omni.kit.app.get_app().next_update_async()


_task = asyncio.ensure_future(main())
