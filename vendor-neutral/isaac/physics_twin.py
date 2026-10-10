# physics_twin.py
# Isaac Sim physics twin — KR210 articulation driver.
#
# Subscribes: /servo_setpoints (JointState, radians)
# Publishes:  /joint_states   (JointState, radians) @ 60 Hz

import asyncio
import numpy as np
import omni.kit.app
import omni.usd
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from isaacsim.core.prims import Articulation
from pxr import UsdPhysics


# -------------------------------------------------------------------------
# Robot configuration
# -------------------------------------------------------------------------
ROBOT_PATH  = "/World/Main_Robot_L_URDF/base_link"
ROBOT_SCOPE = "/World/Main_Robot_L_URDF"

DOF_ORDER = ["joint_1", "joint_2", "joint_3", "joint_5", "joint_4"]

PUBLISH_HZ = 60.0


# -------------------------------------------------------------------------
# Prim paths for each joint
# -------------------------------------------------------------------------
JOINT_PATHS = {
    "joint_1": f"{ROBOT_SCOPE}/base_link/joint_1",
    "joint_2": f"{ROBOT_SCOPE}/link_1/joint_2",
    "joint_3": f"{ROBOT_SCOPE}/link_2/joint_3",
    "joint_4": f"{ROBOT_SCOPE}/link_4/joint_4",
    "joint_5": f"{ROBOT_SCOPE}/link_3/joint_5",
}


# -------------------------------------------------------------------------
# Drive gains
# -------------------------------------------------------------------------
DRIVES = {
    "joint_1": {"stiffness": 1000000.0, "damping": 100000.0, "max_force": 50000000.0},
    "joint_2": {"stiffness": 1000000.0, "damping": 100000.0, "max_force": 50000000.0},
    "joint_3": {"stiffness": 1000000.0, "damping": 100000.0, "max_force": 50000000.0},
    "joint_4": {"stiffness":  200000.0, "damping":  20000.0, "max_force":  5000000.0},
    "joint_5": {"stiffness":  200000.0, "damping":  20000.0, "max_force":  5000000.0},
}


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

        self.create_subscription(
            JointState, "/servo_setpoints", self.on_setpoint, 10
        )
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        # no create_timer: we publish every frame from main loop

        self.get_logger().info(f"Physics twin ready. DOF={DOF_ORDER}.")

    def on_setpoint(self, msg: JointState):
        self.latest_setpoint = dict(zip(msg.name, msg.position))

    def step(self):
        if self.latest_setpoint is None:
            return

        current = self.robot.get_joint_positions()
        if current is None:
            return
        current = current[0]

        target = np.zeros(len(DOF_ORDER), dtype=np.float32)
        for i, jname in enumerate(DOF_ORDER):
            if jname in self.latest_setpoint:
                target[i] = float(self.latest_setpoint[jname])
            else:
                target[i] = float(current[i])

        self.robot.set_joint_position_targets(
            positions=target.reshape(1, -1),
            joint_indices=list(range(len(DOF_ORDER))),
        )

    def publish_states(self):
        pos = self.robot.get_joint_positions()
        if pos is None:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(DOF_ORDER)
        msg.position = [float(pos[0][i]) for i in range(len(DOF_ORDER))]
        self.js_pub.publish(msg)


# -------------------------------------------------------------------------
# Main
# -------------------------------------------------------------------------
async def main():
    if not rclpy.ok():
        try:
            rclpy.init()
        except RuntimeError:
            pass

    print("=" * 60)
    print("Initializing physics twin...")
    print("=" * 60)

    inspect_limits()
    configure_drives()

    print("Initializing articulation...")
    robot = Articulation(ROBOT_PATH)
    robot.initialize()
    print(f"Articulation DOF names: {robot.dof_names}")
    print(f"Expected DOF_ORDER  : {DOF_ORDER}")

    if list(robot.dof_names) != DOF_ORDER:
        print("!!! WARNING: DOF order mismatch !!!")

    node = PhysicsTwin(robot)
    print("Entering main loop. Publishing /joint_states @ 60 Hz.")

    while True:
        node.step()
        node.publish_states()          # publish every frame = 60 Hz
        try:
            rclpy.spin_once(node, timeout_sec=0.0)
        except Exception:
            pass
        await omni.kit.app.get_app().next_update_async()


_task = asyncio.ensure_future(main())