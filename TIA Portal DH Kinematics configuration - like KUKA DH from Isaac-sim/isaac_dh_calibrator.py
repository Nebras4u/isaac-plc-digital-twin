import rclpy
from rclpy.node import Node
import time
import os

# Required to parse the Isaac Sim Action Graph TF trees
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# =========================================================
# Configuration
# =========================================================
WORLD_FRAME = "World"  # Your absolute global origin frame

# List of the sequential joint frame names published by Isaac Sim.
ISAAC_FRAMES = [
    "joint_a1",       # Axis 1 Pivot Frame
    "joint_a2",       # Axis 2 Pivot Frame
    "joint_a3",       # Axis 3 Pivot Frame
    "joint_a4",       # Axis 4 Pivot Frame
    "joint_a5",       # Axis 5 Pivot Frame
    "joint_a6",       # Axis 6 Pivot Frame
    "link_6_tool0",   # Intermediate Flange Link
    "tool0"           # End Effector (Flange/TCP) Frame
]

class IsaacDHCalibrator(Node):
    def __init__(self):
        super().__init__("isaac_dh_calibrator")
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Run printing at 2 Hz (every 0.5 seconds) for clear reading
        self.create_timer(0.5, self.print_all_joint_wcs)
        self.get_logger().info("DH Frame Calibrator Started. Listening to Isaac TF tree...")

    def print_all_joint_wcs(self):
        # Clear the terminal screen dynamically for an easily scannable dashboard overview
        os.system('clear' if os.name == 'posix' else 'cls')
        
        print("=" * 80)
        print(f" ISAAC SIM JOINT WCS CALIBRATION DASHBOARD (Target Origin: {WORLD_FRAME})")
        print("=" * 80)
        print(f"{'Frame Name':<15} | {'X Position (mm)':>18} | {'Y Position (mm)':>18} | {'Z Position (mm)':>18}")
        print("-" * 80)

        missing_frames = []

        for frame in ISAAC_FRAMES:
            try:
                # Query the transform tree from the absolute world origin down to the joint link node
                trans = self.tf_buffer.lookup_transform(
                    WORLD_FRAME,
                    frame,
                    rclpy.time.Time()
                )
                
                # Convert translations natively from meters to millimeters for precise TIA Portal DH entry
                x_mm = trans.transform.translation.x * 1000.0
                y_mm = trans.transform.translation.y * 1000.0
                z_mm = trans.transform.translation.z * 1000.0
                
                print(f"{frame:<15} | {x_mm:>18.3f} | {y_mm:>18.3f} | {z_mm:>18.3f}")
                
            except TransformException:
                missing_frames.append(frame)
                print(f"{frame:<15} | {'[Waiting for TF...]':>18} | {'':>18} | {'':>18}")

        print("-" * 80)
        print("💡 HOW TO USE THIS DATA TO CALIBRATE DH PARAMETERS IN TIA PORTAL:")
        print("1. Your static Global Base Offset = 'joint_a1' position values directly.")
        print("2. Distance from joint_a1 to joint_a2 along Z = your DH 'd1' height parameter.")
        print("3. Distance from joint_a1 to joint_a2 along X = your DH 'a1' horizontal shoulder offset.")
        print("4. Distance from joint_a2 to joint_a3 along Z = your DH 'a2' main link boom length.")
        print("5. Distance from joint_a3 to joint_a4 along X = your DH 'a3' forearm length parameter.")
        print("=" * 80)
        
        if missing_frames:
            print(f"⚠️ Missing links in TF tree (Check names): {', '.join(missing_frames)}")


def main(args=None):
    rclpy.init(args=args)
    node = IsaacDHCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nCalibrator terminated.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
