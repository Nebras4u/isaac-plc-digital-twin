import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('kuka_supervised_control')
    xacro_file = os.path.join(pkg_share, 'config', 'kuka_kr210_test.urdf.xacro')
    controllers_file = os.path.join(pkg_share, 'config', 'test_controllers.yaml')

    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_file],
        output='screen',
    )

    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    spawn_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['kuka_arm_controller'],
        output='screen',
    )

    spawn_safety = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['safety_enable_controller'],
        output='screen',
    )

    return LaunchDescription([
        rsp_node,
        control_node,
        spawn_jsb,
        spawn_arm,
        spawn_safety, Node(package="controller_manager", executable="spawner", arguments=["safety_estop_controller"]), Node(package="controller_manager", executable="spawner", arguments=["safety_reset_controller"]),
    ])
