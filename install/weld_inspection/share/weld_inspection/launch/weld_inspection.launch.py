import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory("weld_inspection")
    world_file = os.path.join(pkg_dir, "worlds", "weld_inspection.sdf")

    model_path_arg = DeclareLaunchArgument("model_path", default_value=str(Path.home() / "weld_model.onnx"))
    spawn_interval_arg = DeclareLaunchArgument("spawn_interval", default_value="12.0")
    belt_speed_arg = DeclareLaunchArgument("belt_speed", default_value="0.10")
    conf_arg = DeclareLaunchArgument("confidence_threshold", default_value="0.25")

    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", world_file, "-r"],
        output="screen",
        additional_env={"QT_QPA_PLATFORM": "xcb", "GZ_SIM_RESOURCE_PATH": os.path.join(pkg_dir, "models")},
    )

    bridge = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            "/weld_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/diverter/cmd@std_msgs/msg/Float64]gz.msgs.Double",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/robot_arm/waist_cmd@std_msgs/msg/Float64]gz.msgs.Double",
            "/robot_arm/elbow_cmd@std_msgs/msg/Float64]gz.msgs.Double",
            "/robot_arm/wrist_cmd@std_msgs/msg/Float64]gz.msgs.Double",
        ],
        output="screen",
    )

    inspector = Node(
        package="weld_inspection",
        executable="weld_inspector_node",
        name="weld_inspector",
        output="screen",
        parameters=[{
            "model_path": LaunchConfiguration("model_path"),
            "confidence_threshold": LaunchConfiguration("confidence_threshold"),
            "diverter_angle": -1.30,
            "diverter_reset_delay": 2.5,
            "debug_display": False,
        }],
    )

    spawner = Node(
        package="weld_inspection",
        executable="spawn_welds_node",
        name="spawn_welds",
        output="screen",
        parameters=[{
            "spawn_interval": LaunchConfiguration("spawn_interval"),
            "belt_speed": LaunchConfiguration("belt_speed"),
        }],
    )

    rviz_config = os.path.join(pkg_dir, "config", "weld_inspection.rviz")
    rviz = Node(package="rviz2", executable="rviz2", name="rviz2", arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [], output="screen")

    return LaunchDescription([
        model_path_arg, spawn_interval_arg, belt_speed_arg, conf_arg,
        gz_sim,
        TimerAction(period=10.0, actions=[bridge]),
        TimerAction(period=15.0, actions=[inspector, rviz]),
        TimerAction(period=18.0, actions=[spawner]),
    ])