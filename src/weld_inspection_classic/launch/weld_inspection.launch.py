import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory("weld_inspection_classic")
    world_file = os.path.join(pkg_dir, "worlds", "weld_inspection.world")
    rviz_config = os.path.join(pkg_dir, "config", "weld_inspection.rviz")

    # Path to YOLO model
    model_path = os.path.expanduser("~/Downloads/weld_model.onnx")

    # Launch Gazebo Classic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    inspector = Node(
        package="weld_inspection_classic",
        executable="weld_inspector_node",
        name="weld_inspector",
        output="screen",
        parameters=[{
            "model_path": model_path,
            "confidence_threshold": 0.25,
        }],
    )

    orchestrator = Node(
        package="weld_inspection_classic",
        executable="orchestrator_node",
        name="orchestrator",
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        inspector,
        orchestrator,
        #rviz
    ])