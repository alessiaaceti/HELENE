import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Explicitly find the URDF inside the hw_description package
    urdf_path = os.path.join(
        get_package_share_directory("hw_description"),
        "urdf",
        "helene_hw.urdf.xacro"
    )

    # Build MoveIt configurations
    moveit_config = (
        MoveItConfigsBuilder("helene", package_name="helene_moveit_config")
        .robot_description(file_path=urdf_path) # Pass the absolute path here
        .to_moveit_configs()
    )

    # Fetch your servo parameter file
    servo_yaml = os.path.join(
        get_package_share_directory("controller_helene"),
        "config",
        "servo_params.yaml",
    )
    
    # Standalone MoveIt Servo Node
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        parameters=[
            servo_yaml,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    return LaunchDescription([servo_node])