import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml
import subprocess

def generate_launch_description():
    # Trova le directory dei pacchetti dinamicamente
    hw_desc_pkg = get_package_share_directory('hw_description')
    moveit_cfg_pkg = get_package_share_directory('helene_moveit_config')
    controller_pkg = get_package_share_directory('controller_helene')

    xacro_path = os.path.join(hw_desc_pkg, 'urdf', 'helene_hw.urdf.xacro')
    srdf_path = os.path.join(moveit_cfg_pkg, 'config', 'helene.srdf')
    kinematics_path = os.path.join(moveit_cfg_pkg, 'config', 'kinematics.yaml')
    servo_yaml_path = os.path.join(controller_pkg, 'config', 'servo_params.yaml')
    
    # Xacro
    robot_description_config = subprocess.check_output(['xacro', xacro_path]).decode('utf-8')
    robot_description = {"robot_description": robot_description_config}

    # SRDF
    with open(srdf_path, 'r') as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    # Kinematics
    with open(kinematics_path, 'r') as file:
        kinematics_yaml = yaml.safe_load(file)
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            servo_yaml_path,
            {'use_sim_time': False} 
        ]
    )

    return LaunchDescription([servo_node])
