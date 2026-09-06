"""Launch AMR robot description for visualization in RViz."""

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("amr_description")

    # Get XACRO file path (includes base_footprint for navigation)
    xacro_file = os.path.join(
        get_package_share_directory("amr_description"),
        "urdf", "amr.gazebo.xacro"
    )

    # Process xacro to generate URDF
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_content = doc.toxml()

    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Launch RViz with the robot model.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=[
                FindPackageShare("amr_description"),
                "/rviz/robot.rviz"
            ],
            description="Path to RViz configuration file.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time.",
        ),
        DeclareLaunchArgument(
            "publish_joints",
            default_value="true",
            description="Run joint_state_publisher for manual joint testing.",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "robot_description": robot_description_content,
            }],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            condition=IfCondition(LaunchConfiguration("publish_joints")),
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
    ])
