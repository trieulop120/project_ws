from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("amr_description")

    urdf_path = PathJoinSubstitution(
        [package_share, "urdf", "robots", "complete_robot_AMR.urdf.xacro"]
    )
    rviz_config_default = PathJoinSubstitution(
        [package_share, "rviz", "display.rviz"]
    )

    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("urdf"),
            " robot_name:=",
            LaunchConfiguration("robot_name"),
            " sim:=",
            LaunchConfiguration("sim"),
            " base_profile:=",
            LaunchConfiguration("base_profile"),
            " sensor_profile:=",
            LaunchConfiguration("sensor_profile"),
            " lift_profile:=",
            LaunchConfiguration("lift_profile"),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="amr"),
        DeclareLaunchArgument("urdf", default_value=urdf_path),
        DeclareLaunchArgument("base_profile", default_value="",
            description="Reserved compatibility argument for future base variants."),
        DeclareLaunchArgument("sensor_profile", default_value="",
            description="Reserved compatibility argument (LiDAR/camera variants)."),
        DeclareLaunchArgument("lift_profile", default_value="",
            description="Reserved compatibility argument for lift mechanism variants."),
        DeclareLaunchArgument("publish_joints", default_value="true",
            description="Run joint_state_publisher for manual wheel and lift testing."),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("rviz_config", default_value=rviz_config_default),
        DeclareLaunchArgument("rviz_software_rendering", default_value="0"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("sim", default_value="false",
            description="Enable Gazebo simulation plugins in the robot description."),

        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            condition=IfCondition(LaunchConfiguration("publish_joints")),
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "robot_description": robot_description,
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            additional_env={
                "LIBGL_ALWAYS_SOFTWARE": LaunchConfiguration("rviz_software_rendering"),
            },
        ),
    ])