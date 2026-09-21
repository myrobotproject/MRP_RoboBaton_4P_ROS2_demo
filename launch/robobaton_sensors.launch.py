from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = PathJoinSubstitution(
        [FindPackageShare("robobaton_4p_ros2_demo"), "config", "robobaton_sensors.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="robobaton_4p_ros2_demo",
                executable="robobaton_sensors_node",
                name="robobaton_sensors_node",
                output="screen",
                parameters=[config_path],
            )
        ]
    )
