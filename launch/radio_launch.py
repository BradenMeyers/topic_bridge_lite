from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    output = "screen"
    pkg_name = "network_bridge"
    share = get_package_share_directory(pkg_name)

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=share + "/config/radio.yaml",
        description="Full path to the ROS2 parameters file for the radio nodes",
    )
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="radio1",
        description="Namespace for the radio encoder/transport node pair",
    )

    params_file = LaunchConfiguration("params_file")
    namespace = LaunchConfiguration("namespace")

    return LaunchDescription(
        [
            params_file_arg,
            namespace_arg,
            Node(
                package=pkg_name,
                executable="encoder_node",
                name="encoder_node",
                namespace=namespace,
                output=output,
                parameters=[params_file],
            ),
            Node(
                package=pkg_name,
                executable="radio_bridge.py",
                name="radio_transport_node",
                namespace=namespace,
                output=output,
                parameters=[params_file],
            ),
        ]
    )
