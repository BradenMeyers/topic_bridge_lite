import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("network_bridge")
    params_file  = os.path.join(share, "config", "bluetooth.yaml")
    topics_file  = os.path.join(share, "config", "bluetooth_topics.yaml")
    scripts_dir  = os.path.join(share, "scripts")

    return LaunchDescription(
        [
            Node(
                package="network_bridge",
                executable="encoder_node",
                name="encoder_node",
                namespace="bt",
                output="screen",
                parameters=[params_file, {"config_file": topics_file}],
            ),
            Node(
                package="network_bridge",
                executable="bluetooth_bridge.py",
                name="bluetooth_transport_node",
                namespace="bt",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
