from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("network_bridge")
    params_file = share + "/config/udp_split.yaml"
    topics1_file = share + "/config/udp1_topics.yaml"
    topics2_file = share + "/config/udp2_topics.yaml"

    return LaunchDescription(
        [
            # udp1 — encoder decodes inbound and encodes outbound TX topics
            Node(
                package="network_bridge",
                executable="encoder_node",
                name="encoder_node",
                namespace="udp1",
                output="screen",
                parameters=[params_file, {"config_file": topics1_file}],
            ),
            Node(
                package="network_bridge",
                executable="transport_node",
                name="transport_node",
                namespace="udp1",
                output="screen",
                parameters=[params_file],
            ),
            # udp2 — mirror: encoder decodes inbound and encodes outbound TX topics
            Node(
                package="network_bridge",
                executable="encoder_node",
                name="encoder_node",
                namespace="udp2",
                output="screen",
                parameters=[params_file, {"config_file": topics2_file}],
            ),
            Node(
                package="network_bridge",
                executable="transport_node",
                name="transport_node",
                namespace="udp2",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
