#!/usr/bin/env python3

"""
modem_bridge.py

Python transport node for the Seatrac acoustic modem.
Mirrors radio_bridge.py / bluetooth_bridge.py — subscribes to BridgeFrame on
outbound_topic, but instead of writing raw bytes to a network interface it
converts each frame into a ModemSend message (CID_DAT_SEND, one-way) and
publishes it for the modem driver node to transmit.

Encoding, compression, and topic routing are all handled by the encoder node.
This node only translates BridgeFrame payloads into the modem's data-send
command.
"""

import rclpy
from rclpy.node import Node

from network_bridge.msg import BridgeFrame
from seatrac_interfaces.msg import ModemSend

CID_DAT_SEND = 0x60
MSGTYPE_OWAY = 0
MAX_PACKET_LEN = 30


class ModemTransportNode(Node):

    def __init__(self):
        super().__init__("modem_transport_node")

        self.declare_parameter("outbound_topic", "bridge_frame_out")
        self.declare_parameter("modem_send_topic", "modem_send")

        outbound = self.get_parameter("outbound_topic").get_parameter_value().string_value
        modem_send_topic = self.get_parameter("modem_send_topic").get_parameter_value().string_value

        self._frame_sub = self.create_subscription(
            BridgeFrame, outbound, self._on_outbound_frame, 20
        )
        self._modem_send_pub = self.create_publisher(ModemSend, modem_send_topic, 10)

        self.get_logger().info(
            f"ModemTransportNode: outbound='{outbound}' modem_send='{modem_send_topic}'"
        )

    def _on_outbound_frame(self, msg: BridgeFrame) -> None:
        payload = bytes(msg.payload)

        if len(payload) > MAX_PACKET_LEN:
            self.get_logger().error(
                f"Frame payload of {len(payload)} bytes exceeds modem packet limit "
                f"of {MAX_PACKET_LEN} bytes — discarding"
            )
            return

        send = ModemSend()
        send.header.stamp = self.get_clock().now().to_msg()
        send.msg_id = CID_DAT_SEND
        send.dest_id = msg.dst_addr
        send.msg_type = MSGTYPE_OWAY
        send.packet_len = len(payload)
        send.packet_data = list(payload) 

        self._modem_send_pub.publish(send)


def main(args=None):
    rclpy.init(args=args)
    node = ModemTransportNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
