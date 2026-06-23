#!/usr/bin/env python3

"""
modem_bridge.py

Python transport node for the Seatrac acoustic modem.
Mirrors radio_bridge.py / bluetooth_bridge.py — subscribes to BridgeFrame on
outbound_topic, but instead of writing raw bytes to a network interface it
converts each frame into a ModemSend message (CID_DAT_SEND, one-way) and
publishes it for the modem driver node to transmit. On the inbound side, it
takes ModemRec reports from the modem driver and forwards the packet data as
a BridgeFrame for the encoder node to decode.

Encoding, compression, and topic routing are all handled by the encoder node.
This node only translates between BridgeFrame payloads and the modem's
data-send/data-receive messages.
"""

import rclpy
from rclpy.node import Node

from network_bridge.msg import BridgeFrame
from seatrac_interfaces.msg import ModemRec, ModemSend

CID_DAT_SEND = 0x60
MSGTYPE_OWAY = 0
MAX_PACKET_LEN = 30


class ModemTransportNode(Node):

    def __init__(self):
        super().__init__("modem_transport_node")

        self.declare_parameter("outbound_topic", "bridge_frame_out")
        self.declare_parameter("inbound_topic", "bridge_frame_in")
        self.declare_parameter("modem_send_topic", "modem_send")
        self.declare_parameter("modem_rec_topic", "modem_rec")

        outbound = self.get_parameter("outbound_topic").get_parameter_value().string_value
        inbound = self.get_parameter("inbound_topic").get_parameter_value().string_value
        modem_send_topic = self.get_parameter("modem_send_topic").get_parameter_value().string_value
        modem_rec_topic = self.get_parameter("modem_rec_topic").get_parameter_value().string_value

        self._frame_sub = self.create_subscription(
            BridgeFrame, outbound, self._on_outbound_frame, 20
        )
        self._modem_send_pub = self.create_publisher(ModemSend, modem_send_topic, 10)

        self._modem_rec_sub = self.create_subscription(
            ModemRec, modem_rec_topic, self._on_modem_rec, 20
        )
        self._inbound_pub = self.create_publisher(BridgeFrame, inbound, 10)

        self.get_logger().info(
            f"ModemTransportNode: outbound='{outbound}' inbound='{inbound}' "
            f"modem_send='{modem_send_topic}' modem_rec='{modem_rec_topic}'"
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

    def _on_modem_rec(self, msg: ModemRec) -> None:
        frame = BridgeFrame()
        frame.dst_addr = msg.dest_id
        frame.src_addr = msg.src_id
        frame.payload = list(msg.packet_data[: msg.packet_len])
        self._inbound_pub.publish(frame)


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
