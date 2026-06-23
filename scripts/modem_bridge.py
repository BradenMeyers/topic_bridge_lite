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

from network_bridge.msg import BridgeFrame, InterfaceStatus
from seatrac_interfaces.msg import ModemCmdUpdate, ModemRec, ModemSend

CID_DAT_SEND = 0x60
MSGTYPE_OWAY = 0
MAX_PACKET_LEN = 30
QUEUE_MAX = 8


class ModemTransportNode(Node):

    def __init__(self):
        super().__init__("modem_transport_node")

        self.declare_parameter("outbound_topic", "bridge_frame_out")
        self.declare_parameter("inbound_topic", "bridge_frame_in")
        self.declare_parameter("modem_send_topic", "modem_send")
        self.declare_parameter("modem_rec_topic", "modem_rec")
        self.declare_parameter("modem_cmd_update_topic", "modem_cmd_update")
        self.declare_parameter("status_topic", "interface_status")
        self.declare_parameter("window_ms", 100)

        outbound = self.get_parameter("outbound_topic").value
        inbound = self.get_parameter("inbound_topic").value
        modem_send_topic = self.get_parameter("modem_send_topic").value
        modem_rec_topic = self.get_parameter("modem_rec_topic").value
        modem_cmd_update_topic = self.get_parameter("modem_cmd_update_topic").value
        status_topic = self.get_parameter("status_topic").value
        self._win_ms = self.get_parameter("window_ms").value

        self._queue_depth = 0
        self._last_seq: dict[int, int] = {}
        self._gaps_window = 0
        self._bytes_sent_window = 0
        self._packets_sent_window = 0

        self._frame_sub = self.create_subscription(
            BridgeFrame, outbound, self._on_outbound_frame, 20
        )
        self._modem_send_pub = self.create_publisher(ModemSend, modem_send_topic, 10)

        self._modem_rec_sub = self.create_subscription(
            ModemRec, modem_rec_topic, self._on_modem_rec, 20
        )
        self._inbound_pub = self.create_publisher(BridgeFrame, inbound, 10)

        self._modem_cmd_update_sub = self.create_subscription(
            ModemCmdUpdate, modem_cmd_update_topic, self._on_modem_cmd_update, 10
        )
        self._status_pub = self.create_publisher(InterfaceStatus, status_topic, 10)

        self.create_timer(self._win_ms / 1000.0, self._publish_status)

        self.get_logger().info(
            f"ModemTransportNode: outbound='{outbound}' inbound='{inbound}' "
            f"modem_send='{modem_send_topic}' modem_rec='{modem_rec_topic}' "
            f"modem_cmd_update='{modem_cmd_update_topic}' status='{status_topic}'"
        )

    def _on_outbound_frame(self, msg: BridgeFrame) -> None:
        last = self._last_seq.get(msg.topic_id)
        if last is not None:
            expected = (last + 1) & 0xFF
            if msg.sequence != expected:
                gap = (msg.sequence - expected) & 0xFF
                self._gaps_window += gap
                self.get_logger().warning(
                    f"Seq gap on topic_id={msg.topic_id}: "
                    f"expected {expected} got {msg.sequence} ({gap} missing)"
                )
        self._last_seq[msg.topic_id] = msg.sequence

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
        self._bytes_sent_window += len(payload)
        self._packets_sent_window += 1

    def _on_modem_rec(self, msg: ModemRec) -> None:
        frame = BridgeFrame()
        frame.dst_addr = msg.dest_id
        frame.src_addr = msg.src_id
        frame.payload = list(msg.packet_data[: msg.packet_len])
        self._inbound_pub.publish(frame)

    def _on_modem_cmd_update(self, msg: ModemCmdUpdate) -> None:
        self._queue_depth = msg.queue_size

    def _compute_medium_state(self) -> int:
        fill = self._queue_depth / QUEUE_MAX
        if fill >= 0.8:
            return InterfaceStatus.OVERLOADED
        if fill >= 0.5:
            return InterfaceStatus.CONGESTED
        return InterfaceStatus.OK

    def _publish_status(self) -> None:
        gaps = self._gaps_window
        self._gaps_window = 0
        bytes_sent = self._bytes_sent_window
        self._bytes_sent_window = 0
        packets_sent = self._packets_sent_window
        self._packets_sent_window = 0

        status = InterfaceStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.queue_depth = min(self._queue_depth, 255)
        status.queue_max = QUEUE_MAX
        status.dropped_last_window = 0
        status.seq_gaps_last_window = min(gaps, 255)
        status.medium_state = self._compute_medium_state()
        status.bytes_sent_last_window = min(bytes_sent, 2**32 - 1)
        status.packets_sent_last_window = min(packets_sent, 2**32 - 1)
        status.window_ms = min(self._win_ms, 65535)
        self._status_pub.publish(status)


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
