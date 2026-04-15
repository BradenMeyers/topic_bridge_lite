#!/usr/bin/env python3

"""
bluetooth_bridge.py

Python transport node for Bluetooth RFCOMM.
Mirrors radio_bridge.py — subscribes to BridgeFrame on outbound_topic,
writes payload bytes over Bluetooth; on receive, publishes a BridgeFrame
on inbound_topic.

Encoding, compression, and topic routing are handled by the encoder node.
This node is a pure byte passthrough between ROS2 and the Bluetooth socket.

Parameters
----------
bt_role         : 'server' or 'client'
bt_mac_address  : remote MAC address (client mode only, e.g. "AA:BB:CC:DD:EE:FF")
bt_channel      : RFCOMM channel number (default 1)
queue_max       : max outbound frames to buffer (default 32)
window_ms       : stats accounting window in ms (default 100)
outbound_topic  : BridgeFrame topic to consume (default 'bridge_frame_out')
inbound_topic   : BridgeFrame topic to publish (default 'bridge_frame_in')
status_topic    : InterfaceStatus topic to publish (default 'interface_status')
"""

import collections
import threading

import rclpy
from rclpy.node import Node

from network_bridge.msg import BridgeFrame, InterfaceStatus
from bluetooth_manager import BluetoothDevice


class BluetoothTransportNode(Node):

    def __init__(self):
        super().__init__("bluetooth_transport_node")

        self.declare_parameter("bt_role", "server")
        self.declare_parameter("bt_mac_address", "")
        self.declare_parameter("bt_channel", 1)
        self.declare_parameter("queue_max", 32)
        self.declare_parameter("window_ms", 100)
        self.declare_parameter("outbound_topic", "bridge_frame_out")
        self.declare_parameter("inbound_topic", "bridge_frame_in")
        self.declare_parameter("status_topic", "interface_status")

        role     = self.get_parameter("bt_role").get_parameter_value().string_value
        mac      = self.get_parameter("bt_mac_address").get_parameter_value().string_value
        channel  = self.get_parameter("bt_channel").get_parameter_value().integer_value
        self._q_max  = self.get_parameter("queue_max").get_parameter_value().integer_value
        self._win_ms = self.get_parameter("window_ms").get_parameter_value().integer_value
        outbound = self.get_parameter("outbound_topic").get_parameter_value().string_value
        inbound  = self.get_parameter("inbound_topic").get_parameter_value().string_value
        status   = self.get_parameter("status_topic").get_parameter_value().string_value

        self._bt = BluetoothDevice(
            role=role,
            mac_address=mac or "",
            channel=channel,
            logger=self.get_logger(),
        )
        self._bt.set_receive_callback(self._on_bt_receive)

        self._send_queue: collections.deque = collections.deque()
        self._cv = threading.Condition()
        self._shutting_down = False

        self._dropped_window = 0
        self._gaps_window = 0
        self._bytes_sent_window = 0
        self._stats_lock = threading.Lock()

        self._last_seq: dict = {}

        self._frame_sub = self.create_subscription(
            BridgeFrame, outbound, self._on_outbound_frame, 20
        )
        self._inbound_pub = self.create_publisher(BridgeFrame, inbound, 10)
        self._status_pub  = self.create_publisher(InterfaceStatus, status, 10)

        self.create_timer(0.1, self._publish_status)
        self.create_timer(0.5, self._check_health)

        self._send_thread = threading.Thread(
            target=self._send_thread_fn, daemon=True, name="bt_send"
        )
        self._send_thread.start()

        self._bt.open()

        self.get_logger().info(
            f"BluetoothTransportNode: role={role} mac={mac or 'any'} "
            f"channel={channel} outbound='{outbound}' inbound='{inbound}' "
            f"status='{status}' queue_max={self._q_max}"
        )

    def _on_outbound_frame(self, msg: BridgeFrame) -> None:
        last = self._last_seq.get(msg.topic_id)
        if last is not None:
            expected = (last + 1) & 0xFF
            if msg.sequence != expected:
                gap = (msg.sequence - expected) & 0xFF
                with self._stats_lock:
                    self._gaps_window += gap
                self.get_logger().warning(
                    f"Seq gap on topic_id={msg.topic_id}: "
                    f"expected {expected} got {msg.sequence} ({gap} missing)"
                )
        self._last_seq[msg.topic_id] = msg.sequence

        with self._cv:
            if len(self._send_queue) >= self._q_max:
                with self._stats_lock:
                    self._dropped_window += 1
                self.get_logger().debug(
                    f"Send queue full (max {self._q_max}) — dropping frame seq={msg.sequence}"
                )
                return
            self._send_queue.append(bytes(msg.payload))
            self._cv.notify()

    def _send_thread_fn(self) -> None:
        while not self._shutting_down:
            with self._cv:
                self._cv.wait_for(
                    lambda: (bool(self._send_queue) and self._bt.is_ready())
                            or self._shutting_down,
                    timeout=0.05,
                )
                if self._shutting_down:
                    break
                if not self._send_queue or not self._bt.is_ready():
                    continue
                payload = self._send_queue.popleft()

            self._bt.write(payload)
            with self._stats_lock:
                self._bytes_sent_window += len(payload)

    def _on_bt_receive(self, data: bytes) -> None:
        if not rclpy.ok() or not data:
            return
        frame = BridgeFrame()
        frame.payload = list(data)
        self._inbound_pub.publish(frame)

    def _check_health(self) -> None:
        if self._bt.has_failed():
            self.get_logger().info("Bluetooth failed — reopening")
            self._bt.close()
            self._bt.open()

    def _compute_medium_state(self) -> int:
        if not self._bt.is_ready():
            return InterfaceStatus.DISCONNECTED
        with self._cv:
            depth = len(self._send_queue)
        fill = depth / max(self._q_max, 1)
        if fill >= 0.8:
            return InterfaceStatus.OVERLOADED
        if fill >= 0.5:
            return InterfaceStatus.CONGESTED
        return InterfaceStatus.OK

    def _publish_status(self) -> None:
        with self._cv:
            depth = len(self._send_queue)
        with self._stats_lock:
            dropped = self._dropped_window
            self._dropped_window = 0
            gaps = self._gaps_window
            self._gaps_window = 0
            bytes_sent = self._bytes_sent_window
            self._bytes_sent_window = 0

        msg = InterfaceStatus()
        msg.header.stamp         = self.get_clock().now().to_msg()
        msg.queue_depth          = min(depth, 255)
        msg.queue_max            = min(self._q_max, 255)
        msg.dropped_last_window  = min(dropped, 255)
        msg.seq_gaps_last_window = min(gaps, 255)
        msg.medium_state         = self._compute_medium_state()
        msg.bytes_sent_last_window = min(bytes_sent, 2**32 - 1)
        msg.window_ms            = min(self._win_ms, 65535)
        self._status_pub.publish(msg)

    def destroy_node(self) -> None:
        self._shutting_down = True
        with self._cv:
            self._cv.notify_all()
        self._send_thread.join(timeout=2.0)
        self._bt.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BluetoothTransportNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
