#!/usr/bin/env python3

"""
radio_bridge.py

Python transport node for XBee radio hardware.
Mirrors transport_node.cpp — subscribes to BridgeFrame on outbound_topic,
writes payload bytes to the radio; on radio receive, publishes a BridgeFrame
on inbound_topic with the raw bytes as payload.

Encoding, compression, and topic routing are all handled by the encoder node.
This node is a pure byte passthrough between ROS2 and the XBee radio.
"""

import collections
import threading

import rclpy
from rclpy.node import Node

from network_bridge.msg import BridgeFrame, InterfaceStatus
from radio_manager import XBeeRadioDevice


class RadioTransportNode(Node):

    def __init__(self):
        super().__init__("radio_transport_node")

        self.declare_parameter("xbee_port", "/dev/ttyUSB0")
        self.declare_parameter("xbee_baud", 9600)
        self.declare_parameter("xbee_dest_address", "")
        self.declare_parameter("queue_max", 32)
        self.declare_parameter("window_ms", 100)
        self.declare_parameter("outbound_topic", "bridge_frame_out")
        self.declare_parameter("inbound_topic", "bridge_frame_in")
        self.declare_parameter("status_topic", "interface_status")

        port          = self.get_parameter("xbee_port").get_parameter_value().string_value
        baud          = self.get_parameter("xbee_baud").get_parameter_value().integer_value
        dest          = self.get_parameter("xbee_dest_address").get_parameter_value().string_value or None
        self._q_max   = self.get_parameter("queue_max").get_parameter_value().integer_value
        self._win_ms  = self.get_parameter("window_ms").get_parameter_value().integer_value
        outbound      = self.get_parameter("outbound_topic").get_parameter_value().string_value
        inbound       = self.get_parameter("inbound_topic").get_parameter_value().string_value
        status        = self.get_parameter("status_topic").get_parameter_value().string_value

        self._radio = XBeeRadioDevice(port, baud, dest_address=dest, logger=self.get_logger())
        self._radio.set_receive_callback(self._on_radio_receive)

        self._send_queue: collections.deque = collections.deque()
        self._cv = threading.Condition()
        self._shutting_down = False

        self._dropped_window = 0
        self._gaps_window = 0
        self._bytes_sent_window = 0
        self._packets_sent_window = 0
        self._stats_lock = threading.Lock()

        self._last_seq: dict[int, int] = {}

        self._frame_sub = self.create_subscription(
            BridgeFrame, outbound, self._on_outbound_frame, 20
        )
        self._inbound_pub = self.create_publisher(BridgeFrame, inbound, 10)
        self._status_pub  = self.create_publisher(InterfaceStatus, status, 10)

        self.create_timer(0.1,  self._publish_status)
        self.create_timer(0.5,  self._check_health)

        self._send_thread = threading.Thread(target=self._send_thread_fn, daemon=True)
        self._send_thread.start()

        self._radio.open()

        self.get_logger().info(
            f"RadioTransportNode: outbound='{outbound}' inbound='{inbound}' "
            f"status='{status}' port={port} baud={baud} "
            f"dest={dest or 'broadcast'} queue_max={self._q_max}"
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
                    lambda: (bool(self._send_queue) and self._radio.is_ready())
                            or self._shutting_down,
                    timeout=0.01,
                )
                if self._shutting_down:
                    break
                if not self._send_queue or not self._radio.is_ready():
                    continue
                payload = self._send_queue.popleft()

            self._radio.write(payload)
            with self._stats_lock:
                self._bytes_sent_window += len(payload)
                self._packets_sent_window += 1

    def _on_radio_receive(self, data: bytes) -> None:
        if not rclpy.ok() or not data:
            return
        frame = BridgeFrame()
        frame.payload = list(data)
        self._inbound_pub.publish(frame)

    def _check_health(self) -> None:
        if self._radio.has_failed():
            self.get_logger().info("Radio failed — reopening")
            self._radio.close()
            self._radio.open()

    def _compute_medium_state(self) -> int:
        if not self._radio.is_ready():
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
            packets_sent = self._packets_sent_window
            self._packets_sent_window = 0

        msg = InterfaceStatus()
        msg.header.stamp              = self.get_clock().now().to_msg()
        msg.queue_depth               = min(depth, 255)
        msg.queue_max                 = min(self._q_max, 255)
        msg.dropped_last_window       = min(dropped, 255)
        msg.seq_gaps_last_window      = min(gaps, 255)
        msg.medium_state              = self._compute_medium_state()
        msg.bytes_sent_last_window    = min(bytes_sent, 2**32 - 1)
        msg.packets_sent_last_window  = min(packets_sent, 2**32 - 1)
        msg.window_ms                 = min(self._win_ms, 65535)
        self._status_pub.publish(msg)

    def destroy_node(self) -> None:
        self._shutting_down = True
        with self._cv:
            self._cv.notify_all()
        self._send_thread.join(timeout=2.0)
        self._radio.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RadioTransportNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
