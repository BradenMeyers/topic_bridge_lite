#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>

#include "network_interfaces/network_interface_base.hpp"
#include "network_bridge/msg/bridge_frame.hpp"
#include "network_bridge/msg/interface_status.hpp"

/**
 * @class TransportNode
 * @brief Owns the NetworkInterface plugin and shuttles BridgeFrame payloads
 *        to/from the radio.
 *
 * Responsibilities:
 *  - Subscribe to BridgeFrame messages from the encoder node and write their
 *    payloads to the network interface via a dedicated send thread.
 *  - Forward bytes received from the radio back onto an inbound BridgeFrame
 *    topic for any consumer (future use).
 *  - Publish InterfaceStatus at 10 Hz so the encoder node can gate traffic.
 *
 * This node is intentionally minimal — all encoding intelligence lives in the
 * encoder node.
 */
class TransportNode : public rclcpp::Node
{
public:
  explicit TransportNode(const std::string & node_name);
  ~TransportNode() override;

  void initialize();
  void shutdown();

private:
  void load_parameters();
  void load_network_interface();

  /** Called by the encoder-facing BridgeFrame subscriber. */
  void on_outbound_frame(const network_bridge::msg::BridgeFrame::SharedPtr msg);

  /** Called from the NetworkInterface receive callback (io_context thread). */
  void on_radio_receive(std::span<const uint8_t> data);

  /** Publish InterfaceStatus and reset window counters. */
  void publish_status();

  /** Reopen the interface if it has failed. */
  void check_network_health();

  /** Compute medium_state from current queue fill level. */
  uint8_t compute_medium_state() const;

  /** Dedicated thread: pops from send_queue_ and calls network_interface_->write(). */
  void send_thread_fn();

  // ── pluginlib ─────────────────────────────────────────────────────────────
  pluginlib::ClassLoader<network_bridge::NetworkInterface> loader_;
  std::shared_ptr<network_bridge::NetworkInterface> network_interface_;
  std::string network_interface_name_;

  // ── ROS2 I/O ──────────────────────────────────────────────────────────────
  rclcpp::Subscription<network_bridge::msg::BridgeFrame>::SharedPtr frame_sub_;
  rclcpp::Publisher<network_bridge::msg::BridgeFrame>::SharedPtr inbound_pub_;
  rclcpp::Publisher<network_bridge::msg::InterfaceStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;

  std::string outbound_topic_{"bridge_frame_out"};
  std::string inbound_topic_{"bridge_frame_in"};
  std::string status_topic_{"interface_status"};

  // ── bounded send queue ────────────────────────────────────────────────────
  mutable std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<std::vector<uint8_t>> send_queue_;
  uint8_t queue_max_{32};

  // ── per-window stats (written from multiple threads) ──────────────────────
  std::atomic<uint32_t> bytes_sent_window_{0};
  std::atomic<uint8_t> dropped_window_{0};
  uint16_t window_ms_{100};

  // ── send thread ───────────────────────────────────────────────────────────
  std::thread send_thread_;
  std::atomic<bool> shutting_down_{false};
};
