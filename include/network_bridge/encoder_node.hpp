#pragma once

#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "network_bridge/subscription_manager.hpp"
#include "network_bridge/msg/bridge_frame.hpp"
#include "network_bridge/msg/interface_status.hpp"

namespace network_bridge
{

enum class Priority : uint8_t { LOW = 0, MEDIUM = 1, HIGH = 2 };
enum class DropPolicy { FIFO, LATEST };

struct TopicConfig
{
  uint8_t id{0};
  std::string mode;         // "tx" or "rx"
  std::string input;        // ROS2 topic to subscribe (tx)
  std::string output;       // ROS2 topic to publish (rx)
  std::string type;         // ROS2 message type string
  float rate{5.0f};
  Priority priority{Priority::MEDIUM};
  DropPolicy drop_policy{DropPolicy::LATEST};
  int queue_max{8};
  int zstd_level{3};
  bool publish_stale_data{false};
  bool is_tf{false};
  bool is_static_tf{false};
  std::vector<std::string> tf_include;
  std::vector<std::string> tf_exclude;
  uint8_t dst_addr{0x00};  // TX: destination address (0x00 = unset)
  uint8_t src_addr{0x00};  // RX: expected source address for registry lookup
};

struct TxQueue
{
  TopicConfig config;
  std::shared_ptr<SubscriptionManager> sub_mgr;
  std::deque<std::vector<uint8_t>> queue;
};

struct RxEntry
{
  TopicConfig config;
  rclcpp::GenericPublisher::SharedPtr publisher;
};

}  // namespace network_bridge

class EncoderNode : public rclcpp::Node
{
public:
  explicit EncoderNode(const std::string & node_name);
  ~EncoderNode() override;

  virtual void initialize();
  virtual void shutdown();

protected:
  virtual void load_parameters();
  virtual void load_topic_config(const std::string & path);

  virtual void encode_and_publish(network_bridge::TxQueue & tq);
  virtual void on_inbound_frame(const network_bridge::msg::BridgeFrame::SharedPtr msg);
  virtual void on_interface_status(const network_bridge::msg::InterfaceStatus::SharedPtr msg);

  virtual void compress(
    const std::vector<uint8_t> & data, std::vector<uint8_t> & out, int level = 3);
  virtual void decompress(
    std::span<const uint8_t> data, std::vector<uint8_t> & out);

  network_bridge::Priority min_publishable_priority() const;

  std::vector<network_bridge::TxQueue> tx_queues_;

  // Keyed by (src_addr, topic_id) so the same topic_id can be reused
  // across different source addresses.
  std::map<std::pair<uint8_t, uint8_t>, network_bridge::RxEntry> rx_registry_;

  std::vector<rclcpp::TimerBase::SharedPtr> timers_;

  rclcpp::Publisher<network_bridge::msg::BridgeFrame>::SharedPtr bridge_frame_pub_;
  rclcpp::Subscription<network_bridge::msg::BridgeFrame>::SharedPtr inbound_sub_;
  rclcpp::Subscription<network_bridge::msg::InterfaceStatus>::SharedPtr status_sub_;

  uint8_t sequence_{0};
  uint8_t medium_state_{1};

  bool use_addressing_{false};
  uint8_t src_addr_{0x00};   // this node's own address on the link

  std::string outbound_topic_{"bridge_frame_out"};
  std::string inbound_topic_{"bridge_frame_in"};
  std::string status_topic_{"interface_status"};
};
