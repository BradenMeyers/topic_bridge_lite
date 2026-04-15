#include "network_bridge/transport_node.hpp"

#include <pluginlib/class_loader.hpp>

// Construction / destruction

TransportNode::TransportNode(const std::string & node_name)
: Node(node_name),
  loader_("network_bridge", "network_bridge::NetworkInterface") {}

TransportNode::~TransportNode()
{
  shutdown();
}

void TransportNode::initialize()
{
  load_parameters();
  load_network_interface();
  network_interface_->open();
}

void TransportNode::shutdown()
{
  RCLCPP_INFO(this->get_logger(), "TransportNode: shutting down");
  shutting_down_ = true;
  queue_cv_.notify_all();
  if (send_thread_.joinable()) {send_thread_.join();}

  if (network_interface_) {
    network_interface_->close();
  }
  network_interface_.reset();

  status_timer_.reset();
  health_timer_.reset();
  frame_sub_.reset();
  inbound_pub_.reset();
  status_pub_.reset();
}

// Parameter loading

void TransportNode::load_parameters()
{
  this->declare_parameter("network_interface", std::string("network_bridge::UdpInterface"));
  this->declare_parameter("queue_max", 32);
  this->declare_parameter("window_ms", 100);
  this->declare_parameter("outbound_topic", outbound_topic_);
  this->declare_parameter("inbound_topic", inbound_topic_);
  this->declare_parameter("status_topic", status_topic_);
  int queue_max_int, window_ms_int;
  this->get_parameter("network_interface", network_interface_name_);
  this->get_parameter("queue_max", queue_max_int);
  this->get_parameter("window_ms", window_ms_int);
  this->get_parameter("outbound_topic", outbound_topic_);
  this->get_parameter("inbound_topic", inbound_topic_);
  this->get_parameter("status_topic", status_topic_);

  queue_max_ = static_cast<uint8_t>(std::min(queue_max_int, 255));
  window_ms_ = static_cast<uint16_t>(std::clamp(window_ms_int, 1, 65535));

  // Outbound BridgeFrame subscriber (from encoder_node)
  frame_sub_ = this->create_subscription<network_bridge::msg::BridgeFrame>(
    outbound_topic_, 20,
    [this](const network_bridge::msg::BridgeFrame::SharedPtr msg) {
      on_outbound_frame(msg);
    });

  // Inbound BridgeFrame publisher (radio → ROS2)
  rclcpp::QoS inbound_qos(10);
  inbound_pub_ =
    this->create_publisher<network_bridge::msg::BridgeFrame>(inbound_topic_, inbound_qos);

  // InterfaceStatus publisher
  rclcpp::QoS status_qos(10);
  status_pub_ =
    this->create_publisher<network_bridge::msg::InterfaceStatus>(status_topic_, status_qos);

  // 10 Hz status timer (window_ms controls the accounting window)
  status_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    [this]() {publish_status();});

  // 500 ms health check (same cadence as original NetworkBridge)
  health_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    [this]() {check_network_health();});

  RCLCPP_INFO(
    this->get_logger(),
    "TransportNode: outbound='%s' inbound='%s' status='%s' queue_max=%u window_ms=%u",
    outbound_topic_.c_str(), inbound_topic_.c_str(), status_topic_.c_str(),
    queue_max_, window_ms_);
}

// Network interface

void TransportNode::load_network_interface()
{
  try {
    network_interface_ = loader_.createSharedInstance(network_interface_name_);
    network_interface_->initialize(
      shared_from_this(),
      [this](std::span<const uint8_t> data) {on_radio_receive(data);});

    RCLCPP_INFO(
      this->get_logger(),
      "Loaded network interface: %s", network_interface_name_.c_str());
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      this->get_logger(),
      "Failed to load network interface: %s", ex.what());
    rclcpp::shutdown();
    exit(1);
  }

  // Start send thread after interface is initialised
  send_thread_ = std::thread([this]() {send_thread_fn();});
}

void TransportNode::check_network_health()
{
  if (!network_interface_) {return;}
  if (network_interface_->has_failed()) {
    RCLCPP_INFO(this->get_logger(), "Network interface failed — reopening");
    network_interface_->close();
    network_interface_->open();
  }
}

// Send path (ROS callback → queue → send thread → radio)

void TransportNode::on_outbound_frame(const network_bridge::msg::BridgeFrame::SharedPtr msg)
{
  auto it = rx_last_seq_.find(msg->topic_id);
  if (it != rx_last_seq_.end()) {
    uint8_t expected = static_cast<uint8_t>(it->second + 1);
    if (msg->sequence != expected) {
      uint32_t gap = static_cast<uint8_t>(msg->sequence - expected);
      gaps_window_.fetch_add(gap, std::memory_order_relaxed);
      RCLCPP_WARN(
        this->get_logger(),
        "Seq gap on topic_id=%u: expected %u got %u (%u missing)",
        msg->topic_id, expected, msg->sequence, gap);
    }
  }
  rx_last_seq_[msg->topic_id] = msg->sequence;

  std::lock_guard<std::mutex> lock(queue_mutex_);
  if (send_queue_.size() >= queue_max_) {
    dropped_window_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_DEBUG(
      this->get_logger(),
      "Send queue full (max %u) — dropping frame seq=%u",
      queue_max_, msg->sequence);
    return;
  }
  send_queue_.push_back(msg->payload);
  queue_cv_.notify_one();
}

void TransportNode::send_thread_fn()
{
  while (!shutting_down_.load(std::memory_order_relaxed)) {
    std::vector<uint8_t> payload;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      // Wait up to 10 ms; re-check is_ready so we don't spin-busy when TCP
      // is not yet connected.
      queue_cv_.wait_for(lock, std::chrono::milliseconds(10), [this] {
        return (!send_queue_.empty() &&
               network_interface_ &&
               network_interface_->is_ready()) ||
               shutting_down_.load(std::memory_order_relaxed);
      });

      if (shutting_down_.load(std::memory_order_relaxed)) {break;}
      if (send_queue_.empty() || !network_interface_ || !network_interface_->is_ready()) {
        continue;
      }

      payload = std::move(send_queue_.front());
      send_queue_.pop_front();
    }

    network_interface_->write(payload);
    bytes_sent_window_.fetch_add(
      static_cast<uint32_t>(payload.size()), std::memory_order_relaxed);
  }
}

// Receive path (radio → ROS2 inbound topic)

void TransportNode::on_radio_receive(std::span<const uint8_t> data)
{
  if (!rclcpp::ok() || data.empty()) {return;}

  network_bridge::msg::BridgeFrame frame;
  frame.payload.assign(data.begin(), data.end());
  inbound_pub_->publish(frame);
}

// Status publisher

uint8_t TransportNode::compute_medium_state() const
{
  if (!network_interface_ || !network_interface_->is_ready()) {return 0;}

  size_t depth;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    depth = send_queue_.size();
  }
  float fill = static_cast<float>(depth) / static_cast<float>(queue_max_);
  if (fill >= 0.8f) {return 3;}  // overloaded
  if (fill >= 0.5f) {return 2;}  // congested
  return 1;                       // ok
}

void TransportNode::publish_status()
{
  size_t depth;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    depth = send_queue_.size();
  }

  network_bridge::msg::InterfaceStatus msg;
  msg.header.stamp = this->now();
  msg.queue_depth = static_cast<uint8_t>(std::min(depth, size_t(255)));
  msg.queue_max = queue_max_;
  msg.dropped_last_window  = dropped_window_.exchange(0, std::memory_order_relaxed);
  msg.seq_gaps_last_window = static_cast<uint8_t>(
    std::min(gaps_window_.exchange(0, std::memory_order_relaxed), 255u));
  msg.medium_state = compute_medium_state();
  msg.bytes_sent_last_window = bytes_sent_window_.exchange(0, std::memory_order_relaxed);
  msg.window_ms = window_ms_;

  status_pub_->publish(msg);
}

// Main

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::string node_name = "transport_node_" + std::to_string(::getpid());

  auto node = std::make_shared<TransportNode>(node_name);
  node->initialize();

  rclcpp::spin(node);

  node->shutdown();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
