#include "network_bridge/encoder_node.hpp"

#include <zstd.h>
#include <span>

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "network_bridge/subscription_manager_tf.hpp"

EncoderNode::EncoderNode(const std::string & node_name)
: Node(node_name) {}

EncoderNode::~EncoderNode()
{
  shutdown();
}

void EncoderNode::initialize()
{
  load_parameters();
}

void EncoderNode::shutdown()
{
  RCLCPP_INFO(this->get_logger(), "EncoderNode: shutting down");
  timers_.clear();
  tx_queues_.clear();
  rx_registry_.clear();
  bridge_frame_pub_.reset();
  bridge_status_pub_.reset();
  inbound_sub_.reset();
  status_sub_.reset();
}

void EncoderNode::load_parameters()
{
  this->declare_parameter("outbound_topic", outbound_topic_);
  this->declare_parameter("inbound_topic", inbound_topic_);
  this->declare_parameter("status_topic", status_topic_);
  this->declare_parameter("bridge_status_topic", bridge_status_topic_);
  this->declare_parameter("throughput_alpha", throughput_alpha_);
  this->declare_parameter("config_file", std::string(""));
  this->declare_parameter("config_file_absolute", false);
  this->declare_parameter("use_addressing", false);

  this->get_parameter("outbound_topic", outbound_topic_);
  this->get_parameter("inbound_topic", inbound_topic_);
  this->get_parameter("status_topic", status_topic_);
  this->get_parameter("bridge_status_topic", bridge_status_topic_);
  this->get_parameter("throughput_alpha", throughput_alpha_);
  this->get_parameter("use_addressing", use_addressing_);

  std::string config_file;
  bool config_file_absolute;
  this->get_parameter("config_file", config_file);
  this->get_parameter("config_file_absolute", config_file_absolute);

  if (!config_file.empty() && !config_file_absolute) {
    config_file = ament_index_cpp::get_package_share_directory("network_bridge") +
      "/" + config_file;
  }

  bridge_frame_pub_ =
    this->create_publisher<network_bridge::msg::BridgeFrame>(outbound_topic_, 10);
  bridge_status_pub_ =
    this->create_publisher<network_bridge::msg::BridgeStatus>(bridge_status_topic_, 10);

  inbound_sub_ = this->create_subscription<network_bridge::msg::BridgeFrame>(
    inbound_topic_, 20,
    [this](const network_bridge::msg::BridgeFrame::SharedPtr msg) {
      on_inbound_frame(msg);
    });

  status_sub_ = this->create_subscription<network_bridge::msg::InterfaceStatus>(
    status_topic_, 10,
    [this](const network_bridge::msg::InterfaceStatus::SharedPtr msg) {
      on_interface_status(msg);
    });
    
    if (config_file.empty()) {
      RCLCPP_WARN(this->get_logger(), "No config_file set — no topics will be bridged");
      return;
    }
    
    load_topic_config(config_file);

    RCLCPP_INFO(
      this->get_logger(),
      "out='%s' in='%s' status='%s' addressing=%s src=0x%02X",
      outbound_topic_.c_str(), inbound_topic_.c_str(), status_topic_.c_str(),
      use_addressing_ ? "on" : "off", src_addr_);
}

void EncoderNode::load_topic_config(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to load config_file '%s': %s", path.c_str(), e.what());
    return;
  }
  src_addr_ = root["node_addr"].as<uint8_t>(0);

  auto topics_node = root["topics"];
  if (!topics_node || !topics_node.IsSequence()) {
    RCLCPP_ERROR(this->get_logger(), "config_file has no 'topics' list");
    return;
  }

  for (auto entry : topics_node) {
    network_bridge::TopicConfig cfg;
    cfg.id = entry["id"].as<uint8_t>(0);
    cfg.mode = entry["mode"].as<std::string>("tx");
    cfg.input = entry["input"].as<std::string>("");
    cfg.output = entry["output"].as<std::string>("");
    cfg.type = entry["type"].as<std::string>("");
    cfg.rate = entry["rate"].as<float>(5.0f);
    cfg.queue_max = entry["queue_max"].as<int>(8);
    cfg.zstd_level = entry["zstd_level"].as<int>(3);
    cfg.publish_stale_data = entry["publish_stale_data"].as<bool>(false);
    cfg.dst_addr = entry["dst_addr"].as<uint8_t>(0);
    cfg.src_addr = entry["src_addr"].as<uint8_t>(0);

    // TODO - dont do this as a string but a number priority. Decide how priorities are handled.
    std::string prio_str = entry["priority"].as<std::string>("medium");
    if (prio_str == "low") {
      cfg.priority = network_bridge::Priority::LOW;
    } else if (prio_str == "high") {
      cfg.priority = network_bridge::Priority::HIGH;
    }

    std::string drop_str = entry["drop_policy"].as<std::string>("latest");
    cfg.drop_policy = (drop_str == "fifo") ?
      network_bridge::DropPolicy::FIFO : network_bridge::DropPolicy::LATEST;

    std::string t = cfg.input;
    cfg.is_tf = (t == "/tf" || t == "tf" || t == "/tf_static" || t == "tf_static");
    cfg.is_tf = entry["is_tf"].as<bool>(cfg.is_tf);
    cfg.is_static_tf = cfg.is_tf && (t == "/tf_static" || t == "tf_static");
    cfg.is_static_tf = entry["is_static_tf"].as<bool>(cfg.is_static_tf);

    if (entry["tf_include"] && entry["tf_include"].IsSequence()) {
      for (auto v : entry["tf_include"]) {cfg.tf_include.push_back(v.as<std::string>());}
    }
    if (entry["tf_exclude"] && entry["tf_exclude"].IsSequence()) {
      for (auto v : entry["tf_exclude"]) {cfg.tf_exclude.push_back(v.as<std::string>());}
    }

    if (cfg.type.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Topic id=%u has no 'type' — skipping", cfg.id);
      continue;
    }

    bool do_tx = (cfg.mode == "tx" || cfg.mode == "duplex");
    bool do_rx = (cfg.mode == "rx" || cfg.mode == "duplex");

    if (!do_tx && !do_rx) {
      RCLCPP_ERROR(
        this->get_logger(), "Unknown mode '%s' for id=%u", cfg.mode.c_str(), cfg.id);
      continue;
    }

    if (do_tx) {
      if (cfg.input.empty()) {
        RCLCPP_ERROR(this->get_logger(), "TX id=%u has no 'input' — skipping TX", cfg.id);
      } else {
        std::shared_ptr<SubscriptionManager> mgr;
        if (cfg.is_tf) {
          auto tf_mgr = std::make_shared<SubscriptionManagerTF>(
            shared_from_this(), cfg.input, "",
            cfg.zstd_level, cfg.publish_stale_data, cfg.is_static_tf);
          if (!cfg.tf_include.empty()) {tf_mgr->set_include_pattern(cfg.tf_include);}
          if (!cfg.tf_exclude.empty()) {tf_mgr->set_exclude_pattern(cfg.tf_exclude);}
          tf_mgr->setup_subscription();
          mgr = tf_mgr;
        } else {
          mgr = std::make_shared<SubscriptionManager>(
            shared_from_this(), cfg.input, "",
            cfg.zstd_level, cfg.publish_stale_data, cfg.queue_max);
          mgr->setup_subscription();
        }

        network_bridge::TxQueue tq;
        tq.config = cfg;
        tq.sub_mgr = mgr;
        tx_queues_.push_back(std::move(tq));

        int ms = static_cast<int>(1000.0f / cfg.rate);
        size_t idx = tx_queues_.size() - 1;
        timers_.push_back(this->create_wall_timer(
            std::chrono::milliseconds(ms),
            [this, idx]() {encode_and_publish(tx_queues_[idx]);}));

        RCLCPP_INFO(
          this->get_logger(),
          "TX id=%u dst=0x%02X  %s [%s]  %.1fHz  pri=%s  drop=%s  q=%d",
          cfg.id, cfg.dst_addr, cfg.input.c_str(), cfg.type.c_str(), cfg.rate,
          prio_str.c_str(), drop_str.c_str(), cfg.queue_max);
      }
    }

    if (do_rx) {
      if (cfg.output.empty()) {
        RCLCPP_ERROR(this->get_logger(), "RX id=%u has no 'output' — skipping RX", cfg.id);
      } else {
        auto key = std::make_pair(cfg.src_addr, cfg.id);
        network_bridge::RxEntry rx;
        rx.config = cfg;
        rx_registry_[key] = std::move(rx);

        RCLCPP_INFO(
          this->get_logger(),
          "RX id=%u src=0x%02X  → %s [%s]",
          cfg.id, cfg.src_addr, cfg.output.c_str(), cfg.type.c_str());
      }
    }
  }
}

void EncoderNode::encode_and_publish(network_bridge::TxQueue & tq)
{
  tq.sub_mgr->check_subscription();

  if (!tq.sub_mgr->has_data()) {return;}

  bool is_valid = false;
  std::vector<uint8_t> raw = tq.sub_mgr->get_data(is_valid);

  if (!is_valid || raw.empty()) {return;}

  bytes_offered_window_ += static_cast<uint32_t>(raw.size());

  if (window_bytes_used_ >= window_bytes_budget_) {
    frames_skipped_window_++;
    return;
  }

  std::vector<uint8_t> compressed;
  try {
    compress(raw, compressed, tq.config.zstd_level);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Compression failed: %s", e.what());
    return;
  }

  window_bytes_used_ += static_cast<uint32_t>(compressed.size());
  frames_encoded_window_++;

  std::vector<uint8_t> wire;
  wire.reserve((use_addressing_ ? 2 : 0) + 2 + compressed.size());
  if (use_addressing_) {
    wire.push_back(tq.config.dst_addr);
    wire.push_back(src_addr_);
  }
  wire.push_back(tq.config.id);
  wire.push_back(sequence_);
  wire.insert(wire.end(), compressed.begin(), compressed.end());

  network_bridge::msg::BridgeFrame frame;
  frame.dst_addr = use_addressing_ ? tq.config.dst_addr : 0;
  frame.src_addr = use_addressing_ ? src_addr_ : 0;
  frame.topic_id = tq.config.id;
  frame.sequence = sequence_++;
  frame.payload  = std::move(wire);

  bridge_frame_pub_->publish(frame);
}

void EncoderNode::on_inbound_frame(const network_bridge::msg::BridgeFrame::SharedPtr msg)
{
  if (msg->payload.size() < (use_addressing_ ? 4u : 2u)) {
    RCLCPP_WARN(this->get_logger(), "Inbound frame too short (%zu bytes)", msg->payload.size());
    return;
  }

  size_t offset = 0;
  uint8_t src_addr = 0x00;
  // TODO maybe we always force addressing but the lower layer can 
  // pull off the header and add it back on on the other side
  if (use_addressing_) {
    uint8_t dst_addr = msg->payload[0];
    if (dst_addr != src_addr_ && dst_addr != 0x00) {
      RCLCPP_DEBUG(
        this->get_logger(), "Frame for dst_addr=0x%02X, not us (0x%02X) — dropping",
        dst_addr, src_addr_);
      return;
    }
    src_addr = msg->payload[1];
    offset = 2;
  }
  uint8_t topic_id = msg->payload[offset];
  // uint8_t sequence = msg->payload[offset + 1];  // available for future use

  // Try exact src_addr match first, then fall back to wildcard (src_addr=0).
  auto it = rx_registry_.find({src_addr, topic_id});
  if (it == rx_registry_.end()) {
    it = rx_registry_.find({0, topic_id});
  }
  if (it == rx_registry_.end()) {
    RCLCPP_DEBUG(
      this->get_logger(), "No RX entry for (src=0x%02X, id=%u)", src_addr, topic_id);
    return;
  }
  auto & entry = it->second;

  std::span<const uint8_t> zstd_data(
    msg->payload.data() + offset + 2,
    msg->payload.size() - offset - 2);

  std::vector<uint8_t> decompressed;
  try {
    decompress(zstd_data, decompressed);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "Decompression failed (src=0x%02X id=%u): %s",
      src_addr, topic_id, e.what());
    return;
  }

  if (!entry.publisher) {
    rclcpp::QoS qos(10);
    qos.reliable().transient_local();
    entry.publisher =
      this->create_generic_publisher(entry.config.output, entry.config.type, qos);
    RCLCPP_INFO(
      this->get_logger(), "Created RX publisher %s [%s]",
      entry.config.output.c_str(), entry.config.type.c_str());
  }

  rclcpp::SerializedMessage serialized(decompressed.size());
  auto & rcl_msg = serialized.get_rcl_serialized_message();
  std::copy(decompressed.begin(), decompressed.end(), rcl_msg.buffer);
  rcl_msg.buffer_length = decompressed.size();

  entry.publisher->publish(serialized);

  RCLCPP_DEBUG(
    this->get_logger(), "RX src=0x%02X id=%u", src_addr, topic_id);
}

void EncoderNode::on_interface_status(
  const network_bridge::msg::InterfaceStatus::SharedPtr msg)
{
  medium_state_ = msg->medium_state;

  bool active_window = msg->bytes_sent_last_window > 0 || msg->queue_depth > 0;
  if (msg->window_ms > 0 && active_window) {
    float sample_bps =
      static_cast<float>(msg->bytes_sent_last_window) * 8000.0f /
      static_cast<float>(msg->window_ms);
    estimated_bps_ = (estimated_bps_ == 0.0f && sample_bps > 0.0f)
      ? sample_bps
      : throughput_alpha_ * sample_bps + (1.0f - throughput_alpha_) * estimated_bps_;
  }

  float win_sec = static_cast<float>(msg->window_ms) / 1000.0f;
  uint32_t in  = frames_encoded_window_;
  uint32_t out = msg->packets_sent_last_window;
  uint32_t skipped = frames_skipped_window_;
  uint32_t dropped = msg->dropped_last_window;
  uint32_t total_attempted = in + skipped;

  network_bridge::msg::BridgeStatus bs;
  bs.header.stamp           = msg->header.stamp;
  bs.window_ms              = msg->window_ms;
  bs.estimated_throughput_bps = estimated_bps_;
  bs.output_bps             = (win_sec > 0.0f)
    ? static_cast<float>(msg->bytes_sent_last_window) * 8.0f / win_sec : 0.0f;
  bs.input_bps              = (win_sec > 0.0f)
    ? static_cast<float>(bytes_offered_window_) * 8.0f / win_sec : 0.0f;
  bs.packets_in_last_window      = in;
  bs.packets_out_last_window     = out;
  bs.packets_skipped_last_window = skipped;
  bs.packets_dropped_last_window = dropped;
  bs.packet_loss_rate       = (total_attempted > 0)
    ? static_cast<float>(skipped + dropped) / static_cast<float>(total_attempted) : 0.0f;
  bs.queue_depth   = msg->queue_depth;
  bs.queue_max     = msg->queue_max;
  bs.medium_state  = msg->medium_state;
  bridge_status_pub_->publish(bs);

  frames_encoded_window_ = 0;
  frames_skipped_window_ = 0;
  bytes_offered_window_  = 0;
  window_bytes_used_ = 0;
  if (estimated_bps_ > 0.0f && msg->window_ms > 0) {
    window_bytes_budget_ = static_cast<uint32_t>(
      estimated_bps_ * static_cast<float>(msg->window_ms) / 8000.0f);
  } else {
    window_bytes_budget_ = UINT32_MAX;
  }

  if (msg->queue_max == 0) {return;}
  float fill = static_cast<float>(msg->queue_depth) / static_cast<float>(msg->queue_max);

  if (fill >= 0.8f) {
    for (auto & tq : tx_queues_) {
      tq.sub_mgr->trim_to(1);
    }
  } else if (fill >= 0.5f) {
    for (auto & tq : tx_queues_) {
      tq.sub_mgr->trim_to(std::max(size_t(1), tq.sub_mgr->queue_size() / 2));
    }
  }
}

void EncoderNode::compress(
  const std::vector<uint8_t> & data, std::vector<uint8_t> & out, int level)
{
  size_t capacity = ZSTD_compressBound(data.size());
  out.resize(capacity);
  size_t result = ZSTD_compress(out.data(), capacity, data.data(), data.size(), level);
  if (ZSTD_isError(result)) {throw std::runtime_error(ZSTD_getErrorName(result));}
  out.resize(result);
}

void EncoderNode::decompress(std::span<const uint8_t> data, std::vector<uint8_t> & out)
{
  size_t decompressed_size = ZSTD_getFrameContentSize(data.data(), data.size());
  if (decompressed_size == ZSTD_CONTENTSIZE_ERROR) {
    throw std::runtime_error("Not a zstd frame");
  }
  if (decompressed_size == ZSTD_CONTENTSIZE_UNKNOWN) {
    throw std::runtime_error("zstd frame size unknown");
  }
  out.resize(decompressed_size);
  size_t result = ZSTD_decompress(out.data(), decompressed_size, data.data(), data.size());
  if (ZSTD_isError(result)) {throw std::runtime_error(ZSTD_getErrorName(result));}
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EncoderNode>(
    "encoder_node_" + std::to_string(::getpid()));
  node->initialize();
  rclcpp::spin(node);
  node->shutdown();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
