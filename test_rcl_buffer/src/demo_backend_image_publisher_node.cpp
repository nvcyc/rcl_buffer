// Copyright 2024 NVIDIA Corporation
//
// Demo backend image publisher node for demonstrating buffer backend plugin system
// Supports both CPU (std::vector) and demo backend modes via parameter

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "demo_buffer_backend/demo_buffer_impl.hpp"
#include "rosidl_runtime_cpp/buffer.hpp"

using namespace std::chrono_literals;

class DemoBackendImagePublisher : public rclcpp::Node
{
public:
  DemoBackendImagePublisher()
  : Node("demo_backend_image_publisher"), count_(0)
  {
    // Declare parameters
    this->declare_parameter<std::string>("backend_mode", "demo");
    this->declare_parameter<std::string>("topic_name", "demo_backend_image");
    this->declare_parameter<int>("publish_rate_ms", 500);
    this->declare_parameter<std::string>("count_topic_prefix", "");
    this->declare_parameter<int>("max_publish_count", 0);  // 0 = unlimited

    // Get parameters
    backend_mode_ = this->get_parameter("backend_mode").as_string();
    std::string topic_name = this->get_parameter("topic_name").as_string();
    int publish_rate_ms = this->get_parameter("publish_rate_ms").as_int();
    std::string count_prefix = this->get_parameter("count_topic_prefix").as_string();
    max_publish_count_ = static_cast<size_t>(this->get_parameter("max_publish_count").as_int());

    // Validate backend mode
    if (backend_mode_ != "cpu" && backend_mode_ != "demo") {
      RCLCPP_ERROR(this->get_logger(), "Invalid backend_mode: %s. Must be 'cpu' or 'demo'",
        backend_mode_.c_str());
      backend_mode_ = "cpu";
    }

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);

    // Support prefixed count topic for multi-publisher tests
    std::string count_topic = count_prefix.empty() ? "publisher_count" :
                              count_prefix + "_publisher_count";
    count_publisher_ = this->create_publisher<std_msgs::msg::UInt32>(count_topic, 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(publish_rate_ms),
      std::bind(&DemoBackendImagePublisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(),
      "Demo backend image publisher started (backend_mode: %s, topic: %s, max_count: %zu)",
      backend_mode_.c_str(), topic_name.c_str(),
      max_publish_count_ == 0 ? SIZE_MAX : max_publish_count_);
  }

private:
  void timer_callback()
  {
    // Stop publishing if max count reached (0 = unlimited)
    if (max_publish_count_ > 0 && count_ >= max_publish_count_) {
      return;
    }

    auto msg = sensor_msgs::msg::Image();

    // Create demo image: 8x8 RGB
    msg.header.stamp = this->now();
    msg.header.frame_id = "demo_backend_frame";
    msg.height = 8;
    msg.width = 8;
    msg.encoding = "rgb8";
    msg.step = 8 * 3;
    msg.is_bigendian = 0;

    const size_t data_size = 8 * 8 * 3;

    // Fill with pattern based on count
    std::vector<uint8_t> host_data(data_size);
    for (size_t i = 0; i < data_size; ++i) {
      host_data[i] = static_cast<uint8_t>((count_ + i) % 256);
    }

    if (backend_mode_ == "demo") {
      // Create DemoBufferImpl with the data
      auto demo_impl = std::make_unique<demo_buffer_backend::DemoBufferImpl<uint8_t>>(
        std::move(host_data));

      // Set demo backend implementation on msg.data
      msg.data.set_impl(std::move(demo_impl), "demo");
    } else {
      // CPU mode: use default std::vector assignment
      msg.data = host_data;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Publishing image #%zu with %s backend (size: %zu, backend: %s)",
      count_ + 1, backend_mode_.c_str(), msg.data.size(), msg.data.get_backend_type().c_str());

    publisher_->publish(msg);

    // Publish count for test verification
    auto count_msg = std_msgs::msg::UInt32();
    count_msg.data = ++count_;
    count_publisher_->publish(count_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr count_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string backend_mode_;
  size_t count_;
  size_t max_publish_count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DemoBackendImagePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
