// Copyright 2024 NVIDIA Corporation
//
// Demo backend image publisher node for demonstrating buffer backend plugin system

#include <chrono>
#include <memory>
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
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("demo_backend_image", 10);
    count_publisher_ = this->create_publisher<std_msgs::msg::UInt32>("publisher_count", 10);

    timer_ = this->create_wall_timer(
      500ms, std::bind(&DemoBackendImagePublisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Demo backend image publisher started");
  }

private:
  void timer_callback()
  {
    auto msg = sensor_msgs::msg::Image();

    // Create demo image: 8x8 RGB
    msg.header.stamp = this->now();
    msg.header.frame_id = "demo_backend_frame";
    msg.height = 8;
    msg.width = 8;
    msg.encoding = "rgb8";
    msg.step = 8 * 3;
    msg.is_bigendian = 0;

    // Create DemoBufferImpl for image data
    const size_t data_size = 8 * 8 * 3;

    // Fill with pattern based on count
    std::vector<uint8_t> host_data(data_size);
    for (size_t i = 0; i < data_size; ++i) {
      host_data[i] = static_cast<uint8_t>((count_ + i) % 256);
    }

    // Create DemoBufferImpl with the data
    auto demo_impl = std::make_unique<demo_buffer_backend::DemoBufferImpl<uint8_t>>(
      std::move(host_data));

    // Set demo backend implementation on msg.data
    msg.data.set_impl(std::move(demo_impl), "demo");

    RCLCPP_INFO(
      this->get_logger(),
      "Publishing image #%zu with demo backend (size: %zu, backend: %s)",
      count_ + 1, msg.data.size(), msg.data.get_backend_type().c_str());

    publisher_->publish(msg);

    // Publish count for test verification
    auto count_msg = std_msgs::msg::UInt32();
    count_msg.data = ++count_;
    count_publisher_->publish(count_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr count_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DemoBackendImagePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
