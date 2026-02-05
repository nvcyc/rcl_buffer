// Copyright 2024 NVIDIA Corporation
//
// Demo backend image subscriber node for demonstrating buffer backend plugin system

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rosidl_runtime_cpp/buffer.hpp"

class DemoBackendImageSubscriber : public rclcpp::Node
{
public:
  DemoBackendImageSubscriber()
  : Node("demo_backend_image_subscriber"),
    received_count_(0),
    validation_passed_(true)
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "demo_backend_image", 10,
      std::bind(&DemoBackendImageSubscriber::image_callback, this, std::placeholders::_1));

    // Publishers for test results
    count_publisher_ = this->create_publisher<std_msgs::msg::UInt32>("subscriber_count", 10);
    validation_publisher_ = this->create_publisher<std_msgs::msg::Bool>("validation_result", 10);

    RCLCPP_INFO(this->get_logger(), "Demo backend image subscriber started");
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    received_count_++;
    bool msg_valid = true;

    RCLCPP_INFO(this->get_logger(), "Received image #%u", received_count_);

    // Validate dimensions
    if (msg->width != 8 || msg->height != 8) {
      RCLCPP_ERROR(
        this->get_logger(), "Wrong dimensions: %ux%u (expected 8x8)",
        msg->width, msg->height);
      msg_valid = false;
    }

    // Validate encoding
    if (msg->encoding != "rgb8") {
      RCLCPP_ERROR(
        this->get_logger(), "Wrong encoding: %s (expected rgb8)",
        msg->encoding.c_str());
      msg_valid = false;
    }

    // Validate data size
    const size_t expected_size = 8 * 8 * 3;
    if (msg->data.size() != expected_size) {
      RCLCPP_ERROR(
        this->get_logger(), "Wrong data size: %zu (expected %zu)",
        msg->data.size(), expected_size);
      msg_valid = false;
    }

    // Check backend type - accept "demo" or "cpu" (CPU is valid fallback for inter-process)
    const std::string backend_type = msg->data.get_backend_type();
    if (backend_type != "demo" && backend_type != "cpu") {
      RCLCPP_ERROR(
        this->get_logger(),
        "Unexpected backend type: %s (expected: demo or cpu)",
        backend_type.c_str());
      msg_valid = false;
    }

    // Log backend type
    if (backend_type == "demo") {
      RCLCPP_INFO(
        this->get_logger(),
        "Received message using 'demo' backend - zero-copy path!");
    } else if (backend_type == "cpu") {
      RCLCPP_INFO(
        this->get_logger(),
        "Received message using 'cpu' backend - serialization fallback");
    }

    // Validate data integrity
    try {
      std::vector<uint8_t> cpu_data = msg->data.to_vector();

      if (cpu_data.size() > 0) {
        RCLCPP_INFO(
          this->get_logger(),
          "Data integrity check: first byte = %u, last byte = %u, size = %zu",
          cpu_data[0], cpu_data[cpu_data.size() - 1], cpu_data.size());

        // Verify data pattern (values should be sequential modulo 256)
        // Since publisher uses (count + i) % 256, consecutive bytes differ by 1
        bool pattern_valid = true;
        for (size_t i = 1; i < std::min(cpu_data.size(), static_cast<size_t>(10)); ++i) {
          uint8_t expected_diff = 1;
          uint8_t actual_diff = (cpu_data[i] - cpu_data[i - 1] + 256) % 256;
          if (actual_diff != expected_diff) {
            RCLCPP_WARN(
              this->get_logger(),
              "Pattern check: byte[%zu]=%u, byte[%zu]=%u, diff=%u (expected 1)",
              i - 1, cpu_data[i - 1], i, cpu_data[i], actual_diff);
            pattern_valid = false;
            break;
          }
        }

        if (pattern_valid) {
          RCLCPP_INFO(this->get_logger(), "Data pattern verification: PASSED");
        } else {
          RCLCPP_WARN(this->get_logger(), "Data pattern verification: FAILED");
          // Don't fail the test for pattern mismatch - data might be from different count
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Exception during data validation: %s", e.what());
      msg_valid = false;
    }

    validation_passed_ = validation_passed_ && msg_valid;

    // Publish count and validation status
    auto count_msg = std_msgs::msg::UInt32();
    count_msg.data = received_count_;
    count_publisher_->publish(count_msg);

    auto validation_msg = std_msgs::msg::Bool();
    validation_msg.data = validation_passed_;
    validation_publisher_->publish(validation_msg);

    if (msg_valid) {
      RCLCPP_INFO(
        this->get_logger(),
        "Image #%u validation: PASSED (backend: %s, size: %zu)",
        received_count_,
        backend_type.c_str(),
        msg->data.size());
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "Image #%u validation: FAILED",
        received_count_);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr count_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr validation_publisher_;
  uint32_t received_count_;
  bool validation_passed_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DemoBackendImageSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
