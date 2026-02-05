// Copyright 2024 NVIDIA Corporation
//
// Demo backend image subscriber node for demonstrating buffer backend plugin system
// Supports configurable topic and expected backend type via parameters

#include <memory>
#include <sstream>
#include <string>
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
    // Declare parameters
    this->declare_parameter<std::string>("topic_name", "demo_backend_image");
    this->declare_parameter<std::string>("expected_backends", "demo,cpu");
    this->declare_parameter<std::string>("count_topic_suffix", "");
    this->declare_parameter<std::string>("count_topic_prefix", "");

    // Get parameters
    std::string topic_name = this->get_parameter("topic_name").as_string();
    expected_backends_str_ = this->get_parameter("expected_backends").as_string();
    std::string count_suffix = this->get_parameter("count_topic_suffix").as_string();
    std::string count_prefix = this->get_parameter("count_topic_prefix").as_string();

    // Parse expected backends
    parse_expected_backends(expected_backends_str_);

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic_name, 10,
      std::bind(&DemoBackendImageSubscriber::image_callback, this, std::placeholders::_1));

    // Publishers for test results (with optional prefix and suffix for multiple test configurations)
    std::string count_topic = count_prefix.empty() ? "subscriber_count" :
                              count_prefix + "_subscriber_count";
    count_topic += count_suffix;
    std::string validation_topic = count_prefix.empty() ? "validation_result" :
                                   count_prefix + "_validation_result";
    validation_topic += count_suffix;

    count_publisher_ = this->create_publisher<std_msgs::msg::UInt32>(count_topic, 10);
    validation_publisher_ = this->create_publisher<std_msgs::msg::Bool>(validation_topic, 10);

    RCLCPP_INFO(this->get_logger(),
      "Demo backend image subscriber started (topic: %s, expected_backends: %s)",
      topic_name.c_str(), expected_backends_str_.c_str());
  }

private:
  void parse_expected_backends(const std::string & backends_str)
  {
    expected_backends_.clear();
    std::string token;
    std::istringstream tokenStream(backends_str);
    while (std::getline(tokenStream, token, ',')) {
      // Trim whitespace
      size_t start = token.find_first_not_of(" \t");
      size_t end = token.find_last_not_of(" \t");
      if (start != std::string::npos && end != std::string::npos) {
        expected_backends_.push_back(token.substr(start, end - start + 1));
      }
    }
  }

  bool is_backend_expected(const std::string & backend) const
  {
    for (const auto & expected : expected_backends_) {
      if (expected == backend) {
        return true;
      }
    }
    return false;
  }

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

    // Check backend type against expected backends
    const std::string backend_type = msg->data.get_backend_type();
    if (!is_backend_expected(backend_type)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Unexpected backend type: %s (expected one of: %s)",
        backend_type.c_str(), expected_backends_str_.c_str());
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
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "Received message using '%s' backend", backend_type.c_str());
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
  std::string expected_backends_str_;
  std::vector<std::string> expected_backends_;
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
