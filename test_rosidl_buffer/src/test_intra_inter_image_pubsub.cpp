// Copyright 2026 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Combined publisher + intra-process subscriber in a single process.
//
// Designed to run alongside an external demo_backend_image_subscriber_node
// to test the scenario where one publisher sends to both an intra-process
// and an inter-process subscriber simultaneously.
//
// Parameters:
//   backend_mode                      "cpu" or "demo"
//   topic_name                        image topic (default: "test_image")
//   publish_rate_ms                   publishing rate in ms
//   max_publish_count                 max messages to publish (0 = unlimited)
//   intra_expected_backends           expected backend types for intra sub
//   intra_acceptable_buffer_backends  acceptable backends for intra sub
//
// Publishes results:
//   publisher_count         (std_msgs/UInt32) publisher message count
//   intra_subscriber_count  (std_msgs/UInt32) intra subscriber message count
//   intra_validation_result (std_msgs/Bool)   intra subscriber validation

#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rosidl_buffer/buffer.hpp"
#include "demo_buffer/demo_buffer_impl.hpp"

using namespace std::chrono_literals;

class IntraInterPubSub : public rclcpp::Node
{
public:
  IntraInterPubSub()
  : Node("intra_inter_pubsub"),
    pub_count_(0),
    sub_count_(0),
    validation_passed_(true)
  {
    this->declare_parameter<std::string>("backend_mode", "cpu");
    this->declare_parameter<std::string>("topic_name", "test_image");
    this->declare_parameter<int>("publish_rate_ms", 200);
    this->declare_parameter<int>("max_publish_count", 50);
    this->declare_parameter<std::string>("intra_expected_backends", "cpu");
    this->declare_parameter<std::string>("intra_acceptable_buffer_backends", "__default__");

    backend_mode_ = this->get_parameter("backend_mode").as_string();
    std::string topic_name = this->get_parameter("topic_name").as_string();
    int publish_rate_ms = this->get_parameter("publish_rate_ms").as_int();
    max_publish_count_ = static_cast<size_t>(this->get_parameter("max_publish_count").as_int());
    std::string intra_expected = this->get_parameter("intra_expected_backends").as_string();
    std::string intra_acceptable =
      this->get_parameter("intra_acceptable_buffer_backends").as_string();

    parse_expected_backends(intra_expected);

    // Publisher
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
    pub_count_publisher_ = this->create_publisher<std_msgs::msg::UInt32>("publisher_count", 10);

    // Intra-process subscriber
    rclcpp::SubscriptionOptions sub_options;
    if (intra_acceptable != "__default__") {
      sub_options.acceptable_buffer_backends = intra_acceptable;
    }
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic_name, 10,
      std::bind(&IntraInterPubSub::image_callback, this, std::placeholders::_1),
      sub_options);

    // Intra-process result publishers
    intra_count_publisher_ =
      this->create_publisher<std_msgs::msg::UInt32>("intra_subscriber_count", 10);
    intra_validation_publisher_ =
      this->create_publisher<std_msgs::msg::Bool>("intra_validation_result", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(publish_rate_ms),
      std::bind(&IntraInterPubSub::timer_callback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "IntraInterPubSub started (backend: %s, topic: %s, expected: %s)",
      backend_mode_.c_str(), topic_name.c_str(), intra_expected.c_str());
  }

private:
  void parse_expected_backends(const std::string & backends_str)
  {
    std::string token;
    std::istringstream tokenStream(backends_str);
    while (std::getline(tokenStream, token, ',')) {
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

  void timer_callback()
  {
    if (max_publish_count_ > 0 && pub_count_ >= max_publish_count_) {
      return;
    }

    auto msg = sensor_msgs::msg::Image();
    msg.header.stamp = this->now();
    msg.header.frame_id = "test_frame";
    msg.height = 8;
    msg.width = 8;
    msg.encoding = "rgb8";
    msg.step = 8 * 3;
    msg.is_bigendian = 0;

    const size_t data_size = 8 * 8 * 3;
    std::vector<uint8_t> host_data(data_size);
    for (size_t i = 0; i < data_size; ++i) {
      host_data[i] = static_cast<uint8_t>((pub_count_ + i) % 256);
    }

    if (backend_mode_ == "demo") {
      auto demo_impl = std::make_unique<demo_buffer_backend::DemoBufferImpl<uint8_t>>(
        std::move(host_data));
      msg.data = rosidl::Buffer<uint8_t>(std::move(demo_impl));
    } else {
      msg.data = host_data;
    }

    image_publisher_->publish(msg);

    auto count_msg = std_msgs::msg::UInt32();
    count_msg.data = ++pub_count_;
    pub_count_publisher_->publish(count_msg);
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    sub_count_++;
    bool msg_valid = true;

    if (msg->width != 8 || msg->height != 8) {
      RCLCPP_ERROR(this->get_logger(), "[intra] Wrong dimensions: %ux%u", msg->width, msg->height);
      msg_valid = false;
    }

    if (msg->encoding != "rgb8") {
      RCLCPP_ERROR(this->get_logger(), "[intra] Wrong encoding: %s", msg->encoding.c_str());
      msg_valid = false;
    }

    const size_t expected_size = 8 * 8 * 3;
    if (msg->data.size() != expected_size) {
      RCLCPP_ERROR(
        this->get_logger(), "[intra] Wrong data size: %zu (expected %zu)",
        msg->data.size(), expected_size);
      msg_valid = false;
    }

    const std::string backend_type = msg->data.get_backend_type();
    if (!is_backend_expected(backend_type)) {
      RCLCPP_ERROR(
        this->get_logger(), "[intra] Unexpected backend: %s", backend_type.c_str());
      msg_valid = false;
    }

    try {
      std::vector<uint8_t> cpu_data = msg->data.to_vector();
      if (cpu_data.size() > 0) {
        bool pattern_valid = true;
        for (size_t i = 1; i < std::min(cpu_data.size(), static_cast < size_t > (10)); ++i) {
          uint8_t actual_diff = (cpu_data[i] - cpu_data[i - 1] + 256) % 256;
          if (actual_diff != 1) {
            RCLCPP_WARN(this->get_logger(), "[intra] Pattern mismatch at byte %zu", i);
            pattern_valid = false;
            break;
          }
        }
        if (!pattern_valid) {
          msg_valid = false;
        }

        // Duplicate detection — data[0] == (pub_count % 256) is unique per message
        uint8_t first_byte = cpu_data[0];
        if (seen_first_bytes_.count(first_byte) > 0) {
          RCLCPP_ERROR(
            this->get_logger(),
            "[intra] Duplicate message detected! data[0]=%u was already received",
            first_byte);
          msg_valid = false;
        } else {
          seen_first_bytes_.insert(first_byte);
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "[intra] Validation exception: %s", e.what());
      msg_valid = false;
    }

    validation_passed_ = validation_passed_ && msg_valid;

    auto count_msg = std_msgs::msg::UInt32();
    count_msg.data = sub_count_;
    intra_count_publisher_->publish(count_msg);

    auto val_msg = std_msgs::msg::Bool();
    val_msg.data = validation_passed_;
    intra_validation_publisher_->publish(val_msg);

    RCLCPP_INFO(
      this->get_logger(), "[intra] Image #%u (backend: %s, valid: %s)",
      sub_count_, backend_type.c_str(), msg_valid ? "true" : "false");
  }

  // Publisher members
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub_count_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string backend_mode_;
  size_t pub_count_;
  size_t max_publish_count_;

  // Intra-process subscriber members
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr intra_count_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr intra_validation_publisher_;
  std::vector<std::string> expected_backends_;
  uint32_t sub_count_;
  bool validation_passed_;
  std::unordered_set<uint8_t> seen_first_bytes_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IntraInterPubSub>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
