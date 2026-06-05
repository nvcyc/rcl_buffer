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

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int32.hpp"

class PlainImageSubscriber : public rclcpp::Node
{
public:
  PlainImageSubscriber()
  : Node("plain_image_subscriber"),
    received_count_(0),
    validation_passed_(true)
  {
    this->declare_parameter<std::string>("topic_name", "plain_image");
    this->declare_parameter<std::string>("count_topic_suffix", "");
    this->declare_parameter<std::string>("count_topic_prefix", "");
    this->declare_parameter<int>("log_every_n", 1);

    std::string topic_name = this->get_parameter("topic_name").as_string();
    std::string count_suffix = this->get_parameter("count_topic_suffix").as_string();
    std::string count_prefix = this->get_parameter("count_topic_prefix").as_string();
    log_every_n_ = this->get_parameter("log_every_n").as_int();

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic_name, 10,
      std::bind(&PlainImageSubscriber::callback, this, std::placeholders::_1));

    std::string count_topic = count_prefix.empty() ? "subscriber_count" :
      count_prefix + "_subscriber_count";
    count_topic += count_suffix;
    std::string validation_topic = count_prefix.empty() ? "validation_result" :
      count_prefix + "_validation_result";
    validation_topic += count_suffix;

    count_publisher_ = this->create_publisher<std_msgs::msg::UInt32>(count_topic, 10);
    validation_publisher_ = this->create_publisher<std_msgs::msg::Bool>(validation_topic, 10);

    RCLCPP_INFO(this->get_logger(),
      "Plain image subscriber started (topic: %s)",
      topic_name.c_str());
  }

private:
  void callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    received_count_++;

    bool msg_valid = validate_image(*msg);
    validation_passed_ = validation_passed_ && msg_valid;

    auto count_msg = std_msgs::msg::UInt32();
    count_msg.data = received_count_;
    count_publisher_->publish(count_msg);

    auto validation_msg = std_msgs::msg::Bool();
    validation_msg.data = validation_passed_;
    validation_publisher_->publish(validation_msg);

    if (log_every_n_ > 0 && received_count_ % static_cast<uint32_t>(log_every_n_) == 0) {
      RCLCPP_INFO(this->get_logger(),
        "Received image #%u (valid: %s)",
        received_count_,
        msg_valid ? "true" : "false");
    }
  }

  bool validate_image(const sensor_msgs::msg::Image & msg) const
  {
    if (msg.height != image_height_ || msg.width != image_width_) {
      RCLCPP_ERROR(this->get_logger(),
        "Wrong dimensions: %ux%u",
        msg.width,
        msg.height);
      return false;
    }
    if (msg.encoding != "rgb8") {
      RCLCPP_ERROR(this->get_logger(),
        "Wrong encoding: %s",
        msg.encoding.c_str());
      return false;
    }
    if (msg.is_bigendian != 0) {
      RCLCPP_ERROR(this->get_logger(),
        "Wrong endian flag: %u",
        msg.is_bigendian);
      return false;
    }
    if (msg.step != image_width_ * channel_count_) {
      RCLCPP_ERROR(this->get_logger(),
        "Wrong step: %u",
        msg.step);
      return false;
    }
    if (msg.data.size() != image_height_ * msg.step) {
      RCLCPP_ERROR(this->get_logger(),
        "Wrong data size: %zu",
        msg.data.size());
      return false;
    }
    if (msg.data.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Image data is empty");
      return false;
    }

    const uint8_t first_byte = msg.data[0];
    for (size_t i = 0; i < msg.data.size(); ++i) {
      const uint8_t expected = static_cast<uint8_t>((first_byte + i) % 256);
      if (msg.data[i] != expected) {
        RCLCPP_ERROR(this->get_logger(),
          "Unexpected data byte at %zu: got %u, expected %u",
          i,
          msg.data[i],
          expected);
        return false;
      }
    }

    return true;
  }

  static constexpr uint32_t image_width_ = 64;
  static constexpr uint32_t image_height_ = 48;
  static constexpr uint32_t channel_count_ = 3;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr count_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr validation_publisher_;
  uint32_t received_count_;
  bool validation_passed_;
  int log_every_n_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlainImageSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
