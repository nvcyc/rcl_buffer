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

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/u_int32.hpp"

using namespace std::chrono_literals;

class PlainImagePublisher : public rclcpp::Node
{
public:
  PlainImagePublisher()
  : Node("plain_image_publisher"), count_(0)
  {
    this->declare_parameter<std::string>("topic_name", "plain_image");
    this->declare_parameter<int>("publish_rate_ms", 500);
    this->declare_parameter<std::string>("count_topic_prefix", "");
    this->declare_parameter<int>("max_publish_count", 0);
    this->declare_parameter<int>("log_every_n", 1);

    std::string topic_name = this->get_parameter("topic_name").as_string();
    int publish_rate_ms = this->get_parameter("publish_rate_ms").as_int();
    std::string count_prefix = this->get_parameter("count_topic_prefix").as_string();
    max_publish_count_ = static_cast<size_t>(this->get_parameter("max_publish_count").as_int());
    log_every_n_ = this->get_parameter("log_every_n").as_int();

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);

    std::string count_topic = count_prefix.empty() ? "publisher_count" :
      count_prefix + "_publisher_count";
    count_publisher_ = this->create_publisher<std_msgs::msg::UInt32>(count_topic, 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(publish_rate_ms),
      std::bind(&PlainImagePublisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(),
      "Plain image publisher started (topic: %s, max_count: %zu)",
      topic_name.c_str(),
      max_publish_count_ == 0 ? SIZE_MAX : max_publish_count_);
  }

private:
  void timer_callback()
  {
    if (max_publish_count_ > 0 && count_ >= max_publish_count_) {
      return;
    }

    auto msg = sensor_msgs::msg::Image();
    msg.header.stamp = this->now();
    msg.header.frame_id = "plain_image_frame";
    msg.height = image_height_;
    msg.width = image_width_;
    msg.encoding = "rgb8";
    msg.is_bigendian = 0;
    msg.step = image_width_ * channel_count_;
    msg.data.resize(image_height_ * msg.step);

    for (size_t i = 0; i < msg.data.size(); ++i) {
      msg.data[i] = static_cast<uint8_t>((count_ + i) % 256);
    }

    if (log_every_n_ > 0 && count_ % static_cast<size_t>(log_every_n_) == 0) {
      RCLCPP_INFO(this->get_logger(), "Publishing image #%zu", count_);
    }
    publisher_->publish(msg);

    auto count_msg = std_msgs::msg::UInt32();
    count_msg.data = ++count_;
    count_publisher_->publish(count_msg);
  }

  static constexpr uint32_t image_width_ = 64;
  static constexpr uint32_t image_height_ = 48;
  static constexpr uint32_t channel_count_ = 3;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr count_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
  size_t max_publish_count_;
  int log_every_n_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlainImagePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
