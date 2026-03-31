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
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"

using namespace std::chrono_literals;

class PlainStringPublisher : public rclcpp::Node
{
public:
  PlainStringPublisher()
  : Node("plain_string_publisher"), count_(0)
  {
    this->declare_parameter<std::string>("topic_name", "plain_string");
    this->declare_parameter<int>("publish_rate_ms", 500);
    this->declare_parameter<std::string>("count_topic_prefix", "");
    this->declare_parameter<int>("max_publish_count", 0);

    std::string topic_name = this->get_parameter("topic_name").as_string();
    int publish_rate_ms = this->get_parameter("publish_rate_ms").as_int();
    std::string count_prefix = this->get_parameter("count_topic_prefix").as_string();
    max_publish_count_ = static_cast<size_t>(this->get_parameter("max_publish_count").as_int());

    publisher_ = this->create_publisher<std_msgs::msg::String>(topic_name, 10);

    std::string count_topic = count_prefix.empty() ? "publisher_count" :
      count_prefix + "_publisher_count";
    count_publisher_ = this->create_publisher<std_msgs::msg::UInt32>(count_topic, 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(publish_rate_ms),
      std::bind(&PlainStringPublisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(),
      "Plain string publisher started (topic: %s, max_count: %zu)",
      topic_name.c_str(),
      max_publish_count_ == 0 ? SIZE_MAX : max_publish_count_);
  }

private:
  void timer_callback()
  {
    if (max_publish_count_ > 0 && count_ >= max_publish_count_) {
      return;
    }

    auto msg = std_msgs::msg::String();
    msg.data = "message_" + std::to_string(count_);

    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
    publisher_->publish(msg);

    auto count_msg = std_msgs::msg::UInt32();
    count_msg.data = ++count_;
    count_publisher_->publish(count_msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr count_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
  size_t max_publish_count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlainStringPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
