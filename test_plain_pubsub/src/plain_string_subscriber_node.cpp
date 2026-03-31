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
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/bool.hpp"

class PlainStringSubscriber : public rclcpp::Node
{
public:
  PlainStringSubscriber()
  : Node("plain_string_subscriber"),
    received_count_(0),
    validation_passed_(true)
  {
    this->declare_parameter<std::string>("topic_name", "plain_string");
    this->declare_parameter<std::string>("count_topic_suffix", "");
    this->declare_parameter<std::string>("count_topic_prefix", "");

    std::string topic_name = this->get_parameter("topic_name").as_string();
    std::string count_suffix = this->get_parameter("count_topic_suffix").as_string();
    std::string count_prefix = this->get_parameter("count_topic_prefix").as_string();

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      topic_name, 10,
      std::bind(&PlainStringSubscriber::callback, this, std::placeholders::_1));

    std::string count_topic = count_prefix.empty() ? "subscriber_count" :
      count_prefix + "_subscriber_count";
    count_topic += count_suffix;
    std::string validation_topic = count_prefix.empty() ? "validation_result" :
      count_prefix + "_validation_result";
    validation_topic += count_suffix;

    count_publisher_ = this->create_publisher<std_msgs::msg::UInt32>(count_topic, 10);
    validation_publisher_ = this->create_publisher<std_msgs::msg::Bool>(validation_topic, 10);

    RCLCPP_INFO(this->get_logger(),
      "Plain string subscriber started (topic: %s)",
      topic_name.c_str());
  }

private:
  void callback(const std_msgs::msg::String::SharedPtr msg)
  {
    received_count_++;

    bool msg_valid = !msg->data.empty() && msg->data.rfind("message_", 0) == 0;
    validation_passed_ = validation_passed_ && msg_valid;

    auto count_msg = std_msgs::msg::UInt32();
    count_msg.data = received_count_;
    count_publisher_->publish(count_msg);

    auto validation_msg = std_msgs::msg::Bool();
    validation_msg.data = validation_passed_;
    validation_publisher_->publish(validation_msg);

    RCLCPP_INFO(this->get_logger(),
      "Received #%u: '%s' (valid: %s)",
      received_count_, msg->data.c_str(),
      msg_valid ? "true" : "false");
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr count_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr validation_publisher_;
  uint32_t received_count_;
  bool validation_passed_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlainStringSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
