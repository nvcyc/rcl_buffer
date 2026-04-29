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

// Publisher node for test_msgs::msg::MultiNested over rclcpp.
//
// MultiNested contains three UnboundedSequences[...] fields; each
// UnboundedSequences has a `uint8[] uint8_values` field that the rosidl C++
// generator emits as `rosidl::Buffer<uint8_t>`. This means the outer message
// carries a "list of uint8[]" through several nesting shapes. The
// `backend_mode` parameter switches how every inner `uint8_values` is
// constructed:
//
//   * "cpu"  — resize() + operator[] (CPU-backed Buffer, default behavior)
//   * "demo" — DemoBufferImpl<uint8_t> wrapped in rosidl::Buffer<uint8_t>
//
// This lets us exercise both construction paths for nested Buffer fields in
// pub/sub launch tests (demo-to-demo, cpu-to-cpu, demo-to-cpu).

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rosidl_buffer/buffer.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "demo_buffer/demo_buffer_impl.hpp"
#include "test_msgs/msg/multi_nested.hpp"
#include "test_msgs/msg/unbounded_sequences.hpp"

using namespace std::chrono_literals;
using MultiNested = test_msgs::msg::MultiNested;
using UnboundedSequences = test_msgs::msg::UnboundedSequences;
using U8Buffer = rosidl::Buffer<uint8_t>;

namespace
{

std::vector<uint8_t> make_payload(
  std::size_t bucket,
  std::size_t index,
  std::size_t seq,
  std::size_t length)
{
  std::vector<uint8_t> out(length);
  for (std::size_t i = 0; i < length; ++i) {
    out[i] = static_cast<uint8_t>((bucket * 53 + index * 17 + seq * 7 + i) % 256);
  }
  return out;
}

std::size_t payload_length(std::size_t bucket, std::size_t index, std::size_t seq)
{
  return 1 + ((bucket + index + seq) % 7) * 4;
}

void populate_inner(
  UnboundedSequences & inner,
  std::size_t bucket,
  std::size_t index,
  std::size_t seq,
  const std::string & backend_mode)
{
  const std::size_t length = payload_length(bucket, index, seq);
  auto payload = make_payload(bucket, index, seq, length);

  if (backend_mode == "demo") {
    auto impl = std::make_unique<demo_buffer_backend::DemoBufferImpl<uint8_t>>(
      std::move(payload));
    inner.uint8_values = U8Buffer(std::move(impl));
  } else {
    inner.uint8_values.resize(length);
    for (std::size_t i = 0; i < length; ++i) {
      inner.uint8_values[i] = payload[i];
    }
  }
}

}  // namespace

class MultiNestedPublisherNode : public rclcpp::Node
{
public:
  MultiNestedPublisherNode()
  : Node("multi_nested_publisher")
  {
    this->declare_parameter<std::string>("backend_mode", "demo");
    this->declare_parameter<std::string>("topic_name", "test_multi_nested");
    this->declare_parameter<int>("publish_rate_ms", 200);
    this->declare_parameter<std::string>("count_topic_prefix", "");
    this->declare_parameter<int>("max_publish_count", 0);

    backend_mode_ = this->get_parameter("backend_mode").as_string();
    const auto topic_name = this->get_parameter("topic_name").as_string();
    const auto publish_rate_ms = this->get_parameter("publish_rate_ms").as_int();
    const auto count_prefix =
      this->get_parameter("count_topic_prefix").as_string();
    max_publish_count_ =
      static_cast<std::size_t>(this->get_parameter("max_publish_count").as_int());

    if (backend_mode_ != "cpu" && backend_mode_ != "demo") {
      RCLCPP_ERROR(
        this->get_logger(),
        "Invalid backend_mode: %s. Must be 'cpu' or 'demo'",
        backend_mode_.c_str());
      backend_mode_ = "cpu";
    }

    publisher_ = this->create_publisher<MultiNested>(topic_name, 10);

    const auto count_topic = count_prefix.empty() ?
      std::string{"publisher_count"} :
    count_prefix + "_publisher_count";
    count_publisher_ =
      this->create_publisher<std_msgs::msg::UInt32>(count_topic, 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(publish_rate_ms),
      std::bind(&MultiNestedPublisherNode::on_timer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "MultiNested publisher started (backend_mode=%s, topic=%s, max_count=%zu)",
      backend_mode_.c_str(), topic_name.c_str(),
      max_publish_count_ == 0 ? SIZE_MAX : max_publish_count_);
  }

private:
  void on_timer()
  {
    if (max_publish_count_ > 0 && count_ >= max_publish_count_) {
      return;
    }

    MultiNested msg;

    // Bucket 0: fixed-size array[3] of UnboundedSequences.
    for (std::size_t i = 0; i < msg.array_of_unbounded_sequences.size(); ++i) {
      populate_inner(
        msg.array_of_unbounded_sequences[i], 0, i, count_, backend_mode_);
    }

    // Bucket 1: bounded sequence<=3 — exercise 2 elements.
    msg.bounded_sequence_of_unbounded_sequences.resize(2);
    for (std::size_t i = 0; i < msg.bounded_sequence_of_unbounded_sequences.size(); ++i) {
      populate_inner(
        msg.bounded_sequence_of_unbounded_sequences[i],
        1, i, count_, backend_mode_);
    }

    // Bucket 2: unbounded sequence — the literal "list of uint8[]".
    const std::size_t outer_len = 1 + (count_ % 4);
    msg.unbounded_sequence_of_unbounded_sequences.resize(outer_len);
    for (std::size_t i = 0; i < outer_len; ++i) {
      populate_inner(
        msg.unbounded_sequence_of_unbounded_sequences[i],
        2, i, count_, backend_mode_);
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Publishing MultiNested #%zu (backend_mode=%s, outer_unbounded_len=%zu)",
      count_ + 1, backend_mode_.c_str(), outer_len);
    publisher_->publish(msg);

    auto count_msg = std_msgs::msg::UInt32();
    count_msg.data = static_cast<std::uint32_t>(++count_);
    count_publisher_->publish(count_msg);
  }

  rclcpp::Publisher<MultiNested>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr count_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string backend_mode_;
  std::size_t count_ = 0;
  std::size_t max_publish_count_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiNestedPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
