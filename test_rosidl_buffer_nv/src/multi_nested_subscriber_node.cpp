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

// Subscriber node for test_msgs::msg::MultiNested. Validates that the
// publisher's deterministic pattern for every nested `uint8_values` field
// round-trips correctly, regardless of whether the publisher used the CPU
// or demo backend to build them.
//
// Reports progress on three std_msgs topics for launch_testing:
//   subscriber_count    (UInt32)  — total messages received so far
//   validation_result   (Bool)    — cumulative AND of per-message validation
//   backend_report      (String)  — observed backend on the first nested
//                                   uint8_values of the first received msg,
//                                   useful to confirm whether zero-copy was
//                                   preserved across the RMW boundary.
//
// The `expected_backends` parameter is a comma-separated allowlist that is
// checked against the observed backend of the first nested uint8_values.
// A value of "any" (or omitting the parameter) disables the check.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rosidl_buffer/buffer.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "test_msgs/msg/multi_nested.hpp"
#include "test_msgs/msg/unbounded_sequences.hpp"

using MultiNested = test_msgs::msg::MultiNested;
using UnboundedSequences = test_msgs::msg::UnboundedSequences;

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

std::string verify_inner(
  const UnboundedSequences & inner,
  std::size_t bucket,
  std::size_t index,
  std::size_t seq)
{
  const std::size_t length = payload_length(bucket, index, seq);
  const auto expected = make_payload(bucket, index, seq, length);

  if (inner.uint8_values.size() != expected.size()) {
    std::ostringstream oss;
    oss << "bucket=" << bucket << " index=" << index
        << " size mismatch: expected " << expected.size()
        << " got " << inner.uint8_values.size();
    return oss.str();
  }
  // Buffer<uint8_t>::operator[] requires the CPU backend; for non-CPU buffers
  // (e.g. the demo backend) we materialise a CPU copy via to_vector() before
  // comparing element-wise.
  const auto received = inner.uint8_values.to_vector();
  for (std::size_t i = 0; i < expected.size(); ++i) {
    if (received[i] != expected[i]) {
      std::ostringstream oss;
      oss << "bucket=" << bucket << " index=" << index
          << " byte[" << i << "]: expected "
          << static_cast<int>(expected[i])
          << " got " << static_cast<int>(received[i]);
      return oss.str();
    }
  }
  return {};
}

}  // namespace

class MultiNestedSubscriberNode : public rclcpp::Node
{
public:
  MultiNestedSubscriberNode()
  : Node("multi_nested_subscriber")
  {
    this->declare_parameter<std::string>("topic_name", "test_multi_nested");
    this->declare_parameter<std::string>("expected_backends", "any");
    this->declare_parameter<std::string>("count_topic_prefix", "");
    this->declare_parameter<std::string>("count_topic_suffix", "");
    this->declare_parameter<std::string>(
      "acceptable_buffer_backends", "__default__");

    const auto topic_name = this->get_parameter("topic_name").as_string();
    expected_backends_str_ =
      this->get_parameter("expected_backends").as_string();
    const auto count_prefix =
      this->get_parameter("count_topic_prefix").as_string();
    const auto count_suffix =
      this->get_parameter("count_topic_suffix").as_string();
    const auto acceptable_backends =
      this->get_parameter("acceptable_buffer_backends").as_string();

    parse_expected_backends(expected_backends_str_);

    rclcpp::SubscriptionOptions sub_options;
    if (acceptable_backends != "__default__") {
      sub_options.acceptable_buffer_backends = acceptable_backends;
    }
    subscription_ = this->create_subscription<MultiNested>(
      topic_name, 10,
      std::bind(
        &MultiNestedSubscriberNode::on_message, this, std::placeholders::_1),
      sub_options);

    const auto count_topic =
      (count_prefix.empty() ? std::string{"subscriber_count"} :
      count_prefix + "_subscriber_count") +
      count_suffix;
    const auto validation_topic =
      (count_prefix.empty() ? std::string{"validation_result"} :
      count_prefix + "_validation_result") +
      count_suffix;
    const auto backend_topic =
      (count_prefix.empty() ? std::string{"backend_report"} :
      count_prefix + "_backend_report") +
      count_suffix;

    count_publisher_ =
      this->create_publisher<std_msgs::msg::UInt32>(count_topic, 10);
    validation_publisher_ =
      this->create_publisher<std_msgs::msg::Bool>(validation_topic, 10);
    backend_publisher_ =
      this->create_publisher<std_msgs::msg::String>(backend_topic, 10);

    RCLCPP_INFO(
      this->get_logger(),
      "MultiNested subscriber started (topic=%s, expected_backends=%s, "
      "acceptable_buffer_backends=%s)",
      topic_name.c_str(), expected_backends_str_.c_str(),
      acceptable_backends.c_str());
  }

private:
  void parse_expected_backends(const std::string & s)
  {
    expected_backends_.clear();
    std::string token;
    std::istringstream stream(s);
    while (std::getline(stream, token, ',')) {
      const auto start = token.find_first_not_of(" \t");
      const auto end = token.find_last_not_of(" \t");
      if (start != std::string::npos && end != std::string::npos) {
        expected_backends_.push_back(token.substr(start, end - start + 1));
      }
    }
  }

  bool backend_allowed(const std::string & backend) const
  {
    if (expected_backends_str_ == "any") {
      return true;
    }
    for (const auto & expected : expected_backends_) {
      if (expected == backend) {
        return true;
      }
    }
    return false;
  }

  void on_message(const MultiNested::SharedPtr msg)
  {
    const std::size_t seq = received_count_;
    ++received_count_;
    bool msg_valid = true;

    RCLCPP_INFO(
      this->get_logger(), "Received MultiNested #%zu", received_count_);

    // Bucket 0: fixed array[3].
    for (std::size_t i = 0; i < msg->array_of_unbounded_sequences.size(); ++i) {
      const auto err =
        verify_inner(msg->array_of_unbounded_sequences[i], 0, i, seq);
      if (!err.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Bucket0 mismatch: %s", err.c_str());
        msg_valid = false;
      }
    }

    // Bucket 1: bounded sequence — publisher fills 2 elements.
    if (msg->bounded_sequence_of_unbounded_sequences.size() != 2) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Bucket1 size mismatch: expected 2, got %zu",
        msg->bounded_sequence_of_unbounded_sequences.size());
      msg_valid = false;
    } else {
      for (std::size_t i = 0; i < msg->bounded_sequence_of_unbounded_sequences.size(); ++i) {
        const auto err = verify_inner(
          msg->bounded_sequence_of_unbounded_sequences[i], 1, i, seq);
        if (!err.empty()) {
          RCLCPP_ERROR(this->get_logger(), "Bucket1 mismatch: %s", err.c_str());
          msg_valid = false;
        }
      }
    }

    // Bucket 2: unbounded sequence — publisher sends 1 + (seq % 4) elements.
    const std::size_t expected_outer = 1 + (seq % 4);
    if (msg->unbounded_sequence_of_unbounded_sequences.size() != expected_outer) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Bucket2 size mismatch: expected %zu, got %zu",
        expected_outer,
        msg->unbounded_sequence_of_unbounded_sequences.size());
      msg_valid = false;
    } else {
      for (std::size_t i = 0; i < expected_outer; ++i) {
        const auto err = verify_inner(
          msg->unbounded_sequence_of_unbounded_sequences[i], 2, i, seq);
        if (!err.empty()) {
          RCLCPP_ERROR(this->get_logger(), "Bucket2 mismatch: %s", err.c_str());
          msg_valid = false;
        }
      }
    }

    // Report the observed backend on the first nested uint8_values.
    const auto & sample = msg->array_of_unbounded_sequences[0].uint8_values;
    const auto backend_type = sample.get_backend_type();
    if (!backend_allowed(backend_type)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Unexpected nested backend: %s (expected one of: %s)",
        backend_type.c_str(), expected_backends_str_.c_str());
      msg_valid = false;
    }

    // Duplicate detection — seq 0 starts at a bucket=0/index=0/seq=0 pattern
    // whose first byte is unique for the first ~256 messages. Use to_vector()
    // so this works for both CPU and demo backends.
    std::uint8_t first_byte = 0;
    {
      const auto & inner = msg->array_of_unbounded_sequences[0].uint8_values;
      if (!inner.empty()) {
        const auto vec = inner.to_vector();
        first_byte = vec.empty() ? 0 : vec[0];
      }
    }
    if (!seen_first_bytes_.insert(first_byte).second) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Duplicate detected: first_byte=%u was already received",
        first_byte);
      msg_valid = false;
    }

    validation_passed_ = validation_passed_ && msg_valid;

    std_msgs::msg::UInt32 count_msg;
    count_msg.data = static_cast<std::uint32_t>(received_count_);
    count_publisher_->publish(count_msg);

    std_msgs::msg::Bool validation_msg;
    validation_msg.data = validation_passed_;
    validation_publisher_->publish(validation_msg);

    std_msgs::msg::String backend_msg;
    backend_msg.data = backend_type;
    backend_publisher_->publish(backend_msg);

    RCLCPP_INFO(
      this->get_logger(),
      "MultiNested #%zu validation: %s (nested backend: %s)",
      received_count_, msg_valid ? "PASSED" : "FAILED",
      backend_type.c_str());
  }

  rclcpp::Subscription<MultiNested>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr count_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr validation_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr backend_publisher_;
  std::string expected_backends_str_;
  std::vector<std::string> expected_backends_;
  std::size_t received_count_ = 0;
  bool validation_passed_ = true;
  std::unordered_set<std::uint8_t> seen_first_bytes_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiNestedSubscriberNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
