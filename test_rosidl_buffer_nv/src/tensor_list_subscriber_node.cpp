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

// Subscriber node for isaac_ros_tensor_list_interfaces::msg::TensorList.
// Validates that the publisher's deterministic pattern for every nested
// `tensor.data` buffer round-trips correctly, regardless of whether the
// publisher used the CPU or demo backend to build them.
//
// Reports progress on three std_msgs topics for launch_testing:
//   subscriber_count    (UInt32)  — total messages received so far
//   validation_result   (Bool)    — cumulative AND of per-message validation
//   backend_report      (String)  — observed backend on the first nested
//                                   tensor.data of the latest received msg,
//                                   useful to confirm whether zero-copy was
//                                   preserved across the RMW boundary.
//
// The `expected_backends` parameter is a comma-separated allowlist that is
// checked against the observed backend of the first nested tensor.data
// Buffer. A value of "any" (or omitting the parameter) disables the check.

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
#include "isaac_ros_tensor_list_interfaces/msg/tensor.hpp"
#include "isaac_ros_tensor_list_interfaces/msg/tensor_list.hpp"
#include "isaac_ros_tensor_list_interfaces/msg/tensor_shape.hpp"

using TensorList = isaac_ros_tensor_list_interfaces::msg::TensorList;
using Tensor = isaac_ros_tensor_list_interfaces::msg::Tensor;

namespace
{

constexpr std::int32_t kDataTypeUInt8 = 2;

std::vector<uint8_t> make_payload(
  std::size_t index,
  std::size_t seq,
  std::size_t length)
{
  std::vector<uint8_t> out(length);
  for (std::size_t i = 0; i < length; ++i) {
    out[i] = static_cast<uint8_t>((index * 131 + seq * 7 + i) % 256);
  }
  return out;
}

std::size_t payload_length(std::size_t index, std::size_t seq)
{
  return 4 + ((index + seq) % 6) * 8;
}

std::string verify_tensor(
  const Tensor & tensor,
  std::size_t index,
  std::size_t seq)
{
  const std::size_t length = payload_length(index, seq);
  const auto expected = make_payload(index, seq, length);

  if (tensor.data.size() != expected.size()) {
    std::ostringstream oss;
    oss << "tensor[" << index << "] size mismatch: expected "
        << expected.size() << " got " << tensor.data.size();
    return oss.str();
  }
  // Buffer<uint8_t>::operator[] requires the CPU backend; for non-CPU buffers
  // (e.g. the demo backend) we materialise a CPU copy via to_vector() before
  // comparing element-wise.
  const auto received = tensor.data.to_vector();
  for (std::size_t i = 0; i < expected.size(); ++i) {
    if (received[i] != expected[i]) {
      std::ostringstream oss;
      oss << "tensor[" << index << "] byte[" << i << "]: expected "
          << static_cast<int>(expected[i])
          << " got " << static_cast<int>(received[i]);
      return oss.str();
    }
  }
  if (tensor.shape.rank != 2 ||
    tensor.shape.dims.size() != 2 ||
    tensor.shape.dims[0] != static_cast<std::uint32_t>(1 + index))
  {
    std::ostringstream oss;
    oss << "tensor[" << index << "] shape mismatch (rank="
        << static_cast<int>(tensor.shape.rank)
        << " dims.size=" << tensor.shape.dims.size() << ")";
    return oss.str();
  }
  if (tensor.data_type != kDataTypeUInt8) {
    std::ostringstream oss;
    oss << "tensor[" << index << "] data_type expected "
        << kDataTypeUInt8 << " got " << tensor.data_type;
    return oss.str();
  }
  return {};
}

}  // namespace

class TensorListSubscriberNode : public rclcpp::Node
{
public:
  TensorListSubscriberNode()
  : Node("tensor_list_subscriber")
  {
    this->declare_parameter<std::string>("topic_name", "test_tensor_list");
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
    subscription_ = this->create_subscription<TensorList>(
      topic_name, 10,
      std::bind(
        &TensorListSubscriberNode::on_message, this, std::placeholders::_1),
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
      "TensorList subscriber started (topic=%s, expected_backends=%s, "
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

  void on_message(const TensorList::SharedPtr msg)
  {
    const std::size_t seq = received_count_;
    ++received_count_;
    bool msg_valid = true;

    RCLCPP_INFO(
      this->get_logger(), "Received TensorList #%zu (tensors=%zu)",
      received_count_, msg->tensors.size());

    const std::size_t expected_outer = 1 + (seq % 4);
    if (msg->tensors.size() != expected_outer) {
      RCLCPP_ERROR(
        this->get_logger(),
        "TensorList size mismatch: expected %zu, got %zu",
        expected_outer, msg->tensors.size());
      msg_valid = false;
    } else {
      for (std::size_t i = 0; i < expected_outer; ++i) {
        const auto err = verify_tensor(msg->tensors[i], i, seq);
        if (!err.empty()) {
          RCLCPP_ERROR(this->get_logger(), "Validation: %s", err.c_str());
          msg_valid = false;
        }
      }
    }

    // Report the observed backend on the first tensor's data buffer.
    std::string backend_type = "<empty>";
    if (!msg->tensors.empty()) {
      backend_type = msg->tensors[0].data.get_backend_type();
      if (!backend_allowed(backend_type)) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Unexpected nested backend: %s (expected one of: %s)",
          backend_type.c_str(), expected_backends_str_.c_str());
        msg_valid = false;
      }
    }

    // Duplicate detection — for the first ~256 messages the first byte of
    // tensor[0].data is unique because it is a function of (index=0, seq).
    // Use to_vector() so this works for both CPU and demo backends.
    std::uint8_t first_byte = 0;
    if (!msg->tensors.empty() && !msg->tensors[0].data.empty()) {
      const auto vec = msg->tensors[0].data.to_vector();
      first_byte = vec.empty() ? 0 : vec[0];
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
      "TensorList #%zu validation: %s (nested backend: %s)",
      received_count_, msg_valid ? "PASSED" : "FAILED",
      backend_type.c_str());
  }

  rclcpp::Subscription<TensorList>::SharedPtr subscription_;
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
  auto node = std::make_shared<TensorListSubscriberNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
