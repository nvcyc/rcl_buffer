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

// Publisher node for isaac_ros_tensor_list_interfaces::msg::TensorList over
// rclcpp.
//
// TensorList carries a `Tensor[]` field; each Tensor has a `uint8[] data`
// field that the rosidl C++ generator emits as `rosidl::Buffer<uint8_t>`.
// This means the outer message is literally a "list of uint8[]". The
// `backend_mode` parameter switches how every tensor's `data` buffer is
// constructed:
//
//   * "cpu"  — resize() + operator[] (CPU-backed Buffer, default behavior)
//   * "demo" — DemoBufferImpl<uint8_t> wrapped in rosidl::Buffer<uint8_t>
//
// This lets us exercise both construction paths for nested Buffer fields
// in pub/sub launch tests (demo-to-demo, cpu-to-cpu, demo-to-cpu).

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rosidl_buffer/buffer.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "demo_buffer/demo_buffer_impl.hpp"
#include "isaac_ros_tensor_list_interfaces/msg/tensor.hpp"
#include "isaac_ros_tensor_list_interfaces/msg/tensor_list.hpp"
#include "isaac_ros_tensor_list_interfaces/msg/tensor_shape.hpp"

using namespace std::chrono_literals;
using TensorList = isaac_ros_tensor_list_interfaces::msg::TensorList;
using Tensor = isaac_ros_tensor_list_interfaces::msg::Tensor;
using U8Buffer = rosidl::Buffer<uint8_t>;

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

void populate_tensor(
  Tensor & tensor,
  std::size_t index,
  std::size_t seq,
  const std::string & backend_mode)
{
  std::ostringstream name;
  name << "t" << index << "_seq" << seq;
  tensor.name = name.str();
  tensor.data_type = kDataTypeUInt8;

  const std::size_t length = payload_length(index, seq);
  tensor.shape.rank = 2;
  tensor.shape.dims.resize(2);
  tensor.shape.dims[0] = static_cast<std::uint32_t>(1 + index);
  tensor.shape.dims[1] =
    static_cast<std::uint32_t>(length / std::max<std::size_t>(1, 1 + index));

  tensor.strides.resize(2);
  tensor.strides[0] = static_cast<std::uint64_t>(length);
  tensor.strides[1] = 1u;

  auto payload = make_payload(index, seq, length);
  if (backend_mode == "demo") {
    auto impl = std::make_unique<demo_buffer_backend::DemoBufferImpl<uint8_t>>(
      std::move(payload));
    tensor.data = U8Buffer(std::move(impl));
  } else {
    tensor.data.resize(length);
    for (std::size_t i = 0; i < length; ++i) {
      tensor.data[i] = payload[i];
    }
  }
}

}  // namespace

class TensorListPublisherNode : public rclcpp::Node
{
public:
  TensorListPublisherNode()
  : Node("tensor_list_publisher")
  {
    this->declare_parameter<std::string>("backend_mode", "demo");
    this->declare_parameter<std::string>("topic_name", "test_tensor_list");
    this->declare_parameter<int>("publish_rate_ms", 200);
    this->declare_parameter<std::string>("count_topic_prefix", "");
    this->declare_parameter<int>("max_publish_count", 0);
    this->declare_parameter<int>("expected_subscription_count", 0);

    backend_mode_ = this->get_parameter("backend_mode").as_string();
    const auto topic_name = this->get_parameter("topic_name").as_string();
    const auto publish_rate_ms = this->get_parameter("publish_rate_ms").as_int();
    const auto count_prefix =
      this->get_parameter("count_topic_prefix").as_string();
    max_publish_count_ =
      static_cast<std::size_t>(this->get_parameter("max_publish_count").as_int());
    const auto expected_subscription_count =
      this->get_parameter("expected_subscription_count").as_int();
    expected_subscription_count_ = expected_subscription_count > 0 ?
      static_cast<std::size_t>(expected_subscription_count) : 0u;

    if (backend_mode_ != "cpu" && backend_mode_ != "demo") {
      RCLCPP_ERROR(
        this->get_logger(),
        "Invalid backend_mode: %s. Must be 'cpu' or 'demo'",
        backend_mode_.c_str());
      backend_mode_ = "cpu";
    }

    publisher_ = this->create_publisher<TensorList>(topic_name, 10);

    const auto count_topic = count_prefix.empty() ?
      std::string{"publisher_count"} :
    count_prefix + "_publisher_count";
    count_publisher_ =
      this->create_publisher<std_msgs::msg::UInt32>(count_topic, 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(publish_rate_ms),
      std::bind(&TensorListPublisherNode::on_timer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "TensorList publisher started "
      "(backend_mode=%s, topic=%s, max_count=%zu, expected_subscriptions=%zu)",
      backend_mode_.c_str(), topic_name.c_str(),
      max_publish_count_ == 0 ? SIZE_MAX : max_publish_count_,
      expected_subscription_count_);
  }

private:
  void on_timer()
  {
    if (max_publish_count_ > 0 && count_ >= max_publish_count_) {
      return;
    }
    if (publisher_->get_subscription_count() < expected_subscription_count_) {
      return;
    }

    TensorList msg;
    msg.header.frame_id = "tensor_list_launch";
    msg.header.stamp = this->now();

    // Vary the number of tensors per message: 1..4. This forces nested
    // Buffer fields to resize across messages and exercises a real
    // "list of uint8[]" of varying length.
    const std::size_t num_tensors = 1 + (count_ % 4);
    msg.tensors.resize(num_tensors);
    for (std::size_t i = 0; i < num_tensors; ++i) {
      populate_tensor(msg.tensors[i], i, count_, backend_mode_);
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Publishing TensorList #%zu (backend_mode=%s, tensors=%zu)",
      count_ + 1, backend_mode_.c_str(), num_tensors);
    publisher_->publish(msg);

    auto count_msg = std_msgs::msg::UInt32();
    count_msg.data = static_cast<std::uint32_t>(++count_);
    count_publisher_->publish(count_msg);
  }

  rclcpp::Publisher<TensorList>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr count_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string backend_mode_;
  std::size_t count_ = 0;
  std::size_t max_publish_count_ = 0;
  std::size_t expected_subscription_count_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TensorListPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
