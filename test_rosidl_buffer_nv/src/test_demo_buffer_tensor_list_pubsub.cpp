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

// Integration test: verify that a real-world "list of uint8[]" message —
// `isaac_ros_tensor_list_interfaces::msg::TensorList` — works end-to-end with
// the demo buffer backend.
//
// TensorList carries a `Tensor[] tensors` field; each Tensor has a
// `uint8[] data` field that the rosidl C++ generator materialises as
// `rosidl::Buffer<uint8_t>`. The list-of-tensors thus literally is a
// list-of-uint8[] from the rosidl::Buffer perspective.
//
// The publisher fills every Tensor's `data` field, mixing:
//   * plain CPU-backed Buffers (via resize/operator[]), and
//   * explicit demo-backend Buffers constructed from a DemoBufferImpl<uint8_t>.
//
// The subscriber asserts that every tensor's data buffer round-trips with
// the correct contents. Nested Buffers are expected to arrive on the CPU
// backend after RMW deserialisation (only the top-level field of a message
// — when applicable — is eligible for native demo-descriptor transport),
// so the test's primary contract is content equivalence, not preserved
// backend identity.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rosidl_buffer/buffer.hpp"
#include "demo_buffer/demo_buffer_impl.hpp"
#include "isaac_ros_tensor_list_interfaces/msg/tensor.hpp"
#include "isaac_ros_tensor_list_interfaces/msg/tensor_list.hpp"
#include "isaac_ros_tensor_list_interfaces/msg/tensor_shape.hpp"

using namespace std::chrono_literals;

namespace
{

using TensorList = isaac_ros_tensor_list_interfaces::msg::TensorList;
using Tensor = isaac_ros_tensor_list_interfaces::msg::Tensor;
using TensorShape = isaac_ros_tensor_list_interfaces::msg::TensorShape;
using U8Buffer = rosidl::Buffer<uint8_t>;

constexpr std::int32_t kDataTypeUInt8 = 2;

/// Deterministic data-buffer pattern. `index` is the tensor's position in
/// the list, `seq` is the publish iteration counter.
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

/// Length of the tensor `data` buffer for a given (index, seq).
std::size_t payload_length(std::size_t index, std::size_t seq)
{
  // Choose lengths that exercise both small and chunky buffers.
  return 4 + ((index + seq) % 6) * 8;
}

/// Builds a demo-backed Buffer<uint8_t> from a payload.
U8Buffer make_demo_buffer(std::vector<uint8_t> payload)
{
  auto impl = std::make_unique<demo_buffer_backend::DemoBufferImpl<uint8_t>>(
    std::move(payload));
  return U8Buffer(std::move(impl));
}

/// Fills a Tensor with a deterministic shape, name, strides, and `data`.
/// `use_demo` toggles between the plain CPU path and the demo-backend path
/// for the `data` field.
void populate_tensor(
  Tensor & tensor,
  std::size_t index,
  std::size_t seq,
  bool use_demo)
{
  std::ostringstream name;
  name << "t" << index << "_seq" << seq;
  tensor.name = name.str();
  tensor.data_type = kDataTypeUInt8;

  // Shape: a small rank-2 tensor; dims[0] = 1 + index, dims[1] derived from
  // the data length so the shape "matches" the payload size.
  const std::size_t length = payload_length(index, seq);
  tensor.shape.rank = 2;
  tensor.shape.dims.resize(2);
  tensor.shape.dims[0] = static_cast<std::uint32_t>(1 + index);
  tensor.shape.dims[1] =
    static_cast<std::uint32_t>(length / std::max<std::size_t>(1, 1 + index));

  // Strides: arbitrary deterministic values, also exercising a uint64[].
  tensor.strides.resize(2);
  tensor.strides[0] = static_cast<std::uint64_t>(length);
  tensor.strides[1] = 1u;

  auto payload = make_payload(index, seq, length);
  if (use_demo) {
    tensor.data = make_demo_buffer(std::move(payload));
  } else {
    tensor.data.resize(length);
    for (std::size_t i = 0; i < length; ++i) {
      tensor.data[i] = payload[i];
    }
  }
}

/// Verifies one Tensor against the canonical pattern. Returns an empty
/// string on success, otherwise a human-readable description of the
/// mismatch.
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
  // (e.g. the demo backend preserved via the descriptor wire path) we
  // materialise a CPU copy via to_vector() before comparing element-wise.
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

class TensorListPublisher : public rclcpp::Node
{
public:
  TensorListPublisher()
  : Node("tensor_list_demo_publisher")
  {
    publisher_ =
      this->create_publisher<TensorList>("test_tensor_list", 10);
    timer_ = this->create_wall_timer(
      100ms, std::bind(&TensorListPublisher::timer_callback, this));
    RCLCPP_INFO(
      this->get_logger(), "TensorList demo publisher started");
  }

  std::size_t get_publish_count() const {return count_;}

private:
  void timer_callback()
  {
    TensorList msg;
    msg.header.frame_id = "tensor_list_demo";
    msg.header.stamp = this->now();

    // Vary the number of tensors across messages to stress nested-buffer
    // resizing and avoid identical message shapes.
    const std::size_t num_tensors = 1 + (count_ % 4);  // 1..4
    msg.tensors.resize(num_tensors);
    for (std::size_t i = 0; i < num_tensors; ++i) {
      // Mix CPU and demo construction within a single message.
      const bool use_demo = ((i + count_) % 2 == 0);
      populate_tensor(msg.tensors[i], i, count_, use_demo);
    }

    publisher_->publish(msg);

    RCLCPP_INFO(
      this->get_logger(),
      "Published TensorList #%zu (tensors=%zu)",
      count_, num_tensors);

    ++count_;
  }

  rclcpp::Publisher<TensorList>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::size_t count_ = 0;
};

class TensorListSubscriber : public rclcpp::Node
{
public:
  TensorListSubscriber()
  : Node("tensor_list_demo_subscriber")
  {
    subscription_ = this->create_subscription<TensorList>(
      "test_tensor_list", 10,
      std::bind(
        &TensorListSubscriber::callback, this, std::placeholders::_1));
    RCLCPP_INFO(
      this->get_logger(), "TensorList demo subscriber started");
  }

  std::size_t get_received_count() const {return received_count_;}
  bool all_valid() const {return all_valid_;}
  const std::string & last_failure() const {return last_failure_;}

private:
  void callback(const TensorList::SharedPtr msg)
  {
    const std::size_t seq = received_count_;  // matches publisher count_
    ++received_count_;

    const std::size_t expected_outer = 1 + (seq % 4);
    if (msg->tensors.size() != expected_outer) {
      std::ostringstream oss;
      oss << "tensors.size() == " << msg->tensors.size()
          << " (expected " << expected_outer << ")";
      fail(oss.str());
      return;
    }

    for (std::size_t i = 0; i < expected_outer; ++i) {
      const auto err = verify_tensor(msg->tensors[i], i, seq);
      if (!err.empty()) {
        fail(err);
        return;
      }
    }

    // Sanity: every nested data Buffer should report a known backend type
    // (typically "cpu" after RMW round-trip; any future improvement that
    // preserves "demo" end-to-end should still pass this test).
    const auto & sample = msg->tensors[0].data;
    if (sample.get_backend_type().empty()) {
      fail("empty backend_type on received tensor.data Buffer");
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Received TensorList #%zu OK (tensors=%zu, nested backend=%s)",
      seq, msg->tensors.size(), sample.get_backend_type().c_str());
  }

  void fail(const std::string & reason)
  {
    all_valid_ = false;
    last_failure_ = reason;
    RCLCPP_ERROR(this->get_logger(), "Validation failed: %s", reason.c_str());
  }

  rclcpp::Subscription<TensorList>::SharedPtr subscription_;
  std::size_t received_count_ = 0;
  bool all_valid_ = true;
  std::string last_failure_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::cout << "\n=== TensorList (list-of-uint8[]) Demo Backend Pub/Sub ===\n";

  auto publisher = std::make_shared<TensorListPublisher>();
  auto subscriber = std::make_shared<TensorListSubscriber>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher);
  executor.add_node(subscriber);

  std::cout << "Waiting for discovery...\n" << std::flush;
  std::this_thread::sleep_for(1s);
  executor.spin_some(100ms);

  const auto start = std::chrono::steady_clock::now();
  constexpr auto kTimeout = 5s;
  while (std::chrono::steady_clock::now() - start < kTimeout) {
    executor.spin_some(100ms);
    if (subscriber->get_received_count() >= 3) {
      break;
    }
  }

  const bool success = subscriber->get_received_count() >= 3 &&
    subscriber->all_valid();

  std::cout << "\nTest Results:\n";
  std::cout << "  Published: " << publisher->get_publish_count() << "\n";
  std::cout << "  Received:  " << subscriber->get_received_count() << "\n";
  std::cout << "  Validation: "
            << (subscriber->all_valid() ? "OK" : "FAIL") << "\n";
  if (!subscriber->all_valid()) {
    std::cout << "  First failure: " << subscriber->last_failure() << "\n";
  }

  rclcpp::shutdown();
  publisher.reset();
  subscriber.reset();

  if (success) {
    std::cout << "\n=== ALL TESTS PASSED ===\n" << std::flush;
    return 0;
  }
  std::cout << "\n=== TESTS FAILED ===\n" << std::flush;
  return 1;
}
