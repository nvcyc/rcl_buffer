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

// Integration test: verify that a message type containing a "list of uint8[]"
// — i.e. a sequence of sub-messages whose fields include `uint8[]` (generated
// as `rosidl::Buffer<uint8_t>`) — works end-to-end with the demo buffer
// backend.
//
// We reuse the existing `test_msgs/msg/MultiNested` fixture, which embeds
// `test_msgs/msg/UnboundedSequences` in three different shapes:
//
//   UnboundedSequences[3]    array_of_unbounded_sequences
//   UnboundedSequences[<=3]  bounded_sequence_of_unbounded_sequences
//   UnboundedSequences[]     unbounded_sequence_of_unbounded_sequences
//
// Each inner UnboundedSequences has a `uint8[] uint8_values` field which the
// rosidl C++ generator materialises as `rosidl::Buffer<uint8_t>`. This is
// the literal "list of uint8[]" the test is exercising.
//
// The publisher populates every inner `uint8_values` field, mixing:
//   * plain CPU-backed Buffers (via resize/operator[]), and
//   * explicit demo-backend Buffers constructed from a DemoBufferImpl<uint8_t>.
//
// The subscriber asserts that every inner buffer round-trips with the
// correct contents. Nested Buffers are expected to arrive on the CPU backend
// after RMW deserialisation (only the top-level field of a message — when
// applicable — is eligible for native demo-descriptor transport), so the
// test's primary contract is content equivalence, not preserved backend
// identity.

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
#include "test_msgs/msg/multi_nested.hpp"
#include "test_msgs/msg/unbounded_sequences.hpp"

using namespace std::chrono_literals;

namespace
{

using MultiNested = test_msgs::msg::MultiNested;
using UnboundedSequences = test_msgs::msg::UnboundedSequences;
using U8Buffer = rosidl::Buffer<uint8_t>;

/// Deterministic pattern so the subscriber can reconstruct exactly what the
/// publisher built. `bucket` distinguishes the three sequence shapes inside
/// MultiNested; `index` is the position within that shape; `seq` is the
/// publish iteration counter.
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

/// Builds a demo-backed Buffer<uint8_t> from the canonical payload.
U8Buffer make_demo_buffer(std::vector<uint8_t> payload)
{
  auto impl = std::make_unique<demo_buffer_backend::DemoBufferImpl<uint8_t>>(
    std::move(payload));
  return U8Buffer(std::move(impl));
}

/// Fills one inner UnboundedSequences element. `use_demo` toggles between
/// the plain CPU path (resize + operator[]) and the demo-backend path.
void populate_inner(
  UnboundedSequences & inner,
  std::size_t bucket,
  std::size_t index,
  std::size_t seq,
  bool use_demo)
{
  // Pick lengths that exercise both empty-ish and sizeable buffers.
  const std::size_t length = 1 + ((bucket + index + seq) % 7) * 4;
  auto payload = make_payload(bucket, index, seq, length);

  if (use_demo) {
    inner.uint8_values = make_demo_buffer(payload);
  } else {
    inner.uint8_values.resize(length);
    for (std::size_t i = 0; i < length; ++i) {
      inner.uint8_values[i] = payload[i];
    }
  }
}

/// Verifies one inner UnboundedSequences element against the canonical
/// payload. Returns an empty string on success, otherwise a human-readable
/// description of the mismatch.
std::string verify_inner(
  const UnboundedSequences & inner,
  std::size_t bucket,
  std::size_t index,
  std::size_t seq)
{
  const std::size_t length = 1 + ((bucket + index + seq) % 7) * 4;
  const auto expected = make_payload(bucket, index, seq, length);

  if (inner.uint8_values.size() != expected.size()) {
    std::ostringstream oss;
    oss << "bucket=" << bucket << " index=" << index
        << " size mismatch: expected " << expected.size()
        << " got " << inner.uint8_values.size();
    return oss.str();
  }
  // Buffer<uint8_t>::operator[] requires the CPU backend; for non-CPU buffers
  // (e.g. the demo backend preserved via the descriptor wire path) we
  // materialise a CPU copy via to_vector() before comparing element-wise.
  const auto received = inner.uint8_values.to_vector();
  for (std::size_t i = 0; i < expected.size(); ++i) {
    if (received[i] != expected[i]) {
      std::ostringstream oss;
      oss << "bucket=" << bucket << " index=" << index
          << " byte[" << i << "]: expected " << static_cast<int>(expected[i])
          << " got " << static_cast<int>(received[i]);
      return oss.str();
    }
  }
  return {};
}

}  // namespace

class MultiNestedPublisher : public rclcpp::Node
{
public:
  MultiNestedPublisher()
  : Node("multi_nested_demo_publisher")
  {
    publisher_ =
      this->create_publisher<MultiNested>("test_multi_nested", 10);
    timer_ = this->create_wall_timer(
      100ms, std::bind(&MultiNestedPublisher::timer_callback, this));
    RCLCPP_INFO(
      this->get_logger(), "MultiNested demo publisher started");
  }

  std::size_t get_publish_count() const {return count_;}

private:
  void timer_callback()
  {
    MultiNested msg;

    // Bucket 0: fixed-size array (exactly 3 elements). Use a mix: the first
    // element is CPU-backed, the others demo-backed.
    for (std::size_t i = 0; i < msg.array_of_unbounded_sequences.size(); ++i) {
      const bool use_demo = (i != 0);
      populate_inner(
        msg.array_of_unbounded_sequences[i], 0, i, count_, use_demo);
    }

    // Bucket 1: bounded sequence. Exercise 2 of the up-to-3 slots. Mix again
    // but flip the pattern so ordering is clearly different from bucket 0.
    msg.bounded_sequence_of_unbounded_sequences.resize(2);
    for (std::size_t i = 0; i < msg.bounded_sequence_of_unbounded_sequences.size(); ++i) {
      const bool use_demo = (i == 0);
      populate_inner(
        msg.bounded_sequence_of_unbounded_sequences[i], 1, i, count_, use_demo);
    }

    // Bucket 2: unbounded sequence — the core "list of uint8[]" shape.
    // Make the length depend on the publish count so we also cover a
    // run of differently-sized outer sequences across messages.
    const std::size_t outer_len = 1 + (count_ % 4);  // 1..4
    msg.unbounded_sequence_of_unbounded_sequences.resize(outer_len);
    for (std::size_t i = 0; i < outer_len; ++i) {
      const bool use_demo = ((i + count_) % 2 == 0);
      populate_inner(
        msg.unbounded_sequence_of_unbounded_sequences[i],
        2, i, count_, use_demo);
    }

    publisher_->publish(msg);

    RCLCPP_INFO(
      this->get_logger(),
      "Published MultiNested #%zu "
      "(fixed=%zu, bounded=%zu, unbounded=%zu)",
      count_,
      msg.array_of_unbounded_sequences.size(),
      msg.bounded_sequence_of_unbounded_sequences.size(),
      msg.unbounded_sequence_of_unbounded_sequences.size());

    ++count_;
  }

  rclcpp::Publisher<MultiNested>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::size_t count_ = 0;
};

class MultiNestedSubscriber : public rclcpp::Node
{
public:
  MultiNestedSubscriber()
  : Node("multi_nested_demo_subscriber")
  {
    subscription_ = this->create_subscription<MultiNested>(
      "test_multi_nested", 10,
      std::bind(
        &MultiNestedSubscriber::callback, this, std::placeholders::_1));
    RCLCPP_INFO(
      this->get_logger(), "MultiNested demo subscriber started");
  }

  std::size_t get_received_count() const {return received_count_;}
  bool all_valid() const {return all_valid_;}
  const std::string & last_failure() const {return last_failure_;}

private:
  void callback(const MultiNested::SharedPtr msg)
  {
    const std::size_t seq = received_count_;  // matches publisher count_
    ++received_count_;

    // Bucket 0: array_of_unbounded_sequences has exactly 3 elements.
    for (std::size_t i = 0; i < msg->array_of_unbounded_sequences.size(); ++i) {
      const auto err =
        verify_inner(msg->array_of_unbounded_sequences[i], 0, i, seq);
      if (!err.empty()) {
        fail(err);
        return;
      }
    }

    // Bucket 1: bounded sequence — publisher resized it to 2.
    if (msg->bounded_sequence_of_unbounded_sequences.size() != 2) {
      std::ostringstream oss;
      oss << "bounded_sequence_of_unbounded_sequences.size() == "
          << msg->bounded_sequence_of_unbounded_sequences.size()
          << " (expected 2)";
      fail(oss.str());
      return;
    }
    for (std::size_t i = 0; i < msg->bounded_sequence_of_unbounded_sequences.size(); ++i) {
      const auto err = verify_inner(
        msg->bounded_sequence_of_unbounded_sequences[i], 1, i, seq);
      if (!err.empty()) {
        fail(err);
        return;
      }
    }

    // Bucket 2: unbounded sequence — publisher uses 1 + (seq % 4) elements.
    const std::size_t expected_outer = 1 + (seq % 4);
    if (msg->unbounded_sequence_of_unbounded_sequences.size() != expected_outer) {
      std::ostringstream oss;
      oss << "unbounded_sequence_of_unbounded_sequences.size() == "
          << msg->unbounded_sequence_of_unbounded_sequences.size()
          << " (expected " << expected_outer << ")";
      fail(oss.str());
      return;
    }
    for (std::size_t i = 0; i < expected_outer; ++i) {
      const auto err = verify_inner(
        msg->unbounded_sequence_of_unbounded_sequences[i], 2, i, seq);
      if (!err.empty()) {
        fail(err);
        return;
      }
    }

    // Sanity: confirm every nested Buffer reports a known backend type
    // (should be "cpu" after RMW round-trip, but we only require a
    // non-empty string — any future improvement that preserves "demo"
    // end-to-end should still pass this test).
    const auto & sample = msg->array_of_unbounded_sequences[0].uint8_values;
    if (sample.get_backend_type().empty()) {
      fail("empty backend_type on received nested Buffer");
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Received MultiNested #%zu OK (nested backend=%s)",
      seq, sample.get_backend_type().c_str());
  }

  void fail(const std::string & reason)
  {
    all_valid_ = false;
    last_failure_ = reason;
    RCLCPP_ERROR(this->get_logger(), "Validation failed: %s", reason.c_str());
  }

  rclcpp::Subscription<MultiNested>::SharedPtr subscription_;
  std::size_t received_count_ = 0;
  bool all_valid_ = true;
  std::string last_failure_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::cout << "\n=== MultiNested (list-of-uint8[]) Demo Backend Pub/Sub ===\n";

  auto publisher = std::make_shared<MultiNestedPublisher>();
  auto subscriber = std::make_shared<MultiNestedSubscriber>();

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
