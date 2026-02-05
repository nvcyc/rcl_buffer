// Copyright 2024 NVIDIA Corporation
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

#ifndef RCL_BUFFER_TEST_BACKEND__TEST_BUFFER_IMPL_HPP_
#define RCL_BUFFER_TEST_BACKEND__TEST_BUFFER_IMPL_HPP_

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <typeinfo>
#include <vector>

#include "rosidl_runtime_cpp/buffer_impl_base.hpp"
#include "rosidl_runtime_cpp/cpu_buffer_impl.hpp"
#include "rcl_buffer_test_backend_msgs/msg/test_buffer_descriptor.hpp"
#include "rmw/types.h"

namespace rcl_buffer_test_backend
{

/// Compute FNV-1a hash of data bytes.
/// This is a simple, fast, non-cryptographic hash function.
/// @param data Pointer to data bytes
/// @param size Number of bytes
/// @return 64-bit hash value
inline uint64_t compute_fnv1a_hash(const uint8_t * data, size_t size)
{
  constexpr uint64_t FNV_OFFSET_BASIS = 14695981039346656037ULL;
  constexpr uint64_t FNV_PRIME = 1099511628211ULL;

  uint64_t hash = FNV_OFFSET_BASIS;
  for (size_t i = 0; i < size; ++i) {
    hash ^= static_cast<uint64_t>(data[i]);
    hash *= FNV_PRIME;
  }
  return hash;
}

/// Test buffer implementation that stores data in a std::vector<uint8_t>.
/// This is a simple implementation for testing the buffer backend plugin system.
/// It serializes data by copying and includes a hash for verification.
template<typename T>
class TestBufferImpl : public rosidl_runtime_cpp::BufferImplBase<T>
{
public:
  TestBufferImpl() = default;

  explicit TestBufferImpl(size_t size)
  {
    storage_.resize(size);
  }

  /// Constructor from existing vector (move)
  explicit TestBufferImpl(std::vector<T> && data)
  : storage_(std::move(data)) {}

  /// Constructor from existing vector (copy)
  explicit TestBufferImpl(const std::vector<T> & data)
  : storage_(data) {}

  ~TestBufferImpl() = default;

  // Allow copy and move
  TestBufferImpl(const TestBufferImpl &) = default;
  TestBufferImpl & operator=(const TestBufferImpl &) = default;
  TestBufferImpl(TestBufferImpl &&) = default;
  TestBufferImpl & operator=(TestBufferImpl &&) = default;

  /// Get mutable reference to underlying std::vector.
  std::vector<T> & get_storage() {return storage_;}

  /// Get const reference to underlying std::vector.
  const std::vector<T> & get_storage() const {return storage_;}

  // ========== BufferImplBase interface implementation ==========

  size_t size() const override {return storage_.size();}

  void resize(size_t n) override
  {
    storage_.resize(n);
  }

  void clear() override
  {
    storage_.clear();
  }

  const void * get_backend_handle() const override
  {
    return storage_.empty() ? nullptr : storage_.data();
  }

  std::unique_ptr<rosidl_runtime_cpp::BufferImplBase<T>> to_cpu() const override
  {
    // TestBufferImpl stores data on CPU, so just copy to CpuBufferImpl
    auto cpu = std::make_unique<rosidl_runtime_cpp::CpuBufferImpl<T>>();
    cpu->get_storage() = storage_;
    return cpu;
  }

  // ========== Descriptor-based Serialization Interface ==========

  std::string get_descriptor_type_name() const override
  {
    return "rcl_buffer_test_backend_msgs/msg/TestBufferDescriptor";
  }

  std::shared_ptr<void> create_descriptor(const rmw_gid_t & subscriber_gid) const override
  {
    (void)subscriber_gid;  // Not used for test backend

    std::cerr << "[TestBufferImpl] create_descriptor() called, size=" << storage_.size()
              << " elements\n";

    auto descriptor = std::make_shared<rcl_buffer_test_backend_msgs::msg::TestBufferDescriptor>();

    descriptor->size = storage_.size();
    descriptor->element_type_name = typeid(T).name();

    // Copy data to descriptor
    descriptor->data.resize(storage_.size() * sizeof(T));
    if (!storage_.empty()) {
      std::memcpy(
        descriptor->data.data(),
        storage_.data(),
        storage_.size() * sizeof(T));
    }

    // Compute hash of the data
    descriptor->data_hash = compute_fnv1a_hash(
      reinterpret_cast<const uint8_t *>(storage_.data()),
      storage_.size() * sizeof(T));

    std::cerr << "[TestBufferImpl] Descriptor created: size=" << descriptor->size
              << ", data_hash=" << descriptor->data_hash << "\n";

    return descriptor;
  }

  std::unique_ptr<rosidl_runtime_cpp::BufferImplBase<T>> from_descriptor(
    const std::shared_ptr<void> & descriptor_ptr,
    const rmw_gid_t & publisher_gid) const override
  {
    (void)publisher_gid;  // Not used for test backend

    auto descriptor = std::static_pointer_cast<rcl_buffer_test_backend_msgs::msg::TestBufferDescriptor>(
      descriptor_ptr);

    std::cerr << "[TestBufferImpl] from_descriptor() called, size=" << descriptor->size
              << " elements, data_hash=" << descriptor->data_hash << "\n";

    // Validate element type
    if (descriptor->element_type_name != typeid(T).name()) {
      throw std::runtime_error(
              "TestBufferDescriptor element type mismatch: expected " +
              std::string(typeid(T).name()) + ", got " +
              descriptor->element_type_name);
    }

    // Create new buffer and copy data
    auto impl = std::make_unique<TestBufferImpl<T>>(descriptor->size);

    if (descriptor->size > 0 && !descriptor->data.empty()) {
      std::memcpy(
        impl->storage_.data(),
        descriptor->data.data(),
        descriptor->size * sizeof(T));

      // Verify hash
      uint64_t computed_hash = compute_fnv1a_hash(
        reinterpret_cast<const uint8_t *>(impl->storage_.data()),
        impl->storage_.size() * sizeof(T));

      if (computed_hash != descriptor->data_hash) {
        std::cerr << "[TestBufferImpl] WARNING: Hash mismatch! Expected "
                  << descriptor->data_hash << ", computed " << computed_hash << "\n";
        throw std::runtime_error("TestBufferDescriptor hash verification failed");
      }

      std::cerr << "[TestBufferImpl] Hash verified successfully\n";
    }

    return impl;
  }

  std::unique_ptr<rosidl_runtime_cpp::BufferImplBase<T>> clone() const override
  {
    auto copy = std::make_unique<TestBufferImpl<T>>();
    copy->storage_ = storage_;  // Deep copy
    return copy;
  }

private:
  std::vector<T> storage_;
};

}  // namespace rcl_buffer_test_backend

#endif  // RCL_BUFFER_TEST_BACKEND__TEST_BUFFER_IMPL_HPP_
