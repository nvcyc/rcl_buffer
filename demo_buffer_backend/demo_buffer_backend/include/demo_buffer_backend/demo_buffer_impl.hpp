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

#ifndef DEMO_BUFFER_BACKEND__DEMO_BUFFER_IMPL_HPP_
#define DEMO_BUFFER_BACKEND__DEMO_BUFFER_IMPL_HPP_

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <typeinfo>
#include <vector>

#include "rcl_buffer/buffer_impl_base.hpp"
#include "rcl_buffer/cpu_buffer_impl.hpp"
#include "demo_buffer_backend_msgs/msg/demo_buffer_descriptor.hpp"
#include "rmw/types.h"

namespace demo_buffer_backend
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

/// Demo buffer implementation that stores data in a std::vector<uint8_t>.
/// This is a simple implementation for demonstrating the buffer backend plugin system.
/// It serializes data by copying and includes a hash for verification.
template<typename T>
class DemoBufferImpl : public rcl_buffer::BufferImplBase<T>
{
public:
  DemoBufferImpl() = default;

  explicit DemoBufferImpl(size_t size)
  {
    storage_.resize(size);
  }

  /// Constructor from existing vector (move)
  explicit DemoBufferImpl(std::vector<T> && data)
  : storage_(std::move(data)) {}

  /// Constructor from existing vector (copy)
  explicit DemoBufferImpl(const std::vector<T> & data)
  : storage_(data) {}

  ~DemoBufferImpl() = default;

  // Allow copy and move
  DemoBufferImpl(const DemoBufferImpl &) = default;
  DemoBufferImpl & operator=(const DemoBufferImpl &) = default;
  DemoBufferImpl(DemoBufferImpl &&) = default;
  DemoBufferImpl & operator=(DemoBufferImpl &&) = default;

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

  std::unique_ptr<rcl_buffer::BufferImplBase<T>> to_cpu() const override
  {
    // DemoBufferImpl stores data on CPU, so just copy to CpuBufferImpl
    auto cpu = std::make_unique<rcl_buffer::CpuBufferImpl<T>>();
    cpu->get_storage() = storage_;
    return cpu;
  }

  // ========== Descriptor-based Serialization Interface ==========

  std::string get_descriptor_type_name() const override
  {
    return "demo_buffer_backend_msgs/msg/DemoBufferDescriptor";
  }

  std::shared_ptr<void> create_descriptor(const rmw_gid_t & subscriber_gid) const override
  {
    (void)subscriber_gid;  // Not used for demo backend

    std::cerr << "[DemoBufferImpl] create_descriptor() called, size=" << storage_.size()
              << " elements\n";

    auto descriptor = std::make_shared<demo_buffer_backend_msgs::msg::DemoBufferDescriptor>();

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

    std::cerr << "[DemoBufferImpl] Descriptor created: size=" << descriptor->size
              << ", data_hash=" << descriptor->data_hash << "\n";

    return descriptor;
  }

  std::unique_ptr<rcl_buffer::BufferImplBase<T>> from_descriptor(
    const std::shared_ptr<void> & descriptor_ptr,
    const rmw_gid_t & publisher_gid) const override
  {
    (void)publisher_gid;  // Not used for demo backend

    auto descriptor = std::static_pointer_cast<demo_buffer_backend_msgs::msg::DemoBufferDescriptor>(
      descriptor_ptr);

    std::cerr << "[DemoBufferImpl] from_descriptor() called, size=" << descriptor->size
              << " elements, data_hash=" << descriptor->data_hash << "\n";

    // Validate element type
    if (descriptor->element_type_name != typeid(T).name()) {
      throw std::runtime_error(
              "DemoBufferDescriptor element type mismatch: expected " +
              std::string(typeid(T).name()) + ", got " +
              descriptor->element_type_name);
    }

    // Create new buffer and copy data
    auto impl = std::make_unique<DemoBufferImpl<T>>(descriptor->size);

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
        std::cerr << "[DemoBufferImpl] WARNING: Hash mismatch! Expected "
                  << descriptor->data_hash << ", computed " << computed_hash << "\n";
        throw std::runtime_error("DemoBufferDescriptor hash verification failed");
      }

      std::cerr << "[DemoBufferImpl] Hash verified successfully\n";
    }

    return impl;
  }

  std::unique_ptr<rcl_buffer::BufferImplBase<T>> clone() const override
  {
    auto copy = std::make_unique<DemoBufferImpl<T>>();
    copy->storage_ = storage_;  // Deep copy
    return copy;
  }

private:
  std::vector<T> storage_;
};

}  // namespace demo_buffer_backend

#endif  // DEMO_BUFFER_BACKEND__DEMO_BUFFER_IMPL_HPP_
