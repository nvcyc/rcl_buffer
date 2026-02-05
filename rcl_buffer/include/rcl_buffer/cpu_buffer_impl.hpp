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

#ifndef RCL_BUFFER__CPU_BUFFER_IMPL_HPP_
#define RCL_BUFFER__CPU_BUFFER_IMPL_HPP_

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rcl_buffer/buffer_impl_base.hpp"

namespace rcl_buffer
{

/// CPU buffer implementation wrapping std::vector.
/// Provides the reference implementation for CPU memory buffers.
template<typename T>
class CpuBufferImpl : public BufferImplBase<T>
{
public:
  CpuBufferImpl() = default;

  /// Get mutable reference to underlying std::vector.
  std::vector<T> & get_storage() {return storage_;}

  /// Get const reference to underlying std::vector.
  const std::vector<T> & get_storage() const {return storage_;}

  // BufferImplBase interface implementation
  size_t size() const override {return storage_.size();}

  void resize(size_t n) override {storage_.resize(n);}

  void clear() override {storage_.clear();}

  const void * get_backend_handle() const override
  {
    return storage_.empty() ? nullptr : storage_.data();
  }

  std::unique_ptr<BufferImplBase<T>> to_cpu() const override
  {
    auto copy = std::make_unique<CpuBufferImpl<T>>();
    copy->storage_ = storage_;
    return copy;
  }

  // ========== Descriptor-based Serialization Interface ==========
  // CPU backend doesn't use descriptors - serializes directly as std::vector

  std::string get_descriptor_type_name() const override
  {
    // Not used for CPU backend
    return "std::vector";
  }

  std::shared_ptr<void> create_descriptor(const rmw_gid_t & subscriber_gid) const override
  {
    // Not used for CPU backend - serializes directly as std::vector
    (void)subscriber_gid;
    throw std::runtime_error("CPU backend does not use descriptors");
  }

  std::unique_ptr<BufferImplBase<T>> from_descriptor(
    const std::shared_ptr<void> & descriptor,
    const rmw_gid_t & publisher_gid) const override
  {
    // Not used for CPU backend - deserializes directly as std::vector
    (void)descriptor;
    (void)publisher_gid;
    throw std::runtime_error("CPU backend does not use descriptors");
  }

  std::unique_ptr<BufferImplBase<T>> clone() const override
  {
    auto copy = std::make_unique<CpuBufferImpl<T>>();
    copy->storage_ = storage_;  // Deep copy of vector
    return copy;
  }

private:
  std::vector<T> storage_;
};

}  // namespace rcl_buffer

#endif  // RCL_BUFFER__CPU_BUFFER_IMPL_HPP_
