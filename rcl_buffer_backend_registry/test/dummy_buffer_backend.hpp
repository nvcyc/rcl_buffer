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

#ifndef TEST__DUMMY_BUFFER_BACKEND_HPP_
#define TEST__DUMMY_BUFFER_BACKEND_HPP_

#include <cstring>
#include <memory>
#include <vector>

#include "rcl_buffer_backend/buffer_backend.hpp"
#include "rcl_buffer/buffer_impl_base.hpp"
#include "rcl_buffer/cpu_buffer_impl.hpp"

// For pluginlib export
#include <pluginlib/class_list_macros.hpp>

namespace rcl_buffer_backend_registry
{
namespace test
{

/// Dummy backend implementation for testing.
/// Acts like CPU backend but adds a marker to verify it's being used.
template<typename T>
class DummyBufferImpl : public rcl_buffer::BufferImplBase<T>
{
public:
  DummyBufferImpl()
  : marker_(0xDEADBEEF) {}

  explicit DummyBufferImpl(size_t size)
  : marker_(0xDEADBEEF)
  {
    data_.resize(size);
  }

  size_t size() const override {return data_.size();}
  void resize(size_t n) override {data_.resize(n);}
  void clear() override {data_.clear();}

  const void * get_backend_handle() const override
  {
    return data_.empty() ? nullptr : data_.data();
  }

  std::unique_ptr<rcl_buffer::BufferImplBase<T>> to_cpu() const override
  {
    auto cpu = std::make_unique<rcl_buffer::CpuBufferImpl<T>>();
    cpu->get_storage() = data_;
    return cpu;
  }

  std::string get_descriptor_type_name() const override
  {
    return "dummy_descriptor";
  }

  std::shared_ptr<void> create_descriptor(const rmw_gid_t & subscriber_gid) const override
  {
    // Return nullptr - not used in these tests
    (void)subscriber_gid;
    return nullptr;
  }

  std::unique_ptr<rcl_buffer::BufferImplBase<T>> from_descriptor(
    const std::shared_ptr<void> & descriptor,
    const rmw_gid_t & publisher_gid) const override
  {
    // Return nullptr - not used in these tests
    (void)descriptor;
    (void)publisher_gid;
    return nullptr;
  }

  std::unique_ptr<rcl_buffer::BufferImplBase<T>> clone() const override
  {
    auto cloned = std::make_unique<DummyBufferImpl<T>>();
    cloned->data_ = data_;
    cloned->marker_ = marker_;
    return cloned;
  }

  std::vector<T> & get_data() {return data_;}
  const std::vector<T> & get_data() const {return data_;}

  uint32_t get_marker() const {return marker_;}

private:
  std::vector<T> data_;
  uint32_t marker_;  // Marker to identify this backend
};

/// Dummy backend for testing the BufferBackend interface.
/// Implements all 4+4 methods with test markers.
class DummyBufferBackend : public rcl_buffer::BufferBackend
{
public:
  DummyBufferBackend()
  : call_count_(0) {}

  // Track which methods were called
  mutable size_t call_count_;
  mutable std::string last_method_;

  std::string get_backend_type() const override
  {
    return "dummy";
  }

  std::string get_descriptor_type_name() const override
  {
    return "dummy_descriptor";
  }

  std::shared_ptr<void> create_descriptor_with_endpoint(
    const std::shared_ptr<void> & impl,
    const rmw_topic_endpoint_info_t & endpoint_info) const override
  {
    // Return nullptr - not used in these tests
    (void)impl;
    (void)endpoint_info;
    return nullptr;
  }

  std::shared_ptr<void> from_descriptor_with_endpoint(
    const std::shared_ptr<void> & descriptor,
    const rmw_topic_endpoint_info_t & endpoint_info) const override
  {
    // Return nullptr - not used in these tests
    (void)descriptor;
    (void)endpoint_info;
    return nullptr;
  }
};

}  // namespace test
}  // namespace rcl_buffer_backend_registry

// Export the DummyBufferBackend as a pluginlib plugin
PLUGINLIB_EXPORT_CLASS(
  rcl_buffer_backend_registry::test::DummyBufferBackend,
  rcl_buffer::BufferBackend)

#endif  // TEST__DUMMY_BUFFER_BACKEND_HPP_
