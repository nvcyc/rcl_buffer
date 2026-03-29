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

#include "demo_buffer_backend/demo_buffer_backend.hpp"

#include <rcutils/logging_macros.h>

#include <pluginlib/class_list_macros.hpp>

#include "demo_buffer_backend_msgs/msg/demo_buffer_descriptor.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"

namespace demo_buffer_backend
{

//==============================================================================
DemoBufferBackend::DemoBufferBackend()
{
  RCUTILS_LOG_INFO_NAMED("demo_buffer_backend", "DemoBufferBackend created");
}

//==============================================================================
const rosidl_message_type_support_t *
DemoBufferBackend::get_descriptor_type_support() const
{
  return rosidl_typesupport_cpp::get_message_type_support_handle<
    demo_buffer_backend_msgs::msg::DemoBufferDescriptor>();
}

//==============================================================================
std::shared_ptr<void>
DemoBufferBackend::create_empty_descriptor() const
{
  return std::make_shared<demo_buffer_backend_msgs::msg::DemoBufferDescriptor>();
}

//==============================================================================
void DemoBufferBackend::on_creating_endpoint(
  const rmw_topic_endpoint_info_t & endpoint_info) const
{
  RCUTILS_LOG_INFO_NAMED(
    "demo_buffer_backend",
    "on_creating_endpoint() called for topic: %s (endpoint_type=%d)",
    endpoint_info.topic_type,
    static_cast<int>(endpoint_info.endpoint_type));
}

//==============================================================================
std::pair<bool, std::vector<std::set<uint32_t>>>
DemoBufferBackend::on_discovering_endpoint(
  const rmw_topic_endpoint_info_t & endpoint_info,
  const std::vector<rmw_topic_endpoint_info_t> & existing_endpoints,
  const std::unordered_map<std::string, std::string> & endpoint_supported_backends)
{
  (void)existing_endpoints;

  bool supports_demo = endpoint_supported_backends.find("demo") !=
    endpoint_supported_backends.end();

  if (supports_demo) {
    RCUTILS_LOG_INFO_NAMED(
      "demo_buffer_backend",
      "Discovered endpoint supports 'demo' backend for topic: %s",
      endpoint_info.topic_type);
  } else {
    RCUTILS_LOG_INFO_NAMED(
      "demo_buffer_backend",
      "Discovered endpoint does NOT support 'demo' backend");
  }

  {
    std::lock_guard<std::mutex> lock(compat_mutex_);
    endpoint_compat_cache_[gid_hash(endpoint_info.endpoint_gid)] = supports_demo;
  }

  return {supports_demo, {}};
}

//==============================================================================
std::shared_ptr<void> DemoBufferBackend::create_descriptor_with_endpoint(
  const void * impl,
  const rmw_topic_endpoint_info_t & endpoint_info) const
{
  {
    std::lock_guard<std::mutex> lock(compat_mutex_);
    auto it = endpoint_compat_cache_.find(gid_hash(endpoint_info.endpoint_gid));
    if (it != endpoint_compat_cache_.end() && !it->second) {
      return nullptr;
    }
  }

  const auto * demo_impl = static_cast<const DemoBufferImpl<uint8_t> *>(impl);

  rmw_gid_t dummy_gid;
  std::memset(&dummy_gid, 0, sizeof(dummy_gid));

  return demo_impl->create_descriptor(dummy_gid);
}

//==============================================================================
std::unique_ptr<void, void (*)(void *)> DemoBufferBackend::from_descriptor_with_endpoint(
  const void * descriptor,
  const rmw_topic_endpoint_info_t & endpoint_info) const
{
  (void)endpoint_info;

  DemoBufferImpl<uint8_t> temp_impl;

  rmw_gid_t dummy_gid;
  std::memset(&dummy_gid, 0, sizeof(dummy_gid));

  auto result = temp_impl.from_descriptor(descriptor, dummy_gid);
  return {result.release(), [](void * p) {
      delete static_cast<rosidl::BufferImplBase<uint8_t> *>(p);
    }};
}

}  // namespace demo_buffer_backend

// Export the DemoBufferBackend as a pluginlib plugin
PLUGINLIB_EXPORT_CLASS(
  demo_buffer_backend::DemoBufferBackend,
  rosidl::BufferBackend)
