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

#include <pluginlib/class_list_macros.hpp>
#include <rcutils/logging_macros.h>

#include "demo_buffer_backend_msgs/msg/demo_buffer_descriptor.hpp"
#include "rcl_buffer_backend/register_buffer_descriptor.hpp"

namespace demo_buffer_backend
{

//==============================================================================
DemoBufferBackend::DemoBufferBackend()
{
  RCUTILS_LOG_INFO_NAMED("demo_buffer_backend", "DemoBufferBackend created");

  // Register FastCDR descriptor serializers automatically using the
  // rosidl-generated type support for DemoBufferDescriptor.
  rcl_buffer::register_buffer_descriptor<
    demo_buffer_backend_msgs::msg::DemoBufferDescriptor>(get_backend_type());
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

  // Check if discovered endpoint supports demo backend
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

  // Demo backend is always compatible with other demo backends
  // No special grouping logic needed
  return {supports_demo, {}};
}

//==============================================================================
std::shared_ptr<void> DemoBufferBackend::create_descriptor_with_endpoint(
  const std::shared_ptr<void> & impl,
  const rmw_topic_endpoint_info_t & endpoint_info) const
{
  (void)endpoint_info;  // Demo backend doesn't use endpoint info for descriptor

  auto demo_impl = std::static_pointer_cast<DemoBufferImpl<uint8_t>>(impl);

  // Create descriptor using the impl's create_descriptor method
  rmw_gid_t dummy_gid;
  std::memset(&dummy_gid, 0, sizeof(dummy_gid));

  return demo_impl->create_descriptor(dummy_gid);
}

//==============================================================================
std::shared_ptr<void> DemoBufferBackend::from_descriptor_with_endpoint(
  const std::shared_ptr<void> & descriptor,
  const rmw_topic_endpoint_info_t & endpoint_info) const
{
  (void)endpoint_info;  // Demo backend doesn't use endpoint info for descriptor

  auto temp_impl = std::make_shared<DemoBufferImpl<uint8_t>>();

  rmw_gid_t dummy_gid;
  std::memset(&dummy_gid, 0, sizeof(dummy_gid));

  auto result = temp_impl->from_descriptor(descriptor, dummy_gid);
  return result;
}

}  // namespace demo_buffer_backend

// Export the DemoBufferBackend as a pluginlib plugin
PLUGINLIB_EXPORT_CLASS(
  demo_buffer_backend::DemoBufferBackend,
  rcl_buffer::BufferBackend)
