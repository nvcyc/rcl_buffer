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

#include "demo_buffer_backend/demo_buffer_backend.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <iostream>

// Declare external registration function from the descriptor message package
// This is provided by demo_buffer_backend_msgs_fastrtps_registration library
extern "C" void register_demo_buffer_descriptor_fastrtps();

namespace demo_buffer_backend
{

//==============================================================================
DemoBufferBackend::DemoBufferBackend()
{
  std::cerr << "[DemoBufferBackend] DemoBufferBackend created\n";
}

//==============================================================================
void DemoBufferBackend::on_creating_endpoint(
  const rmw_topic_endpoint_info_t & endpoint_info) const
{
  std::cerr << "[DemoBufferBackend] on_creating_endpoint() called for topic: "
            << endpoint_info.topic_type
            << " (endpoint_type=" << static_cast<int>(endpoint_info.endpoint_type) << ")\n";
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
    std::cerr << "[DemoBufferBackend] Discovered endpoint supports 'demo' backend for topic: "
              << endpoint_info.topic_type << "\n";
  } else {
    std::cerr << "[DemoBufferBackend] Discovered endpoint does NOT support 'demo' backend\n";
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

//==============================================================================
void * DemoBufferBackend::get_descriptor_registration_function() const
{
  return reinterpret_cast<void *>(&register_demo_buffer_descriptor_fastrtps);
}

}  // namespace demo_buffer_backend

// Export the DemoBufferBackend as a pluginlib plugin
PLUGINLIB_EXPORT_CLASS(
  demo_buffer_backend::DemoBufferBackend,
  rcl_buffer::BufferBackend)
