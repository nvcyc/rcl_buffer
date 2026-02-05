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

#include "rcl_buffer_test_backend/test_buffer_backend.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <iostream>

// Declare external registration function from the descriptor message package
// This is provided by rcl_buffer_test_backend_msgs_fastrtps_registration library
extern "C" void register_test_buffer_descriptor_fastrtps();

namespace rcl_buffer_test_backend
{

//==============================================================================
TestBufferBackend::TestBufferBackend()
{
  std::cerr << "[TestBufferBackend] TestBufferBackend created\n";
}

//==============================================================================
void TestBufferBackend::on_creating_endpoint(
  const rmw_topic_endpoint_info_t & endpoint_info) const
{
  std::cerr << "[TestBufferBackend] on_creating_endpoint() called for topic: "
            << endpoint_info.topic_type
            << " (endpoint_type=" << static_cast<int>(endpoint_info.endpoint_type) << ")\n";
}

//==============================================================================
std::pair<bool, std::vector<std::set<uint32_t>>>
TestBufferBackend::on_discovering_endpoint(
  const rmw_topic_endpoint_info_t & endpoint_info,
  const std::vector<rmw_topic_endpoint_info_t> & existing_endpoints,
  const std::unordered_map<std::string, std::string> & endpoint_supported_backends)
{
  (void)existing_endpoints;

  // Check if discovered endpoint supports test backend
  bool supports_test = endpoint_supported_backends.find("test") !=
    endpoint_supported_backends.end();

  if (supports_test) {
    std::cerr << "[TestBufferBackend] Discovered endpoint supports 'test' backend for topic: "
              << endpoint_info.topic_type << "\n";
  } else {
    std::cerr << "[TestBufferBackend] Discovered endpoint does NOT support 'test' backend\n";
  }

  // Test backend is always compatible with other test backends
  // No special grouping logic needed
  return {supports_test, {}};
}

//==============================================================================
std::shared_ptr<void> TestBufferBackend::create_descriptor_with_endpoint(
  const std::shared_ptr<void> & impl,
  const rmw_topic_endpoint_info_t & endpoint_info) const
{
  (void)endpoint_info;  // Test backend doesn't use endpoint info for descriptor

  auto test_impl = std::static_pointer_cast<TestBufferImpl<uint8_t>>(impl);

  // Create descriptor using the impl's create_descriptor method
  rmw_gid_t dummy_gid;
  std::memset(&dummy_gid, 0, sizeof(dummy_gid));

  return test_impl->create_descriptor(dummy_gid);
}

//==============================================================================
std::shared_ptr<void> TestBufferBackend::from_descriptor_with_endpoint(
  const std::shared_ptr<void> & descriptor,
  const rmw_topic_endpoint_info_t & endpoint_info) const
{
  (void)endpoint_info;  // Test backend doesn't use endpoint info for descriptor

  auto temp_impl = std::make_shared<TestBufferImpl<uint8_t>>();

  rmw_gid_t dummy_gid;
  std::memset(&dummy_gid, 0, sizeof(dummy_gid));

  auto result = temp_impl->from_descriptor(descriptor, dummy_gid);
  return result;
}

//==============================================================================
void * TestBufferBackend::get_descriptor_registration_function() const
{
  return reinterpret_cast<void *>(&register_test_buffer_descriptor_fastrtps);
}

}  // namespace rcl_buffer_test_backend

// Export the TestBufferBackend as a pluginlib plugin
PLUGINLIB_EXPORT_CLASS(
  rcl_buffer_test_backend::TestBufferBackend,
  rosidl_buffer_registry::BufferBackend)
