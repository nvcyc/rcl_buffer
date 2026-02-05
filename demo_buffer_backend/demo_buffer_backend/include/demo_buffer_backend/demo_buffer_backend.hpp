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

#ifndef DEMO_BUFFER_BACKEND__DEMO_BUFFER_BACKEND_HPP_
#define DEMO_BUFFER_BACKEND__DEMO_BUFFER_BACKEND_HPP_

#include <cstring>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>

#include "rosidl_buffer_registry/buffer_backend.hpp"
#include "demo_buffer_backend/demo_buffer_impl.hpp"

namespace demo_buffer_backend
{

/// Demo buffer backend implementation for demonstrating the buffer backend plugin system.
/// This backend is intentionally simple - it stores data on CPU and serializes by copying.
/// It includes a hash verification mechanism to ensure data integrity.
class DemoBufferBackend : public rosidl_buffer_registry::BufferBackend
{
public:
  /// Constructor
  DemoBufferBackend();
  ~DemoBufferBackend() override = default;

  // ========== BufferBackend interface implementation ==========

  /// Get backend type name
  std::string get_backend_type() const override
  {
    return "demo";
  }

  /// Get backend aux info (empty for demo backend)
  std::string get_backend_aux_info() const override
  {
    return "version=1.0";
  }

  /// Get descriptor message type name
  std::string get_descriptor_type_name() const override
  {
    return "demo_buffer_backend_msgs::msg::DemoBufferDescriptor";
  }

  /// Create descriptor with endpoint awareness
  std::shared_ptr<void> create_descriptor_with_endpoint(
    const std::shared_ptr<void> & impl,
    const rmw_topic_endpoint_info_t & endpoint_info) const override;

  /// Create BufferImpl from descriptor with endpoint awareness
  std::shared_ptr<void> from_descriptor_with_endpoint(
    const std::shared_ptr<void> & descriptor,
    const rmw_topic_endpoint_info_t & endpoint_info) const override;

  /// Hook for creating local endpoint (logs endpoint creation)
  void on_creating_endpoint(
    const rmw_topic_endpoint_info_t & endpoint_info) const override;

  /// Hook for discovering remote endpoint (checks backend compatibility)
  std::pair<bool, std::vector<std::set<uint32_t>>> on_discovering_endpoint(
    const rmw_topic_endpoint_info_t & endpoint_info,
    const std::vector<rmw_topic_endpoint_info_t> & existing_endpoints,
    const std::unordered_map<std::string, std::string> & endpoint_supported_backends) override;

  /// Provide the FastRTPS registration function
  void * get_descriptor_registration_function() const override;
};

}  // namespace demo_buffer_backend

#endif  // DEMO_BUFFER_BACKEND__DEMO_BUFFER_BACKEND_HPP_
