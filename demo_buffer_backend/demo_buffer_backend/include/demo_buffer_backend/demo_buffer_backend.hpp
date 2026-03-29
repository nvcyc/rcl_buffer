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

#ifndef DEMO_BUFFER_BACKEND__DEMO_BUFFER_BACKEND_HPP_
#define DEMO_BUFFER_BACKEND__DEMO_BUFFER_BACKEND_HPP_

#include <array>
#include <cstring>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rosidl_buffer_backend/buffer_backend.hpp"
#include "demo_buffer/demo_buffer_impl.hpp"
#include "demo_buffer_backend_msgs/msg/demo_buffer_descriptor.hpp"
#include "demo_buffer_backend/visibility_control.h"
#include "rmw/types.h"

namespace demo_buffer_backend
{

/// Demo buffer backend implementation for demonstrating the buffer backend plugin system.
/// This backend is intentionally simple - it stores data on CPU and serializes by copying.
/// It includes a hash verification mechanism to ensure data integrity.
class DEMO_BUFFER_BACKEND_PUBLIC DemoBufferBackend : public rosidl::BufferBackend
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

  /// Get backend metadata
  std::string get_backend_metadata() const override
  {
    return "version=1.0";
  }

  const rosidl_message_type_support_t * get_descriptor_type_support() const override;

  std::shared_ptr<void> create_empty_descriptor() const override;

  /// Create descriptor with endpoint awareness.
  /// Returns nullptr if the endpoint is not compatible with the demo backend,
  /// signaling that the serialization layer should fall back to CPU.
  std::shared_ptr<void> create_descriptor_with_endpoint(
    const void * impl,
    const rmw_topic_endpoint_info_t & endpoint_info) const override;

  /// Create BufferImpl from descriptor with endpoint awareness
  std::unique_ptr<void, void (*)(void *)> from_descriptor_with_endpoint(
    const void * descriptor,
    const rmw_topic_endpoint_info_t & endpoint_info) const override;

  /// Hook for creating local endpoint (logs endpoint creation)
  void on_creating_endpoint(
    const rmw_topic_endpoint_info_t & endpoint_info) const override;

  /// Hook for discovering remote endpoint (checks backend compatibility)
  std::pair<bool, std::vector<std::set<uint32_t>>> on_discovering_endpoint(
    const rmw_topic_endpoint_info_t & endpoint_info,
    const std::vector<rmw_topic_endpoint_info_t> & existing_endpoints,
    const std::unordered_map<std::string, std::string> & endpoint_supported_backends) override;

private:
  using GidKey = std::array<uint8_t, RMW_GID_STORAGE_SIZE>;
  mutable std::mutex compat_mutex_;
  std::unordered_map<std::size_t, bool> endpoint_compat_cache_;

  static std::size_t gid_hash(const uint8_t * gid)
  {
    std::size_t h = 0;
    for (size_t i = 0; i < RMW_GID_STORAGE_SIZE; ++i) {
      h ^= std::hash<uint8_t>{}(gid[i]) + 0x9e3779b9 + (h << 6) + (h >> 2);
    }
    return h;
  }
};

}  // namespace demo_buffer_backend

#endif  // DEMO_BUFFER_BACKEND__DEMO_BUFFER_BACKEND_HPP_
