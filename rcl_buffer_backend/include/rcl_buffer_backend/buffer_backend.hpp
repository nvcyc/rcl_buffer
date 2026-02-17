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

#ifndef RCL_BUFFER_BACKEND__BUFFER_BACKEND_HPP_
#define RCL_BUFFER_BACKEND__BUFFER_BACKEND_HPP_

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <set>
#include <unordered_map>
#include <vector>

#include "rmw/topic_endpoint_info.h"

namespace rcl_buffer
{

/// Abstract interface for vendor-specific buffer backend implementations.
/// Backends must implement all 8 methods: 4 serialization modes + 4 deserialization modes.
///
/// This interface uses type erasure (void*) since it must work across different
/// element types (uint8_t, float, etc.) at the plugin boundary.
class BufferBackend
{
public:
  virtual ~BufferBackend() = default;

  // ========== Descriptor Message Support (Serialization-Technology Independent) ==========

  /// Get the backend type name (e.g., "cuda", "rocm", "vulkan")
  virtual std::string get_backend_type() const = 0;

  /// Get the backend aux info string
  /// This is used by the backend to pass backend-specific information to other endpoints.
  virtual std::string get_backend_aux_info() const
  {
    return "";
  }

  /// Get the descriptor message type name (e.g., "cpu_buffer_msgs::msg::CpuBufferDescriptor")
  /// This is used by the serialization layer to find the appropriate type support.
  virtual std::string get_descriptor_type_name() const = 0;

  /// Create a descriptor message with endpoint awareness.
  /// @param impl Type-erased BufferImplBase pointer.
  /// @param endpoint_info Endpoint info for the peer.
  /// @return Type-erased descriptor message.
  virtual std::shared_ptr<void> create_descriptor_with_endpoint(
    const std::shared_ptr<void> & impl,
    const rmw_topic_endpoint_info_t & endpoint_info) const = 0;

  /// Create a BufferImpl from descriptor with endpoint awareness.
  /// @param descriptor Type-erased descriptor message pointer.
  /// @param endpoint_info Endpoint info for the peer.
  /// @return Type-erased BufferImplBase pointer.
  virtual std::shared_ptr<void> from_descriptor_with_endpoint(
    const std::shared_ptr<void> & descriptor,
    const rmw_topic_endpoint_info_t & endpoint_info) const = 0;

  /// Hook invoked when creating a local endpoint.
  virtual void on_creating_endpoint(
    const rmw_topic_endpoint_info_t & endpoint_info) const
  {
    (void)endpoint_info;
  }

  /// Hook invoked when discovering a remote endpoint.
  /// Returns {compatible, groups of endpoint GID hashes}.
  /// @param endpoint_info Information about the discovered endpoint
  /// @param existing_endpoints List of existing endpoints for grouping decisions
  /// @param endpoint_supported_backends Map of backend type to aux info string for the discovered endpoint
  virtual std::pair<bool, std::vector<std::set<uint32_t>>> on_discovering_endpoint(
    const rmw_topic_endpoint_info_t & endpoint_info,
    const std::vector<rmw_topic_endpoint_info_t> & existing_endpoints,
    const std::unordered_map<std::string, std::string> & endpoint_supported_backends)
  {
    (void)endpoint_info;
    (void)existing_endpoints;
    (void)endpoint_supported_backends;
    return {true, {}};
  }

};

}  // namespace rcl_buffer

#endif  // RCL_BUFFER_BACKEND__BUFFER_BACKEND_HPP_
