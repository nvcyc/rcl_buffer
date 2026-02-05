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

#ifndef RCL_BUFFER__BUFFER_IMPL_BASE_HPP_
#define RCL_BUFFER__BUFFER_IMPL_BASE_HPP_

#include <cstddef>
#include <memory>
#include <string>

// Forward declaration to avoid dependency on rmw
struct rmw_gid_s;
typedef struct rmw_gid_s rmw_gid_t;

namespace rcl_buffer
{

/// Abstract base class for all buffer implementations (CPU, CUDA, ROCm, etc.).
/// Provides a common interface for buffer operations and serialization.
template<typename T>
class BufferImplBase
{
public:
  virtual ~BufferImplBase() = default;

  // ========== Core Buffer Operations ==========

  /// Get the number of elements in the buffer.
  virtual size_t size() const = 0;

  /// Resize the buffer to hold n elements.
  /// @param n Number of elements
  virtual void resize(size_t n) = 0;

  /// Clear all elements from the buffer.
  virtual void clear() = 0;

  /// Get a backend-specific handle to the underlying buffer.
  /// For CPU buffers, this returns a pointer to the data.
  /// For GPU buffers, this returns a device pointer.
  /// @return Opaque pointer to backend-specific buffer handle
  virtual const void * get_backend_handle() const = 0;

  /// Create a CPU copy of this buffer.
  /// If already on CPU, may return a copy or the same instance.
  /// @return New BufferImplBase instance on CPU
  virtual std::unique_ptr<BufferImplBase<T>> to_cpu() const = 0;

  // ========== Descriptor-based Serialization Interface ==========

  /// Get the descriptor message type name for this backend.
  /// For example, "isaac_ros_cuda_buffer_msgs::msg::CudaBufferDescriptor"
  /// @return Fully qualified descriptor type name
  virtual std::string get_descriptor_type_name() const = 0;

  /// Create a descriptor message from this buffer implementation.
  /// The descriptor contains metadata needed to serialize/deserialize the buffer.
  /// @param subscriber_gid The GID of the target subscriber for peer-to-peer channels
  /// @return Type-erased descriptor message (std::shared_ptr<DescriptorMessageType>)
  virtual std::shared_ptr<void> create_descriptor(const rmw_gid_t & subscriber_gid) const = 0;

  /// Reconstruct a buffer from a descriptor message.
  /// @param descriptor Type-erased descriptor message pointer
  /// @param publisher_gid The GID of the publisher for peer-to-peer channels
  /// @return New BufferImplBase instance reconstructed from descriptor
  virtual std::unique_ptr<BufferImplBase<T>> from_descriptor(
    const std::shared_ptr<void> & descriptor,
    const rmw_gid_t & publisher_gid) const = 0;

  /// Create a deep copy of this buffer.
  /// @return New BufferImplBase instance with copied data
  virtual std::unique_ptr<BufferImplBase<T>> clone() const = 0;
};

}  // namespace rcl_buffer

#endif  // RCL_BUFFER__BUFFER_IMPL_BASE_HPP_
