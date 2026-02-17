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

#ifndef RCL_BUFFER_BACKEND__REGISTER_BUFFER_DESCRIPTOR_HPP_
#define RCL_BUFFER_BACKEND__REGISTER_BUFFER_DESCRIPTOR_HPP_

#include <string>

#include "rosidl_typesupport_fastrtps_cpp/buffer_serialization.hpp"

namespace rcl_buffer
{

/// Register FastCDR descriptor serializers for a buffer descriptor message type.
///
/// Backend implementors should call this once in their constructor so that
/// the framework can serialize/deserialize the backend's descriptor message.
/// This wrapper is provided by rcl_buffer_backend so that backends do not
/// need to depend directly on rosidl_typesupport_fastrtps_cpp.
///
/// @tparam DescriptorMsgT  The rosidl-generated descriptor message type
///                         (e.g., my_backend_msgs::msg::MyDescriptor).
/// @param backend_name     The backend type name used as the registry key
///                         (e.g., "demo", "cuda").
template<typename DescriptorMsgT>
inline void register_buffer_descriptor(const std::string & backend_name)
{
  rosidl_typesupport_fastrtps_cpp::register_descriptor_serializers<DescriptorMsgT>(
    backend_name);
}

}  // namespace rcl_buffer

#endif  // RCL_BUFFER_BACKEND__REGISTER_BUFFER_DESCRIPTOR_HPP_
