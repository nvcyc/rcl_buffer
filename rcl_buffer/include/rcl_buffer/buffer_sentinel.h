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

#ifndef RCL_BUFFER__BUFFER_SENTINEL_H_
#define RCL_BUFFER__BUFFER_SENTINEL_H_

#include <stddef.h>

/// Sentinel value for rosidl_runtime_c sequence capacity field.
///
/// When a rosidl_runtime_c__uint8__Sequence has capacity == RCL_BUFFER_SENTINEL_CAPACITY,
/// the data pointer holds a reinterpret_cast'd pointer to an rcl_buffer::Buffer<uint8_t>
/// instead of a malloc'd byte array. This enables passing vendor-backed buffer objects
/// through the C message layer (used by rclpy) without changing the struct definition.
///
/// SIZE_MAX can never occur from a real allocation, making it a reliable discriminant.
///
/// Usage:
///   if (sequence->capacity == RCL_BUFFER_SENTINEL_CAPACITY) {
///     // data is actually a rcl_buffer::Buffer<uint8_t>*
///     auto * buffer = reinterpret_cast<rcl_buffer::Buffer<uint8_t>*>(sequence->data);
///   }
#define RCL_BUFFER_SENTINEL_CAPACITY ((size_t)-1)

#endif  // RCL_BUFFER__BUFFER_SENTINEL_H_
