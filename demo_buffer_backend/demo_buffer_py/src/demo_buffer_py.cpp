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

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "rcl_buffer/buffer.hpp"
#include "demo_buffer/demo_buffer_impl.hpp"

namespace py = pybind11;

/// Create an rcl_buffer::Buffer<uint8_t> backed by DemoBufferImpl.
///
/// This is the Python-facing factory function that backend vendors would
/// provide. It creates a Buffer with the demo backend and returns it as
/// a raw pointer integer, which the Python wrapper then uses to construct
/// an rcl_buffer.Buffer Python object.
///
/// @param data The initial data as bytes
/// @return A new heap-allocated Buffer backed by DemoBufferImpl
static rcl_buffer::Buffer<uint8_t> * create_demo_buffer_from_bytes(
  const std::string & data)
{
  auto * buffer = new rcl_buffer::Buffer<uint8_t>();

  // Create DemoBufferImpl with the data
  std::vector<uint8_t> vec(data.begin(), data.end());
  auto demo_impl = std::make_unique<demo_buffer_backend::DemoBufferImpl<uint8_t>>(
    std::move(vec));
  buffer->set_impl(std::move(demo_impl), "demo");

  return buffer;
}

/// Create an rcl_buffer::Buffer<uint8_t> backed by DemoBufferImpl of given size.
///
/// @param size The number of bytes (zero-initialized)
/// @return A new heap-allocated Buffer backed by DemoBufferImpl
static rcl_buffer::Buffer<uint8_t> * create_demo_buffer_from_size(size_t size)
{
  auto * buffer = new rcl_buffer::Buffer<uint8_t>();

  auto demo_impl = std::make_unique<demo_buffer_backend::DemoBufferImpl<uint8_t>>(size);
  buffer->set_impl(std::move(demo_impl), "demo");

  return buffer;
}


PYBIND11_MODULE(_demo_buffer_py, m)
{
  m.doc() = "Python bindings for the demo buffer backend (DemoBufferImpl)";

  // Factory: create a demo-backed Buffer from bytes, return as uintptr_t
  // The Python layer wraps this into an rcl_buffer.Buffer
  m.def("_create_demo_buffer_from_bytes", [](py::bytes data) -> uintptr_t {
      std::string s = data;
      auto * buf = create_demo_buffer_from_bytes(s);
      return reinterpret_cast<uintptr_t>(buf);
    },
    py::arg("data"),
    "Create a demo-backed Buffer from bytes (internal, returns raw pointer)");

  m.def("_create_demo_buffer_from_size", [](size_t size) -> uintptr_t {
      auto * buf = create_demo_buffer_from_size(size);
      return reinterpret_cast<uintptr_t>(buf);
    },
    py::arg("size"),
    "Create a demo-backed Buffer of given size (internal, returns raw pointer)");
}
