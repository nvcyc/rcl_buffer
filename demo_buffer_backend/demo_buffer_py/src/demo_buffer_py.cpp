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
#include <memory>
#include <string>
#include <vector>

#include "rosidl_buffer/buffer.hpp"
#include "demo_buffer/demo_buffer_impl.hpp"

namespace py = pybind11;

/// Create a demo-backed rosidl::Buffer<uint8_t> and return it as a
/// Python rosidl_buffer.Buffer by calling _take_buffer_from_ptr on the
/// C++ side.  The uintptr_t never leaks to Python.
static py::object wrap_demo_buffer(std::vector<uint8_t> && data)
{
  auto impl = std::make_unique<demo_buffer_backend::DemoBufferImpl<uint8_t>>(
    std::move(data));
  auto * buf = new rosidl::Buffer<uint8_t>(std::move(impl));

  py::module_ rbuf = py::module_::import("rosidl_buffer._rosidl_buffer_py");
  return rbuf.attr("_take_buffer_from_ptr")(reinterpret_cast<uintptr_t>(buf));
}

PYBIND11_MODULE(_demo_buffer_py, m)
{
  m.doc() = "Python bindings for the demo buffer backend (DemoBufferImpl)";

  m.def("_from_cpu_data", [](py::bytes data) -> py::object {
      std::string s = data;
      std::vector<uint8_t> vec(s.begin(), s.end());
      return wrap_demo_buffer(std::move(vec));
    },
    py::arg("data"));

  m.def("_from_size", [](size_t size) -> py::object {
      std::vector<uint8_t> vec(size, 0);
      return wrap_demo_buffer(std::move(vec));
    },
    py::arg("size"));
}
