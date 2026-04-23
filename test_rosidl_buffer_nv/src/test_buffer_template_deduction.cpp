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

// Reproducer for the template-argument-deduction failure reported by
// downstream packages (e.g. bag2_to_image) when they pass
// `sensor_msgs::msg::CompressedImage::_data_type` (== rosidl::Buffer<uint8_t>)
// to OpenCV APIs such as:
//
//     auto mat = cv::imdecode(cv::Mat(image->data), imdecode_flag_);
//
// This file contains NO OpenCV dependency. It defines a minimal
// `MockCvMat` whose templated constructor has the same signature as
// cv::Mat(const std::vector<_Tp> &) so the compiler reproduces the exact
// same error:
//
//     error: no matching function for call to
//       'MockCvMat::MockCvMat(rosidl::Buffer<uint8_t>&)'
//
// Flip the `REPRO_COMPILE_ERROR` compile definition on to trigger the
// failing call at compile time. Without it, this file compiles cleanly
// and asserts the expected behaviour via static_assert.
//
// Build-time toggle (manual):
//   colcon build --packages-select test_rosidl_buffer_nv \
//     --cmake-args -DTEST_BUFFER_REPRO_COMPILE_ERROR=ON
// or directly:
//   g++ -DREPRO_COMPILE_ERROR ...

#include <cstdint>
#include <iostream>
#include <type_traits>
#include <vector>

#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

// ---------------------------------------------------------------------------
// Minimal stand-in for cv::Mat.
//
// IMPORTANT: cv::Mat is NOT a class template. It is a plain class with a
// *templated constructor*:
//
//   class Mat {
//    public:
//     template<typename _Tp>
//     explicit Mat(const std::vector<_Tp> & vec, bool copyData = false);
//   };
//
// That distinction is crucial. If we instead made MockCvMat a class
// template (MockCvMat<_Tp>), the caller would name the instantiation
// explicitly (MockCvMat<uint8_t>(buffer)), the constructor would collapse
// to a concrete signature `(const std::vector<uint8_t> &)`, and the
// Buffer's user-defined conversion operator would take over -- masking the
// very deduction failure we want to reproduce. Keeping the class
// non-templated forces the compiler to deduce `_Tp` from the argument
// at the call site, which is where user-defined conversions are ignored.
// ---------------------------------------------------------------------------
struct MockCvMat
{
  MockCvMat() = default;

  template<typename _Tp>
  explicit MockCvMat(const std::vector<_Tp> & vec, bool /*copyData*/ = false)
  : size(vec.size()),
    first(vec.empty() ? std::uint64_t{0} : static_cast<std::uint64_t>(vec[0]))
  {
  }

  std::size_t size = 0;
  std::uint64_t first = 0;
};

int main()
{
  // ------------------------------------------------------------------
  // Set up a CompressedImage exactly as bag2_to_image does.
  // image->data is rosidl::Buffer<uint8_t>.
  // ------------------------------------------------------------------
  auto image = std::make_shared<sensor_msgs::msg::CompressedImage>();
  image->format = "jpeg";
  image->data.resize(16);
  for (std::size_t i = 0; i < image->data.size(); ++i) {
    image->data[i] = static_cast<uint8_t>(i);
  }

  // ------------------------------------------------------------------
  // (A) Document the failure at compile time.
  //
  // MockCvMat's ctor is a function template. Template argument deduction
  // does NOT consider user-defined conversions, so it cannot deduce `_Tp`
  // from an argument of type `rosidl::Buffer<uint8_t> &` and the call
  // below would fail to compile. We assert that fact here.
  // ------------------------------------------------------------------
  using BufferType = decltype(image->data);  // rosidl::Buffer<uint8_t>
  static_assert(
    !std::is_constructible_v<MockCvMat, BufferType &>,
    "If this assertion fires, rosidl::Buffer<uint8_t> is now directly "
    "usable with MockCvMat's templated vector constructor. Either the "
    "Buffer class added a matching constructor template, or this test "
    "must be updated.");
  static_assert(
    std::is_constructible_v<MockCvMat, const std::vector<uint8_t> &>,
    "Sanity check: MockCvMat must accept a real std::vector<uint8_t>.");

  std::cout << "[A] static_assert: cannot construct MockCvMat "
            << "directly from rosidl::Buffer<uint8_t> ... OK (expected)\n";

  // ------------------------------------------------------------------
  // (B) Optional: actually trigger the compile error so a human can read
  // the diagnostic. Enable with -DREPRO_COMPILE_ERROR.
  // ------------------------------------------------------------------
#ifdef REPRO_COMPILE_ERROR
  // The next line mirrors the exact downstream failure:
  //   auto mat = cv::imdecode(cv::Mat(image->data), imdecode_flag_);
  MockCvMat repro(image->data);
  (void)repro;
#endif

  // ------------------------------------------------------------------
  // (C) Workaround 1: explicit cast forces the user-defined conversion
  //     operator `rosidl::Buffer::operator std::vector<T, A> &()` to run
  //     before overload resolution picks the MockCvMat ctor, so template
  //     deduction sees a std::vector and succeeds.
  // ------------------------------------------------------------------
  {
    MockCvMat mat{static_cast<const std::vector<uint8_t> &>(image->data)};
    if (mat.size != image->data.size() || mat.first != image->data[0]) {
      std::cerr << "[C] static_cast workaround produced wrong values\n";
      return 1;
    }
    std::cout << "[C] static_cast<const std::vector<uint8_t> &>(...) ... OK\n";
  }

  // ------------------------------------------------------------------
  // (D) Workaround 2: to_vector() escape hatch. Always copies but also
  //     works for non-CPU-backed buffers (e.g. demo/CUDA backends).
  // ------------------------------------------------------------------
  {
    MockCvMat mat{image->data.to_vector()};
    if (mat.size != image->data.size() || mat.first != image->data[0]) {
      std::cerr << "[D] to_vector() workaround produced wrong values\n";
      return 1;
    }
    std::cout << "[D] image->data.to_vector() ... OK\n";
  }

  // ------------------------------------------------------------------
  // (E) Workaround 3: bind through a local std::vector reference. This
  //     triggers the same user-defined conversion but reads a little
  //     cleaner than static_cast in client code.
  // ------------------------------------------------------------------
  {
    const std::vector<uint8_t> & as_vec = image->data;
    MockCvMat mat{as_vec};
    if (mat.size != image->data.size() || mat.first != image->data[0]) {
      std::cerr << "[E] reference-binding workaround produced wrong values\n";
      return 1;
    }
    std::cout << "[E] const std::vector<uint8_t> & as_vec = image->data ... OK\n";
  }

  // ------------------------------------------------------------------
  // (F) Same pattern, but with sensor_msgs::msg::Image to confirm the
  // failure mode is not specific to CompressedImage.
  // ------------------------------------------------------------------
  {
    sensor_msgs::msg::Image img;
    img.data.resize(8, 42);
    static_assert(
      !std::is_constructible_v<
        MockCvMat,
        decltype(img.data) &>,
      "Image::_data_type also must not be directly constructible.");

    MockCvMat mat{static_cast<const std::vector<uint8_t> &>(img.data)};
    if (mat.size != img.data.size() || mat.first != 42) {
      std::cerr << "[F] Image variant produced wrong values\n";
      return 1;
    }
    std::cout << "[F] Same failure/workaround reproduces on Image too ... OK\n";
  }

  std::cout << "\n=== ALL REPRODUCER CHECKS PASSED ===\n";
  std::cout << "To actually see the compile error, rebuild with\n"
            << "  --cmake-args -DTEST_BUFFER_REPRO_COMPILE_ERROR=ON\n";
  return 0;
}
