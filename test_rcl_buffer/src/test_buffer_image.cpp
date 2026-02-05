// Copyright 2024 NVIDIA Corporation
//
// Simple test to verify rosidl_runtime_cpp::Buffer works with sensor_msgs::msg::Image

#include <iostream>
#include <vector>

#include "sensor_msgs/msg/image.hpp"

int main()
{
  // Test 1: Create an Image message (uses Buffer<uint8_t> for data field)
  sensor_msgs::msg::Image img;
  img.width = 640;
  img.height = 480;
  img.encoding = "rgb8";
  img.step = 640 * 3;

  std::cout << "Test 1: Creating Image message... OK\n";

  // Test 2: Resize and fill data (backward compatibility with std::vector API)
  img.data.resize(640 * 480 * 3);
  std::cout << "Test 2: Resizing buffer to " << img.data.size() << " bytes... OK\n";

  // Test 3: Fill with test pattern
  for (size_t i = 0; i < img.data.size(); ++i) {
    img.data[i] = static_cast<uint8_t>(i % 255);
  }
  std::cout << "Test 3: Filling buffer via operator[]... OK\n";

  // Test 4: Read back values
  if (img.data[0] == 0 && img.data[100] == 100) {
    std::cout << "Test 4: Reading values via operator[]... OK\n";
  } else {
    std::cerr << "Test 4: FAILED - unexpected values\n";
    return 1;
  }

  // Test 5: Implicit conversion to std::vector<uint8_t>&
  std::vector<uint8_t> & vec_ref = img.data;
  if (vec_ref.size() == img.data.size() && vec_ref[50] == img.data[50]) {
    std::cout << "Test 5: Implicit conversion to std::vector<uint8_t>&... OK\n";
  } else {
    std::cerr << "Test 5: FAILED - implicit conversion issue\n";
    return 1;
  }

  // Test 6: Use with legacy code expecting std::vector
  auto process_vector = [](const std::vector<uint8_t> & data) {
    return data.size();
  };
  size_t result = process_vector(img.data);
  if (result == img.data.size()) {
    std::cout << "Test 6: Passing to legacy function expecting std::vector... OK\n";
  } else {
    std::cerr << "Test 6: FAILED\n";
    return 1;
  }

  // Test 7: Push back (CPU backend)
  sensor_msgs::msg::Image small_img;
  small_img.data.push_back(255);
  small_img.data.push_back(128);
  small_img.data.push_back(64);
  if (small_img.data.size() == 3 && small_img.data[1] == 128) {
    std::cout << "Test 7: Using push_back()... OK\n";
  } else {
    std::cerr << "Test 7: FAILED\n";
    return 1;
  }

  // Test 8: Empty and clear
  small_img.data.clear();
  if (small_img.data.empty()) {
    std::cout << "Test 8: clear() and empty()... OK\n";
  } else {
    std::cerr << "Test 8: FAILED\n";
    return 1;
  }

  // Test 9: Get backend type (should be "cpu" by default)
  if (img.data.get_backend_type() == "cpu") {
    std::cout << "Test 9: Backend type is 'cpu'... OK\n";
  } else {
    std::cerr << "Test 9: FAILED - backend type is '" 
              << img.data.get_backend_type() << "'\n";
    return 1;
  }

  // Test 10: to_vector() escape hatch
  std::vector<uint8_t> copied = img.data.to_vector();
  if (copied.size() == img.data.size() && copied[10] == img.data[10]) {
    std::cout << "Test 10: to_vector() escape hatch... OK\n";
  } else {
    std::cerr << "Test 10: FAILED\n";
    return 1;
  }

  std::cout << "\n=== ALL TESTS PASSED ===\n";
  std::cout << "sensor_msgs::msg::Image works correctly with rosidl_runtime_cpp::Buffer!\n";
  
  return 0;
}

