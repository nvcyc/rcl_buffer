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

#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include "demo_buffer/demo_buffer_impl.hpp"
#include "demo_buffer_backend/demo_buffer_backend.hpp"
#include "demo_buffer_backend_msgs/msg/demo_buffer_descriptor.hpp"
#include "rosidl_buffer/cpu_buffer_impl.hpp"

using demo_buffer_backend::DemoBufferImpl;
using demo_buffer_backend::DemoBufferBackend;
using demo_buffer_backend::compute_fnv1a_hash;

class DemoBufferImplTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

// Test basic buffer operations
TEST_F(DemoBufferImplTest, BasicOperations)
{
  DemoBufferImpl<uint8_t> buffer;

  // Test initial state
  EXPECT_EQ(buffer.get_storage().size(), 0u);
  EXPECT_TRUE(buffer.get_storage().empty());

  // Test resize
  buffer.get_storage().resize(100);
  EXPECT_EQ(buffer.get_storage().size(), 100u);
  EXPECT_NE(buffer.get_storage().data(), nullptr);

  // Test clear
  buffer.get_storage().clear();
  EXPECT_EQ(buffer.get_storage().size(), 0u);
}

// Test constructor with size
TEST_F(DemoBufferImplTest, ConstructorWithSize)
{
  DemoBufferImpl<uint8_t> buffer(256);
  EXPECT_EQ(buffer.get_storage().size(), 256u);
  EXPECT_NE(buffer.get_storage().data(), nullptr);
}

// Test constructor with vector
TEST_F(DemoBufferImplTest, ConstructorWithVector)
{
  std::vector<uint8_t> data = {1, 2, 3, 4, 5};
  DemoBufferImpl<uint8_t> buffer(data);

  EXPECT_EQ(buffer.get_storage().size(), 5u);
  EXPECT_EQ(buffer.get_storage(), data);
}

// Test hash computation
TEST_F(DemoBufferImplTest, HashComputation)
{
  std::vector<uint8_t> data1 = {1, 2, 3, 4, 5};
  std::vector<uint8_t> data2 = {1, 2, 3, 4, 6};  // Different last byte

  uint64_t hash1 = compute_fnv1a_hash(data1.data(), data1.size());
  uint64_t hash2 = compute_fnv1a_hash(data2.data(), data2.size());

  // Same data should produce same hash
  uint64_t hash1_again = compute_fnv1a_hash(data1.data(), data1.size());
  EXPECT_EQ(hash1, hash1_again);

  // Different data should produce different hash
  EXPECT_NE(hash1, hash2);

  // Empty data should still produce a hash
  uint64_t empty_hash = compute_fnv1a_hash(nullptr, 0);
  EXPECT_NE(empty_hash, 0u);
}

// Test descriptor creation and reconstruction
TEST_F(DemoBufferImplTest, DescriptorRoundTrip)
{
  // Create buffer with test data
  std::vector<uint8_t> test_data(1024);
  for (size_t i = 0; i < test_data.size(); ++i) {
    test_data[i] = static_cast<uint8_t>(i % 256);
  }

  DemoBufferImpl<uint8_t> original(test_data);
  EXPECT_EQ(original.get_storage().size(), test_data.size());

  // Create descriptor
  rmw_gid_t dummy_gid;
  std::memset(&dummy_gid, 0, sizeof(dummy_gid));

  auto descriptor_shared = original.create_descriptor(dummy_gid);
  ASSERT_NE(descriptor_shared, nullptr);

  auto descriptor = std::static_pointer_cast<
    demo_buffer_backend_msgs::msg::DemoBufferDescriptor>(descriptor_shared);

  // Verify descriptor contents
  EXPECT_EQ(descriptor->size, test_data.size());
  EXPECT_EQ(descriptor->data.size(), test_data.size());
  EXPECT_NE(descriptor->data_hash, 0u);

  // Reconstruct buffer from descriptor
  DemoBufferImpl<uint8_t> temp;
  auto reconstructed_ptr = temp.from_descriptor(descriptor_shared.get(), dummy_gid);
  ASSERT_NE(reconstructed_ptr, nullptr);

  auto reconstructed = dynamic_cast<DemoBufferImpl<uint8_t> *>(reconstructed_ptr.get());
  ASSERT_NE(reconstructed, nullptr);

  // Verify reconstructed buffer matches original
  EXPECT_EQ(reconstructed->get_storage().size(), original.get_storage().size());
  EXPECT_EQ(reconstructed->get_storage(), original.get_storage());
}

// Test to_cpu conversion
TEST_F(DemoBufferImplTest, ToCpuConversion)
{
  std::vector<uint8_t> test_data = {10, 20, 30, 40, 50};
  DemoBufferImpl<uint8_t> buffer(test_data);

  auto cpu_buffer = buffer.to_cpu();
  ASSERT_NE(cpu_buffer, nullptr);
  auto * cpu_impl = static_cast<rosidl::CpuBufferImpl<uint8_t> *>(cpu_buffer.get());
  EXPECT_EQ(cpu_impl->get_storage().size(), test_data.size());
}

// Test clone
TEST_F(DemoBufferImplTest, Clone)
{
  std::vector<uint8_t> test_data = {100, 200, 150, 50, 25};
  DemoBufferImpl<uint8_t> original(test_data);

  auto cloned = original.clone();
  ASSERT_NE(cloned, nullptr);
  auto * cloned_as_demo = dynamic_cast<DemoBufferImpl<uint8_t> *>(cloned.get());
  ASSERT_NE(cloned_as_demo, nullptr);
  EXPECT_EQ(cloned_as_demo->get_storage().size(), original.get_storage().size());

  // Verify it's a deep copy by modifying original
  original.get_storage()[0] = 0;
  auto cloned_impl = dynamic_cast<DemoBufferImpl<uint8_t> *>(cloned.get());
  ASSERT_NE(cloned_impl, nullptr);
  EXPECT_EQ(cloned_impl->get_storage()[0], 100);  // Unchanged
}

class DemoBufferBackendTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    backend_ = std::make_unique<DemoBufferBackend>();
  }
  void TearDown() override {}

  std::unique_ptr<DemoBufferBackend> backend_;
};

// Test backend type name
TEST_F(DemoBufferBackendTest, BackendTypeName)
{
  EXPECT_EQ(backend_->get_backend_type(), "demo");
}

// Test backend metadata
TEST_F(DemoBufferBackendTest, BackendMetadata)
{
  EXPECT_EQ(backend_->get_backend_metadata(), "version=1.0");
}

// Test create_descriptor_with_endpoint
TEST_F(DemoBufferBackendTest, CreateDescriptorWithEndpoint)
{
  // Create test buffer impl
  std::vector<uint8_t> test_data = {1, 2, 3, 4, 5};
  auto impl = std::make_shared<DemoBufferImpl<uint8_t>>(test_data);

  // Create endpoint info
  rmw_topic_endpoint_info_t endpoint_info;
  std::memset(&endpoint_info, 0, sizeof(endpoint_info));
  endpoint_info.topic_type = "test_type";

  // Create descriptor
  auto descriptor = backend_->create_descriptor_with_endpoint(impl.get(), endpoint_info);
  ASSERT_NE(descriptor, nullptr);

  auto typed_desc = std::static_pointer_cast<
    demo_buffer_backend_msgs::msg::DemoBufferDescriptor>(descriptor);
  EXPECT_EQ(typed_desc->size, test_data.size());
}

// Test from_descriptor_with_endpoint
TEST_F(DemoBufferBackendTest, FromDescriptorWithEndpoint)
{
  // Create descriptor directly
  auto descriptor = std::make_shared<demo_buffer_backend_msgs::msg::DemoBufferDescriptor>();
  descriptor->size = 5;
  descriptor->element_type_name = typeid(uint8_t).name();
  // Buffer doesn't support initializer list, use vector assignment
  std::vector<uint8_t> test_data = {1, 2, 3, 4, 5};
  descriptor->data = test_data;
  descriptor->data_hash = compute_fnv1a_hash(descriptor->data.data(), descriptor->data.size());

  // Create endpoint info
  rmw_topic_endpoint_info_t endpoint_info;
  std::memset(&endpoint_info, 0, sizeof(endpoint_info));
  endpoint_info.topic_type = "test_type";

  // Reconstruct from descriptor
  auto result = backend_->from_descriptor_with_endpoint(descriptor.get(), endpoint_info);
  ASSERT_NE(result.get(), nullptr);
}

// Test on_discovering_endpoint
TEST_F(DemoBufferBackendTest, OnDiscoveringEndpoint)
{
  rmw_topic_endpoint_info_t endpoint_info;
  std::memset(&endpoint_info, 0, sizeof(endpoint_info));
  endpoint_info.topic_type = "test_type";

  std::vector<rmw_topic_endpoint_info_t> existing_endpoints;

  // Test with demo backend supported
  std::unordered_map<std::string, std::string> supported_backends_with_test;
  supported_backends_with_test["demo"] = "version=1.0";

  auto result_with = backend_->on_discovering_endpoint(
    endpoint_info, existing_endpoints, supported_backends_with_test);
  EXPECT_TRUE(result_with.first);  // Should be compatible

  // Test without test backend supported
  std::unordered_map<std::string, std::string> supported_backends_without_test;
  supported_backends_without_test["cuda"] = "device=0";

  auto result_without = backend_->on_discovering_endpoint(
    endpoint_info, existing_endpoints, supported_backends_without_test);
  EXPECT_FALSE(result_without.first);  // Should not be compatible
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
