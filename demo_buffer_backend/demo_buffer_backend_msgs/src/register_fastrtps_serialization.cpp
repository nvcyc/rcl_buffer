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

#include <iostream>

#include "demo_buffer_backend_msgs/msg/demo_buffer_descriptor.hpp"
#include \
  "demo_buffer_backend_msgs/msg/detail/demo_buffer_descriptor__rosidl_typesupport_fastrtps_cpp.hpp"
#include "rosidl_typesupport_fastrtps_cpp/buffer_serialization.hpp"

namespace demo_buffer_backend_msgs
{

// Explicit registration function that can be called by RMW layer
// Export with C linkage to make it discoverable without name mangling
// IMPORTANT: This must be called AFTER the FastCDR backend entry is created
extern "C" void register_demo_buffer_descriptor_fastrtps()
{
  std::cerr << "[Demo Buffer Msgs] register_demo_buffer_descriptor_fastrtps() called\n";

  // Register serialize and deserialize functions
  auto & serializers = rosidl_typesupport_fastrtps_cpp::get_descriptor_serializers();
  rosidl_typesupport_fastrtps_cpp::DescriptorSerializers desc_ser;

  desc_ser.serialize = [](eprosima::fastcdr::Cdr & cdr,
    const std::shared_ptr<void> & desc_ptr) {
      std::cerr << "[Demo Buffer Msgs] Serializing DemoBufferDescriptor\n";
      auto desc =
        std::static_pointer_cast<demo_buffer_backend_msgs::msg::DemoBufferDescriptor>(desc_ptr);
      std::cerr << "[Demo Buffer Msgs]   size=" << desc->size
                << ", data_hash=" << desc->data_hash
                << ", data.size()=" << desc->data.size() << "\n";
      demo_buffer_backend_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(*desc, cdr);
      std::cerr << "[Demo Buffer Msgs] DemoBufferDescriptor serialization complete\n";
    };

  desc_ser.deserialize = [](eprosima::fastcdr::Cdr & cdr) -> std::shared_ptr<void> {
      std::cerr << "[Demo Buffer Msgs] Deserializing DemoBufferDescriptor\n";
      auto desc = std::make_shared<demo_buffer_backend_msgs::msg::DemoBufferDescriptor>();
      demo_buffer_backend_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(cdr, *desc);
      std::cerr << "[Demo Buffer Msgs]   size=" << desc->size
                << ", data_hash=" << desc->data_hash
                << ", data.size()=" << desc->data.size() << "\n";
      std::cerr << "[Demo Buffer Msgs] DemoBufferDescriptor deserialization complete\n";
      return desc;
    };

  serializers["demo"] = desc_ser;

  std::cerr << "[Demo Buffer Msgs] Demo buffer descriptor registered with FastCDR\n";
}

// NO static initializer - registration must happen after FastCDR backend entry exists
// The RMW layer will call this function via dlsym after creating the backend entry

}  // namespace demo_buffer_backend_msgs
