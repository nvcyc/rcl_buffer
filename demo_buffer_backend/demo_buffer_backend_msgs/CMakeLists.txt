cmake_minimum_required(VERSION 3.8)
project(demo_buffer_backend_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_fastrtps_cpp REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DemoBufferDescriptor.msg"
)

# Build FastRTPS serialization registration library
# This allows RMW layer to dynamically register test buffer descriptor serialization
add_library(${PROJECT_NAME}_fastrtps_registration SHARED
  src/register_fastrtps_serialization.cpp
)

# Ensure the registration library is built after the message library
rosidl_get_typesupport_target(fastrtps_target ${PROJECT_NAME} rosidl_typesupport_fastrtps_cpp)

target_link_libraries(${PROJECT_NAME}_fastrtps_registration
  ${fastrtps_target}
)

ament_target_dependencies(${PROJECT_NAME}_fastrtps_registration
  rosidl_typesupport_fastrtps_cpp
)

# Create an alias to support namespaced access
add_library(${PROJECT_NAME}::${PROJECT_NAME}_fastrtps_registration ALIAS ${PROJECT_NAME}_fastrtps_registration)

install(TARGETS ${PROJECT_NAME}_fastrtps_registration
  EXPORT export_${PROJECT_NAME}_fastrtps_registration
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export the targets - makes them available to packages that depend on this package
ament_export_targets(export_${PROJECT_NAME}_fastrtps_registration HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
