cmake_minimum_required(VERSION 3.8)
project(demo_buffer_backend)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcl_buffer_backend REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rcl_buffer_backend_registry REQUIRED)
find_package(pluginlib REQUIRED)
find_package(demo_buffer REQUIRED)
find_package(demo_buffer_backend_msgs REQUIRED)
find_package(rcutils REQUIRED)

# Build the demo backend plugin library
add_library(${PROJECT_NAME} SHARED
  src/demo_buffer_backend_plugin.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  demo_buffer_backend_msgs::demo_buffer_backend_msgs__rosidl_typesupport_fastrtps_cpp
  rcl_buffer_backend::rcl_buffer_backend
  rosidl_runtime_cpp::rosidl_runtime_cpp
  rcl_buffer_backend_registry::rcl_buffer_backend_registry
  pluginlib::pluginlib
  demo_buffer::demo_buffer
  rcutils::rcutils
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

# Install plugin descriptor (CRITICAL for pluginlib discovery)
install(
  FILES demo_buffer_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Register plugin with ament index for pluginlib discovery
pluginlib_export_plugin_description_file(rcl_buffer_backend demo_buffer_plugin.xml)

# Export dependencies
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rcl_buffer_backend
  rosidl_runtime_cpp
  rcl_buffer_backend_registry
  pluginlib
  demo_buffer
  demo_buffer_backend_msgs
  rcutils
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Basic backend functionality test
  ament_add_gtest(test_demo_buffer_backend
    test/test_demo_buffer_backend.cpp
  )
  if(TARGET test_demo_buffer_backend)
    target_link_libraries(test_demo_buffer_backend
      ${PROJECT_NAME}
      rcl_buffer_backend::rcl_buffer_backend
      rosidl_runtime_cpp::rosidl_runtime_cpp
      rcl_buffer_backend_registry::rcl_buffer_backend_registry
      ${demo_buffer_backend_msgs_TARGETS}
    )
  endif()
endif()

ament_package()
