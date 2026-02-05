cmake_minimum_required(VERSION 3.8)
project(test_rcl_buffer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)

# Optional dependencies for test backend nodes
find_package(rclcpp_components QUIET)
find_package(rcl_buffer_test_backend QUIET)

# Build standalone test executable (no ROS)
add_executable(test_buffer_image src/test_buffer_image.cpp)
ament_target_dependencies(test_buffer_image sensor_msgs rosidl_runtime_cpp)

# Build pub/sub integration test
add_executable(test_image_pubsub src/test_image_pubsub.cpp)
ament_target_dependencies(test_image_pubsub rclcpp sensor_msgs rosidl_runtime_cpp)

install(TARGETS
  test_buffer_image
  test_image_pubsub
  DESTINATION lib/${PROJECT_NAME}
)

# Test backend publisher/subscriber nodes (only if rcl_buffer_test_backend is available)
if(rcl_buffer_test_backend_FOUND)
  message(STATUS "rcl_buffer_test_backend found - building test backend nodes")

  # Test backend image publisher (standalone executable)
  add_executable(test_backend_image_publisher_node
    src/test_backend_image_publisher_node.cpp
  )
  target_link_libraries(test_backend_image_publisher_node
    rcl_buffer_test_backend::rcl_buffer_test_backend
  )
  ament_target_dependencies(test_backend_image_publisher_node
    rclcpp
    sensor_msgs
    std_msgs
    rosidl_runtime_cpp
  )

  # Test backend image subscriber (standalone executable)
  add_executable(test_backend_image_subscriber_node
    src/test_backend_image_subscriber_node.cpp
  )
  ament_target_dependencies(test_backend_image_subscriber_node
    rclcpp
    sensor_msgs
    std_msgs
    rosidl_runtime_cpp
  )

  install(TARGETS
    test_backend_image_publisher_node
    test_backend_image_subscriber_node
    DESTINATION lib/${PROJECT_NAME}
  )

else()
  message(STATUS "rcl_buffer_test_backend NOT found - skipping test backend nodes")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Launch test for test backend (only if rcl_buffer_test_backend is available)
  if(rcl_buffer_test_backend_FOUND)
    find_package(launch_testing_ament_cmake REQUIRED)
    add_launch_test(test/test_test_backend_image_pubsub_launch.py
      TARGET test_test_backend_image_pubsub_launch
      TIMEOUT 60
    )
  endif()
endif()

ament_package()
