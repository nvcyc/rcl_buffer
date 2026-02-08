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
find_package(demo_buffer QUIET)
find_package(demo_buffer_backend QUIET)

# Build standalone test executable (no ROS)
add_executable(test_cpu_buffer_image src/test_cpu_buffer_image.cpp)
target_link_libraries(test_cpu_buffer_image
  rosidl_runtime_cpp::rosidl_runtime_cpp
  ${sensor_msgs_TARGETS}
)

# Build pub/sub integration test
add_executable(test_cpu_buffer_image_pubsub src/test_cpu_buffer_image_pubsub.cpp)
target_link_libraries(test_cpu_buffer_image_pubsub
  rclcpp::rclcpp
  rosidl_runtime_cpp::rosidl_runtime_cpp
  ${sensor_msgs_TARGETS}
)

install(TARGETS
  test_cpu_buffer_image
  test_cpu_buffer_image_pubsub
  DESTINATION lib/${PROJECT_NAME}
)

# Demo backend publisher/subscriber nodes (need demo_buffer for the impl)
# NOTE: We link demo_buffer (the impl-only header package), NOT demo_buffer_backend
# (the pluginlib plugin). Directly linking a pluginlib plugin library causes a
# double-free on shutdown because class_loader's ProxyExec static is destroyed
# twice (once by pluginlib::ClassLoader and once by __cxa_finalize).
if(demo_buffer_FOUND)
  message(STATUS "demo_buffer found - building demo backend nodes")

  # Demo backend image publisher (standalone executable)
  add_executable(demo_backend_image_publisher_node
    src/demo_backend_image_publisher_node.cpp
  )
  target_link_libraries(demo_backend_image_publisher_node
    demo_buffer::demo_buffer
    rclcpp::rclcpp
    rosidl_runtime_cpp::rosidl_runtime_cpp
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
  )

  # Demo backend image subscriber (standalone executable)
  add_executable(demo_backend_image_subscriber_node
    src/demo_backend_image_subscriber_node.cpp
  )
  target_link_libraries(demo_backend_image_subscriber_node
    rclcpp::rclcpp
    rosidl_runtime_cpp::rosidl_runtime_cpp
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
  )

  # Demo backend pub/sub in single process
  add_executable(test_demo_buffer_image_pubsub
    src/test_demo_buffer_image_pubsub.cpp
  )
  target_link_libraries(test_demo_buffer_image_pubsub
    demo_buffer::demo_buffer
    rclcpp::rclcpp
    rosidl_runtime_cpp::rosidl_runtime_cpp
    ${sensor_msgs_TARGETS}
  )

  install(TARGETS
    demo_backend_image_publisher_node
    demo_backend_image_subscriber_node
    test_demo_buffer_image_pubsub
    DESTINATION lib/${PROJECT_NAME}
  )

else()
  message(STATUS "demo_buffer NOT found - skipping demo backend nodes")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Launch tests for buffer backend (need demo_buffer_backend plugin installed for RMW)
  if(demo_buffer_backend_FOUND)
    find_package(launch_testing_ament_cmake REQUIRED)

    # 1 publisher to 1 subscriber tests
    add_launch_test(test/test_1pub_1sub_cpu_to_cpu.py
      TARGET test_1pub_1sub_cpu_to_cpu
      TIMEOUT 30
    )
    add_launch_test(test/test_1pub_1sub_demo_to_demo.py
      TARGET test_1pub_1sub_demo_to_demo
      TIMEOUT 30
    )
  endif()
endif()

ament_package()
