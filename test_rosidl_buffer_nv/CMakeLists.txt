cmake_minimum_required(VERSION 3.8)
project(test_rosidl_buffer_nv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(test_msgs REQUIRED)
find_package(isaac_ros_tensor_list_interfaces REQUIRED)

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

# Reproducer for the template-argument-deduction failure reported by
# downstream OpenCV users (e.g. bag2_to_image). By default this target
# compiles cleanly and asserts the expected failure via static_assert.
# Configure with -DTEST_BUFFER_REPRO_COMPILE_ERROR=ON to force the
# reproducer to emit the actual compile error.
option(TEST_BUFFER_REPRO_COMPILE_ERROR
  "Enable the failing line in test_buffer_template_deduction to surface \
the exact downstream compile error." OFF)
add_executable(test_buffer_template_deduction src/test_buffer_template_deduction.cpp)
target_link_libraries(test_buffer_template_deduction
  rosidl_runtime_cpp::rosidl_runtime_cpp
  ${sensor_msgs_TARGETS}
)
if(TEST_BUFFER_REPRO_COMPILE_ERROR)
  target_compile_definitions(test_buffer_template_deduction
    PRIVATE REPRO_COMPILE_ERROR=1)
endif()

install(TARGETS
  test_cpu_buffer_image
  test_cpu_buffer_image_pubsub
  test_buffer_template_deduction
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

  # Intra-process pub+sub combined with inter-process subscriber support
  add_executable(test_intra_inter_image_pubsub
    src/test_intra_inter_image_pubsub.cpp
  )
  target_link_libraries(test_intra_inter_image_pubsub
    demo_buffer::demo_buffer
    rclcpp::rclcpp
    rosidl_runtime_cpp::rosidl_runtime_cpp
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
  )

  install(TARGETS
    demo_backend_image_publisher_node
    demo_backend_image_subscriber_node
    test_demo_buffer_image_pubsub
    test_intra_inter_image_pubsub
    DESTINATION lib/${PROJECT_NAME}
  )

  # MultiNested ("list of uint8[]") pub/sub — single-process smoke test and
  # launch-test driver nodes.
  #
  # test_msgs::msg::MultiNested embeds UnboundedSequences[], each with a
  # `uint8[] uint8_values` field that the rosidl C++ generator emits as
  # rosidl::Buffer<uint8_t>. These targets verify nested Buffer fields
  # round-trip and can be constructed with an explicit demo backend.
  # test_msgs is a required dependency of this package; missing it is a
  # build-time error.

  # Single-process integration test (no launch).
  add_executable(test_demo_buffer_multi_nested_pubsub
    src/test_demo_buffer_multi_nested_pubsub.cpp
  )
  target_link_libraries(test_demo_buffer_multi_nested_pubsub
    demo_buffer::demo_buffer
    rclcpp::rclcpp
    rosidl_runtime_cpp::rosidl_runtime_cpp
    ${test_msgs_TARGETS}
  )

  # Configurable publisher node for launch tests
  # (cpu/demo selectable via parameter).
  add_executable(multi_nested_publisher_node
    src/multi_nested_publisher_node.cpp
  )
  target_link_libraries(multi_nested_publisher_node
    demo_buffer::demo_buffer
    rclcpp::rclcpp
    rosidl_runtime_cpp::rosidl_runtime_cpp
    ${std_msgs_TARGETS}
    ${test_msgs_TARGETS}
  )

  # Subscriber node for launch tests (content + backend validation,
  # emits counters/validation/backend_report topics for launch_testing).
  add_executable(multi_nested_subscriber_node
    src/multi_nested_subscriber_node.cpp
  )
  target_link_libraries(multi_nested_subscriber_node
    rclcpp::rclcpp
    rosidl_runtime_cpp::rosidl_runtime_cpp
    ${std_msgs_TARGETS}
    ${test_msgs_TARGETS}
  )

  install(TARGETS
    test_demo_buffer_multi_nested_pubsub
    multi_nested_publisher_node
    multi_nested_subscriber_node
    DESTINATION lib/${PROJECT_NAME}
  )

  # TensorList ("list of uint8[]") pub/sub — same shape of test as MultiNested
  # but driven by a real-world Isaac ROS message type
  # (isaac_ros_tensor_list_interfaces::msg::TensorList). Each Tensor in the
  # outer list has a `uint8[] data` field that the rosidl C++ generator
  # emits as rosidl::Buffer<uint8_t>. isaac_ros_tensor_list_interfaces is a
  # required dependency of this package; missing it is a build-time error.

  # Single-process integration test (no launch).
  add_executable(test_demo_buffer_tensor_list_pubsub
    src/test_demo_buffer_tensor_list_pubsub.cpp
  )
  target_link_libraries(test_demo_buffer_tensor_list_pubsub
    demo_buffer::demo_buffer
    rclcpp::rclcpp
    rosidl_runtime_cpp::rosidl_runtime_cpp
    ${isaac_ros_tensor_list_interfaces_TARGETS}
  )

  # Configurable publisher node for launch tests
  # (cpu/demo selectable via parameter).
  add_executable(tensor_list_publisher_node
    src/tensor_list_publisher_node.cpp
  )
  target_link_libraries(tensor_list_publisher_node
    demo_buffer::demo_buffer
    rclcpp::rclcpp
    rosidl_runtime_cpp::rosidl_runtime_cpp
    ${std_msgs_TARGETS}
    ${isaac_ros_tensor_list_interfaces_TARGETS}
  )

  # Subscriber node for launch tests (content + backend validation,
  # emits counters/validation/backend_report topics for launch_testing).
  add_executable(tensor_list_subscriber_node
    src/tensor_list_subscriber_node.cpp
  )
  target_link_libraries(tensor_list_subscriber_node
    rclcpp::rclcpp
    rosidl_runtime_cpp::rosidl_runtime_cpp
    ${std_msgs_TARGETS}
    ${isaac_ros_tensor_list_interfaces_TARGETS}
  )

  install(TARGETS
    test_demo_buffer_tensor_list_pubsub
    tensor_list_publisher_node
    tensor_list_subscriber_node
    DESTINATION lib/${PROJECT_NAME}
  )

else()
  message(STATUS "demo_buffer NOT found - skipping demo backend nodes")
endif()

# Install Python package for rclpy test nodes.
# SCRIPTS_DESTINATION makes setuptools generate proper .exe wrappers on Windows
# (extensionless shebang scripts don't work there — WinError 193).
ament_python_install_package(test_rosidl_buffer_nv
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Unit test for Buffer <-> array.array backward compatibility (no ROS nodes needed)
  ament_add_pytest_test(test_buffer_array_compat
    test/test_buffer_array_compat.py
    TIMEOUT 30
  )

  # Launch tests for buffer backend (need demo_buffer_backend plugin installed for RMW)
  if(demo_buffer_backend_FOUND)
    find_package(launch_testing_ament_cmake REQUIRED)

    # Zenoh tests: C++ publisher/subscriber: 1 pub to 1 sub tests
    add_launch_test(test/test_1pub_1sub_cpu_to_cpu_zenoh.py
      TARGET test_1pub_1sub_cpu_to_cpu_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_1pub_1sub_demo_to_demo_zenoh.py
      TARGET test_1pub_1sub_demo_to_demo_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_1pub_2sub_cpu_to_cpu_zenoh.py
      TARGET test_1pub_2sub_cpu_to_cpu_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_1pub_2sub_demo_to_demo_zenoh.py
      TARGET test_1pub_2sub_demo_to_demo_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_2pub_2sub_cpu_to_cpu_zenoh.py
      TARGET test_2pub_2sub_cpu_to_cpu_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_2pub_2sub_demo_to_demo_zenoh.py
      TARGET test_2pub_2sub_demo_to_demo_zenoh
      TIMEOUT 30
    )

    # Zenoh tests: MultiNested ("list of uint8[]") — nested Buffer fields.
    # Exercise both CPU and demo construction paths; subscriber validates
    # content round-trip and reports the observed backend of the nested
    # uint8_values fields.
    add_launch_test(test/test_multi_nested_demo_to_demo_zenoh.py
      TARGET test_multi_nested_demo_to_demo_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_multi_nested_cpu_to_cpu_zenoh.py
      TARGET test_multi_nested_cpu_to_cpu_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_multi_nested_demo_to_cpu_zenoh.py
      TARGET test_multi_nested_demo_to_cpu_zenoh
      TIMEOUT 30
    )

    # Zenoh tests: TensorList ("list of uint8[]") — nested Buffer fields in a
    # real-world Isaac ROS message type. Same matrix of demo/cpu construction
    # paths as MultiNested above; subscriber validates content round-trip and
    # reports the observed backend of nested tensor.data buffers.
    add_launch_test(test/test_tensor_list_demo_to_demo_zenoh.py
      TARGET test_tensor_list_demo_to_demo_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_tensor_list_cpu_to_cpu_zenoh.py
      TARGET test_tensor_list_cpu_to_cpu_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_tensor_list_demo_to_cpu_zenoh.py
      TARGET test_tensor_list_demo_to_cpu_zenoh
      TIMEOUT 30
    )

    # Zenoh tests: acceptable_buffer_backends option tests
    add_launch_test(test/test_1pub_1sub_demo_to_demo_sub_any_zenoh.py
      TARGET test_1pub_1sub_demo_to_demo_sub_any_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_1pub_1sub_demo_to_demo_sub_demo_zenoh.py
      TARGET test_1pub_1sub_demo_to_demo_sub_demo_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_1pub_1sub_cpu_to_cpu_sub_empty_zenoh.py
      TARGET test_1pub_1sub_cpu_to_cpu_sub_empty_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_1pub_1sub_cpu_to_cpu_sub_cpu_zenoh.py
      TARGET test_1pub_1sub_cpu_to_cpu_sub_cpu_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_1pub_1sub_cpu_to_cpu_sub_default_zenoh.py
      TARGET test_1pub_1sub_cpu_to_cpu_sub_default_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_1pub_1sub_demo_to_cpu_sub_empty_zenoh.py
      TARGET test_1pub_1sub_demo_to_cpu_sub_empty_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_1pub_1sub_demo_to_cpu_sub_cpu_zenoh.py
      TARGET test_1pub_1sub_demo_to_cpu_sub_cpu_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_1pub_1sub_demo_to_cpu_sub_default_zenoh.py
      TARGET test_1pub_1sub_demo_to_cpu_sub_default_zenoh
      TIMEOUT 30
    )

    # Zenoh tests: rclpy publisher/subscriber: 1 pub to 1 sub tests
    add_launch_test(test/test_rclpy_1pub_1sub_cpu_to_cpu_zenoh.py
      TARGET test_rclpy_1pub_1sub_cpu_to_cpu_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_rclpy_1pub_1sub_cpu_to_cpu_explicit_zenoh.py
      TARGET test_rclpy_1pub_1sub_cpu_to_cpu_explicit_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_rclpy_1pub_1sub_demo_to_demo_zenoh.py
      TARGET test_rclpy_1pub_1sub_demo_to_demo_zenoh
      TIMEOUT 30
    )

    # Zenoh tests: Cross-language (rclcpp <-> rclpy) tests
    add_launch_test(test/test_rclcpp_pub_cpu_rclpy_sub_cpu_zenoh.py
      TARGET test_rclcpp_pub_cpu_rclpy_sub_cpu_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_rclpy_pub_cpu_rclcpp_sub_cpu_zenoh.py
      TARGET test_rclpy_pub_cpu_rclcpp_sub_cpu_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_rclcpp_pub_demo_rclpy_sub_demo_zenoh.py
      TARGET test_rclcpp_pub_demo_rclpy_sub_demo_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_rclpy_pub_demo_rclcpp_sub_demo_zenoh.py
      TARGET test_rclpy_pub_demo_rclcpp_sub_demo_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_rclcpp_pub_cpu_rclpy_sub_cpu_explicit_zenoh.py
      TARGET test_rclcpp_pub_cpu_rclpy_sub_cpu_explicit_zenoh
      TIMEOUT 30
    )

    # Zenoh tests: Cross-language 1 pub to 2 subs (rclcpp + rclpy) tests
    add_launch_test(test/test_rclcpp_pub_cpu_rclcpp_sub_rclpy_sub_cpu_zenoh.py
      TARGET test_rclcpp_pub_cpu_rclcpp_sub_rclpy_sub_cpu_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_rclpy_pub_cpu_rclcpp_sub_rclpy_sub_cpu_zenoh.py
      TARGET test_rclpy_pub_cpu_rclcpp_sub_rclpy_sub_cpu_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_rclcpp_pub_demo_rclcpp_sub_rclpy_sub_demo_zenoh.py
      TARGET test_rclcpp_pub_demo_rclcpp_sub_rclpy_sub_demo_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_rclpy_pub_demo_rclcpp_sub_rclpy_sub_demo_zenoh.py
      TARGET test_rclpy_pub_demo_rclcpp_sub_rclpy_sub_demo_zenoh
      TIMEOUT 30
    )

    # Zenoh tests: Demo publisher to legacy array.array subscriber (backward-compat)
    add_launch_test(test/test_rclpy_demo_pub_legacy_array_sub_zenoh.py
      TARGET test_rclpy_demo_pub_legacy_array_sub_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_rclcpp_demo_pub_rclpy_legacy_array_sub_zenoh.py
      TARGET test_rclcpp_demo_pub_rclpy_legacy_array_sub_zenoh
      TIMEOUT 30
    )

    # Zenoh tests: Legacy array.array publisher to demo-aware subscriber (backward-compat)
    add_launch_test(test/test_rclpy_legacy_array_pub_demo_sub_zenoh.py
      TARGET test_rclpy_legacy_array_pub_demo_sub_zenoh
      TIMEOUT 30
    )

    # Zenoh tests: Same-process (pub + sub in one process)
    add_launch_test(test/test_same_process_cpu_to_cpu_zenoh.py
      TARGET test_same_process_cpu_to_cpu_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_same_process_demo_to_demo_zenoh.py
      TARGET test_same_process_demo_to_demo_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_rclpy_same_process_cpu_to_cpu_zenoh.py
      TARGET test_rclpy_same_process_cpu_to_cpu_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_rclpy_same_process_demo_to_demo_zenoh.py
      TARGET test_rclpy_same_process_demo_to_demo_zenoh
      TIMEOUT 30
    )

    # Service / action route tests (rclpy) - Zenoh
    add_launch_test(test/test_rclpy_service_cpu_zenoh.py
      TARGET test_rclpy_service_cpu_zenoh
      TIMEOUT 45
    )
    add_launch_test(test/test_rclpy_action_zenoh.py
      TARGET test_rclpy_action_zenoh
      TIMEOUT 45
    )

    # Service / action route tests (rclpy) - FastRTPS
    add_launch_test(test/test_rclpy_service_cpu_fastrtps.py
      TARGET test_rclpy_service_cpu_fastrtps
      TIMEOUT 45
    )
    # TODO: Re-enable when full demo-buffer service response path is stable in this test scope.
    # add_launch_test(test/test_rclpy_service_demo_fastrtps.py
    #   TARGET test_rclpy_service_demo_fastrtps
    #   TIMEOUT 45
    # )
    add_launch_test(test/test_rclpy_action_fastrtps.py
      TARGET test_rclpy_action_fastrtps
      TIMEOUT 45
    )

    # acceptable_buffer_backends option tests: demo to demo with 'any' and 'demo'

    # CycloneDDS tests: acceptable_buffer_backends option tests: cpu to cpu with '', 'cpu', and default
    add_launch_test(test/test_1pub_1sub_cpu_to_cpu_sub_empty_cyclonedds.py
      TARGET test_1pub_1sub_cpu_to_cpu_sub_empty_cyclonedds
      TIMEOUT 30
    )
    add_launch_test(test/test_1pub_1sub_cpu_to_cpu_sub_cpu_cyclonedds.py
      TARGET test_1pub_1sub_cpu_to_cpu_sub_cpu_cyclonedds
      TIMEOUT 30
    )
    add_launch_test(test/test_1pub_1sub_cpu_to_cpu_sub_default_cyclonedds.py
      TARGET test_1pub_1sub_cpu_to_cpu_sub_default_cyclonedds
      TIMEOUT 30
    )

    # CycloneDDS tests: acceptable_buffer_backends option tests: demo to cpu with '', 'cpu', and default
    # TODO: Re-enable when CycloneDDS demo-to-cpu backend fallback is stable.
    # add_launch_test(test/test_1pub_1sub_demo_to_cpu_sub_empty_cyclonedds.py
    #   TARGET test_1pub_1sub_demo_to_cpu_sub_empty_cyclonedds
    #   TIMEOUT 30
    # )
    # add_launch_test(test/test_1pub_1sub_demo_to_cpu_sub_cpu_cyclonedds.py
    #   TARGET test_1pub_1sub_demo_to_cpu_sub_cpu_cyclonedds
    #   TIMEOUT 30
    # )
    # add_launch_test(test/test_1pub_1sub_demo_to_cpu_sub_default_cyclonedds.py
    #   TARGET test_1pub_1sub_demo_to_cpu_sub_default_cyclonedds
    #   TIMEOUT 30
    # )

    # CycloneDDS tests: CPU backend pub/sub
    add_launch_test(test/test_1pub_1sub_cpu_to_cpu_cyclonedds.py
      TARGET test_1pub_1sub_cpu_to_cpu_cyclonedds
      TIMEOUT 30
    )
    add_launch_test(test/test_1pub_2sub_cpu_to_cpu_cyclonedds.py
      TARGET test_1pub_2sub_cpu_to_cpu_cyclonedds
      TIMEOUT 30
    )
    # TODO: Re-enable when CycloneDDS demo-to-cpu backend fallback is stable.
    # add_launch_test(test/test_1pub_1sub_demo_to_cpu_cyclonedds.py
    #   TARGET test_1pub_1sub_demo_to_cpu_cyclonedds
    #   TIMEOUT 30
    # )

    # FastRTPS tests: buffer-aware pub/sub over rmw_fastrtps_cpp
    # Higher timeouts than Zenoh/CycloneDDS because DDS discovery + per-endpoint
    # DataWriter/DataReader creation adds latency before messages can flow.
    add_launch_test(test/test_1pub_1sub_cpu_to_cpu_fastrtps.py
      TARGET test_1pub_1sub_cpu_to_cpu_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_1pub_1sub_demo_to_demo_fastrtps.py
      TARGET test_1pub_1sub_demo_to_demo_fastrtps
      TIMEOUT 45
    )
    # TODO: demo_to_cpu requires further RMW changes for mixed-backend support
    # add_launch_test(test/test_1pub_1sub_demo_to_cpu_fastrtps.py
    #   TARGET test_1pub_1sub_demo_to_cpu_fastrtps
    #   TIMEOUT 45
    # )
    add_launch_test(test/test_1pub_2sub_cpu_to_cpu_fastrtps.py
      TARGET test_1pub_2sub_cpu_to_cpu_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_1pub_2sub_demo_to_demo_fastrtps.py
      TARGET test_1pub_2sub_demo_to_demo_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_2pub_2sub_cpu_to_cpu_fastrtps.py
      TARGET test_2pub_2sub_cpu_to_cpu_fastrtps
      TIMEOUT 60
    )
    add_launch_test(test/test_2pub_2sub_demo_to_demo_fastrtps.py
      TARGET test_2pub_2sub_demo_to_demo_fastrtps
      TIMEOUT 60
    )

    # FastRTPS tests: MultiNested ("list of uint8[]") — nested Buffer fields.
    add_launch_test(test/test_multi_nested_demo_to_demo_fastrtps.py
      TARGET test_multi_nested_demo_to_demo_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_multi_nested_cpu_to_cpu_fastrtps.py
      TARGET test_multi_nested_cpu_to_cpu_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_multi_nested_demo_to_cpu_fastrtps.py
      TARGET test_multi_nested_demo_to_cpu_fastrtps
      TIMEOUT 45
    )

    # FastRTPS tests: TensorList ("list of uint8[]") — nested Buffer fields in
    # a real-world Isaac ROS message type.
    add_launch_test(test/test_tensor_list_demo_to_demo_fastrtps.py
      TARGET test_tensor_list_demo_to_demo_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_tensor_list_cpu_to_cpu_fastrtps.py
      TARGET test_tensor_list_cpu_to_cpu_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_tensor_list_demo_to_cpu_fastrtps.py
      TARGET test_tensor_list_demo_to_cpu_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_rclcpp_pub_tensor_list_demo_rclpy_sub_demo_fastrtps.py
      TARGET test_rclcpp_pub_tensor_list_demo_rclpy_sub_demo_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_rclpy_pub_tensor_list_demo_rclcpp_sub_demo_fastrtps.py
      TARGET test_rclpy_pub_tensor_list_demo_rclcpp_sub_demo_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_rclpy_pub_tensor_list_demo_rclpy_sub_demo_fastrtps.py
      TARGET test_rclpy_pub_tensor_list_demo_rclpy_sub_demo_fastrtps
      TIMEOUT 45
    )

    # FastRTPS tests: acceptable_buffer_backends option tests
    add_launch_test(test/test_1pub_1sub_demo_to_demo_sub_any_fastrtps.py
      TARGET test_1pub_1sub_demo_to_demo_sub_any_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_1pub_1sub_demo_to_demo_sub_demo_fastrtps.py
      TARGET test_1pub_1sub_demo_to_demo_sub_demo_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_1pub_1sub_cpu_to_cpu_sub_empty_fastrtps.py
      TARGET test_1pub_1sub_cpu_to_cpu_sub_empty_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_1pub_1sub_cpu_to_cpu_sub_cpu_fastrtps.py
      TARGET test_1pub_1sub_cpu_to_cpu_sub_cpu_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_1pub_1sub_cpu_to_cpu_sub_default_fastrtps.py
      TARGET test_1pub_1sub_cpu_to_cpu_sub_default_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_1pub_1sub_demo_to_cpu_sub_empty_fastrtps.py
      TARGET test_1pub_1sub_demo_to_cpu_sub_empty_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_1pub_1sub_demo_to_cpu_sub_cpu_fastrtps.py
      TARGET test_1pub_1sub_demo_to_cpu_sub_cpu_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_1pub_1sub_demo_to_cpu_sub_default_fastrtps.py
      TARGET test_1pub_1sub_demo_to_cpu_sub_default_fastrtps
      TIMEOUT 45
    )

    # FastRTPS tests: rclpy-only publisher/subscriber
    add_launch_test(test/test_rclpy_1pub_1sub_cpu_to_cpu_fastrtps.py
      TARGET test_rclpy_1pub_1sub_cpu_to_cpu_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_rclpy_1pub_1sub_cpu_to_cpu_explicit_fastrtps.py
      TARGET test_rclpy_1pub_1sub_cpu_to_cpu_explicit_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_rclpy_1pub_1sub_demo_to_demo_fastrtps.py
      TARGET test_rclpy_1pub_1sub_demo_to_demo_fastrtps
      TIMEOUT 45
    )

    # FastRTPS tests: Cross-language (rclcpp <-> rclpy) tests
    add_launch_test(test/test_rclcpp_pub_cpu_rclpy_sub_cpu_fastrtps.py
      TARGET test_rclcpp_pub_cpu_rclpy_sub_cpu_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_rclpy_pub_cpu_rclcpp_sub_cpu_fastrtps.py
      TARGET test_rclpy_pub_cpu_rclcpp_sub_cpu_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_rclcpp_pub_demo_rclpy_sub_demo_fastrtps.py
      TARGET test_rclcpp_pub_demo_rclpy_sub_demo_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_rclpy_pub_demo_rclcpp_sub_demo_fastrtps.py
      TARGET test_rclpy_pub_demo_rclcpp_sub_demo_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_rclcpp_pub_cpu_rclpy_sub_cpu_explicit_fastrtps.py
      TARGET test_rclcpp_pub_cpu_rclpy_sub_cpu_explicit_fastrtps
      TIMEOUT 45
    )

    # FastRTPS tests: Cross-language 1 pub to 2 subs (rclcpp + rclpy)
    add_launch_test(test/test_rclcpp_pub_cpu_rclcpp_sub_rclpy_sub_cpu_fastrtps.py
      TARGET test_rclcpp_pub_cpu_rclcpp_sub_rclpy_sub_cpu_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_rclpy_pub_cpu_rclcpp_sub_rclpy_sub_cpu_fastrtps.py
      TARGET test_rclpy_pub_cpu_rclcpp_sub_rclpy_sub_cpu_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_rclcpp_pub_demo_rclcpp_sub_rclpy_sub_demo_fastrtps.py
      TARGET test_rclcpp_pub_demo_rclcpp_sub_rclpy_sub_demo_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_rclpy_pub_demo_rclcpp_sub_rclpy_sub_demo_fastrtps.py
      TARGET test_rclpy_pub_demo_rclcpp_sub_rclpy_sub_demo_fastrtps
      TIMEOUT 45
    )

    # FastRTPS tests: Legacy array.array backward compatibility
    add_launch_test(test/test_rclpy_demo_pub_legacy_array_sub_fastrtps.py
      TARGET test_rclpy_demo_pub_legacy_array_sub_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_rclcpp_demo_pub_rclpy_legacy_array_sub_fastrtps.py
      TARGET test_rclcpp_demo_pub_rclpy_legacy_array_sub_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_rclpy_legacy_array_pub_demo_sub_fastrtps.py
      TARGET test_rclpy_legacy_array_pub_demo_sub_fastrtps
      TIMEOUT 45
    )

    # FastRTPS tests: Same-process (pub + sub in one process)
    add_launch_test(test/test_same_process_cpu_to_cpu_fastrtps.py
      TARGET test_same_process_cpu_to_cpu_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_same_process_demo_to_demo_fastrtps.py
      TARGET test_same_process_demo_to_demo_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_rclpy_same_process_cpu_to_cpu_fastrtps.py
      TARGET test_rclpy_same_process_cpu_to_cpu_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_rclpy_same_process_demo_to_demo_fastrtps.py
      TARGET test_rclpy_same_process_demo_to_demo_fastrtps
      TIMEOUT 45
    )

    # Zenoh tests: Intra-process + inter-process (1 pub → 1 intra sub + 1 inter sub)
    add_launch_test(test/test_intra_cpu_inter_cpu_zenoh.py
      TARGET test_intra_cpu_inter_cpu_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_intra_demo_inter_demo_zenoh.py
      TARGET test_intra_demo_inter_demo_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_intra_demo_inter_cpu_zenoh.py
      TARGET test_intra_demo_inter_cpu_zenoh
      TIMEOUT 30
    )
    add_launch_test(test/test_intra_cpu_inter_demo_zenoh.py
      TARGET test_intra_cpu_inter_demo_zenoh
      TIMEOUT 30
    )

    # FastRTPS tests: Intra-process + inter-process (1 pub → 1 intra sub + 1 inter sub)
    add_launch_test(test/test_intra_cpu_inter_cpu_fastrtps.py
      TARGET test_intra_cpu_inter_cpu_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_intra_demo_inter_demo_fastrtps.py
      TARGET test_intra_demo_inter_demo_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_intra_demo_inter_cpu_fastrtps.py
      TARGET test_intra_demo_inter_cpu_fastrtps
      TIMEOUT 45
    )
    add_launch_test(test/test_intra_cpu_inter_demo_fastrtps.py
      TARGET test_intra_cpu_inter_demo_fastrtps
      TIMEOUT 45
    )
  endif()
endif()

ament_package()
