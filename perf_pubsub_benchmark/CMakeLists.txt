cmake_minimum_required(VERSION 3.8)
project(perf_pubsub_benchmark)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PerfMessage.msg"
)

add_executable(perf_publisher
  src/perf_publisher.cpp
)
target_link_libraries(perf_publisher rclcpp::rclcpp)

add_executable(perf_subscriber
  src/perf_subscriber.cpp
)
target_link_libraries(perf_subscriber rclcpp::rclcpp)

rosidl_get_typesupport_target(cpp_typesupport_target
  "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(perf_publisher "${cpp_typesupport_target}")
target_link_libraries(perf_subscriber "${cpp_typesupport_target}")

install(TARGETS
  perf_publisher
  perf_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/run_benchmark.py
  scripts/perf_publisher_py.py
  scripts/perf_subscriber_py.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
