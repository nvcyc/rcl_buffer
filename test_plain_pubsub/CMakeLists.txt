cmake_minimum_required(VERSION 3.8)
project(test_plain_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(plain_string_publisher_node
  src/plain_string_publisher_node.cpp
)
target_link_libraries(plain_string_publisher_node
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)

add_executable(plain_string_subscriber_node
  src/plain_string_subscriber_node.cpp
)
target_link_libraries(plain_string_subscriber_node
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)

add_executable(plain_image_publisher_node
  src/plain_image_publisher_node.cpp
)
target_link_libraries(plain_image_publisher_node
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

add_executable(plain_image_subscriber_node
  src/plain_image_subscriber_node.cpp
)
target_link_libraries(plain_image_subscriber_node
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

install(TARGETS
  plain_image_publisher_node
  plain_image_subscriber_node
  plain_string_publisher_node
  plain_string_subscriber_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_2pub_2sub_plain_fastrtps.py
    TARGET test_2pub_2sub_plain_fastrtps
    TIMEOUT 60
  )
  add_launch_test(test/test_2pub_2sub_plain_image_fastrtps.py
    TARGET test_2pub_2sub_plain_image_fastrtps
    TIMEOUT 60
  )
endif()

ament_package()
