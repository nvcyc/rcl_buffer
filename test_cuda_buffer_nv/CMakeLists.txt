cmake_minimum_required(VERSION 3.20)
project(test_cuda_buffer_nv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(cuda_buffer REQUIRED)
find_package(cuda_buffer_backend REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(cuda_image_60fps_publisher_node
  src/cuda_image_60fps_publisher_node.cpp
)
target_link_libraries(cuda_image_60fps_publisher_node
  cuda_buffer::cuda_buffer
  CUDA::cudart
  CUDA::cuda_driver
  rclcpp::rclcpp
  rosidl_runtime_cpp::rosidl_runtime_cpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

add_executable(cuda_image_60fps_subscriber_node
  src/cuda_image_60fps_subscriber_node.cpp
)
target_link_libraries(cuda_image_60fps_subscriber_node
  cuda_buffer::cuda_buffer
  CUDA::cudart
  CUDA::cuda_driver
  rclcpp::rclcpp
  rosidl_runtime_cpp::rosidl_runtime_cpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

install(TARGETS
  cuda_image_60fps_publisher_node
  cuda_image_60fps_subscriber_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
