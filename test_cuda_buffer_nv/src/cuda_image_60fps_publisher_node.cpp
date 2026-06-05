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

#include <cuda_runtime.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "cuda_buffer/cuda_buffer_api.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/u_int32.hpp"

class CudaImage60FpsPublisher : public rclcpp::Node
{
public:
  CudaImage60FpsPublisher()
  : Node("cuda_image_60fps_publisher")
  {
    this->declare_parameter<std::string>("topic_name", "cuda_image_60fps");
    this->declare_parameter<int>("publish_rate_ms", 17);
    this->declare_parameter<int>("image_width", 640);
    this->declare_parameter<int>("image_height", 480);
    this->declare_parameter<int>("max_publish_count", 0);
    this->declare_parameter<int>("expected_subscription_count", 0);
    this->declare_parameter<int>("log_every_n", 60);

    const auto topic_name = this->get_parameter("topic_name").as_string();
    const auto publish_rate_ms =
      static_cast<int>(this->get_parameter("publish_rate_ms").as_int());
    image_width_ = this->get_parameter("image_width").as_int();
    image_height_ = this->get_parameter("image_height").as_int();
    max_publish_count_ =
      static_cast<std::size_t>(this->get_parameter("max_publish_count").as_int());
    const auto expected_subscription_count =
      this->get_parameter("expected_subscription_count").as_int();
    expected_subscription_count_ = expected_subscription_count > 0 ?
      static_cast<std::size_t>(expected_subscription_count) : 0u;
    log_every_n_ = this->get_parameter("log_every_n").as_int();

    CUDA_CHECK(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking));

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
    count_publisher_ =
      this->create_publisher<std_msgs::msg::UInt32>("cuda_60fps_publisher_count", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(publish_rate_ms),
      std::bind(&CudaImage60FpsPublisher::on_timer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "CUDA image publisher started (topic=%s, period=%d ms, size=%dx%d, max_count=%zu)",
      topic_name.c_str(), publish_rate_ms, image_width_, image_height_,
      max_publish_count_ == 0 ? SIZE_MAX : max_publish_count_);
  }

  ~CudaImage60FpsPublisher() override
  {
    if (stream_ != nullptr) {
      cudaStreamSynchronize(stream_);
      cudaStreamDestroy(stream_);
    }
  }

private:
  void on_timer()
  {
    if (max_publish_count_ > 0 && count_ >= max_publish_count_) {
      return;
    }
    if (publisher_->get_subscription_count() < expected_subscription_count_) {
      return;
    }

    sensor_msgs::msg::Image msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "cuda_image_60fps_frame";
    msg.height = static_cast<std::uint32_t>(image_height_);
    msg.width = static_cast<std::uint32_t>(image_width_);
    msg.encoding = "rgb8";
    msg.is_bigendian = 0;
    msg.step = static_cast<std::uint32_t>(image_width_ * kChannelCount);

    const std::size_t data_size =
      static_cast<std::size_t>(image_width_) *
      static_cast<std::size_t>(image_height_) *
      kChannelCount;
    msg.data = cuda_buffer_backend::allocate_buffer(data_size);

    {
      auto write_handle = cuda_buffer_backend::from_output_buffer(msg.data, stream_);
      CUDA_CHECK(cudaMemsetAsync(
        write_handle.get_ptr(), static_cast<int>(count_ % 256), data_size, stream_));
    }

    publisher_->publish(msg);

    std_msgs::msg::UInt32 count_msg;
    count_msg.data = static_cast<std::uint32_t>(++count_);
    count_publisher_->publish(count_msg);

    if (log_every_n_ > 0 && count_ % static_cast<std::size_t>(log_every_n_) == 0) {
      RCLCPP_INFO(
        this->get_logger(), "Published %zu CUDA images", count_);
    }
  }

  static constexpr int kChannelCount = 3;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr count_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  cudaStream_t stream_{nullptr};
  std::size_t count_ = 0;
  std::size_t max_publish_count_ = 0;
  std::size_t expected_subscription_count_ = 0;
  int image_width_ = 0;
  int image_height_ = 0;
  int log_every_n_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CudaImage60FpsPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
