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

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "cuda_buffer/cuda_buffer_api.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int32.hpp"

class CudaImage60FpsSubscriber : public rclcpp::Node
{
public:
  CudaImage60FpsSubscriber()
  : Node("cuda_image_60fps_subscriber")
  {
    this->declare_parameter<std::string>("topic_name", "cuda_image_60fps");
    this->declare_parameter<std::string>("expected_backend", "cuda");
    this->declare_parameter<std::string>("acceptable_buffer_backends", "any");
    this->declare_parameter<int>("log_every_n", 60);

    const auto topic_name = this->get_parameter("topic_name").as_string();
    expected_backend_ = this->get_parameter("expected_backend").as_string();
    const auto acceptable_backends =
      this->get_parameter("acceptable_buffer_backends").as_string();
    log_every_n_ = this->get_parameter("log_every_n").as_int();

    CUDA_CHECK(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking));

    rclcpp::SubscriptionOptions sub_options;
    if (!acceptable_backends.empty()) {
      sub_options.acceptable_buffer_backends = acceptable_backends;
    }
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic_name, 10,
      std::bind(&CudaImage60FpsSubscriber::on_image, this, std::placeholders::_1),
      sub_options);

    count_publisher_ =
      this->create_publisher<std_msgs::msg::UInt32>("cuda_60fps_subscriber_count", 10);
    validation_publisher_ =
      this->create_publisher<std_msgs::msg::Bool>("cuda_60fps_validation_result", 10);
    latency_publisher_ =
      this->create_publisher<std_msgs::msg::Float64>("cuda_60fps_latency_ms", 10);

    RCLCPP_INFO(
      this->get_logger(),
      "CUDA image subscriber started (topic=%s, expected_backend=%s, "
      "acceptable_buffer_backends=%s)",
      topic_name.c_str(), expected_backend_.c_str(), acceptable_backends.c_str());
  }

  ~CudaImage60FpsSubscriber() override
  {
    if (stream_ != nullptr) {
      cudaStreamSynchronize(stream_);
      cudaStreamDestroy(stream_);
    }
  }

private:
  void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    ++received_count_;

    bool metadata_valid = true;
    bool backend_valid = true;
    bool content_valid = true;

    if (msg->encoding != "rgb8") {
      RCLCPP_ERROR(this->get_logger(), "Wrong encoding: %s", msg->encoding.c_str());
      metadata_valid = false;
    }
    const std::size_t expected_size =
      static_cast<std::size_t>(msg->width) *
      static_cast<std::size_t>(msg->height) *
      kChannelCount;
    if (msg->step != msg->width * kChannelCount || msg->data.size() != expected_size) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Wrong image layout: step=%u size=%zu expected_step=%u expected_size=%zu",
        msg->step, msg->data.size(), msg->width * kChannelCount, expected_size);
      metadata_valid = false;
    }

    const auto backend_type = msg->data.get_backend_type();
    if (backend_type != expected_backend_) {
      RCLCPP_ERROR(
        this->get_logger(), "Wrong backend type: %s (expected: %s)",
        backend_type.c_str(), expected_backend_.c_str());
      backend_valid = false;
    }

    std::vector<std::uint8_t> cpu_data;
    try {
      if (backend_type == "cuda") {
        auto read_handle = cuda_buffer_backend::from_input_buffer(msg->data, stream_);
        cpu_data.resize(msg->data.size());
        CUDA_CHECK(cudaMemcpyAsync(
          cpu_data.data(), read_handle.get_ptr(), msg->data.size(),
          cudaMemcpyDeviceToHost, stream_));
        CUDA_CHECK(cudaStreamSynchronize(stream_));
      } else {
        cpu_data = msg->data.to_vector();
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Exception while reading image data: %s", e.what());
      content_valid = false;
    }

    if (!cpu_data.empty()) {
      const auto expected_value = cpu_data[0];
      for (std::size_t i = 0; i < cpu_data.size(); ++i) {
        if (cpu_data[i] != expected_value) {
          RCLCPP_ERROR(
            this->get_logger(),
            "Content mismatch at byte %zu: got %u, expected %u",
            i, cpu_data[i], expected_value);
          content_valid = false;
          break;
        }
      }
    } else if (metadata_valid) {
      RCLCPP_ERROR(this->get_logger(), "Image data is empty");
      content_valid = false;
    }

    const bool msg_valid = metadata_valid && backend_valid && content_valid;
    validation_passed_ = validation_passed_ && msg_valid;

    std_msgs::msg::UInt32 count_msg;
    count_msg.data = static_cast<std::uint32_t>(received_count_);
    count_publisher_->publish(count_msg);

    std_msgs::msg::Bool validation_msg;
    validation_msg.data = validation_passed_;
    validation_publisher_->publish(validation_msg);

    std_msgs::msg::Float64 latency_msg;
    latency_msg.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
    latency_publisher_->publish(latency_msg);

    if (!msg_valid) {
      RCLCPP_ERROR(this->get_logger(), "Received invalid CUDA image #%zu", received_count_);
    } else if (log_every_n_ > 0 &&
      received_count_ % static_cast<std::size_t>(log_every_n_) == 0)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Received CUDA image #%zu (%ux%u, backend=%s, latency=%.3f ms)",
        received_count_, msg->width, msg->height, backend_type.c_str(), latency_msg.data);
    }
  }

  static constexpr std::uint32_t kChannelCount = 3;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr count_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr validation_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_publisher_;
  cudaStream_t stream_{nullptr};
  std::string expected_backend_;
  std::size_t received_count_ = 0;
  bool validation_passed_ = true;
  int log_every_n_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CudaImage60FpsSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
