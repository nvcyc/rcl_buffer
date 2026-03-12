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

// Integration test: single-process pub/sub using DemoBufferImpl backend.
// Links demo_buffer (the impl-only package) to avoid the pluginlib double-free.

#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rosidl_buffer/buffer.hpp"
#include "demo_buffer/demo_buffer_impl.hpp"

using namespace std::chrono_literals;

class DemoImagePublisher : public rclcpp::Node
{
public:
  DemoImagePublisher()
  : Node("demo_image_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("test_image", 10);
    timer_ = this->create_wall_timer(
      100ms, std::bind(&DemoImagePublisher::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Demo image publisher started");
  }

  size_t get_publish_count() const {return count_;}

private:
  void timer_callback()
  {
    auto msg = sensor_msgs::msg::Image();
    msg.header.stamp = this->now();
    msg.header.frame_id = "test_frame";
    msg.height = 8;
    msg.width = 8;
    msg.encoding = "rgb8";
    msg.step = 8 * 3;
    msg.is_bigendian = 0;

    const size_t data_size = 8 * 8 * 3;

    // Create data with demo backend
    std::vector<uint8_t> host_data(data_size);
    for (size_t i = 0; i < data_size; ++i) {
      host_data[i] = static_cast<uint8_t>((count_ + i) % 256);
    }

    auto demo_impl = std::make_unique<demo_buffer_backend::DemoBufferImpl<uint8_t>>(
      std::move(host_data));
    msg.data = rosidl::Buffer<uint8_t>(std::move(demo_impl));

    publisher_->publish(msg);
    count_++;
    RCLCPP_INFO(
      this->get_logger(), "Published image #%zu (backend: %s)",
      count_, msg.data.get_backend_type().c_str());
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

class DemoImageSubscriber : public rclcpp::Node
{
public:
  DemoImageSubscriber()
  : Node("demo_image_subscriber"), received_count_(0), last_correct_(true)
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "test_image", 10,
      std::bind(&DemoImageSubscriber::image_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Demo image subscriber started");
  }

  size_t get_received_count() const {return received_count_;}
  bool is_last_correct() const {return last_correct_;}

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    received_count_++;
    last_correct_ = true;

    if (msg->width != 8 || msg->height != 8) {
      RCLCPP_ERROR(this->get_logger(), "Wrong dimensions");
      last_correct_ = false;
      return;
    }
    if (msg->data.size() != 8 * 8 * 3) {
      RCLCPP_ERROR(this->get_logger(), "Wrong data size: %zu", msg->data.size());
      last_correct_ = false;
      return;
    }

    RCLCPP_INFO(
      this->get_logger(), "Received image #%zu (backend: %s, size: %zu)",
      received_count_, msg->data.get_backend_type().c_str(), msg->data.size());
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  size_t received_count_;
  bool last_correct_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::cout << "\n=== Starting Demo Backend Pub/Sub Integration Test ===\n";
  std::cout << "(Single process, links demo_buffer_backend directly)\n\n";

  auto publisher = std::make_shared<DemoImagePublisher>();
  auto subscriber = std::make_shared<DemoImageSubscriber>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher);
  executor.add_node(subscriber);

  std::cout << "Waiting for discovery...\n" << std::flush;
  std::this_thread::sleep_for(1s);
  executor.spin_some(100ms);
  std::cout << "Discovery complete, starting message exchange...\n" << std::flush;

  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < 3s) {
    executor.spin_some(100ms);
    if (subscriber->get_received_count() >= 3) {
      std::cout << "\nReceived " << subscriber->get_received_count()
                << " messages successfully!\n" << std::flush;
      break;
    }
  }

  bool success = subscriber->get_received_count() >= 3 && subscriber->is_last_correct();

  std::cout << "\nTest Results:\n";
  std::cout << "  Published: " << publisher->get_publish_count() << " images\n";
  std::cout << "  Received:  " << subscriber->get_received_count() << " images\n";
  std::cout << "  Valid: " << (subscriber->is_last_correct() ? "YES" : "NO") << "\n";

  std::cout << "\nCalling rclcpp::shutdown()...\n" << std::flush;
  rclcpp::shutdown();
  std::cout << "rclcpp::shutdown() completed\n" << std::flush;

  // Explicitly destroy nodes before main returns
  publisher.reset();
  subscriber.reset();
  std::cout << "Nodes destroyed\n" << std::flush;

  if (success) {
    std::cout << "\n=== ALL TESTS PASSED ===\n" << std::flush;
    return 0;
  } else {
    std::cout << "\n=== TESTS FAILED ===\n" << std::flush;
    return 1;
  }
}
