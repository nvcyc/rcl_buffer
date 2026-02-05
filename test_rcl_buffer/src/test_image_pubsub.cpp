// Copyright 2024 NVIDIA Corporation
//
// Integration test to verify Buffer-based Image messages work in pub/sub

#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class ImagePublisher : public rclcpp::Node
{
public:
  ImagePublisher()
  : Node("image_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("test_image", 10);
    timer_ = this->create_wall_timer(
      100ms, std::bind(&ImagePublisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Image publisher started");
  }

  size_t get_publish_count() const {return count_;}

private:
  void timer_callback()
  {
    auto msg = sensor_msgs::msg::Image();

    // Create test image: 64x64 RGB
    msg.header.stamp = this->now();
    msg.header.frame_id = "test_frame";
    msg.height = 64;
    msg.width = 64;
    msg.encoding = "rgb8";
    msg.step = 64 * 3;
    msg.is_bigendian = 0;

    // Fill with pattern based on count
    msg.data.resize(64 * 64 * 3);
    for (size_t i = 0; i < msg.data.size(); ++i) {
      msg.data[i] = static_cast<uint8_t>((count_ + i) % 256);
    }

    publisher_->publish(msg);
    count_++;

    RCLCPP_INFO(this->get_logger(), "Published image #%zu", count_);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber()
  : Node("image_subscriber"), received_count_(0), last_correct_(true)
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "test_image", 10,
      std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Image subscriber started");
  }

  size_t get_received_count() const {return received_count_;}
  bool is_last_correct() const {return last_correct_;}

  const sensor_msgs::msg::Image::SharedPtr get_last_message() const
  {
    return last_msg_;
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "=== ENTERED image_callback ===");
    received_count_++;
    last_msg_ = msg;
    RCLCPP_INFO(this->get_logger(), "Stored message pointer");

    // Verify message contents
    last_correct_ = true;

    if (msg->width != 64 || msg->height != 64) {
      RCLCPP_ERROR(this->get_logger(), "Wrong dimensions: %ux%u",
                   msg->width, msg->height);
      last_correct_ = false;
      return;
    }

    if (msg->encoding != "rgb8") {
      RCLCPP_ERROR(this->get_logger(), "Wrong encoding: %s",
                   msg->encoding.c_str());
      last_correct_ = false;
      return;
    }

    if (msg->data.size() != 64 * 64 * 3) {
      RCLCPP_ERROR(this->get_logger(), "Wrong data size: %zu",
                   msg->data.size());
      last_correct_ = false;
      return;
    }

    // Test 1: Access via operator[]
    uint8_t first_pixel = msg->data[0];

    // Test 2: Verify Buffer behaves like vector (implicit conversion)
    const std::vector<uint8_t> & vec_ref = msg->data;
    if (vec_ref[0] != first_pixel) {
      RCLCPP_ERROR(this->get_logger(),
                   "Implicit conversion failed: %u != %u",
                   vec_ref[0], first_pixel);
      last_correct_ = false;
      return;
    }

    // Test 3: Iterator access
    auto it = msg->data.begin();
    if (*it != first_pixel) {
      RCLCPP_ERROR(this->get_logger(), "Iterator access failed");
      last_correct_ = false;
      return;
    }

    // Test 4: Backend type should be "cpu"
    if (msg->data.get_backend_type() != "cpu") {
      RCLCPP_ERROR(this->get_logger(),
                   "Wrong backend type: %s",
                   msg->data.get_backend_type().c_str());
      last_correct_ = false;
      return;
    }

    RCLCPP_INFO(this->get_logger(),
                "Received valid image #%zu (backend: %s)",
                received_count_,
                msg->data.get_backend_type().c_str());
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  sensor_msgs::msg::Image::SharedPtr last_msg_;
  size_t received_count_;
  bool last_correct_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::cout << "\n=== Starting Image Pub/Sub Integration Test ===\n";

  // Create nodes
  auto publisher = std::make_shared<ImagePublisher>();
  auto subscriber = std::make_shared<ImageSubscriber>();

  // Create executor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher);
  executor.add_node(subscriber);

  // Allow time for discovery (important for buffer-aware messages)
  std::cout << "Waiting for discovery...\n" << std::flush;
  std::this_thread::sleep_for(1s);
  std::cout << "Spinning for discovery events...\n" << std::flush;
  executor.spin_some(100ms);  // Process discovery events
  std::cout << "Discovery complete, starting message exchange...\n" << std::flush;

  // Spin for a few seconds to allow message exchange
  auto start = std::chrono::steady_clock::now();
  auto timeout = 3s;  // Reduced from 10s to fail faster

  int spin_count = 0;
  while (std::chrono::steady_clock::now() - start < timeout) {
    // std::cout << "Spin iteration " << ++spin_count << ", received: "
    //           << subscriber->get_received_count() << "\n" << std::flush;
    ++spin_count;
    executor.spin_some(100ms);

    // Check if we've received at least 5 messages
    if (subscriber->get_received_count() >= 5) {
      std::cout << "\nReceived " << subscriber->get_received_count()
                << " messages successfully!\n" << std::flush;
      break;
    }
  }

  std::cout << "Exited spin loop\n" << std::flush;

  // Verify results
  bool success = true;

  if (subscriber->get_received_count() == 0) {
    std::cerr << "\nERROR: No messages received!\n";
    success = false;
  } else {
    std::cout << "\nTest Results:\n";
    std::cout << "  Published: " << publisher->get_publish_count() << " images\n";
    std::cout << "  Received:  " << subscriber->get_received_count() << " images\n";
    std::cout << "  Last message valid: "
              << (subscriber->is_last_correct() ? "YES" : "NO") << "\n";

    if (!subscriber->is_last_correct()) {
      std::cerr << "ERROR: Last message validation failed!\n";
      success = false;
    }

    // Additional validation of last message
    if (subscriber->get_last_message()) {
      auto last_msg = subscriber->get_last_message();

      // Test to_vector() escape hatch
      std::vector<uint8_t> copied = last_msg->data.to_vector();
      if (copied.size() == last_msg->data.size() &&
        copied[0] == last_msg->data[0])
      {
        std::cout << "  to_vector() escape hatch: OK\n";
      } else {
        std::cerr << "ERROR: to_vector() failed!\n";
        success = false;
      }

      // Verify Buffer properties
      std::cout << "\nBuffer Properties:\n";
      std::cout << "  Size: " << last_msg->data.size() << " bytes\n";
      std::cout << "  Backend: " << last_msg->data.get_backend_type() << "\n";
      std::cout << "  Empty: " << (last_msg->data.empty() ? "yes" : "no") << "\n";
    }
  }

  std::cout << "Calling rclcpp::shutdown()...\n" << std::flush;
  rclcpp::shutdown();
  std::cout << "rclcpp::shutdown() completed\n" << std::flush;

  if (success) {
    std::cout << "\n=== ALL TESTS PASSED ===\n" << std::flush;
    std::cout << "Image messages with Buffer<T> work correctly in pub/sub!\n\n" << std::flush;
    // Use _Exit to avoid hanging in destructors
    std::_Exit(0);
  } else {
    std::cout << "\n=== TESTS FAILED ===\n\n" << std::flush;
    std::_Exit(1);
  }
}
