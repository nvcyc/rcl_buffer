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

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "perf_pubsub_benchmark/msg/perf_message.hpp"

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using PerfMsg = perf_pubsub_benchmark::msg::PerfMessage;

class PerfPublisher : public rclcpp::Node
{
public:
  PerfPublisher()
  : Node("perf_publisher"), seq_(0), done_(false)
  {
    declare_parameter<std::string>("topic_name", "perf_test");
    declare_parameter<int>("rate_hz", 5000);
    declare_parameter<double>("duration_sec", 10.0);
    declare_parameter<int>("msg_size", 256);
    declare_parameter<int>("pub_id", 0);
    declare_parameter<double>("warmup_sec", 2.0);
    declare_parameter<int>("qos_depth", 1);
    declare_parameter<bool>("reliable", false);

    topic_name_ = get_parameter("topic_name").as_string();
    rate_hz_ = get_parameter("rate_hz").as_int();
    duration_sec_ = get_parameter("duration_sec").as_double();
    msg_size_ = get_parameter("msg_size").as_int();
    pub_id_ = get_parameter("pub_id").as_int();
    warmup_sec_ = get_parameter("warmup_sec").as_double();

    int qos_depth = get_parameter("qos_depth").as_int();
    bool reliable = get_parameter("reliable").as_bool();

    rclcpp::QoS qos{rclcpp::KeepLast(qos_depth)};
    if (reliable) {
      qos.reliable();
    } else {
      qos.best_effort();
    }
    publisher_ = create_publisher<PerfMsg>(topic_name_, qos);

    payload_.assign(std::max(0, msg_size_), 0xAA);

    RCLCPP_INFO(
      get_logger(),
      "PerfPublisher: topic=%s rate=%dHz duration=%.1fs size=%d pub_id=%d warmup=%.1fs qos=%s",
      topic_name_.c_str(), rate_hz_, duration_sec_, msg_size_, pub_id_, warmup_sec_,
      reliable ? "reliable" : "best_effort");

    publish_thread_ = std::thread(&PerfPublisher::publish_loop, this);
  }

  ~PerfPublisher() override
  {
    done_ = true;
    if (publish_thread_.joinable()) {
      publish_thread_.join();
    }
  }

  bool is_done() const {return done_.load();}

private:
  void publish_loop()
  {
    // Warmup: send messages with timestamp_ns=0 to allow DDS discovery
    auto warmup_end = Clock::now() + std::chrono::duration<double>(warmup_sec_);
    while (rclcpp::ok() && !done_.load() && Clock::now() < warmup_end) {
      PerfMsg msg;
      msg.pub_id = pub_id_;
      msg.seq = 0;
      msg.timestamp_ns = 0;
      publisher_->publish(msg);
      std::this_thread::sleep_for(10ms);
    }

    if (!rclcpp::ok() || done_.load()) {
      done_ = true;
      return;
    }

    auto interval_ns = rate_hz_ > 0 ?
      std::chrono::nanoseconds(1000000000L / rate_hz_) :
      std::chrono::nanoseconds(0);

    auto start = Clock::now();
    auto end_time = start + std::chrono::duration<double>(duration_sec_);

    while (rclcpp::ok() && !done_.load() && Clock::now() < end_time) {
      auto now = Clock::now();
      auto ts_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()).count();

      PerfMsg msg;
      msg.pub_id = pub_id_;
      msg.seq = seq_;
      msg.timestamp_ns = ts_ns;
      msg.data = payload_;

      publisher_->publish(msg);
      seq_++;

      if (interval_ns.count() > 0) {
        auto target = start + interval_ns * static_cast<int64_t>(seq_);
        auto now2 = Clock::now();
        if (target > now2) {
          std::this_thread::sleep_for(target - now2);
        }
      }
    }

    auto actual_end = Clock::now();
    double actual_dur = std::chrono::duration<double>(actual_end - start).count();
    double rate = actual_dur > 0.0 ? static_cast<double>(seq_) / actual_dur : 0.0;

    fprintf(stderr,
      "[PERF_RESULT] role=publisher topic=%s pub_id=%d total_sent=%lu "
      "duration_s=%.3f msgs_per_sec=%.1f\n",
      topic_name_.c_str(), pub_id_, static_cast<unsigned long>(seq_),
      actual_dur, rate);
    fflush(stderr);

    std::this_thread::sleep_for(500ms);
    done_ = true;
  }

  std::string topic_name_;
  int rate_hz_;
  double duration_sec_;
  int msg_size_;
  int pub_id_;
  double warmup_sec_;
  std::vector<uint8_t> payload_;
  uint64_t seq_;
  std::atomic<bool> done_;
  rclcpp::Publisher<PerfMsg>::SharedPtr publisher_;
  std::thread publish_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PerfPublisher>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  while (rclcpp::ok() && !node->is_done()) {
    executor.spin_some(100ms);
  }

  rclcpp::shutdown();
  return 0;
}
