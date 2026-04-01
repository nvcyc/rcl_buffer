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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <climits>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "perf_pubsub_benchmark/msg/perf_message.hpp"

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using PerfMsg = perf_pubsub_benchmark::msg::PerfMessage;

class PerfSubscriber : public rclcpp::Node
{
public:
  PerfSubscriber()
  : Node("perf_subscriber"),
    received_(0),
    first_seq_(UINT64_MAX),
    last_seq_(0),
    latency_min_ns_(INT64_MAX),
    latency_max_ns_(0),
    latency_sum_ns_(0),
    first_latency_ns_(-1),
    started_(false),
    summary_printed_(false),
    done_(false)
  {
    declare_parameter<std::string>("topic_name", "perf_test");
    declare_parameter<int>("sub_id", 0);
    declare_parameter<double>("timeout_sec", 3.0);
    declare_parameter<double>("max_duration_sec", 30.0);
    declare_parameter<int>("qos_depth", 1);
    declare_parameter<bool>("reliable", false);

    topic_name_ = get_parameter("topic_name").as_string();
    sub_id_ = get_parameter("sub_id").as_int();
    timeout_sec_ = get_parameter("timeout_sec").as_double();
    max_duration_sec_ = get_parameter("max_duration_sec").as_double();

    int qos_depth = get_parameter("qos_depth").as_int();
    bool reliable = get_parameter("reliable").as_bool();

    rclcpp::QoS qos{rclcpp::KeepLast(qos_depth)};
    if (reliable) {
      qos.reliable();
    } else {
      qos.best_effort();
    }

    subscription_ = create_subscription<PerfMsg>(
      topic_name_, qos,
      std::bind(&PerfSubscriber::msg_callback, this, std::placeholders::_1));

    check_timer_ = create_wall_timer(
      500ms, std::bind(&PerfSubscriber::check_timeout, this));

    node_start_ = Clock::now();

    RCLCPP_INFO(
      get_logger(),
      "PerfSubscriber: topic=%s sub_id=%d timeout=%.1fs max_duration=%.1fs qos=%s",
      topic_name_.c_str(), sub_id_, timeout_sec_, max_duration_sec_,
      reliable ? "reliable" : "best_effort");
  }

  ~PerfSubscriber() override
  {
    if (started_ && !summary_printed_) {
      print_summary();
    }
  }

  bool is_done() const {return done_.load();}

private:
  void msg_callback(const PerfMsg::SharedPtr msg)
  {
    if (msg->timestamp_ns == 0) {
      return;  // warmup message
    }

    auto now = Clock::now();
    if (!started_) {
      started_ = true;
      measure_start_ = now;
    }
    last_msg_time_ = now;

    auto recv_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      now.time_since_epoch()).count();
    int64_t latency_ns = recv_ns - msg->timestamp_ns;
    if (latency_ns >= 0) {
      latencies_ns_.push_back(latency_ns);
      if (first_latency_ns_ < 0) {
        first_latency_ns_ = latency_ns;
      }
      latency_min_ns_ = std::min(latency_min_ns_, latency_ns);
      latency_max_ns_ = std::max(latency_max_ns_, latency_ns);
      latency_sum_ns_ += latency_ns;
    }

    uint64_t seq = msg->seq;
    if (seq < first_seq_) {
      first_seq_ = seq;
    }
    if (seq > last_seq_) {
      last_seq_ = seq;
    }
    received_++;
  }

  void check_timeout()
  {
    auto now = Clock::now();

    double since_start = std::chrono::duration<double>(now - node_start_).count();
    if (since_start > max_duration_sec_) {
      print_summary();
      done_ = true;
      return;
    }

    if (started_) {
      double since_last = std::chrono::duration<double>(now - last_msg_time_).count();
      if (since_last > timeout_sec_) {
        print_summary();
        done_ = true;
        return;
      }
    }
  }

  void print_summary()
  {
    if (summary_printed_) {
      return;
    }
    summary_printed_ = true;

    if (!started_ || received_ == 0) {
      fprintf(stderr,
        "[PERF_RESULT] role=subscriber topic=%s sub_id=%d total_received=0 "
        "total_expected=0 dropped=0 drop_rate_pct=0.00 "
        "duration_s=0.000 msgs_per_sec=0.0 "
        "latency_min_us=0.0 latency_mean_us=0.0 latency_median_us=0.0 "
        "latency_max_us=0.0 first_latency_us=0.0 max_is_first=false\n",
        topic_name_.c_str(), sub_id_);
      fflush(stderr);
      return;
    }

    uint64_t expected = last_seq_ - first_seq_ + 1;
    uint64_t dropped = expected > received_ ? expected - received_ : 0;
    double drop_rate = expected > 0 ? 100.0 * static_cast<double>(dropped) / expected : 0.0;

    double dur = std::chrono::duration<double>(last_msg_time_ - measure_start_).count();
    double rate = dur > 0.0 ? static_cast<double>(received_) / dur : 0.0;

    double lat_min_us = received_ > 0 ? latency_min_ns_ / 1000.0 : 0.0;
    double lat_max_us = received_ > 0 ? latency_max_ns_ / 1000.0 : 0.0;
    double lat_mean_us = received_ > 0 ?
      (static_cast<double>(latency_sum_ns_) / received_) / 1000.0 : 0.0;

    double lat_median_us = 0.0;
    if (!latencies_ns_.empty()) {
      auto tmp = latencies_ns_;
      size_t n = tmp.size();
      std::nth_element(tmp.begin(), tmp.begin() + n / 2, tmp.end());
      if (n % 2 == 1) {
        lat_median_us = tmp[n / 2] / 1000.0;
      } else {
        auto mid = tmp[n / 2];
        std::nth_element(tmp.begin(), tmp.begin() + n / 2 - 1, tmp.end());
        lat_median_us = (tmp[n / 2 - 1] + mid) / 2000.0;
      }
    }

    double first_lat_us = first_latency_ns_ >= 0 ? first_latency_ns_ / 1000.0 : 0.0;
    bool max_is_first = first_latency_ns_ >= 0 && first_latency_ns_ == latency_max_ns_;

    fprintf(stderr,
      "[PERF_RESULT] role=subscriber topic=%s sub_id=%d total_received=%lu "
      "total_expected=%lu dropped=%lu drop_rate_pct=%.2f "
      "duration_s=%.3f msgs_per_sec=%.1f "
      "latency_min_us=%.1f latency_mean_us=%.1f latency_median_us=%.1f "
      "latency_max_us=%.1f first_latency_us=%.1f max_is_first=%s\n",
      topic_name_.c_str(), sub_id_,
      static_cast<unsigned long>(received_),
      static_cast<unsigned long>(expected),
      static_cast<unsigned long>(dropped),
      drop_rate, dur, rate,
      lat_min_us, lat_mean_us, lat_median_us,
      lat_max_us, first_lat_us,
      max_is_first ? "true" : "false");
    fflush(stderr);
  }

  std::string topic_name_;
  int sub_id_;
  double timeout_sec_;
  double max_duration_sec_;

  uint64_t received_;
  uint64_t first_seq_;
  uint64_t last_seq_;
  int64_t latency_min_ns_;
  int64_t latency_max_ns_;
  int64_t latency_sum_ns_;
  int64_t first_latency_ns_;
  std::vector<int64_t> latencies_ns_;
  bool started_;
  bool summary_printed_;
  std::atomic<bool> done_;

  Clock::time_point node_start_;
  Clock::time_point measure_start_;
  Clock::time_point last_msg_time_;

  rclcpp::Subscription<PerfMsg>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr check_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PerfSubscriber>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  while (rclcpp::ok() && !node->is_done()) {
    executor.spin_some(100ms);
  }

  rclcpp::shutdown();
  return 0;
}
