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

// Single-process benchmark with intra-process communication enabled.
// Both publisher and subscriber nodes live in the same process and use
// rclcpp::NodeOptions().use_intra_process_comms(true) so that messages
// bypass the RMW layer entirely.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <climits>
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

// ---------------------------------------------------------------------------
// IntraPerfPublisher
// ---------------------------------------------------------------------------

class IntraPerfPublisher : public rclcpp::Node
{
public:
  IntraPerfPublisher(
    const std::string & node_name,
    const std::string & topic_name,
    int rate_hz, double duration_sec, int msg_size,
    int pub_id, double warmup_sec,
    int qos_depth, bool reliable,
    const rclcpp::NodeOptions & options)
  : Node(node_name, options), seq_(0), done_(false),
    topic_name_(topic_name), rate_hz_(rate_hz),
    duration_sec_(duration_sec), msg_size_(msg_size),
    pub_id_(pub_id), warmup_sec_(warmup_sec)
  {
    rclcpp::QoS qos{rclcpp::KeepLast(qos_depth)};
    if (reliable) {
      qos.reliable();
    } else {
      qos.best_effort();
    }
    publisher_ = create_publisher<PerfMsg>(topic_name, qos);
    payload_.assign(std::max(0, msg_size), 0xAA);

    RCLCPP_INFO(
      get_logger(),
      "IntraPerfPublisher: topic=%s rate=%dHz duration=%.1fs size=%d pub_id=%d warmup=%.1fs",
      topic_name.c_str(), rate_hz, duration_sec, msg_size, pub_id, warmup_sec);

    publish_thread_ = std::thread(&IntraPerfPublisher::publish_loop, this);
  }

  ~IntraPerfPublisher() override
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
    auto warmup_end = Clock::now() + std::chrono::duration<double>(warmup_sec_);
    while (rclcpp::ok() && !done_.load() && Clock::now() < warmup_end) {
      auto msg = std::make_unique<PerfMsg>();
      msg->pub_id = pub_id_;
      msg->seq = 0;
      msg->timestamp_ns = 0;
      publisher_->publish(std::move(msg));
      std::this_thread::sleep_for(10ms);
    }

    if (!rclcpp::ok() || done_.load()) {
      done_ = true;
      return;
    }

    auto interval_ns = rate_hz_ > 0
      ? std::chrono::nanoseconds(1000000000L / rate_hz_)
      : std::chrono::nanoseconds(0);

    auto start = Clock::now();
    auto end_time = start + std::chrono::duration<double>(duration_sec_);

    while (rclcpp::ok() && !done_.load() && Clock::now() < end_time) {
      auto now = Clock::now();
      auto ts_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()).count();

      auto msg = std::make_unique<PerfMsg>();
      msg->pub_id = pub_id_;
      msg->seq = seq_;
      msg->timestamp_ns = ts_ns;
      msg->data = payload_;

      publisher_->publish(std::move(msg));
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

// ---------------------------------------------------------------------------
// IntraPerfSubscriber
// ---------------------------------------------------------------------------

class IntraPerfSubscriber : public rclcpp::Node
{
public:
  IntraPerfSubscriber(
    const std::string & node_name,
    const std::string & topic_name,
    int sub_id, double timeout_sec, double max_duration_sec,
    int qos_depth, bool reliable,
    const rclcpp::NodeOptions & options)
  : Node(node_name, options),
    received_(0), first_seq_(UINT64_MAX), last_seq_(0),
    latency_min_ns_(INT64_MAX), latency_max_ns_(0), latency_sum_ns_(0),
    started_(false), summary_printed_(false), done_(false),
    topic_name_(topic_name), sub_id_(sub_id),
    timeout_sec_(timeout_sec), max_duration_sec_(max_duration_sec)
  {
    rclcpp::QoS qos{rclcpp::KeepLast(qos_depth)};
    if (reliable) {
      qos.reliable();
    } else {
      qos.best_effort();
    }

    subscription_ = create_subscription<PerfMsg>(
      topic_name, qos,
      std::bind(&IntraPerfSubscriber::msg_callback, this, std::placeholders::_1));

    check_timer_ = create_wall_timer(
      500ms, std::bind(&IntraPerfSubscriber::check_timeout, this));

    node_start_ = Clock::now();

    RCLCPP_INFO(
      get_logger(),
      "IntraPerfSubscriber: topic=%s sub_id=%d timeout=%.1fs max_duration=%.1fs",
      topic_name.c_str(), sub_id, timeout_sec, max_duration_sec);
  }

  ~IntraPerfSubscriber() override
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
      return;
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
        "latency_min_us=0.0 latency_mean_us=0.0 latency_max_us=0.0\n",
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
    double lat_mean_us = received_ > 0
      ? (static_cast<double>(latency_sum_ns_) / received_) / 1000.0 : 0.0;

    fprintf(stderr,
      "[PERF_RESULT] role=subscriber topic=%s sub_id=%d total_received=%lu "
      "total_expected=%lu dropped=%lu drop_rate_pct=%.2f "
      "duration_s=%.3f msgs_per_sec=%.1f "
      "latency_min_us=%.1f latency_mean_us=%.1f latency_max_us=%.1f\n",
      topic_name_.c_str(), sub_id_,
      static_cast<unsigned long>(received_),
      static_cast<unsigned long>(expected),
      static_cast<unsigned long>(dropped),
      drop_rate, dur, rate,
      lat_min_us, lat_mean_us, lat_max_us);
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
  bool started_;
  bool summary_printed_;
  std::atomic<bool> done_;

  Clock::time_point node_start_;
  Clock::time_point measure_start_;
  Clock::time_point last_msg_time_;

  rclcpp::Subscription<PerfMsg>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr check_timer_;
};

// ---------------------------------------------------------------------------
// Argument parsing helpers
// ---------------------------------------------------------------------------

static std::string get_arg(
  int argc, char ** argv, const std::string & name,
  const std::string & def)
{
  for (int i = 1; i + 1 < argc; i++) {
    if (std::string(argv[i]) == name) {
      return argv[i + 1];
    }
  }
  return def;
}

static int get_int(int argc, char ** argv, const std::string & name, int def)
{
  auto s = get_arg(argc, argv, name, "");
  return s.empty() ? def : std::stoi(s);
}

static double get_dbl(int argc, char ** argv, const std::string & name, double def)
{
  auto s = get_arg(argc, argv, name, "");
  return s.empty() ? def : std::stod(s);
}

static bool has_flag(int argc, char ** argv, const std::string & name)
{
  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == name) {
      return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  std::string topic = get_arg(argc, argv, "--topic", "perf_test");
  int num_pubs       = get_int(argc, argv, "--num-pubs", 1);
  int num_subs       = get_int(argc, argv, "--num-subs", 1);
  int rate_hz        = get_int(argc, argv, "--rate-hz", 5000);
  double duration    = get_dbl(argc, argv, "--duration", 10.0);
  int msg_size       = get_int(argc, argv, "--msg-size", 256);
  double warmup      = get_dbl(argc, argv, "--warmup", 2.0);
  int qos_depth      = get_int(argc, argv, "--qos-depth", 1);
  bool reliable      = has_flag(argc, argv, "--reliable");
  double timeout     = get_dbl(argc, argv, "--timeout", 3.0);
  double max_dur     = get_dbl(argc, argv, "--max-duration", 30.0);
  int pub_id_start   = get_int(argc, argv, "--pub-id-start", 0);
  int sub_id_start   = get_int(argc, argv, "--sub-id-start", 0);

  rclcpp::init(argc, argv);

  auto ipc_opts = rclcpp::NodeOptions().use_intra_process_comms(true);
  rclcpp::executors::MultiThreadedExecutor executor;

  std::vector<std::shared_ptr<IntraPerfSubscriber>> sub_nodes;
  for (int i = 0; i < num_subs; i++) {
    int sid = sub_id_start + i;
    auto node = std::make_shared<IntraPerfSubscriber>(
      "intra_sub_" + std::to_string(sid),
      topic, sid, timeout, max_dur,
      qos_depth, reliable, ipc_opts);
    executor.add_node(node);
    sub_nodes.push_back(node);
  }

  std::vector<std::shared_ptr<IntraPerfPublisher>> pub_nodes;
  for (int i = 0; i < num_pubs; i++) {
    int pid = pub_id_start + i;
    auto node = std::make_shared<IntraPerfPublisher>(
      "intra_pub_" + std::to_string(pid),
      topic, rate_hz, duration, msg_size,
      pid, warmup, qos_depth, reliable, ipc_opts);
    executor.add_node(node);
    pub_nodes.push_back(node);
  }

  std::thread spin_thread([&executor]() {
      executor.spin();
    });

  while (rclcpp::ok()) {
    std::this_thread::sleep_for(200ms);
    bool all_done = std::all_of(pub_nodes.begin(), pub_nodes.end(),
        [](auto & n) {return n->is_done();});
    if (all_done) {
      all_done = std::all_of(sub_nodes.begin(), sub_nodes.end(),
        [](auto & n) {return n->is_done();});
    }
    if (all_done) {
      break;
    }
  }

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
