#!/usr/bin/env python3
#
# Copyright 2026 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""rclpy performance subscriber – uses PerfMessage with uint8[] payload."""

import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from perf_pubsub_benchmark.msg import PerfMessage


class PerfSubscriberPy(Node):

    def __init__(self):
        super().__init__("perf_subscriber_py")

        self.declare_parameter("topic_name", "perf_test")
        self.declare_parameter("sub_id", 0)
        self.declare_parameter("timeout_sec", 3.0)
        self.declare_parameter("max_duration_sec", 30.0)
        self.declare_parameter("qos_depth", 1)
        self.declare_parameter("reliable", False)

        self._topic_name = self.get_parameter("topic_name").value
        self._sub_id = self.get_parameter("sub_id").value
        self._timeout_sec = self.get_parameter("timeout_sec").value
        self._max_duration_sec = self.get_parameter("max_duration_sec").value
        qos_depth = self.get_parameter("qos_depth").value
        reliable = self.get_parameter("reliable").value

        qos = QoSProfile(
            depth=qos_depth,
            history=HistoryPolicy.KEEP_LAST,
            reliability=(
                ReliabilityPolicy.RELIABLE if reliable
                else ReliabilityPolicy.BEST_EFFORT
            ),
        )

        self._subscription = self.create_subscription(
            PerfMessage, self._topic_name, self._msg_callback, qos,
        )

        self._check_timer = self.create_timer(0.5, self._check_timeout)

        self._received = 0
        self._first_seq = None
        self._last_seq = 0
        self._latency_min_ns = float("inf")
        self._latency_max_ns = 0
        self._latency_sum_ns = 0
        self._started = False
        self._summary_printed = False
        self._done = False

        self._node_start = time.monotonic()
        self._measure_start = 0.0
        self._last_msg_time = 0.0

        self.get_logger().info(
            f"PerfSubscriberPy: topic={self._topic_name} sub_id={self._sub_id} "
            f"timeout={self._timeout_sec:.1f}s max_duration={self._max_duration_sec:.1f}s "
            f"qos={'reliable' if reliable else 'best_effort'}"
        )

    @property
    def done(self):
        return self._done

    def _msg_callback(self, msg: PerfMessage):
        if msg.timestamp_ns == 0:
            return  # warmup message

        recv_ns = time.monotonic_ns()
        now = recv_ns / 1e9
        if not self._started:
            self._started = True
            self._measure_start = now
        self._last_msg_time = now

        latency_ns = recv_ns - msg.timestamp_ns
        if latency_ns >= 0:
            if latency_ns < self._latency_min_ns:
                self._latency_min_ns = latency_ns
            if latency_ns > self._latency_max_ns:
                self._latency_max_ns = latency_ns
            self._latency_sum_ns += latency_ns

        seq = msg.seq
        if self._first_seq is None or seq < self._first_seq:
            self._first_seq = seq
        if seq > self._last_seq:
            self._last_seq = seq
        self._received += 1

    def _check_timeout(self):
        now = time.monotonic()

        if now - self._node_start > self._max_duration_sec:
            self._print_summary()
            self._done = True
            return

        if self._started:
            if now - self._last_msg_time > self._timeout_sec:
                self._print_summary()
                self._done = True
                return

    def _print_summary(self):
        if self._summary_printed:
            return
        self._summary_printed = True

        if not self._started or self._received == 0 or self._first_seq is None:
            print(
                f"[PERF_RESULT] role=subscriber topic={self._topic_name} "
                f"sub_id={self._sub_id} total_received=0 total_expected=0 "
                f"dropped=0 drop_rate_pct=0.00 duration_s=0.000 msgs_per_sec=0.0 "
                f"latency_min_us=0.0 latency_mean_us=0.0 latency_max_us=0.0",
                file=sys.stderr, flush=True,
            )
            return

        expected = self._last_seq - self._first_seq + 1
        dropped = max(0, expected - self._received)
        drop_rate = 100.0 * dropped / expected if expected > 0 else 0.0

        dur = self._last_msg_time - self._measure_start
        rate = self._received / dur if dur > 0 else 0.0

        lat_min_us = self._latency_min_ns / 1000.0 if self._received > 0 else 0.0
        lat_max_us = self._latency_max_ns / 1000.0 if self._received > 0 else 0.0
        lat_mean_us = (
            (self._latency_sum_ns / self._received) / 1000.0
            if self._received > 0 else 0.0
        )

        print(
            f"[PERF_RESULT] role=subscriber topic={self._topic_name} "
            f"sub_id={self._sub_id} total_received={self._received} "
            f"total_expected={expected} dropped={dropped} "
            f"drop_rate_pct={drop_rate:.2f} "
            f"duration_s={dur:.3f} msgs_per_sec={rate:.1f} "
            f"latency_min_us={lat_min_us:.1f} "
            f"latency_mean_us={lat_mean_us:.1f} "
            f"latency_max_us={lat_max_us:.1f}",
            file=sys.stderr, flush=True,
        )

    def destroy_node(self):
        if self._started and not self._summary_printed:
            self._print_summary()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PerfSubscriberPy()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.done:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
