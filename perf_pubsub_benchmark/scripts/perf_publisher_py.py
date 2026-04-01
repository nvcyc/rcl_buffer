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

"""rclpy performance publisher – same wire protocol as the C++ perf_publisher."""

import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class PerfPublisherPy(Node):

    def __init__(self):
        super().__init__("perf_publisher_py")

        self.declare_parameter("topic_name", "perf_test")
        self.declare_parameter("rate_hz", 5000)
        self.declare_parameter("duration_sec", 10.0)
        self.declare_parameter("msg_size", 256)
        self.declare_parameter("pub_id", 0)
        self.declare_parameter("warmup_sec", 2.0)
        self.declare_parameter("qos_depth", 1)
        self.declare_parameter("reliable", False)

        self._topic_name = self.get_parameter("topic_name").value
        self._rate_hz = self.get_parameter("rate_hz").value
        self._duration_sec = self.get_parameter("duration_sec").value
        self._msg_size = self.get_parameter("msg_size").value
        self._pub_id = self.get_parameter("pub_id").value
        self._warmup_sec = self.get_parameter("warmup_sec").value
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
        self._publisher = self.create_publisher(String, self._topic_name, qos)

        header_estimate = 40
        self._padding = "X" * max(0, self._msg_size - header_estimate)

        self.get_logger().info(
            f"PerfPublisherPy: topic={self._topic_name} rate={self._rate_hz}Hz "
            f"duration={self._duration_sec:.1f}s size={self._msg_size} "
            f"pub_id={self._pub_id} warmup={self._warmup_sec:.1f}s "
            f"qos={'reliable' if reliable else 'best_effort'}"
        )

        self._done = False
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()

    @property
    def done(self):
        return self._done

    def _publish_loop(self):
        # Warmup phase
        warmup_end = time.monotonic() + self._warmup_sec
        msg = String()
        while rclpy.ok() and not self._done and time.monotonic() < warmup_end:
            msg.data = "warmup"
            self._publisher.publish(msg)
            time.sleep(0.01)

        if not rclpy.ok() or self._done:
            self._done = True
            return

        interval = 1.0 / self._rate_hz if self._rate_hz > 0 else 0.0
        seq = 0
        start = time.monotonic()
        end_time = start + self._duration_sec

        while rclpy.ok() and not self._done and time.monotonic() < end_time:
            ts_ns = time.monotonic_ns()
            msg = String()
            msg.data = f"{self._pub_id}:{seq}:{ts_ns}:{self._padding}"
            self._publisher.publish(msg)
            seq += 1

            if interval > 0:
                target = start + interval * seq
                now = time.monotonic()
                if target > now:
                    time.sleep(target - now)

        actual_end = time.monotonic()
        actual_dur = actual_end - start
        rate = seq / actual_dur if actual_dur > 0 else 0.0

        print(
            f"[PERF_RESULT] role=publisher topic={self._topic_name} "
            f"pub_id={self._pub_id} total_sent={seq} "
            f"duration_s={actual_dur:.3f} msgs_per_sec={rate:.1f}",
            file=sys.stderr, flush=True,
        )

        time.sleep(0.5)
        self._done = True


def main(args=None):
    rclpy.init(args=args)
    node = PerfPublisherPy()
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
