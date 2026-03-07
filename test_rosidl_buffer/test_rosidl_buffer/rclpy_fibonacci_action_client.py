#!/usr/bin/env python3
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

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, UInt32
from test_msgs.action import Fibonacci


class FibonacciActionClient(Node):
    """Action client that validates Fibonacci action end-to-end."""

    def __init__(self):
        super().__init__('rclpy_fibonacci_action_client')
        self.declare_parameter('action_name', 'test_fibonacci')
        self.declare_parameter('order', 8)

        action_name = self.get_parameter('action_name').value
        self.order = int(self.get_parameter('order').value)
        self.expected = self._expected_sequence(self.order)
        self.done = False
        self.validation_passed = True

        self.result_count_pub = self.create_publisher(UInt32, 'action_result_count', 10)
        self.validation_pub = self.create_publisher(Bool, 'action_validation_result', 10)

        self.client = ActionClient(self, Fibonacci, action_name)
        self.timer = self.create_timer(0.2, self._kickoff_once)

    def _expected_sequence(self, order):
        seq = [0, 1]
        for _ in range(2, max(2, order)):
            seq.append(seq[-1] + seq[-2])
        return seq[:order]

    def _publish_status(self):
        count_msg = UInt32()
        count_msg.data = 1 if self.done else 0
        self.result_count_pub.publish(count_msg)

        val_msg = Bool()
        val_msg.data = self.validation_passed
        self.validation_pub.publish(val_msg)

    def _kickoff_once(self):
        self.timer.cancel()
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.validation_passed = False
            self.done = True
            self._publish_status()
            self.get_logger().error('Action server not available')
            rclpy.shutdown()
            return
        goal = Fibonacci.Goal()
        goal.order = self.order
        send_future = self.client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.validation_passed = False
                self.done = True
                self._publish_status()
                rclpy.shutdown()
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._result_cb)
        except Exception as exc:
            self.validation_passed = False
            self.done = True
            self._publish_status()
            self.get_logger().error(f'Failed to send action goal: {exc}')
            rclpy.shutdown()

    def _result_cb(self, future):
        try:
            result = future.result().result
            if list(result.sequence) != self.expected:
                self.validation_passed = False
                self.get_logger().error(
                    f'Unexpected action result: got {list(result.sequence)}, '
                    f'expected {self.expected}')
        except Exception as exc:
            self.validation_passed = False
            self.get_logger().error(f'Failed to get action result: {exc}')
        finally:
            self.done = True
            self._publish_status()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'client'):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
