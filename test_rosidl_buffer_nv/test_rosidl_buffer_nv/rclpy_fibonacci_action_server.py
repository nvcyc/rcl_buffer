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
from rclpy.action import ActionServer
from rclpy.node import Node
from test_msgs.action import Fibonacci


class FibonacciActionServer(Node):
    """Simple Fibonacci action server for action route smoke testing."""

    def __init__(self):
        super().__init__('rclpy_fibonacci_action_server')
        self.declare_parameter('action_name', 'test_fibonacci')
        action_name = self.get_parameter('action_name').value
        self._server = ActionServer(
            self, Fibonacci, action_name, execute_callback=self.execute_callback)

    def execute_callback(self, goal_handle):
        order = max(1, int(goal_handle.request.order))
        sequence = [0, 1]
        for _ in range(2, order):
            sequence.append(sequence[-1] + sequence[-2])
        result = Fibonacci.Result()
        result.sequence = sequence[:order]
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
