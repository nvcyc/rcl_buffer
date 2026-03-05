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

import array

from rcl_interfaces.srv import GetParameterTypes
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt32


class ParameterTypesClient(Node):
    """Service client validating uint8[] service responses."""

    def __init__(self):
        super().__init__('rclpy_parameter_types_client')
        self.declare_parameter('service_name', 'test_get_parameter_types')
        self.declare_parameter('max_requests', 3)
        self.declare_parameter('request_rate_ms', 250)

        self.service_name = self.get_parameter('service_name').value
        self.max_requests = self.get_parameter('max_requests').value
        request_rate_ms = self.get_parameter('request_rate_ms').value
        self.expected = bytes([1, 2, 3, 255, 42, 7])

        self.client = self.create_client(GetParameterTypes, self.service_name)
        self.request_count = 0
        self.validation_passed = True

        self.count_pub = self.create_publisher(UInt32, 'service_client_count', 10)
        self.validation_pub = self.create_publisher(Bool, 'service_validation_result', 10)
        self.timer = self.create_timer(request_rate_ms / 1000.0, self._timer_callback)

    def _publish_status(self):
        count_msg = UInt32()
        count_msg.data = self.request_count
        self.count_pub.publish(count_msg)

        val_msg = Bool()
        val_msg.data = self.validation_passed
        self.validation_pub.publish(val_msg)

    def _response_callback(self, future):
        try:
            result = future.result()
            data = result.types
            if isinstance(data, array.array):
                as_bytes = data.tobytes()
            elif isinstance(data, (bytes, bytearray)):
                as_bytes = bytes(data)
            else:
                as_bytes = data.to_bytes()

            if as_bytes != self.expected:
                self.validation_passed = False
                self.get_logger().error(
                    f'Response mismatch: expected {list(self.expected)}, got {list(as_bytes)}')
        except Exception as exc:
            self.validation_passed = False
            self.get_logger().error(f'Service call failed: {exc}')
        finally:
            self.request_count += 1
            self._publish_status()
            if self.request_count >= self.max_requests:
                self.get_logger().info('Completed service validation requests; shutting down')
                self.timer.cancel()
                rclpy.shutdown()

    def _timer_callback(self):
        if self.request_count >= self.max_requests:
            return
        if not self.client.wait_for_service(timeout_sec=0.0):
            self.get_logger().info('Waiting for service...')
            return
        req = GetParameterTypes.Request()
        req.names = ['foo', 'bar']
        future = self.client.call_async(req)
        future.add_done_callback(self._response_callback)


def main(args=None):
    rclpy.init(args=args)
    node = ParameterTypesClient()
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
