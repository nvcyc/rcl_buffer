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


class ParameterTypesService(Node):
    """Service server returning uint8[] using CPU or demo buffer inputs."""

    def __init__(self):
        super().__init__('rclpy_parameter_types_service')
        self.declare_parameter('service_name', 'test_get_parameter_types')
        self.declare_parameter('backend_mode', 'cpu')

        self.service_name = self.get_parameter('service_name').value
        self.backend_mode = self.get_parameter('backend_mode').value
        self.payload = bytes([1, 2, 3, 255, 42, 7])

        self.srv = self.create_service(
            GetParameterTypes, self.service_name, self._handle_request)
        self.get_logger().info(
            f'Service ready: {self.service_name} (backend_mode={self.backend_mode})')

    def _handle_request(self, request, response):
        del request
        if self.backend_mode == 'demo':
            from demo_buffer import DemoBuffer
            response.types = DemoBuffer(self.payload)
        else:
            response.types = array.array('B', self.payload)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ParameterTypesService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
