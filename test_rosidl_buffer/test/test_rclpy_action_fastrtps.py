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

import time
import unittest

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from std_msgs.msg import Bool, UInt32


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    action_server = Node(
        package='test_rosidl_buffer',
        executable='rclpy_fibonacci_action_server',
        output='screen',
        parameters=[{'action_name': 'test_fibonacci'}],
    )
    action_client = Node(
        package='test_rosidl_buffer',
        executable='rclpy_fibonacci_action_client',
        output='screen',
        parameters=[{'action_name': 'test_fibonacci', 'order': 8}],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        action_server,
        action_client,
        launch_testing.actions.ReadyToTest(),
    ])


class TestRclpyActionFastRTPS(unittest.TestCase):
    """Validate rclpy action goal/result flow over FastRTPS."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_rclpy_action_fastrtps')
        self.result_count = 0
        self.validation_passed = None
        self.count_sub = self.node.create_subscription(
            UInt32, 'action_result_count', self._count_cb, 10)
        self.validation_sub = self.node.create_subscription(
            Bool, 'action_validation_result', self._validation_cb, 10)

    def tearDown(self):
        self.node.destroy_node()

    def _count_cb(self, msg):
        self.result_count = msg.data

    def _validation_cb(self, msg):
        self.validation_passed = msg.data

    def test_action_smoke(self):
        start = time.time()
        while (
            (self.result_count < 1 or self.validation_passed is None) and
            time.time() - start < 25.0
        ):
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertGreaterEqual(self.result_count, 1, 'No action result observed')
        self.assertTrue(self.validation_passed, 'Action result validation failed')


@launch_testing.post_shutdown_test()
class TestRclpyActionFastRTPSShutdown(unittest.TestCase):
    """Validate clean shutdown for action test processes."""

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2])
