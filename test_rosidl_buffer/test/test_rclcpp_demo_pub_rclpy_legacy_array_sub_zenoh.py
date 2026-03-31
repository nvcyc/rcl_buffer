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
#
# Launch test: rclcpp demo publisher to rclpy legacy array.array subscriber.
#
# The C++ publisher creates DemoBuffer-backed sensor_msgs/Image data.
# The Python subscriber is a "legacy" node that uses ONLY array.array APIs
# and has NO imports from rosidl_buffer.
#
# This validates the cross-language backward-compatibility path:
# C++ demo pub -> RMW serialize -> RMW deserialize -> C-to-Python conversion
# wraps in Buffer -> Python subscriber uses array.array APIs on the Buffer.

import os
import time
import unittest

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable, TimerAction)
from launch.event_handlers import OnProcessStart
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
    """Launch rclcpp demo publisher with rclpy legacy array.array subscriber."""
    publisher_node = Node(
        package='test_rosidl_buffer',
        executable='demo_backend_image_publisher_node',
        name='rclcpp_demo_publisher',
        output='screen',
        parameters=[{
            'backend_mode': 'demo',
            'topic_name': 'test_cpp_demo_to_py_legacy',
            'publish_rate_ms': 200,
            'max_publish_count': 5,
        }],
    )

    legacy_subscriber_node = Node(
        package='test_rosidl_buffer',
        executable='rclpy_legacy_image_subscriber',
        name='rclpy_legacy_subscriber',
        output='screen',
        parameters=[{
            'topic_name': 'test_cpp_demo_to_py_legacy',
            'count_topic_suffix': '',
        }],
    )

    rmw_zenohd = os.path.join(
        get_package_prefix('rmw_zenoh_cpp'), 'lib', 'rmw_zenoh_cpp', 'rmw_zenohd')
    zenoh_router = ExecuteProcess(
        cmd=[rmw_zenohd],
        name='zenoh_router',
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_zenoh_cpp'),
        zenoh_router,
        RegisterEventHandler(
            OnProcessStart(
                target_action=zenoh_router,
                on_start=[
                    TimerAction(period=1.0, actions=[
                        publisher_node,
                        legacy_subscriber_node,
                        launch_testing.actions.ReadyToTest(),
                    ]),
                ],
            ),
        ),
    ])


class TestRclcppDemoPubLegacyArraySub(unittest.TestCase):
    """Test C++ demo publisher to Python legacy array.array subscriber."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_cpp_demo_pub_legacy_sub')
        self.subscriber_count = 0
        self.validation_passed = True

        self.sub_count_sub = self.node.create_subscription(
            UInt32, 'subscriber_count', self._sub_count_cb, 10)
        self.validation_sub = self.node.create_subscription(
            Bool, 'validation_result', self._validation_cb, 10)

    def tearDown(self):
        self.node.destroy_node()

    def _sub_count_cb(self, msg):
        self.subscriber_count = msg.data

    def _validation_cb(self, msg):
        self.validation_passed = msg.data

    def _spin_until(self, target_count=1, timeout_sec=15.0):
        start = time.time()
        while self.subscriber_count < target_count and time.time() - start < timeout_sec:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.subscriber_count >= target_count

    def test_cpp_demo_buffer_received_as_array(self):
        """Legacy Python subscriber using array.array APIs receives C++ demo Buffer."""
        success = self._spin_until(target_count=1, timeout_sec=15.0)

        self.assertTrue(
            success,
            f'Failed to receive at least 1 message. '
            f'Received: {self.subscriber_count}')
        self.assertGreaterEqual(
            self.subscriber_count, 1,
            f'Subscriber should have received at least 1 message. '
            f'Received: {self.subscriber_count}')
        self.assertTrue(
            self.validation_passed,
            'Legacy array.array API validation failed — Buffer is not '
            'backward-compatible with array.array (cross-language path)')


@launch_testing.post_shutdown_test()
class TestRclcppDemoPubLegacyArraySubShutdown(unittest.TestCase):
    """Test shutdown behavior."""

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
