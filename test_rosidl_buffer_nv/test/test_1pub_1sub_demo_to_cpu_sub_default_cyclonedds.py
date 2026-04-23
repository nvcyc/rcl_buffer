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
# Launch test: demo pub to cpu sub without setting acceptable_buffer_backends
# Verifies that the default (not set) causes CPU fallback when publisher uses demo backend.

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
    """Demo pub to sub without setting acceptable_buffer_backends (default to CPU fallback)."""
    publisher_node = Node(
        package='test_rosidl_buffer_nv',
        executable='demo_backend_image_publisher_node',
        name='demo_image_publisher',
        output='screen',
        parameters=[{
            'backend_mode': 'demo',
            'topic_name': 'test_image',
            'publish_rate_ms': 200,
            'max_publish_count': 50,
        }],
    )

    # No acceptable_buffer_backends parameter — uses library default ("cpu")
    subscriber_node = Node(
        package='test_rosidl_buffer_nv',
        executable='demo_backend_image_subscriber_node',
        name='cpu_image_subscriber',
        output='screen',
        parameters=[{
            'topic_name': 'test_image',
            'expected_backends': 'cpu',
            'count_topic_suffix': '',
        }],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        publisher_node,
        subscriber_node,
        launch_testing.actions.ReadyToTest(),
    ])


class TestDemoCpuSubDefault(unittest.TestCase):
    """Demo to CPU with default acceptable_buffer_backends (not set to CPU fallback)."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_demo_cpu_sub_default')
        self.subscriber_count = 0
        self.validation_passed = None

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

    def _spin_until(self, target_count=20, timeout_sec=15.0):
        start = time.time()
        while (
            (self.subscriber_count < target_count
             or self.validation_passed is None)
            and time.time() - start < timeout_sec
        ):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.subscriber_count >= target_count

    def test_demo_to_cpu_sub_default(self):
        """Demo pub to sub with default option should fall back to CPU data."""
        success = self._spin_until(target_count=20, timeout_sec=15.0)
        self.assertTrue(success, f'Received: {self.subscriber_count}')
        self.assertTrue(self.validation_passed, 'Image validation failed')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
