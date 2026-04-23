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
# Launch test: rclpy demo publisher to rclpy legacy array.array subscriber. (FastRTPS)
#
# The publisher creates DemoBuffer-backed sensor_msgs/Image data.
# The subscriber is a "legacy" node that uses ONLY array.array APIs
# (isinstance, tobytes, tolist, indexing, iteration, count, index,
# memoryview) and has NO imports from rosidl_buffer.
#
# This validates that existing rclpy user code does not break when
# the received uint8[] field is backed by an rosidl_buffer.Buffer.

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
    """Launch rclpy demo publisher with legacy array.array subscriber."""
    publisher_node = Node(
        package='test_rosidl_buffer_nv',
        executable='rclpy_image_publisher',
        name='rclpy_demo_publisher',
        output='screen',
        parameters=[{
            'backend_mode': 'demo',
            'topic_name': 'test_demo_to_legacy',
            'publish_rate_ms': 200,
            'max_publish_count': 50,
        }],
    )

    legacy_subscriber_node = Node(
        package='test_rosidl_buffer_nv',
        executable='rclpy_legacy_image_subscriber',
        name='rclpy_legacy_subscriber',
        output='screen',
        parameters=[{
            'topic_name': 'test_demo_to_legacy',
            'count_topic_suffix': '',
        }],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        publisher_node,
        legacy_subscriber_node,
        launch_testing.actions.ReadyToTest(),
    ])


class TestRclpyDemoPubLegacyArraySubFastRTPS(unittest.TestCase):
    """Test demo publisher to legacy array.array subscriber backward compat."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_demo_pub_legacy_sub_fastrtps')
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

    def _spin_until(self, target_count=20, timeout_sec=15.0):
        start = time.time()
        while self.subscriber_count < target_count and time.time() - start < timeout_sec:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.subscriber_count >= target_count

    def test_demo_buffer_received_as_array(self):
        """Legacy subscriber using only array.array APIs receives demo Buffer data."""
        success = self._spin_until(target_count=20, timeout_sec=15.0)

        self.assertTrue(
            success,
            f'Failed to receive at least 20 messages. '
            f'Received: {self.subscriber_count}')
        self.assertGreaterEqual(
            self.subscriber_count, 20,
            f'Subscriber should have received at least 20 messages. '
            f'Received: {self.subscriber_count}')
        self.assertTrue(
            self.validation_passed,
            'Legacy array.array API validation failed — Buffer is not '
            'backward-compatible with array.array')


@launch_testing.post_shutdown_test()
class TestRclpyDemoPubLegacyArraySubFastRTPSShutdown(unittest.TestCase):
    """Test shutdown behavior."""

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
