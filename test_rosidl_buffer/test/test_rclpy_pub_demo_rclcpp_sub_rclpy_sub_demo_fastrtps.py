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
# Launch test: 1 rclpy publisher (demo) to 1 rclcpp subscriber +
# 1 rclpy subscriber (demo). (FastRTPS)
# Validates that a single Python publisher can deliver demo-backend-backed Buffer data
# to both a C++ and a Python subscriber simultaneously via the buffer-aware path.

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
    """Launch rclpy demo pub with rclcpp + rclpy demo subscribers."""
    publisher_node = Node(
        package='test_rosidl_buffer',
        executable='rclpy_image_publisher',
        name='rclpy_demo_publisher',
        output='screen',
        parameters=[{
            'backend_mode': 'demo',
            'topic_name': 'test_mixed_demo_image',
            'publish_rate_ms': 200,
            'max_publish_count': 50,
        }],
    )

    rclcpp_subscriber = Node(
        package='test_rosidl_buffer',
        executable='demo_backend_image_subscriber_node',
        name='rclcpp_demo_subscriber',
        output='screen',
        parameters=[{
            'topic_name': 'test_mixed_demo_image',
            'expected_backends': 'demo',
            'acceptable_buffer_backends': 'any',
            'count_topic_suffix': '_cpp',
        }],
    )

    rclpy_subscriber = Node(
        package='test_rosidl_buffer',
        executable='rclpy_image_subscriber',
        name='rclpy_demo_subscriber',
        output='screen',
        parameters=[{
            'topic_name': 'test_mixed_demo_image',
            'expected_backends': 'demo',
            'acceptable_buffer_backends': 'any',
            'count_topic_suffix': '_py',
        }],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        publisher_node,
        rclcpp_subscriber,
        rclpy_subscriber,
        launch_testing.actions.ReadyToTest(),
    ])


class TestRclpyPubDemoMixedSubFastRTPS(unittest.TestCase):
    """Test rclpy demo pub to rclcpp + rclpy demo subscribers."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_rclpy_pub_demo_mixed_sub_fastrtps')
        self.publisher_count = 0
        self.subscriber_counts = {'_cpp': 0, '_py': 0}
        self.validation_results = {'_cpp': True, '_py': True}

        self.pub_count_sub = self.node.create_subscription(
            UInt32, 'publisher_count', self._pub_count_cb, 10)

        self.sub_count_cpp = self.node.create_subscription(
            UInt32, 'subscriber_count_cpp',
            lambda msg: self._sub_count_cb(msg, '_cpp'), 10)
        self.sub_count_py = self.node.create_subscription(
            UInt32, 'subscriber_count_py',
            lambda msg: self._sub_count_cb(msg, '_py'), 10)

        self.val_cpp = self.node.create_subscription(
            Bool, 'validation_result_cpp',
            lambda msg: self._validation_cb(msg, '_cpp'), 10)
        self.val_py = self.node.create_subscription(
            Bool, 'validation_result_py',
            lambda msg: self._validation_cb(msg, '_py'), 10)

    def tearDown(self):
        self.node.destroy_node()

    def _pub_count_cb(self, msg):
        self.publisher_count = msg.data

    def _sub_count_cb(self, msg, suffix):
        self.subscriber_counts[suffix] = msg.data

    def _validation_cb(self, msg, suffix):
        self.validation_results[suffix] = msg.data

    def _get_min_sub_count(self):
        return min(self.subscriber_counts.values())

    def _all_validations_passed(self):
        return all(self.validation_results.values())

    def _spin_until(self, target_count=1, timeout_sec=15.0):
        start = time.time()
        while self._get_min_sub_count() < target_count and time.time() - start < timeout_sec:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self._get_min_sub_count() >= target_count

    def test_messages_delivered(self):
        """Test rclpy demo pub delivers to both rclcpp and rclpy subscribers."""
        success = self._spin_until(target_count=1, timeout_sec=15.0)

        min_count = self._get_min_sub_count()
        self.assertTrue(
            success,
            f'Failed to receive at least 1 message on all subscribers. '
            f'Counts: {self.subscriber_counts}')
        self.assertGreaterEqual(
            min_count, 1,
            f'All subscribers should have received at least 1 message. '
            f'Counts: {self.subscriber_counts}')
        self.assertTrue(
            self._all_validations_passed(),
            f'Validation failed: {self.validation_results}')


@launch_testing.post_shutdown_test()
class TestRclpyPubDemoMixedSubFastRTPSShutdown(unittest.TestCase):
    """Test shutdown behavior."""

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
