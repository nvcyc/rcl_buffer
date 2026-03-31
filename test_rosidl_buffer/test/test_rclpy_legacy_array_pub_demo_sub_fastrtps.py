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
# Launch test: rclpy legacy array.array publisher to rclpy demo-aware subscriber. (FastRTPS)
#
# The publisher is a "legacy" node that uses ONLY array.array and has NO
# imports from rosidl_buffer or demo_buffer.  The subscriber is the standard
# buffer-aware rclpy_image_subscriber.
#
# This validates the reverse direction of backward compatibility:
# existing publisher code still works correctly when the subscriber
# side has been upgraded to handle both Buffer and array.array objects.
# The Python-to-C conversion must accept plain array.array data and
# the subscriber should receive it as cpu-backed (not demo).

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
    """Launch rclpy legacy publisher with demo-aware subscriber."""
    legacy_publisher_node = Node(
        package='test_rosidl_buffer',
        executable='rclpy_legacy_image_publisher',
        name='rclpy_legacy_publisher',
        output='screen',
        parameters=[{
            'topic_name': 'test_legacy_to_demo',
            'publish_rate_ms': 200,
            'max_publish_count': 50,
        }],
    )

    demo_subscriber_node = Node(
        package='test_rosidl_buffer',
        executable='rclpy_image_subscriber',
        name='rclpy_demo_subscriber',
        output='screen',
        parameters=[{
            'topic_name': 'test_legacy_to_demo',
            'expected_backends': 'cpu',
            'count_topic_suffix': '',
        }],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        legacy_publisher_node,
        demo_subscriber_node,
        launch_testing.actions.ReadyToTest(),
    ])


class TestRclpyLegacyArrayPubDemoSubFastRTPS(unittest.TestCase):
    """Test legacy array.array publisher to demo-aware subscriber."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_legacy_pub_demo_sub_fastrtps')
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

    def test_legacy_array_pub_to_demo_sub(self):
        """Demo-aware subscriber correctly receives array.array data from legacy pub."""
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
            'Validation failed — demo-aware subscriber could not handle '
            'plain array.array data from legacy publisher')


@launch_testing.post_shutdown_test()
class TestRclpyLegacyArrayPubDemoSubFastRTPSShutdown(unittest.TestCase):
    """Test shutdown behavior."""

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
