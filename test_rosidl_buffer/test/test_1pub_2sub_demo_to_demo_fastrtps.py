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
# Launch test: 1 publisher to 2 subscribers, Demo backend to Demo backend (FastRTPS)
# Tests buffer-aware per-subscriber DataWriter fan-out with demo backend.

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
    """Generate launch description for Demo-to-Demo pub/sub with 2 subs over FastRTPS."""
    publisher_node = Node(
        package='test_rosidl_buffer',
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

    subscriber_node_1 = Node(
        package='test_rosidl_buffer',
        executable='demo_backend_image_subscriber_node',
        name='demo_image_subscriber_1',
        output='screen',
        parameters=[{
            'topic_name': 'test_image',
            'expected_backends': 'demo',
            'acceptable_buffer_backends': 'any',
            'count_topic_suffix': '_1',
        }],
    )

    subscriber_node_2 = Node(
        package='test_rosidl_buffer',
        executable='demo_backend_image_subscriber_node',
        name='demo_image_subscriber_2',
        output='screen',
        parameters=[{
            'topic_name': 'test_image',
            'expected_backends': 'demo',
            'acceptable_buffer_backends': 'any',
            'count_topic_suffix': '_2',
        }],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        publisher_node,
        subscriber_node_1,
        subscriber_node_2,
        launch_testing.actions.ReadyToTest(),
    ])


class TestDemoToDemo2SubFastRTPS(unittest.TestCase):
    """Test case for Demo-to-Demo image pub/sub with 2 subscribers over FastRTPS."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_demo_to_demo_2sub_fastrtps')
        self.publisher_count = 0
        self.subscriber_counts = {'_1': 0, '_2': 0}
        self.validation_results = {'_1': None, '_2': None}

        self.pub_count_sub = self.node.create_subscription(
            UInt32, 'publisher_count', self._pub_count_cb, 10)

        self.sub_count_1 = self.node.create_subscription(
            UInt32, 'subscriber_count_1', lambda msg: self._sub_count_cb(msg, '_1'), 10)
        self.sub_count_2 = self.node.create_subscription(
            UInt32, 'subscriber_count_2', lambda msg: self._sub_count_cb(msg, '_2'), 10)

        self.val_1 = self.node.create_subscription(
            Bool, 'validation_result_1', lambda msg: self._validation_cb(msg, '_1'), 10)
        self.val_2 = self.node.create_subscription(
            Bool, 'validation_result_2', lambda msg: self._validation_cb(msg, '_2'), 10)

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

    def _any_validation_pending(self):
        return any(v is None for v in self.validation_results.values())

    def _spin_until(self, target_count=1, timeout_sec=15.0):
        start = time.time()
        while (
            (self._get_min_sub_count() < target_count
             or self._any_validation_pending())
            and time.time() - start < timeout_sec
        ):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self._get_min_sub_count() >= target_count

    def test_demo_to_demo_2sub_messages_delivered(self):
        """Test Demo backend publisher to 2 Demo backend subscribers over FastRTPS."""
        success = self._spin_until(target_count=1, timeout_sec=15.0)

        min_count = self._get_min_sub_count()
        self.assertTrue(
            success,
            f'Failed to receive at least 1 message on all subscribers. '
            f'Min received: {min_count}')
        self.assertGreaterEqual(
            min_count, 1,
            f'All subscribers should have received at least 1 message. '
            f'Counts: {self.subscriber_counts}')
        self.assertTrue(
            self._all_validations_passed(),
            f'Image validation failed on one or more subscribers: '
            f'{self.validation_results}')


@launch_testing.post_shutdown_test()
class TestDemoToDemo2SubFastRTPSShutdown(unittest.TestCase):
    """Test shutdown behavior for Demo to Demo with 2 subscribers over FastRTPS."""

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
