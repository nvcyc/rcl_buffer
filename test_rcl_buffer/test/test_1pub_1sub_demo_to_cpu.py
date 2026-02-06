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
# Launch test: 1 publisher to 1 subscriber, Demo backend to CPU backend
# This tests the serialization fallback path when the subscriber doesn't have
# the demo backend registered.

import unittest
import time

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from std_msgs.msg import UInt32, Bool


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Generate launch description for Demo-to-CPU pub/sub test."""

    # Publisher using demo backend
    publisher_node = Node(
        package='test_rcl_buffer',
        executable='demo_backend_image_publisher_node',
        name='demo_image_publisher',
        output='screen',
        parameters=[{
            'backend_mode': 'demo',
            'topic_name': 'test_image',
            'publish_rate_ms': 200,
            'max_publish_count': 5,
        }],
    )

    # Subscriber expecting CPU backend (the message will be serialized/deserialized
    # and reconstructed as CPU buffer on subscriber side)
    subscriber_node = Node(
        package='test_rcl_buffer',
        executable='demo_backend_image_subscriber_node',
        name='cpu_image_subscriber',
        output='screen',
        parameters=[{
            'topic_name': 'test_image',
            # The subscriber receives via serialization, so data comes as cpu backend
            'expected_backends': 'cpu',
            'count_topic_suffix': '',
        }],
    )

    return LaunchDescription([
        publisher_node,
        subscriber_node,
        launch_testing.actions.ReadyToTest(),
    ])


class TestDemoToCpu(unittest.TestCase):
    """Test case for Demo-to-CPU image pub/sub (serialization fallback)."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_demo_to_cpu')
        self.publisher_count = 0
        self.subscriber_count = 0
        self.validation_passed = True

        self.pub_count_sub = self.node.create_subscription(
            UInt32, 'publisher_count', self._pub_count_cb, 10)
        self.sub_count_sub = self.node.create_subscription(
            UInt32, 'subscriber_count', self._sub_count_cb, 10)
        self.validation_sub = self.node.create_subscription(
            Bool, 'validation_result', self._validation_cb, 10)

    def tearDown(self):
        self.node.destroy_node()

    def _pub_count_cb(self, msg):
        self.publisher_count = msg.data

    def _sub_count_cb(self, msg):
        self.subscriber_count = msg.data

    def _validation_cb(self, msg):
        self.validation_passed = msg.data

    def _spin_until(self, target_count=1, timeout_sec=15.0):
        start = time.time()
        while self.subscriber_count < target_count and time.time() - start < timeout_sec:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.subscriber_count >= target_count

    def test_demo_to_cpu_messages_delivered(self):
        """Test Demo backend publisher to CPU backend subscriber with serialization."""
        success = self._spin_until(target_count=1, timeout_sec=15.0)

        self.assertTrue(success,
            f"Failed to receive at least 1 message. Received: {self.subscriber_count}")
        self.assertGreaterEqual(self.subscriber_count, 1,
            f"Subscriber should have received at least 1 message. Received: {self.subscriber_count}")
        self.assertTrue(self.validation_passed,
            "Image validation failed - serialization fallback may have issues")


@launch_testing.post_shutdown_test()
class TestDemoToCpuShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
