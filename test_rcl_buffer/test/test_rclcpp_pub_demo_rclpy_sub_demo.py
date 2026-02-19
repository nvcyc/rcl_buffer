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
# Launch test: 1 rclcpp publisher (demo backend) to 1 rclpy subscriber (demo backend).
# Validates cross-language interop for vendor-backed Buffer data:
# - C++ publisher creates a Buffer with DemoBufferImpl backend
# - C++ typesupport serializes via serialize_buffer_with_endpoint (demo descriptor)
# - RMW transmits via Zenoh
# - C typesupport deserializes into a Buffer (is_rcl_buffer flag on the C struct)
# - convert_to_py wraps the Buffer* into a Python rcl_buffer.Buffer
# - Python subscriber receives the Buffer object with backend_type == 'demo'

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
    """Generate launch description for rclcpp demo pub to rclpy demo sub test."""
    publisher_node = Node(
        package='test_rcl_buffer',
        executable='demo_backend_image_publisher_node',
        name='rclcpp_demo_publisher',
        output='screen',
        parameters=[{
            'backend_mode': 'demo',
            'topic_name': 'test_cross_lang_demo_image',
            'publish_rate_ms': 200,
            'max_publish_count': 5,
        }],
    )

    subscriber_node = Node(
        package='test_rcl_buffer',
        executable='rclpy_image_subscriber',
        name='rclpy_demo_subscriber',
        output='screen',
        parameters=[{
            'topic_name': 'test_cross_lang_demo_image',
            'expected_backends': 'demo',
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
                        subscriber_node,
                        launch_testing.actions.ReadyToTest(),
                    ]),
                ],
            ),
        ),
    ])


class TestRclcppPubDemoRclpySubDemo(unittest.TestCase):
    """Test case for rclcpp demo publisher to rclpy demo subscriber."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_rclcpp_pub_demo_rclpy_sub_demo')
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

    def test_rclcpp_pub_demo_rclpy_sub_demo_messages_delivered(self):
        """Test rclcpp demo publisher to rclpy demo subscriber."""
        success = self._spin_until(target_count=1, timeout_sec=15.0)

        self.assertTrue(
            success,
            f'Failed to receive at least 1 message. '
            f'Received: {self.subscriber_count}')
        self.assertGreaterEqual(
            self.subscriber_count, 1,
            f'Subscriber should have received at least 1 message. '
            f'Received: {self.subscriber_count}')
        self.assertTrue(self.validation_passed, 'Image validation failed')


@launch_testing.post_shutdown_test()
class TestRclcppPubDemoRclpySubDemoShutdown(unittest.TestCase):
    """Test shutdown behavior."""

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
