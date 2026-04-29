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
# Launch test: MultiNested ("list of uint8[]") CPU-backend publisher ->
# CPU-backend subscriber over rmw_zenoh_cpp. Baseline case — all nested
# uint8_values fields are populated via the standard CPU path
# (resize + operator[]) and must arrive as "cpu"-backed Buffers.

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
from std_msgs.msg import Bool, String, UInt32


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    publisher_node = Node(
        package='test_rosidl_buffer_nv',
        executable='multi_nested_publisher_node',
        name='multi_nested_publisher',
        output='screen',
        parameters=[{
            'backend_mode': 'cpu',
            'topic_name': 'test_multi_nested',
            'publish_rate_ms': 200,
            'max_publish_count': 50,
        }],
    )

    subscriber_node = Node(
        package='test_rosidl_buffer_nv',
        executable='multi_nested_subscriber_node',
        name='multi_nested_subscriber',
        output='screen',
        parameters=[{
            'topic_name': 'test_multi_nested',
            'expected_backends': 'cpu',
            'acceptable_buffer_backends': 'any',
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


class TestMultiNestedCpuToCpuZenoh(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_multi_nested_cpu_to_cpu_zenoh')
        self.publisher_count = 0
        self.subscriber_count = 0
        self.validation_passed = None
        self.last_backend = None

        self.node.create_subscription(
            UInt32, 'publisher_count', self._pub_count_cb, 10)
        self.node.create_subscription(
            UInt32, 'subscriber_count', self._sub_count_cb, 10)
        self.node.create_subscription(
            Bool, 'validation_result', self._validation_cb, 10)
        self.node.create_subscription(
            String, 'backend_report', self._backend_cb, 10)

    def tearDown(self):
        self.node.destroy_node()

    def _pub_count_cb(self, msg):
        self.publisher_count = msg.data

    def _sub_count_cb(self, msg):
        self.subscriber_count = msg.data

    def _validation_cb(self, msg):
        self.validation_passed = msg.data

    def _backend_cb(self, msg):
        self.last_backend = msg.data

    def _spin_until(self, target_count=10, timeout_sec=15.0):
        start = time.time()
        while (
            (self.subscriber_count < target_count
             or self.validation_passed is None)
            and time.time() - start < timeout_sec
        ):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.subscriber_count >= target_count

    def test_cpu_to_cpu_nested_buffers(self):
        success = self._spin_until(target_count=10, timeout_sec=15.0)
        self.assertTrue(
            success,
            f'Failed to receive at least 10 messages. '
            f'Received: {self.subscriber_count}')
        self.assertIsNotNone(
            self.validation_passed,
            'No validation result received from subscriber')
        self.assertTrue(
            self.validation_passed,
            f'MultiNested content validation failed '
            f'(last nested backend: {self.last_backend})')
        self.assertEqual(
            self.last_backend, 'cpu',
            f'Expected nested backend "cpu", got "{self.last_backend}"')


@launch_testing.post_shutdown_test()
class TestMultiNestedCpuToCpuZenohShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
