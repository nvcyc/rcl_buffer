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
# Launch test: 1 pub (demo) → 1 intra-process sub (demo) + 1 inter-process sub (demo) (FastRTPS)
# Both subscribers accept any buffer backend.  The intra-process subscriber may
# receive the original demo buffer; the inter-process subscriber receives
# serialized data (cpu fallback).

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
    intra_inter_node = Node(
        package='test_rosidl_buffer',
        executable='test_intra_inter_image_pubsub',
        name='intra_inter_pubsub',
        output='screen',
        parameters=[{
            'backend_mode': 'demo',
            'topic_name': 'test_image',
            'publish_rate_ms': 200,
            'max_publish_count': 50,
            'intra_expected_backends': 'demo,cpu',
            'intra_acceptable_buffer_backends': 'any',
        }],
    )

    inter_subscriber = Node(
        package='test_rosidl_buffer',
        executable='demo_backend_image_subscriber_node',
        name='inter_image_subscriber',
        output='screen',
        parameters=[{
            'topic_name': 'test_image',
            'expected_backends': 'demo,cpu',
            'acceptable_buffer_backends': 'any',
            'count_topic_suffix': '',
        }],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        intra_inter_node,
        inter_subscriber,
        launch_testing.actions.ReadyToTest(),
    ])


class TestIntraDemoInterDemoFastRTPS(unittest.TestCase):
    """1 demo pub → 1 demo intra-process sub + 1 demo inter-process sub (FastRTPS)."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_intra_demo_inter_demo_fastrtps')
        self.intra_count = 0
        self.inter_count = 0
        self.intra_validation = None
        self.inter_validation = None

        self.node.create_subscription(
            UInt32, 'intra_subscriber_count',
            lambda msg: setattr(self, 'intra_count', msg.data), 10)
        self.node.create_subscription(
            Bool, 'intra_validation_result',
            lambda msg: setattr(self, 'intra_validation', msg.data), 10)
        self.node.create_subscription(
            UInt32, 'subscriber_count',
            lambda msg: setattr(self, 'inter_count', msg.data), 10)
        self.node.create_subscription(
            Bool, 'validation_result',
            lambda msg: setattr(self, 'inter_validation', msg.data), 10)

    def tearDown(self):
        self.node.destroy_node()

    def _spin_until(self, timeout_sec=15.0):
        start = time.time()
        while (
            (self.intra_count < 1 or self.inter_count < 1
             or self.intra_validation is None or self.inter_validation is None)
            and time.time() - start < timeout_sec
        ):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.intra_count >= 1 and self.inter_count >= 1

    def test_intra_and_inter_receive(self):
        success = self._spin_until(timeout_sec=15.0)
        self.assertTrue(
            success,
            f'Intra received: {self.intra_count}, Inter received: {self.inter_count}')
        self.assertTrue(self.intra_validation, 'Intra-process validation failed')
        self.assertTrue(self.inter_validation, 'Inter-process validation failed')


@launch_testing.post_shutdown_test()
class TestIntraDemoInterDemoFastRTPSShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
