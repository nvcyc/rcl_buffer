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
# Launch test: 1 pub (CPU) → 1 intra-process sub (CPU) + 1 inter-process sub (CPU) (Zenoh)
# Verifies that a CPU publisher delivers correctly to both a same-process
# subscriber and a separate-process subscriber simultaneously.

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
    intra_inter_node = Node(
        package='test_rosidl_buffer_nv',
        executable='test_intra_inter_image_pubsub',
        name='intra_inter_pubsub',
        output='screen',
        parameters=[{
            'backend_mode': 'cpu',
            'topic_name': 'test_image',
            'publish_rate_ms': 200,
            'max_publish_count': 50,
            'intra_expected_backends': 'cpu',
        }],
    )

    inter_subscriber = Node(
        package='test_rosidl_buffer_nv',
        executable='demo_backend_image_subscriber_node',
        name='inter_image_subscriber',
        output='screen',
        parameters=[{
            'topic_name': 'test_image',
            'expected_backends': 'cpu',
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
                        intra_inter_node,
                        inter_subscriber,
                        launch_testing.actions.ReadyToTest(),
                    ]),
                ],
            ),
        ),
    ])


class TestIntraCpuInterCpuZenoh(unittest.TestCase):
    """1 CPU pub → 1 CPU intra-process sub + 1 CPU inter-process sub (Zenoh)."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_intra_cpu_inter_cpu_zenoh')
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
            (self.intra_count < 20 or self.inter_count < 20
             or self.intra_validation is None or self.inter_validation is None)
            and time.time() - start < timeout_sec
        ):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.intra_count >= 20 and self.inter_count >= 20

    def test_intra_and_inter_receive(self):
        success = self._spin_until(timeout_sec=15.0)
        self.assertTrue(
            success,
            f'Intra received: {self.intra_count}, Inter received: {self.inter_count}')
        self.assertTrue(self.intra_validation, 'Intra-process validation failed')
        self.assertTrue(self.inter_validation, 'Inter-process validation failed')


@launch_testing.post_shutdown_test()
class TestIntraCpuInterCpuZenohShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
