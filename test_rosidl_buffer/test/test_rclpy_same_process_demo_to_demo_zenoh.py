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
# Launch test: rclpy same-process pub/sub, demo backend (Zenoh)
#
# Runs rclpy_image_pubsub with --backend-mode demo.  Both publisher and
# subscriber nodes live in a single Python process sharing the same Zenoh
# session.  Validates that the Buffer/DemoBuffer path works end-to-end
# in the intra-process scenario.

import os
import unittest

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable, TimerAction)
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import pytest


@pytest.mark.launch_test
def generate_test_description():
    test_proc = Node(
        package='test_rosidl_buffer',
        executable='rclpy_image_pubsub',
        arguments=['--backend-mode', 'demo'],
        output='screen',
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
                        test_proc,
                        launch_testing.actions.ReadyToTest(),
                    ]),
                ],
            ),
        ),
    ])


class TestRclpySameProcessDemoToDemoZenoh(unittest.TestCase):

    def test_completes_successfully(self, proc_output):
        proc_output.assertWaitFor(
            'ALL TESTS PASSED',
            timeout=30.0,
            stream='stdout',
        )


@launch_testing.post_shutdown_test()
class TestRclpySameProcessDemoToDemoZenohShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2])
