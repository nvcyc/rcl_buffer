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
# Launch test: rclcpp same-process pub/sub, CPU backend (FastRTPS)
#
# Runs test_cpu_buffer_image_pubsub which creates both publisher and subscriber
# nodes in a single process.  Validates that Buffer<uint8_t> with the CPU
# backend works correctly when both endpoints share the same DDS participant.

import unittest

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import pytest


@pytest.mark.launch_test
def generate_test_description():
    test_proc = Node(
        package='test_rosidl_buffer',
        executable='test_cpu_buffer_image_pubsub',
        output='screen',
    )
    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        test_proc,
        launch_testing.actions.ReadyToTest(),
    ])


class TestSameProcessCpuToCpuFastRTPS(unittest.TestCase):

    def test_completes_successfully(self, proc_output):
        proc_output.assertWaitFor(
            'ALL TESTS PASSED',
            timeout=30.0,
            stream='stdout',
        )


@launch_testing.post_shutdown_test()
class TestSameProcessCpuToCpuFastRTPSShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2])
