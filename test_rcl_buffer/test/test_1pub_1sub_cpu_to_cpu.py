#!/usr/bin/env python3
# Copyright 2024 NVIDIA Corporation
#
# Launch test: 1 publisher to 1 subscriber, CPU backend to CPU backend

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
    """Generate launch description for CPU-to-CPU pub/sub test."""

    publisher_node = Node(
        package='test_rcl_buffer',
        executable='demo_backend_image_publisher_node',
        name='cpu_image_publisher',
        output='screen',
        parameters=[{
            'backend_mode': 'cpu',
            'topic_name': 'test_image',
            'publish_rate_ms': 200,
            'max_publish_count': 5,
        }],
    )

    subscriber_node = Node(
        package='test_rcl_buffer',
        executable='demo_backend_image_subscriber_node',
        name='cpu_image_subscriber',
        output='screen',
        parameters=[{
            'topic_name': 'test_image',
            'expected_backends': 'cpu',
            'count_topic_suffix': '',
        }],
    )

    return LaunchDescription([
        publisher_node,
        subscriber_node,
        launch_testing.actions.ReadyToTest(),
    ])


class TestCpuToCpu(unittest.TestCase):
    """Test case for CPU-to-CPU image pub/sub."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_cpu_to_cpu')
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

    def _spin_until(self, target_count=5, timeout_sec=15.0):
        start = time.time()
        while self.subscriber_count < target_count and time.time() - start < timeout_sec:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.subscriber_count >= target_count

    def test_cpu_to_cpu_messages_delivered(self):
        """Test CPU backend publisher to CPU backend subscriber."""
        success = self._spin_until(target_count=5, timeout_sec=15.0)

        self.assertTrue(success,
            f"Failed to receive 5 messages. Received: {self.subscriber_count}")
        self.assertGreaterEqual(self.publisher_count, 5,
            f"Publisher should have sent at least 5 messages. Sent: {self.publisher_count}")
        self.assertTrue(self.validation_passed,
            "Image validation failed")
        self.assertLessEqual(abs(self.publisher_count - self.subscriber_count), 3,
            f"Count mismatch: pub={self.publisher_count}, sub={self.subscriber_count}")


@launch_testing.post_shutdown_test()
class TestCpuToCpuShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
