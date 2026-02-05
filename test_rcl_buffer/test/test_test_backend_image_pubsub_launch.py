#!/usr/bin/env python3
# Copyright 2024 NVIDIA Corporation
#
# Launch test for Test Backend-based Image pub/sub

import os
import sys
import unittest
import time

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.node import Node as RclpyNode
from std_msgs.msg import UInt32, Bool
from sensor_msgs.msg import Image


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Generate launch description for test backend image pub/sub test."""

    # Publisher node
    publisher_node = Node(
        package='test_rcl_buffer',
        executable='test_backend_image_publisher_node',
        name='test_backend_image_publisher',
        output='screen',
        parameters=[],
    )

    # Subscriber node
    subscriber_node = Node(
        package='test_rcl_buffer',
        executable='test_backend_image_subscriber_node',
        name='test_backend_image_subscriber',
        output='screen',
        parameters=[],
    )

    return LaunchDescription([
        publisher_node,
        subscriber_node,
        # Tell launch to start the test after nodes are ready
        launch_testing.actions.ReadyToTest(),
    ])


class TestBackendImagePubSub(unittest.TestCase):
    """Test case for Test Backend-based Image pub/sub."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS context for the test."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS context after test."""
        rclpy.shutdown()

    def setUp(self):
        """Set up test fixtures."""
        self.node = rclpy.create_node('test_test_backend_image_pubsub')

        # Track received messages
        self.publisher_count = 0
        self.subscriber_count = 0
        self.validation_passed = True
        self.images_received = []

        # Create subscriptions to monitor test progress
        self.pub_count_sub = self.node.create_subscription(
            UInt32,
            'publisher_count',
            self._publisher_count_callback,
            10
        )

        self.sub_count_sub = self.node.create_subscription(
            UInt32,
            'subscriber_count',
            self._subscriber_count_callback,
            10
        )

        self.validation_sub = self.node.create_subscription(
            Bool,
            'validation_result',
            self._validation_callback,
            10
        )

        self.image_sub = self.node.create_subscription(
            Image,
            'test_backend_image',
            self._image_callback,
            10
        )

    def tearDown(self):
        """Clean up test fixtures."""
        self.node.destroy_node()

    def _publisher_count_callback(self, msg):
        """Track publisher count."""
        self.publisher_count = msg.data

    def _subscriber_count_callback(self, msg):
        """Track subscriber count."""
        self.subscriber_count = msg.data

    def _validation_callback(self, msg):
        """Track validation status."""
        self.validation_passed = msg.data

    def _image_callback(self, msg):
        """Track received images."""
        self.images_received.append(msg)

    def _spin_until_messages(self, target_count=5, timeout_sec=15.0):
        """Spin until we receive target number of messages or timeout."""
        start_time = time.time()

        while (self.subscriber_count < target_count and
               time.time() - start_time < timeout_sec):
            rclpy.spin_once(self.node, timeout_sec=0.1)

        return self.subscriber_count >= target_count

    def test_01_messages_exchanged(self):
        """Test that messages are successfully exchanged."""
        # Wait for at least 5 messages to be received
        success = self._spin_until_messages(target_count=5, timeout_sec=15.0)

        self.assertTrue(
            success,
            f"Failed to receive 5 messages within timeout. "
            f"Received: {self.subscriber_count}"
        )

        self.assertGreaterEqual(
            self.publisher_count, 5,
            f"Publisher should have sent at least 5 messages. Sent: {self.publisher_count}"
        )

    def test_02_validation_passed(self):
        """Test that message validation passed."""
        # Ensure messages were received first
        self._spin_until_messages(target_count=5, timeout_sec=15.0)

        self.assertTrue(
            self.validation_passed,
            "Message validation failed on subscriber side"
        )

    def test_03_test_backend_used(self):
        """Test that test backend is used for buffers (or CPU fallback)."""
        # Ensure messages were received first
        self._spin_until_messages(target_count=5, timeout_sec=15.0)

        self.assertGreater(
            len(self.images_received), 0,
            "No images received to check backend"
        )

        # The C++ subscriber node validates backend type and publishes validation_result
        # So we rely on that validation
        self.assertTrue(
            self.validation_passed,
            "Backend validation failed (checked by C++ subscriber)"
        )

    def test_04_image_dimensions(self):
        """Test that images have correct dimensions."""
        # Ensure messages were received first
        self._spin_until_messages(target_count=5, timeout_sec=15.0)

        self.assertGreater(
            len(self.images_received), 0,
            "No images received to check dimensions"
        )

        for img in self.images_received:
            self.assertEqual(img.width, 8, "Image width should be 8")
            self.assertEqual(img.height, 8, "Image height should be 8")
            self.assertEqual(img.encoding, "rgb8", "Image encoding should be rgb8")
            self.assertEqual(len(img.data), 8 * 8 * 3, "Image data size should be 192 bytes")

    def test_05_publisher_subscriber_counts_match(self):
        """Test that publisher and subscriber counts roughly match."""
        # Wait for messages
        self._spin_until_messages(target_count=5, timeout_sec=15.0)

        # Allow some tolerance for timing
        self.assertLessEqual(
            abs(self.publisher_count - self.subscriber_count), 3,
            f"Publisher count ({self.publisher_count}) and subscriber count "
            f"({self.subscriber_count}) differ by more than 3"
        )


@launch_testing.post_shutdown_test()
class TestBackendImagePubSubShutdown(unittest.TestCase):
    """Test proper shutdown of nodes."""

    def test_exit_codes(self, proc_info):
        """Check that all processes exited cleanly."""
        launch_testing.asserts.assertExitCodes(proc_info)
