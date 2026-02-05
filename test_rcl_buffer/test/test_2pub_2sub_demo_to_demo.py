#!/usr/bin/env python3
# Copyright 2024 NVIDIA Corporation
#
# Launch test: 2 publishers on different topics, each with 2 subscribers
# All using Demo backend

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
    """Generate launch description for 2 pub / 2 sub per topic, Demo-to-Demo test."""

    # Topic 1: Publisher with 2 subscribers
    publisher_1 = Node(
        package='test_rcl_buffer',
        executable='demo_backend_image_publisher_node',
        name='demo_image_publisher_1',
        output='screen',
        parameters=[{
            'backend_mode': 'demo',
            'topic_name': 'topic1_image',
            'publish_rate_ms': 200,
            'count_topic_prefix': 'topic1',
            'max_publish_count': 5,
        }],
    )

    subscriber_1a = Node(
        package='test_rcl_buffer',
        executable='demo_backend_image_subscriber_node',
        name='demo_image_subscriber_1a',
        output='screen',
        parameters=[{
            'topic_name': 'topic1_image',
            'expected_backends': 'demo,cpu',
            'count_topic_prefix': 'topic1',
            'count_topic_suffix': '_a',
        }],
    )

    subscriber_1b = Node(
        package='test_rcl_buffer',
        executable='demo_backend_image_subscriber_node',
        name='demo_image_subscriber_1b',
        output='screen',
        parameters=[{
            'topic_name': 'topic1_image',
            'expected_backends': 'demo,cpu',
            'count_topic_prefix': 'topic1',
            'count_topic_suffix': '_b',
        }],
    )

    # Topic 2: Publisher with 2 subscribers
    publisher_2 = Node(
        package='test_rcl_buffer',
        executable='demo_backend_image_publisher_node',
        name='demo_image_publisher_2',
        output='screen',
        parameters=[{
            'backend_mode': 'demo',
            'topic_name': 'topic2_image',
            'publish_rate_ms': 200,
            'count_topic_prefix': 'topic2',
            'max_publish_count': 5,
        }],
    )

    subscriber_2a = Node(
        package='test_rcl_buffer',
        executable='demo_backend_image_subscriber_node',
        name='demo_image_subscriber_2a',
        output='screen',
        parameters=[{
            'topic_name': 'topic2_image',
            'expected_backends': 'demo,cpu',
            'count_topic_prefix': 'topic2',
            'count_topic_suffix': '_a',
        }],
    )

    subscriber_2b = Node(
        package='test_rcl_buffer',
        executable='demo_backend_image_subscriber_node',
        name='demo_image_subscriber_2b',
        output='screen',
        parameters=[{
            'topic_name': 'topic2_image',
            'expected_backends': 'demo,cpu',
            'count_topic_prefix': 'topic2',
            'count_topic_suffix': '_b',
        }],
    )

    return LaunchDescription([
        publisher_1,
        subscriber_1a,
        subscriber_1b,
        publisher_2,
        subscriber_2a,
        subscriber_2b,
        launch_testing.actions.ReadyToTest(),
    ])


class TestDemoToDemo2Pub2Sub(unittest.TestCase):
    """Test case for 2 pub / 2 sub per topic, Demo-to-Demo."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_demo_to_demo_2pub_2sub')

        # Track topic1
        self.topic1_pub_count = 0
        self.topic1_sub_counts = {'_a': 0, '_b': 0}
        self.topic1_validations = {'_a': True, '_b': True}

        # Track topic2
        self.topic2_pub_count = 0
        self.topic2_sub_counts = {'_a': 0, '_b': 0}
        self.topic2_validations = {'_a': True, '_b': True}

        # Topic 1 subscriptions
        self.node.create_subscription(
            UInt32, 'topic1_publisher_count',
            lambda msg: setattr(self, 'topic1_pub_count', msg.data), 10)
        self.node.create_subscription(
            UInt32, 'topic1_subscriber_count_a',
            lambda msg: self._update_dict(self.topic1_sub_counts, '_a', msg.data), 10)
        self.node.create_subscription(
            UInt32, 'topic1_subscriber_count_b',
            lambda msg: self._update_dict(self.topic1_sub_counts, '_b', msg.data), 10)
        self.node.create_subscription(
            Bool, 'topic1_validation_result_a',
            lambda msg: self._update_dict(self.topic1_validations, '_a', msg.data), 10)
        self.node.create_subscription(
            Bool, 'topic1_validation_result_b',
            lambda msg: self._update_dict(self.topic1_validations, '_b', msg.data), 10)

        # Topic 2 subscriptions
        self.node.create_subscription(
            UInt32, 'topic2_publisher_count',
            lambda msg: setattr(self, 'topic2_pub_count', msg.data), 10)
        self.node.create_subscription(
            UInt32, 'topic2_subscriber_count_a',
            lambda msg: self._update_dict(self.topic2_sub_counts, '_a', msg.data), 10)
        self.node.create_subscription(
            UInt32, 'topic2_subscriber_count_b',
            lambda msg: self._update_dict(self.topic2_sub_counts, '_b', msg.data), 10)
        self.node.create_subscription(
            Bool, 'topic2_validation_result_a',
            lambda msg: self._update_dict(self.topic2_validations, '_a', msg.data), 10)
        self.node.create_subscription(
            Bool, 'topic2_validation_result_b',
            lambda msg: self._update_dict(self.topic2_validations, '_b', msg.data), 10)

    def tearDown(self):
        self.node.destroy_node()

    def _update_dict(self, d, key, value):
        d[key] = value

    def _get_min_count(self):
        all_counts = list(self.topic1_sub_counts.values()) + list(self.topic2_sub_counts.values())
        return min(all_counts) if all_counts else 0

    def _all_validations_passed(self):
        return (all(self.topic1_validations.values()) and
                all(self.topic2_validations.values()))

    def _spin_until(self, target_count=5, timeout_sec=20.0):
        start = time.time()
        while self._get_min_count() < target_count and time.time() - start < timeout_sec:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self._get_min_count() >= target_count

    def test_demo_to_demo_2pub_2sub_messages_delivered(self):
        """Test 2 Demo publishers to 2 Demo subscribers each."""
        success = self._spin_until(target_count=5, timeout_sec=20.0)

        min_count = self._get_min_count()
        self.assertTrue(success,
            f"Failed to receive 5 msgs on all subscribers. Min: {min_count}, "
            f"topic1: {self.topic1_sub_counts}, topic2: {self.topic2_sub_counts}")
        self.assertGreaterEqual(self.topic1_pub_count, 5,
            f"Topic1 publisher should have sent at least 5 messages. Sent: {self.topic1_pub_count}")
        self.assertGreaterEqual(self.topic2_pub_count, 5,
            f"Topic2 publisher should have sent at least 5 messages. Sent: {self.topic2_pub_count}")
        self.assertTrue(self._all_validations_passed(),
            f"Validation failed: topic1={self.topic1_validations}, topic2={self.topic2_validations}")


@launch_testing.post_shutdown_test()
class TestDemoToDemo2Pub2SubShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
