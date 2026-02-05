#!/usr/bin/env python3
# Copyright 2024 NVIDIA Corporation
#
# Common utilities for backend pub/sub launch tests

import time
import rclpy
from rclpy.node import Node as RclpyNode
from std_msgs.msg import UInt32, Bool
from typing import Dict, List, Optional


class BackendTestMonitor:
    """Monitor for tracking pub/sub test progress across multiple subscribers."""

    def __init__(self, node: RclpyNode, subscriber_suffixes: Optional[List[str]] = None):
        """
        Initialize the test monitor.

        Args:
            node: ROS2 node to use for subscriptions
            subscriber_suffixes: List of suffixes for subscriber count/validation topics.
                                 If None, monitors the default topics (no suffix).
        """
        self.node = node
        self.publisher_count = 0
        self.subscriber_counts: Dict[str, int] = {}
        self.validation_results: Dict[str, bool] = {}

        # Subscribe to publisher count
        self.pub_count_sub = node.create_subscription(
            UInt32,
            'publisher_count',
            self._publisher_count_callback,
            10
        )

        # Set up monitoring for each subscriber
        suffixes = subscriber_suffixes if subscriber_suffixes else ['']
        self._subscriber_subs = []
        self._validation_subs = []

        for suffix in suffixes:
            self.subscriber_counts[suffix] = 0
            self.validation_results[suffix] = True

            count_topic = f'subscriber_count{suffix}'
            validation_topic = f'validation_result{suffix}'

            sub_count = node.create_subscription(
                UInt32,
                count_topic,
                lambda msg, s=suffix: self._subscriber_count_callback(msg, s),
                10
            )
            self._subscriber_subs.append(sub_count)

            val_sub = node.create_subscription(
                Bool,
                validation_topic,
                lambda msg, s=suffix: self._validation_callback(msg, s),
                10
            )
            self._validation_subs.append(val_sub)

    def _publisher_count_callback(self, msg):
        self.publisher_count = msg.data

    def _subscriber_count_callback(self, msg, suffix: str):
        self.subscriber_counts[suffix] = msg.data

    def _validation_callback(self, msg, suffix: str):
        self.validation_results[suffix] = msg.data

    def get_total_subscriber_count(self) -> int:
        """Get sum of all subscriber counts."""
        return sum(self.subscriber_counts.values())

    def get_min_subscriber_count(self) -> int:
        """Get minimum subscriber count (to verify all subscribers received messages)."""
        if not self.subscriber_counts:
            return 0
        return min(self.subscriber_counts.values())

    def all_validations_passed(self) -> bool:
        """Check if all subscribers report validation passed."""
        return all(self.validation_results.values())

    def spin_until_messages(self, target_count: int, timeout_sec: float = 15.0,
                            check_all_subscribers: bool = True) -> bool:
        """
        Spin until target messages received or timeout.

        Args:
            target_count: Target number of messages to receive
            timeout_sec: Timeout in seconds
            check_all_subscribers: If True, checks that ALL subscribers reach target.
                                   If False, checks that ANY subscriber reaches target.
        """
        start_time = time.time()

        while time.time() - start_time < timeout_sec:
            rclpy.spin_once(self.node, timeout_sec=0.1)

            if check_all_subscribers:
                if self.get_min_subscriber_count() >= target_count:
                    return True
            else:
                if self.get_total_subscriber_count() >= target_count:
                    return True

        return False


class MultiTopicBackendTestMonitor:
    """Monitor for tracking pub/sub test progress across multiple topics."""

    def __init__(self, node: RclpyNode, topic_configs: List[Dict]):
        """
        Initialize the multi-topic test monitor.

        Args:
            node: ROS2 node to use for subscriptions
            topic_configs: List of dicts with keys:
                - 'topic_prefix': prefix for the topic (e.g., 'topic1', 'topic2')
                - 'subscriber_suffixes': list of suffixes for subscriber nodes
        """
        self.node = node
        self.topic_monitors: Dict[str, Dict] = {}

        self._all_subs = []

        for config in topic_configs:
            topic_prefix = config['topic_prefix']
            subscriber_suffixes = config.get('subscriber_suffixes', [''])

            self.topic_monitors[topic_prefix] = {
                'publisher_count': 0,
                'subscriber_counts': {s: 0 for s in subscriber_suffixes},
                'validation_results': {s: True for s in subscriber_suffixes},
            }

            # Subscribe to publisher count for this topic
            pub_count_topic = f'{topic_prefix}_publisher_count'
            pub_sub = node.create_subscription(
                UInt32,
                pub_count_topic,
                lambda msg, t=topic_prefix: self._pub_count_callback(msg, t),
                10
            )
            self._all_subs.append(pub_sub)

            # Subscribe to each subscriber's count/validation for this topic
            for suffix in subscriber_suffixes:
                sub_count_topic = f'{topic_prefix}_subscriber_count{suffix}'
                val_topic = f'{topic_prefix}_validation_result{suffix}'

                sub_count = node.create_subscription(
                    UInt32,
                    sub_count_topic,
                    lambda msg, t=topic_prefix, s=suffix: self._sub_count_callback(msg, t, s),
                    10
                )
                self._all_subs.append(sub_count)

                val_sub = node.create_subscription(
                    Bool,
                    val_topic,
                    lambda msg, t=topic_prefix, s=suffix: self._val_callback(msg, t, s),
                    10
                )
                self._all_subs.append(val_sub)

    def _pub_count_callback(self, msg, topic_prefix: str):
        self.topic_monitors[topic_prefix]['publisher_count'] = msg.data

    def _sub_count_callback(self, msg, topic_prefix: str, suffix: str):
        self.topic_monitors[topic_prefix]['subscriber_counts'][suffix] = msg.data

    def _val_callback(self, msg, topic_prefix: str, suffix: str):
        self.topic_monitors[topic_prefix]['validation_results'][suffix] = msg.data

    def get_min_subscriber_count_for_topic(self, topic_prefix: str) -> int:
        """Get minimum subscriber count for a topic."""
        counts = self.topic_monitors[topic_prefix]['subscriber_counts'].values()
        return min(counts) if counts else 0

    def get_min_subscriber_count_overall(self) -> int:
        """Get minimum subscriber count across all topics and subscribers."""
        all_counts = []
        for topic_data in self.topic_monitors.values():
            all_counts.extend(topic_data['subscriber_counts'].values())
        return min(all_counts) if all_counts else 0

    def all_validations_passed(self) -> bool:
        """Check if all subscribers on all topics report validation passed."""
        for topic_data in self.topic_monitors.values():
            if not all(topic_data['validation_results'].values()):
                return False
        return True

    def spin_until_messages(self, target_count: int, timeout_sec: float = 15.0) -> bool:
        """Spin until all subscribers on all topics reach target count or timeout."""
        start_time = time.time()

        while time.time() - start_time < timeout_sec:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.get_min_subscriber_count_overall() >= target_count:
                return True

        return False
