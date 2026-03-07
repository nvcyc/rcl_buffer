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

"""
rclpy image subscriber node for testing Buffer support.

Subscribes to sensor_msgs/Image and validates received data.
Reports message count and validation status via topics.
"""

import array

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, UInt32


class RclpyImageSubscriber(Node):
    """Subscriber node that receives images and validates data integrity."""

    def __init__(self):
        super().__init__('rclpy_image_subscriber')

        # Declare parameters
        self.declare_parameter('topic_name', 'test_image')
        self.declare_parameter('expected_backends', 'cpu')
        self.declare_parameter('count_topic_suffix', '')

        # Read parameters
        topic_name = self.get_parameter('topic_name').value
        self.expected_backends = (
            self.get_parameter('expected_backends').value.split(',')
        )
        count_suffix = self.get_parameter('count_topic_suffix').value

        self.received_count = 0
        self.validation_passed = True

        # Create subscriber
        self.subscription = self.create_subscription(
            Image, topic_name, self.image_callback, 10)

        # Create status publishers
        count_topic = f'subscriber_count{count_suffix}'
        validation_topic = f'validation_result{count_suffix}'
        self.count_publisher = self.create_publisher(UInt32, count_topic, 10)
        self.validation_publisher = self.create_publisher(
            Bool, validation_topic, 10)

        self.get_logger().info(
            f'rclpy image subscriber started '
            f'(topic: {topic_name}, expected_backends: {self.expected_backends})')

    def image_callback(self, msg):
        self.received_count += 1

        # Check the type of msg.data
        data = msg.data
        data_type_name = type(data).__name__

        # Determine backend type
        backend_type = 'cpu'
        try:
            from rosidl_buffer import is_buffer
            if is_buffer(data):
                backend_type = data.backend_type
        except ImportError:
            pass

        # Validate basic image properties
        valid = True
        expected_size = msg.height * msg.width * 3  # rgb8

        if len(data) != expected_size:
            self.get_logger().error(
                f'Image #{self.received_count}: size mismatch '
                f'(expected {expected_size}, got {len(data)})')
            valid = False

        if msg.encoding != 'rgb8':
            self.get_logger().error(
                f'Image #{self.received_count}: encoding mismatch '
                f'(expected rgb8, got {msg.encoding})')
            valid = False

        # Check backend type against expected backends
        if backend_type not in self.expected_backends:
            self.get_logger().error(
                f'Image #{self.received_count}: unexpected backend '
                f"'{backend_type}' (expected one of: {self.expected_backends})")
            valid = False

        # Log backend type
        if backend_type == 'demo':
            self.get_logger().info(
                "Received message using 'demo' backend - zero-copy path!")
        elif backend_type == 'cpu':
            self.get_logger().info(
                "Received message using 'cpu' backend - serialization fallback")
        else:
            self.get_logger().info(
                f"Received message using '{backend_type}' backend")

        # Validate data integrity for all backends
        if valid:
            try:
                # Get CPU-accessible bytes regardless of backend
                if isinstance(data, array.array):
                    data_bytes = data.tobytes()
                elif isinstance(data, (bytes, bytearray)):
                    data_bytes = bytes(data)
                else:
                    # rosidl_buffer.Buffer — use to_bytes() which handles
                    # non-CPU backends via to_vector() internally
                    data_bytes = data.to_bytes()

                if len(data_bytes) > 0:
                    self.get_logger().info(
                        f'Data integrity check: first byte = {data_bytes[0]}, '
                        f'last byte = {data_bytes[-1]}, size = {len(data_bytes)}')

                    # Verify sequential pattern: each consecutive byte
                    # differs by 1 (modulo 256), matching the publisher's
                    # pattern of (count + i) % 256
                    check_len = min(len(data_bytes), 10)
                    pattern_valid = True
                    for i in range(1, check_len):
                        actual_diff = (data_bytes[i] - data_bytes[i - 1] + 256) % 256
                        if actual_diff != 1:
                            self.get_logger().warn(
                                f'Pattern check: byte[{i - 1}]={data_bytes[i - 1]}, '
                                f'byte[{i}]={data_bytes[i]}, '
                                f'diff={actual_diff} (expected 1)')
                            pattern_valid = False
                            break

                    if pattern_valid:
                        self.get_logger().info('Data pattern verification: PASSED')
                    else:
                        self.get_logger().warn('Data pattern verification: FAILED')
                        valid = False

            except Exception as e:
                self.get_logger().error(
                    f'Exception during data validation: {e}')
                valid = False

        if not valid:
            self.validation_passed = False

        self.get_logger().info(
            f'Image #{self.received_count} validation: '
            f'{"PASSED" if valid else "FAILED"} '
            f'(type: {data_type_name}, backend: {backend_type}, size: {len(data)})')

        # Publish status
        count_msg = UInt32()
        count_msg.data = self.received_count
        self.count_publisher.publish(count_msg)

        val_msg = Bool()
        val_msg.data = self.validation_passed
        self.validation_publisher.publish(val_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RclpyImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
