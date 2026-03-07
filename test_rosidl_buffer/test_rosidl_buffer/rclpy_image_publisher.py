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
rclpy image publisher node for testing Buffer support.

Supports CPU and demo backends via parameter. Publishes sensor_msgs/Image
with a known data pattern for verification by the subscriber.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import UInt32


class RclpyImagePublisher(Node):
    """Publisher node that sends images using rosidl_buffer.Buffer or array.array."""

    def __init__(self):
        super().__init__('rclpy_image_publisher')

        # Declare parameters
        self.declare_parameter('backend_mode', 'cpu')
        self.declare_parameter('topic_name', 'test_image')
        self.declare_parameter('publish_rate_ms', 200)
        self.declare_parameter('max_publish_count', 5)
        self.declare_parameter('count_topic_prefix', '')

        # Read parameters
        self.backend_mode = self.get_parameter('backend_mode').value
        topic_name = self.get_parameter('topic_name').value
        publish_rate_ms = self.get_parameter('publish_rate_ms').value
        self.max_publish_count = self.get_parameter('max_publish_count').value
        count_prefix = self.get_parameter('count_topic_prefix').value

        self.count = 0

        # Create publishers
        self.publisher = self.create_publisher(Image, topic_name, 10)
        count_topic = (
            f'{count_prefix}_publisher_count' if count_prefix
            else 'publisher_count'
        )
        self.count_publisher = self.create_publisher(UInt32, count_topic, 10)

        # Create timer
        self.timer = self.create_timer(
            publish_rate_ms / 1000.0, self.timer_callback)

        self.get_logger().info(
            f'rclpy image publisher started '
            f'(backend_mode: {self.backend_mode}, topic: {topic_name}, '
            f'max_count: {self.max_publish_count})')

    def timer_callback(self):
        if self.max_publish_count > 0 and self.count >= self.max_publish_count:
            return

        msg = Image()

        # Create demo image: 8x8 RGB (same as C++ publisher)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'rclpy_test_frame'
        msg.height = 8
        msg.width = 8
        msg.encoding = 'rgb8'
        msg.step = 8 * 3
        msg.is_bigendian = 0

        data_size = 8 * 8 * 3

        # Fill with pattern based on count (same formula as C++ publisher)
        host_data = bytes([(self.count + i) % 256 for i in range(data_size)])

        if self.backend_mode == 'demo':
            from demo_buffer import DemoBuffer
            msg.data = DemoBuffer(host_data)
            backend_label = msg.data.backend_type
        else:
            # CPU mode: use standard array.array (default rclpy behavior)
            import array
            msg.data = array.array('B', host_data)
            backend_label = 'cpu'

        self.get_logger().info(
            f'Publishing image #{self.count + 1} with {backend_label} backend '
            f'(size: {len(msg.data)})')

        self.publisher.publish(msg)

        # Publish count
        count_msg = UInt32()
        self.count += 1
        count_msg.data = self.count
        self.count_publisher.publish(count_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RclpyImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
