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
Legacy rclpy image publisher for backward-compatibility testing.

This publisher deliberately uses ONLY array.array and has NO imports
from rcl_buffer or demo_buffer.  It represents existing user code
written before the native buffer feature was added.

When paired with a buffer-aware subscriber, this validates that the
Python-to-C conversion path still works correctly when the message
field is a plain array.array (not a Buffer).
"""

import array

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import UInt32


class RclpyLegacyImagePublisher(Node):
    """Legacy publisher using only array.array — no rcl_buffer imports."""

    def __init__(self):
        super().__init__('rclpy_legacy_image_publisher')

        self.declare_parameter('topic_name', 'test_image')
        self.declare_parameter('publish_rate_ms', 200)
        self.declare_parameter('max_publish_count', 5)

        topic_name = self.get_parameter('topic_name').value
        publish_rate_ms = self.get_parameter('publish_rate_ms').value
        self.max_publish_count = self.get_parameter('max_publish_count').value

        self.count = 0

        self.publisher = self.create_publisher(Image, topic_name, 10)
        self.count_publisher = self.create_publisher(UInt32, 'publisher_count', 10)

        self.timer = self.create_timer(
            publish_rate_ms / 1000.0, self.timer_callback)

        self.get_logger().info(
            f'Legacy array.array publisher started '
            f'(topic: {topic_name}, max_count: {self.max_publish_count})')

    def timer_callback(self):
        if self.max_publish_count > 0 and self.count >= self.max_publish_count:
            return

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'legacy_test_frame'
        msg.height = 8
        msg.width = 8
        msg.encoding = 'rgb8'
        msg.step = 8 * 3
        msg.is_bigendian = 0

        data_size = 8 * 8 * 3

        # Same sequential pattern as the demo publisher: (count + i) % 256
        msg.data = array.array('B', [(self.count + i) % 256 for i in range(data_size)])

        self.get_logger().info(
            f'Publishing image #{self.count + 1} with array.array '
            f'(size: {len(msg.data)})')

        self.publisher.publish(msg)

        count_msg = UInt32()
        self.count += 1
        count_msg.data = self.count
        self.count_publisher.publish(count_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RclpyLegacyImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
