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
Legacy rclpy image subscriber for backward-compatibility testing.

This subscriber deliberately uses ONLY array.array APIs and has NO imports
from rosidl_buffer.  It represents existing user code written before the
native buffer feature was added.  When a demo-backend publisher sends
Buffer-backed data, the subscriber should still work transparently
because rosidl_buffer.Buffer is a drop-in replacement for array.array('B').

Validated array.array APIs exercised here:
  - isinstance(data, array.array)
  - len(data)
  - data.typecode
  - data.itemsize
  - data.tobytes()
  - data.tolist()
  - data[i]  (indexing)
  - iteration (for byte in data)
  - data.count(x)
  - data.index(x)
  - memoryview(data)
"""

import array

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, UInt32


class RclpyLegacyImageSubscriber(Node):
    """Legacy subscriber using only array.array APIs — no rosidl_buffer imports."""

    def __init__(self):
        super().__init__('rclpy_legacy_image_subscriber')

        self.declare_parameter('topic_name', 'test_image')
        self.declare_parameter('count_topic_suffix', '')

        topic_name = self.get_parameter('topic_name').value
        count_suffix = self.get_parameter('count_topic_suffix').value

        self.received_count = 0
        self.validation_passed = True
        self.seen_first_bytes = set()

        self.subscription = self.create_subscription(
            Image, topic_name, self.image_callback, 10)

        count_topic = f'subscriber_count{count_suffix}'
        validation_topic = f'validation_result{count_suffix}'
        self.count_publisher = self.create_publisher(UInt32, count_topic, 10)
        self.validation_publisher = self.create_publisher(
            Bool, validation_topic, 10)

        self.get_logger().info(
            f'Legacy array.array subscriber started (topic: {topic_name})')

    def image_callback(self, msg):
        self.received_count += 1
        data = msg.data
        valid = True
        expected_size = msg.height * msg.width * 3  # rgb8

        # --- Exercise array.array APIs that existing user code would use ---

        # 1. isinstance check — the primary backward-compat requirement
        if not isinstance(data, array.array):
            self.get_logger().error(
                f'Image #{self.received_count}: isinstance(data, array.array) '
                f'returned False! type={type(data).__name__}')
            valid = False

        # 2. typecode and itemsize
        if valid:
            if data.typecode != 'B':
                self.get_logger().error(
                    f'Image #{self.received_count}: '
                    f'typecode={data.typecode!r}, expected "B"')
                valid = False
            if data.itemsize != 1:
                self.get_logger().error(
                    f'Image #{self.received_count}: '
                    f'itemsize={data.itemsize}, expected 1')
                valid = False

        # 3. len()
        if valid and len(data) != expected_size:
            self.get_logger().error(
                f'Image #{self.received_count}: size mismatch '
                f'(expected {expected_size}, got {len(data)})')
            valid = False

        # 4. tobytes()
        if valid:
            data_bytes = data.tobytes()
            if len(data_bytes) != expected_size:
                self.get_logger().error(
                    f'Image #{self.received_count}: tobytes() size mismatch '
                    f'(expected {expected_size}, got {len(data_bytes)})')
                valid = False

        # 5. tolist()
        if valid:
            data_list = data.tolist()
            if len(data_list) != expected_size:
                self.get_logger().error(
                    f'Image #{self.received_count}: tolist() size mismatch')
                valid = False

        # 6. Indexing — data[i]
        if valid and len(data) > 0:
            first = data[0]
            last = data[-1]
            if not isinstance(first, int) or not isinstance(last, int):
                self.get_logger().error(
                    f'Image #{self.received_count}: indexing returned '
                    f'non-int types: {type(first)}, {type(last)}')
                valid = False

        # 7. Iteration — for byte in data
        if valid:
            iter_count = 0
            for _ in data:
                iter_count += 1
            if iter_count != expected_size:
                self.get_logger().error(
                    f'Image #{self.received_count}: iteration count mismatch '
                    f'(expected {expected_size}, got {iter_count})')
                valid = False

        # 8. count() and index()
        if valid and len(data) > 0:
            first_val = data[0]
            cnt = data.count(first_val)
            if cnt < 1:
                self.get_logger().error(
                    f'Image #{self.received_count}: count({first_val}) = {cnt}')
                valid = False
            idx = data.index(first_val)
            if idx != 0:
                self.get_logger().error(
                    f'Image #{self.received_count}: '
                    f'index({first_val}) = {idx}, expected 0')
                valid = False

        # 9. memoryview (buffer protocol)
        if valid:
            mv = memoryview(data)
            if len(mv) != expected_size:
                self.get_logger().error(
                    f'Image #{self.received_count}: memoryview length mismatch')
                valid = False

        # 10. Verify sequential data pattern
        if valid and len(data) > 1:
            data_bytes = data.tobytes()
            check_len = min(len(data_bytes), 10)
            for i in range(1, check_len):
                actual_diff = (data_bytes[i] - data_bytes[i - 1] + 256) % 256
                if actual_diff != 1:
                    self.get_logger().error(
                        f'Image #{self.received_count}: pattern mismatch at '
                        f'byte[{i}]={data_bytes[i]}, '
                        f'byte[{i - 1}]={data_bytes[i - 1]}')
                    valid = False
                    break

        # 11. Duplicate detection: data[0] == (pub_count % 256) is unique per
        # message for tests sending < 256 messages
        if valid and len(data) > 0:
            first_byte = data[0]
            if first_byte in self.seen_first_bytes:
                self.get_logger().error(
                    f'Image #{self.received_count}: duplicate message '
                    f'detected! data[0]={first_byte} was already received')
                valid = False
            else:
                self.seen_first_bytes.add(first_byte)

        if not valid:
            self.validation_passed = False

        self.get_logger().info(
            f'Image #{self.received_count} legacy validation: '
            f'{"PASSED" if valid else "FAILED"} '
            f'(type: {type(data).__name__}, size: {len(data)})')

        count_msg = UInt32()
        count_msg.data = self.received_count
        self.count_publisher.publish(count_msg)

        val_msg = Bool()
        val_msg.data = self.validation_passed
        self.validation_publisher.publish(val_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RclpyLegacyImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
