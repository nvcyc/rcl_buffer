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

"""rclpy TensorList publisher for nested Buffer transport tests."""

import array

from isaac_ros_tensor_list_interfaces.msg import Tensor, TensorList
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32


DATA_TYPE_UINT8 = 2


def payload_length(index, seq):
    return 4 + ((index + seq) % 6) * 8


def make_payload(index, seq, length):
    return bytes((index * 131 + seq * 7 + i) % 256 for i in range(length))


class RclpyTensorListPublisher(Node):
    """Publisher that sends TensorList messages with nested uint8[] buffers."""

    def __init__(self):
        super().__init__('rclpy_tensor_list_publisher')

        self.declare_parameter('backend_mode', 'demo')
        self.declare_parameter('topic_name', 'test_tensor_list')
        self.declare_parameter('publish_rate_ms', 200)
        self.declare_parameter('max_publish_count', 50)
        self.declare_parameter('count_topic_prefix', '')

        self.backend_mode = self.get_parameter('backend_mode').value
        topic_name = self.get_parameter('topic_name').value
        publish_rate_ms = self.get_parameter('publish_rate_ms').value
        self.max_publish_count = self.get_parameter('max_publish_count').value
        count_prefix = self.get_parameter('count_topic_prefix').value

        self.count = 0
        self.publisher = self.create_publisher(TensorList, topic_name, 10)
        count_topic = (
            f'{count_prefix}_publisher_count' if count_prefix
            else 'publisher_count'
        )
        self.count_publisher = self.create_publisher(UInt32, count_topic, 10)
        self.timer = self.create_timer(
            publish_rate_ms / 1000.0, self.timer_callback)

        self.get_logger().info(
            f'rclpy TensorList publisher started '
            f'(backend_mode: {self.backend_mode}, topic: {topic_name}, '
            f'max_count: {self.max_publish_count})')

    def populate_tensor(self, tensor, index, seq):
        length = payload_length(index, seq)
        payload = make_payload(index, seq, length)

        tensor.name = f't{index}_seq{seq}'
        tensor.data_type = DATA_TYPE_UINT8
        tensor.shape.rank = 2
        tensor.shape.dims = [1 + index, length // max(1, 1 + index)]
        tensor.strides = [length, 1]

        if self.backend_mode == 'demo':
            from demo_buffer import DemoBuffer
            tensor.data = DemoBuffer.from_cpu(payload)
        else:
            tensor.data = array.array('B', payload)

    def timer_callback(self):
        if self.max_publish_count > 0 and self.count >= self.max_publish_count:
            return

        msg = TensorList()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'rclpy_tensor_list'

        num_tensors = 1 + (self.count % 4)
        msg.tensors = [Tensor() for _ in range(num_tensors)]
        for index, tensor in enumerate(msg.tensors):
            self.populate_tensor(tensor, index, self.count)

        self.get_logger().info(
            f'Publishing TensorList #{self.count + 1} '
            f'(backend_mode: {self.backend_mode}, tensors: {num_tensors})')
        self.publisher.publish(msg)

        self.count += 1
        count_msg = UInt32()
        count_msg.data = self.count
        self.count_publisher.publish(count_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RclpyTensorListPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
