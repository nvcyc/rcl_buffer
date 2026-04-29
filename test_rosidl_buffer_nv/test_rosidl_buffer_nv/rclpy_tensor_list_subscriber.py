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

"""rclpy TensorList subscriber for nested Buffer transport tests."""

import array

from isaac_ros_tensor_list_interfaces.msg import TensorList
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, UInt32


DATA_TYPE_UINT8 = 2


def payload_length(index, seq):
    return 4 + ((index + seq) % 6) * 8


def make_payload(index, seq, length):
    return bytes((index * 131 + seq * 7 + i) % 256 for i in range(length))


def bytes_from_buffer(data):
    if isinstance(data, array.array):
        return data.tobytes()
    if isinstance(data, (bytes, bytearray)):
        return bytes(data)
    return data.to_bytes()


def backend_type(data):
    try:
        from rosidl_buffer import Buffer
        if isinstance(data, Buffer):
            return data.backend_type
    except ImportError:
        pass
    return 'cpu'


class RclpyTensorListSubscriber(Node):
    """Subscriber that validates nested Tensor.data buffers."""

    def __init__(self):
        super().__init__('rclpy_tensor_list_subscriber')

        self.declare_parameter('topic_name', 'test_tensor_list')
        self.declare_parameter('expected_backends', 'any')
        self.declare_parameter('count_topic_prefix', '')
        self.declare_parameter('count_topic_suffix', '')
        self.declare_parameter('acceptable_buffer_backends', '__default__')

        topic_name = self.get_parameter('topic_name').value
        self.expected_backends_str = self.get_parameter('expected_backends').value
        self.expected_backends = [
            token.strip() for token in self.expected_backends_str.split(',')
            if token.strip()
        ]
        count_prefix = self.get_parameter('count_topic_prefix').value
        count_suffix = self.get_parameter('count_topic_suffix').value
        acceptable_backends = self.get_parameter(
            'acceptable_buffer_backends').value

        self.received_count = 0
        self.validation_passed = True
        self.seen_first_bytes = set()

        sub_kwargs = {}
        if acceptable_backends != '__default__':
            sub_kwargs['acceptable_buffer_backends'] = acceptable_backends
        self.subscription = self.create_subscription(
            TensorList, topic_name, self.tensor_list_callback, 10, **sub_kwargs)

        count_topic = (
            f'{count_prefix}_subscriber_count' if count_prefix
            else 'subscriber_count'
        ) + count_suffix
        validation_topic = (
            f'{count_prefix}_validation_result' if count_prefix
            else 'validation_result'
        ) + count_suffix
        backend_topic = (
            f'{count_prefix}_backend_report' if count_prefix
            else 'backend_report'
        ) + count_suffix

        self.count_publisher = self.create_publisher(UInt32, count_topic, 10)
        self.validation_publisher = self.create_publisher(
            Bool, validation_topic, 10)
        self.backend_publisher = self.create_publisher(String, backend_topic, 10)

        self.get_logger().info(
            f'rclpy TensorList subscriber started '
            f'(topic: {topic_name}, expected_backends: '
            f'{self.expected_backends_str})')

    def backend_allowed(self, backend):
        return (
            self.expected_backends_str == 'any' or
            backend in self.expected_backends
        )

    def validate_tensor(self, tensor, index, seq):
        length = payload_length(index, seq)
        expected = make_payload(index, seq, length)
        received = bytes_from_buffer(tensor.data)

        if len(received) != len(expected):
            return (
                f'tensor[{index}] size mismatch: expected {len(expected)}, '
                f'got {len(received)}')
        if received != expected:
            for byte_index, (actual, wanted) in enumerate(zip(received, expected)):
                if actual != wanted:
                    return (
                        f'tensor[{index}] byte[{byte_index}]: expected '
                        f'{wanted}, got {actual}')
            return f'tensor[{index}] payload mismatch'
        if tensor.data_type != DATA_TYPE_UINT8:
            return (
                f'tensor[{index}] data_type expected {DATA_TYPE_UINT8}, '
                f'got {tensor.data_type}')
        if (
            tensor.shape.rank != 2 or
            len(tensor.shape.dims) != 2 or
            tensor.shape.dims[0] != 1 + index
        ):
            return (
                f'tensor[{index}] shape mismatch '
                f'(rank={tensor.shape.rank}, dims={list(tensor.shape.dims)})')
        return None

    def tensor_list_callback(self, msg):
        seq = self.received_count
        self.received_count += 1

        msg_valid = True
        expected_outer = 1 + (seq % 4)
        if len(msg.tensors) != expected_outer:
            self.get_logger().error(
                f'TensorList size mismatch: expected {expected_outer}, '
                f'got {len(msg.tensors)}')
            msg_valid = False
        else:
            for index, tensor in enumerate(msg.tensors):
                error = self.validate_tensor(tensor, index, seq)
                if error:
                    self.get_logger().error(error)
                    msg_valid = False

        observed_backend = '<empty>'
        if msg.tensors:
            observed_backend = backend_type(msg.tensors[0].data)
            if not self.backend_allowed(observed_backend):
                self.get_logger().error(
                    f'Unexpected nested backend: {observed_backend} '
                    f'(expected one of: {self.expected_backends_str})')
                msg_valid = False

            data_bytes = bytes_from_buffer(msg.tensors[0].data)
            first_byte = data_bytes[0] if data_bytes else 0
            if first_byte in self.seen_first_bytes:
                self.get_logger().error(
                    f'Duplicate detected: first_byte={first_byte} '
                    'was already received')
                msg_valid = False
            else:
                self.seen_first_bytes.add(first_byte)

        self.validation_passed = self.validation_passed and msg_valid

        count_msg = UInt32()
        count_msg.data = self.received_count
        self.count_publisher.publish(count_msg)

        validation_msg = Bool()
        validation_msg.data = self.validation_passed
        self.validation_publisher.publish(validation_msg)

        backend_msg = String()
        backend_msg.data = observed_backend
        self.backend_publisher.publish(backend_msg)

        self.get_logger().info(
            f'TensorList #{self.received_count} validation: '
            f'{"PASSED" if msg_valid else "FAILED"} '
            f'(nested backend: {observed_backend})')


def main(args=None):
    rclpy.init(args=args)
    node = RclpyTensorListSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
