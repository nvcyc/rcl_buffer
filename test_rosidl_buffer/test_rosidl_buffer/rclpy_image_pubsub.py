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
Same-process rclpy pub/sub test for Buffer support.

Creates publisher and subscriber nodes in a single process using
SingleThreadedExecutor, exchanges messages, validates received data,
and exits with 0 (pass) or 1 (fail).

Supports CPU and demo backends via the --backend-mode CLI argument.
"""

import argparse
import array
import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image


class PubNode(Node):

    def __init__(self, backend_mode):
        super().__init__('same_process_publisher')
        self.backend_mode = backend_mode
        self.count = 0
        self.max_count = 20
        self.publisher = self.create_publisher(Image, 'test_same_process_image', 10)
        self.timer = self.create_timer(0.1, self._timer_cb)
        self.get_logger().info(
            f'Publisher started (backend_mode={self.backend_mode})')

    def _timer_cb(self):
        if self.count >= self.max_count:
            return

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'test_frame'
        msg.height = 8
        msg.width = 8
        msg.encoding = 'rgb8'
        msg.step = 8 * 3
        msg.is_bigendian = 0

        data_size = 8 * 8 * 3
        host_data = bytes([(self.count + i) % 256 for i in range(data_size)])

        if self.backend_mode == 'demo':
            from demo_buffer import DemoBuffer
            msg.data = DemoBuffer.from_cpu(host_data)
        else:
            msg.data = array.array('B', host_data)

        self.publisher.publish(msg)
        self.count += 1
        self.get_logger().info(f'Published image #{self.count}')


class SubNode(Node):

    def __init__(self, expected_backend):
        super().__init__('same_process_subscriber')
        self.expected_backend = expected_backend
        self.received_count = 0
        self.validation_passed = True
        self.seen_first_bytes = set()

        sub_kwargs = {}
        if expected_backend == 'demo':
            sub_kwargs['acceptable_buffer_backends'] = 'any'
        self.subscription = self.create_subscription(
            Image, 'test_same_process_image', self._image_cb, 10, **sub_kwargs)
        self.get_logger().info(
            f'Subscriber started (expected_backend={self.expected_backend})')

    def _image_cb(self, msg):
        self.received_count += 1
        data = msg.data

        backend_type = 'cpu'
        try:
            from rosidl_buffer import Buffer
            if isinstance(data, Buffer):
                backend_type = data.backend_type
        except ImportError:
            pass

        valid = True
        expected_size = msg.height * msg.width * 3

        if len(data) != expected_size:
            self.get_logger().error(
                f'#{self.received_count}: size mismatch '
                f'(expected {expected_size}, got {len(data)})')
            valid = False

        if msg.encoding != 'rgb8':
            self.get_logger().error(
                f'#{self.received_count}: encoding mismatch '
                f'(expected rgb8, got {msg.encoding})')
            valid = False

        if backend_type not in (self.expected_backend, 'cpu'):
            self.get_logger().error(
                f'#{self.received_count}: unexpected backend '
                f"'{backend_type}' (expected '{self.expected_backend}')")
            valid = False

        if valid:
            try:
                if isinstance(data, array.array):
                    data_bytes = data.tobytes()
                elif isinstance(data, (bytes, bytearray)):
                    data_bytes = bytes(data)
                else:
                    data_bytes = data.to_bytes()

                check_len = min(len(data_bytes), 10)
                for i in range(1, check_len):
                    if (data_bytes[i] - data_bytes[i - 1] + 256) % 256 != 1:
                        self.get_logger().error(
                            f'#{self.received_count}: data pattern mismatch at byte {i}')
                        valid = False
                        break

                # Duplicate detection: data[0] == (pub_count % 256)
                first_byte = data_bytes[0]
                if first_byte in self.seen_first_bytes:
                    self.get_logger().error(
                        f'#{self.received_count}: duplicate message detected! '
                        f'data[0]={first_byte} was already received')
                    valid = False
                else:
                    self.seen_first_bytes.add(first_byte)
            except Exception as e:
                self.get_logger().error(f'#{self.received_count}: validation error: {e}')
                valid = False

        if not valid:
            self.validation_passed = False

        self.get_logger().info(
            f'Received image #{self.received_count} '
            f'(backend={backend_type}, valid={valid})')


def main(args=None):
    rclpy.init(args=args)

    non_ros_args = remove_ros_args(args=sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--backend-mode', default='cpu', choices=['cpu', 'demo'])
    parsed = parser.parse_args(non_ros_args[1:])

    print(f'\n=== Starting rclpy Same-Process Pub/Sub Test '
          f'(backend: {parsed.backend_mode}) ===\n', flush=True)

    pub_node = PubNode(parsed.backend_mode)
    sub_node = SubNode(parsed.backend_mode)

    executor = SingleThreadedExecutor()
    executor.add_node(pub_node)
    executor.add_node(sub_node)

    # Allow time for intra-process discovery
    time.sleep(1.0)
    executor.spin_once(timeout_sec=0.1)

    start = time.time()
    timeout = 10.0
    target = 3
    while time.time() - start < timeout:
        executor.spin_once(timeout_sec=0.1)
        if sub_node.received_count >= target:
            break

    success = sub_node.received_count >= target and sub_node.validation_passed

    print('\nTest Results:', flush=True)
    print(f'  Published: {pub_node.count} images', flush=True)
    print(f'  Received:  {sub_node.received_count} images', flush=True)
    print(f'  Valid:     {"YES" if sub_node.validation_passed else "NO"}', flush=True)

    executor.shutdown()
    pub_node.destroy_node()
    sub_node.destroy_node()
    rclpy.shutdown()

    if success:
        print('\n=== ALL TESTS PASSED ===\n', flush=True)
        sys.exit(0)
    else:
        print('\n=== TESTS FAILED ===\n', flush=True)
        sys.exit(1)


if __name__ == '__main__':
    main()
