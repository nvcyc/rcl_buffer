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
rcl_buffer - Python bindings for ROS 2 native buffer feature.

Provides a Buffer type that wraps rcl_buffer::Buffer<uint8_t> and
supports vendor-specific memory backends (CPU, GPU, custom).

Users never construct Buffer directly.  Instead, backend providers
supply factory functions (e.g. demo_buffer.DemoBuffer) that return
Buffer objects.  Existing rclpy code using array.array('B') continues
to work unchanged -- the Buffer type is only relevant when a non-CPU
backend is in use.

Example usage:
    from demo_buffer import DemoBuffer

    # Create a demo-backend buffer from bytes
    buf = DemoBuffer(b'\\x00\\x01\\x02\\x03')
    assert buf.backend_type == 'demo'
    assert len(buf) == 4

    # Assign to a uint8[] message field -- triggers zero-copy path
    msg = Image()
    msg.data = buf

    # Read data from any backend
    raw = buf.to_bytes()
"""

from rcl_buffer._rcl_buffer_py import Buffer  # noqa: F401
from rcl_buffer._rcl_buffer_py import is_buffer  # noqa: F401

__all__ = [
    'Buffer',
    'is_buffer',
]
