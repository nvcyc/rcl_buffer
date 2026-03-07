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
demo_buffer - Python bindings for the demo buffer backend.

Provides a DemoBuffer factory that creates rosidl_buffer.Buffer objects
backed by DemoBufferImpl (the demo/reference buffer backend).

Example usage:
    from demo_buffer import DemoBuffer

    # Create a demo-backend buffer from bytes
    buf = DemoBuffer(b'\\x00\\x01\\x02\\x03')
    assert buf.backend_type == 'demo'

    # Use it in a ROS2 message (sensor_msgs/Image.data)
    msg = Image()
    msg.data = buf  # triggers vendor-aware serialization path
"""

from demo_buffer._demo_buffer_py import (
    _create_demo_buffer_from_bytes,
    _create_demo_buffer_from_size,
)


def DemoBuffer(data=None, *, size=None):
    """
    Create an rosidl_buffer.Buffer backed by the demo buffer backend.

    Parameters
    ----------
    data : bytes or bytearray or None
        Initial data. If provided, creates a buffer with this content.
    size : int or None
        Size in bytes. If provided (and data is None), creates a
        zero-initialized buffer of this size.

    Returns
    -------
    rosidl_buffer.Buffer
        A Buffer object with backend_type == 'demo'.

    Examples
    --------
    >>> buf = DemoBuffer(b'hello')
    >>> len(buf)
    5
    >>> buf.backend_type
    'demo'

    >>> buf = DemoBuffer(size=1024)
    >>> len(buf)
    1024

    """
    from rosidl_buffer import _take_buffer_from_ptr

    if data is not None:
        if not isinstance(data, (bytes, bytearray)):
            data = bytes(data)
        elif isinstance(data, bytearray):
            data = bytes(data)
        ptr = _create_demo_buffer_from_bytes(data)
    elif size is not None:
        ptr = _create_demo_buffer_from_size(size)
    else:
        ptr = _create_demo_buffer_from_size(0)

    return _take_buffer_from_ptr(ptr)


__all__ = ['DemoBuffer']
