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

r"""
demo_buffer - Python bindings for the demo buffer backend.

Provides a DemoBuffer factory that creates rosidl_buffer.Buffer objects
backed by DemoBufferImpl (the demo/reference buffer backend).

Example usage::

    from demo_buffer import DemoBuffer

    # Create a demo-backend buffer from CPU data
    buf = DemoBuffer.from_cpu(b'\x00\x01\x02\x03')
    assert buf.backend_type == 'demo'

    # Convert back to array.array
    cpu_array = buf.to_array()  # array.array('B', ...)

    # Use it in a ROS 2 message (sensor_msgs/Image.data)
    msg = Image()
    msg.data = buf  # triggers vendor-aware serialization path
"""

from demo_buffer._demo_buffer_py import _from_cpu_data, _from_size


class DemoBuffer:
    """Factory for demo-backend rosidl::Buffer objects."""

    @staticmethod
    def from_cpu(data):
        """
        Create a demo-backed Buffer from CPU data.

        Parameters
        ----------
        data : bytes, bytearray, or iterable of ints

        Returns
        -------
        rosidl_buffer.Buffer
            A Buffer with backend_type == 'demo'.

        Examples
        --------
        >>> buf = DemoBuffer.from_cpu(b'hello')
        >>> len(buf)
        5
        >>> buf.backend_type
        'demo'
        """
        if not isinstance(data, bytes):
            data = bytes(data)
        return _from_cpu_data(data)

    @staticmethod
    def from_size(size):
        """
        Create a zero-initialized demo-backed Buffer of given size.

        Parameters
        ----------
        size : int

        Returns
        -------
        rosidl_buffer.Buffer
            A zero-filled Buffer with backend_type == 'demo'.

        Examples
        --------
        >>> buf = DemoBuffer.from_size(1024)
        >>> len(buf)
        1024
        """
        return _from_size(size)


__all__ = ['DemoBuffer']
