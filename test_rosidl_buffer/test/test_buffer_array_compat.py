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
#
# Unit tests for rosidl_buffer.Buffer.
# Buffer is a lightweight wrapper for vendor-backed (non-CPU) data.
# CPU-based data always arrives as array.array('B') in rclpy, so Buffer
# does NOT need array.array API compatibility.

import array
import unittest

from demo_buffer import DemoBuffer
from rosidl_buffer import Buffer, is_buffer


def _make_buffer(data=b'\x00\x01\x02\x03\x04'):
    """Create a DemoBuffer (non-CPU backend) from bytes."""
    return DemoBuffer(data)


class TestBufferType(unittest.TestCase):
    """Basic type identity checks."""

    def test_type_is_buffer(self):
        buf = _make_buffer()
        self.assertIs(type(buf), Buffer)

    def test_isinstance_buffer(self):
        buf = _make_buffer()
        self.assertIsInstance(buf, Buffer)

    def test_array_is_not_buffer(self):
        arr = array.array('B', [1, 2, 3])
        self.assertNotIsInstance(arr, Buffer)

    def test_buffer_is_not_array(self):
        buf = _make_buffer()
        self.assertNotIsInstance(buf, array.array)


class TestBufferProperties(unittest.TestCase):
    """Buffer-specific properties."""

    def test_backend_type(self):
        buf = _make_buffer()
        self.assertEqual(buf.backend_type, 'demo')

    def test_is_cpu_false_for_demo(self):
        buf = _make_buffer()
        self.assertFalse(buf.is_cpu)


class TestBufferLen(unittest.TestCase):
    """len() works without copying data to CPU."""

    def test_len(self):
        buf = _make_buffer(b'\x01\x02\x03')
        self.assertEqual(len(buf), 3)

    def test_len_empty(self):
        buf = _make_buffer(b'')
        self.assertEqual(len(buf), 0)


class TestBufferToBytes(unittest.TestCase):
    """to_bytes() returns buffer contents as Python bytes."""

    def test_to_bytes(self):
        data = b'\x01\x02\x03'
        buf = _make_buffer(data)
        self.assertEqual(buf.to_bytes(), data)

    def test_to_bytes_empty(self):
        buf = _make_buffer(b'')
        self.assertEqual(buf.to_bytes(), b'')

    def test_to_bytes_roundtrip(self):
        data = bytes(range(256))
        buf = _make_buffer(data)
        self.assertEqual(buf.to_bytes(), data)


class TestBufferRepr(unittest.TestCase):
    """repr() shows Buffer-specific info."""

    def test_repr_contains_size(self):
        buf = _make_buffer(b'\x01\x02\x03')
        r = repr(buf)
        self.assertIn('size=3', r)

    def test_repr_contains_backend(self):
        buf = _make_buffer(b'\x01\x02')
        r = repr(buf)
        self.assertIn("backend='demo'", r)

    def test_repr_starts_with_buffer(self):
        buf = _make_buffer(b'\x01')
        self.assertTrue(repr(buf).startswith('Buffer('))


class TestBufferIsBufferFunction(unittest.TestCase):
    """is_buffer() utility function."""

    def test_is_buffer_on_buffer(self):
        buf = _make_buffer()
        self.assertTrue(is_buffer(buf))

    def test_is_buffer_on_array(self):
        arr = array.array('B', [1, 2, 3])
        self.assertFalse(is_buffer(arr))

    def test_is_buffer_on_int(self):
        self.assertFalse(is_buffer(42))


class TestBufferMessageIntegration(unittest.TestCase):
    """Integration with ROS message types (sensor_msgs/Image)."""

    def test_assign_buffer_to_message_field(self):
        from sensor_msgs.msg import Image
        msg = Image()
        msg.height = 2
        msg.width = 2
        msg.encoding = 'rgb8'
        msg.step = 6

        data = bytes(range(12))
        buf = _make_buffer(data)
        msg.data = buf
        self.assertIs(msg.data, buf)

    def test_assign_array_to_message_field(self):
        """array.array assignment should still work normally."""
        from sensor_msgs.msg import Image
        msg = Image()
        msg.height = 2
        msg.width = 2
        msg.encoding = 'rgb8'
        msg.step = 6

        arr = array.array('B', range(12))
        msg.data = arr
        self.assertEqual(list(msg.data), list(range(12)))

    def test_buffer_field_len(self):
        from sensor_msgs.msg import Image
        msg = Image()
        buf = _make_buffer(bytes(range(24)))
        msg.data = buf
        self.assertEqual(len(msg.data), 24)

    def test_buffer_field_to_bytes(self):
        from sensor_msgs.msg import Image
        msg = Image()
        data = bytes(range(10))
        buf = _make_buffer(data)
        msg.data = buf
        self.assertEqual(msg.data.to_bytes(), data)


class TestConvertCompat(unittest.TestCase):
    """rosidl_runtime_py.convert utilities with Buffer data fields."""

    def test_message_to_ordereddict(self):
        from rosidl_runtime_py.convert import message_to_ordereddict
        from sensor_msgs.msg import Image
        msg = Image()
        msg.height = 1
        msg.width = 2
        msg.encoding = 'rgb8'
        msg.step = 6
        msg.data = _make_buffer(bytes(range(6)))

        d = message_to_ordereddict(msg)
        self.assertEqual(d['data'], list(range(6)))

    def test_message_to_csv(self):
        from rosidl_runtime_py.convert import message_to_csv
        from sensor_msgs.msg import Image
        msg = Image()
        msg.height = 1
        msg.width = 1
        msg.encoding = 'rgb8'
        msg.step = 3
        msg.data = _make_buffer(b'\x0a\x0b\x0c')

        csv = message_to_csv(msg)
        self.assertIn('10', csv)
        self.assertIn('11', csv)
        self.assertIn('12', csv)

    def test_message_to_ordereddict_with_array(self):
        """Plain array.array data should still work."""
        from rosidl_runtime_py.convert import message_to_ordereddict
        from sensor_msgs.msg import Image
        msg = Image()
        msg.height = 1
        msg.width = 1
        msg.encoding = 'rgb8'
        msg.step = 3
        msg.data = array.array('B', [10, 11, 12])

        d = message_to_ordereddict(msg)
        self.assertEqual(d['data'], [10, 11, 12])


class TestSetMessageFieldsCompat(unittest.TestCase):
    """rosidl_runtime_py.set_message.set_message_fields with Buffer."""

    def test_set_fields_when_current_is_buffer(self):
        from rosidl_runtime_py.set_message import set_message_fields
        from sensor_msgs.msg import Image
        msg = Image()
        msg.data = _make_buffer(b'\x01\x02\x03')

        set_message_fields(msg, {'data': [0x0a, 0x0b]})
        self.assertEqual(list(msg.data), [0x0a, 0x0b])
        self.assertIsInstance(msg.data, array.array)

    def test_set_fields_when_current_is_array(self):
        """Standard array.array path should still work."""
        from rosidl_runtime_py.set_message import set_message_fields
        from sensor_msgs.msg import Image
        msg = Image()

        set_message_fields(msg, {'data': [0x0a, 0x0b]})
        self.assertEqual(list(msg.data), [0x0a, 0x0b])


if __name__ == '__main__':
    unittest.main()
