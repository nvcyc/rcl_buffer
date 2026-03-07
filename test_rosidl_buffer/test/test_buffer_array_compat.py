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
# Unit tests for rosidl_buffer.Buffer backward compatibility with array.array.
# These tests verify that Buffer is a transparent drop-in replacement for
# array.array('B') so that existing rclpy user code does not break when
# a uint8[] field is backed by a vendor-specific buffer.

import array
import copy
import io
import pickle
import unittest

from demo_buffer import DemoBuffer
from rosidl_buffer import Buffer, is_buffer


def _make_buffer(data=b'\x00\x01\x02\x03\x04'):
    """Create a DemoBuffer (non-CPU backend) from bytes."""
    return DemoBuffer(data)


def _make_cpu_buffer(data=b'\x00\x01\x02\x03\x04'):
    """Create a CPU-backed Buffer by first making demo then converting."""
    # DemoBuffer returns a Buffer with backend_type='demo'
    return DemoBuffer(data)


class TestBufferIsinstanceCompat(unittest.TestCase):
    """isinstance() checks must work for both Buffer and array.array."""

    def test_buffer_isinstance_array(self):
        buf = _make_buffer()
        self.assertIsInstance(buf, array.array)

    def test_buffer_isinstance_buffer(self):
        buf = _make_buffer()
        self.assertIsInstance(buf, Buffer)

    def test_array_isinstance_buffer(self):
        arr = array.array('B', [1, 2, 3])
        self.assertIsInstance(arr, Buffer)

    def test_type_identity_preserved(self):
        """type(buf) should return Buffer, not array.array."""
        buf = _make_buffer()
        self.assertIs(type(buf), Buffer)

    def test_array_type_identity(self):
        """type(arr) should still be array.array, not Buffer."""
        arr = array.array('B', [1, 2, 3])
        self.assertIs(type(arr), array.array)

    def test_issubclass_array_of_buffer(self):
        self.assertTrue(issubclass(array.array, Buffer))

    def test_isinstance_non_B_array(self):
        """Non-'B' array.array should also pass isinstance(x, Buffer)."""
        arr = array.array('i', [1, 2, 3])
        self.assertIsInstance(arr, Buffer)


class TestBufferProperties(unittest.TestCase):
    """array.array properties: typecode, itemsize."""

    def test_typecode(self):
        buf = _make_buffer()
        self.assertEqual(buf.typecode, 'B')

    def test_itemsize(self):
        buf = _make_buffer()
        self.assertEqual(buf.itemsize, 1)

    def test_backend_type(self):
        buf = _make_buffer()
        self.assertEqual(buf.backend_type, 'demo')

    def test_is_cpu_false_for_demo(self):
        buf = _make_buffer()
        self.assertFalse(buf.is_cpu)


class TestBufferLen(unittest.TestCase):
    """len() should work without triggering CPU conversion."""

    def test_len(self):
        buf = _make_buffer(b'\x01\x02\x03')
        self.assertEqual(len(buf), 3)

    def test_len_no_cpu_conversion(self):
        """len() should NOT trigger lazy CPU conversion."""
        buf = _make_buffer()
        _ = len(buf)
        self.assertIsNone(object.__getattribute__(buf, '_cpu_array'))

    def test_len_empty(self):
        buf = _make_buffer(b'')
        self.assertEqual(len(buf), 0)


class TestBufferReadMethods(unittest.TestCase):
    """Read-only array.array methods that trigger lazy CPU conversion."""

    def setUp(self):
        self.data = b'\x0a\x0b\x0c\x0d\x0e'
        self.buf = _make_buffer(self.data)
        self.arr = array.array('B', self.data)

    def test_getitem_int(self):
        self.assertEqual(self.buf[0], 0x0a)
        self.assertEqual(self.buf[-1], 0x0e)

    def test_getitem_slice(self):
        result = self.buf[1:3]
        expected = self.arr[1:3]
        self.assertEqual(result, expected)

    def test_iter(self):
        self.assertEqual(list(self.buf), list(self.arr))

    def test_reversed(self):
        self.assertEqual(list(reversed(self.buf)), list(reversed(self.arr)))

    def test_contains(self):
        self.assertIn(0x0a, self.buf)
        self.assertNotIn(0xff, self.buf)

    def test_tobytes(self):
        self.assertEqual(self.buf.tobytes(), self.arr.tobytes())

    def test_tolist(self):
        self.assertEqual(self.buf.tolist(), self.arr.tolist())

    def test_count(self):
        buf = _make_buffer(b'\x01\x01\x02\x01')
        self.assertEqual(buf.count(0x01), 3)
        self.assertEqual(buf.count(0x02), 1)
        self.assertEqual(buf.count(0xff), 0)

    def test_index(self):
        self.assertEqual(self.buf.index(0x0c), 2)

    def test_index_with_bounds(self):
        buf = _make_buffer(b'\x01\x02\x01\x02')
        self.assertEqual(buf.index(0x02, 2), 3)

    def test_buffer_info(self):
        result = self.buf.buffer_info()
        self.assertIsInstance(result, tuple)
        self.assertEqual(len(result), 2)
        self.assertEqual(result[1], len(self.data))

    def test_tofile(self):
        f = io.BytesIO()
        self.buf.tofile(f)
        f.seek(0)
        self.assertEqual(f.read(), self.data)

    def test_cpu_conversion_triggered(self):
        """Accessing data should trigger CPU conversion."""
        buf = _make_buffer(b'\x01\x02')
        self.assertIsNone(object.__getattribute__(buf, '_cpu_array'))
        _ = buf[0]
        self.assertIsNotNone(object.__getattribute__(buf, '_cpu_array'))


class TestBufferMutatingMethods(unittest.TestCase):
    """Mutating array.array methods."""

    def test_setitem(self):
        buf = _make_buffer(b'\x01\x02\x03')
        buf[1] = 0xff
        self.assertEqual(buf[1], 0xff)

    def test_delitem(self):
        buf = _make_buffer(b'\x01\x02\x03')
        del buf[1]
        self.assertEqual(list(buf), [0x01, 0x03])

    def test_append(self):
        buf = _make_buffer(b'\x01\x02')
        buf.append(0x03)
        self.assertEqual(list(buf), [0x01, 0x02, 0x03])

    def test_extend(self):
        buf = _make_buffer(b'\x01\x02')
        buf.extend([0x03, 0x04])
        self.assertEqual(list(buf), [0x01, 0x02, 0x03, 0x04])

    def test_insert(self):
        buf = _make_buffer(b'\x01\x03')
        buf.insert(1, 0x02)
        self.assertEqual(list(buf), [0x01, 0x02, 0x03])

    def test_pop(self):
        buf = _make_buffer(b'\x01\x02\x03')
        val = buf.pop()
        self.assertEqual(val, 0x03)
        self.assertEqual(list(buf), [0x01, 0x02])

    def test_pop_index(self):
        buf = _make_buffer(b'\x01\x02\x03')
        val = buf.pop(0)
        self.assertEqual(val, 0x01)
        self.assertEqual(list(buf), [0x02, 0x03])

    def test_remove(self):
        buf = _make_buffer(b'\x01\x02\x03')
        buf.remove(0x02)
        self.assertEqual(list(buf), [0x01, 0x03])

    def test_reverse(self):
        buf = _make_buffer(b'\x01\x02\x03')
        buf.reverse()
        self.assertEqual(list(buf), [0x03, 0x02, 0x01])

    def test_frombytes(self):
        buf = _make_buffer(b'\x01')
        buf.frombytes(b'\x02\x03')
        self.assertEqual(list(buf), [0x01, 0x02, 0x03])

    def test_fromlist(self):
        buf = _make_buffer(b'\x01')
        buf.fromlist([0x02, 0x03])
        self.assertEqual(list(buf), [0x01, 0x02, 0x03])


class TestBufferComparison(unittest.TestCase):
    """Comparison operators between Buffer and array.array."""

    def test_eq_with_array(self):
        data = b'\x01\x02\x03'
        buf = _make_buffer(data)
        arr = array.array('B', data)
        self.assertEqual(buf, arr)

    def test_eq_with_buffer(self):
        data = b'\x01\x02\x03'
        buf1 = _make_buffer(data)
        buf2 = _make_buffer(data)
        self.assertEqual(buf1, buf2)

    def test_ne(self):
        buf = _make_buffer(b'\x01\x02')
        arr = array.array('B', [0x01, 0x03])
        self.assertNotEqual(buf, arr)

    def test_lt(self):
        buf = _make_buffer(b'\x01\x02')
        arr = array.array('B', [0x01, 0x03])
        self.assertTrue(buf < arr)

    def test_le(self):
        buf = _make_buffer(b'\x01\x02')
        arr = array.array('B', [0x01, 0x02])
        self.assertTrue(buf <= arr)

    def test_gt(self):
        buf = _make_buffer(b'\x01\x03')
        arr = array.array('B', [0x01, 0x02])
        self.assertTrue(buf > arr)

    def test_ge(self):
        buf = _make_buffer(b'\x01\x02')
        arr = array.array('B', [0x01, 0x02])
        self.assertTrue(buf >= arr)


class TestBufferArithmetic(unittest.TestCase):
    """Arithmetic operators (concatenation, repeat)."""

    def test_add_buffer_and_array(self):
        buf = _make_buffer(b'\x01\x02')
        arr = array.array('B', [0x03, 0x04])
        result = buf + arr
        self.assertEqual(list(result), [0x01, 0x02, 0x03, 0x04])

    def test_add_array_and_buffer(self):
        arr = array.array('B', [0x01, 0x02])
        buf = _make_buffer(b'\x03\x04')
        result = arr + buf
        self.assertIsInstance(result, array.array)
        self.assertEqual(list(result), [0x01, 0x02, 0x03, 0x04])

    def test_add_two_buffers(self):
        buf1 = _make_buffer(b'\x01\x02')
        buf2 = _make_buffer(b'\x03\x04')
        result = buf1 + buf2
        self.assertEqual(list(result), [0x01, 0x02, 0x03, 0x04])

    def test_iadd(self):
        buf = _make_buffer(b'\x01\x02')
        buf += array.array('B', [0x03])
        self.assertEqual(list(buf), [0x01, 0x02, 0x03])

    def test_mul(self):
        buf = _make_buffer(b'\x01\x02')
        result = buf * 2
        self.assertEqual(list(result), [0x01, 0x02, 0x01, 0x02])

    def test_rmul(self):
        buf = _make_buffer(b'\x01\x02')
        result = 2 * buf
        self.assertEqual(list(result), [0x01, 0x02, 0x01, 0x02])


class TestBufferRepr(unittest.TestCase):
    """repr() should match array.array format."""

    def test_repr_matches_array(self):
        data = b'\x01\x02\x03'
        buf = _make_buffer(data)
        arr = array.array('B', data)
        self.assertEqual(repr(buf), repr(arr))

    def test_repr_starts_with_array(self):
        buf = _make_buffer(b'\x01\x02')
        self.assertTrue(repr(buf).startswith("array('B'"))

    def test_repr_empty(self):
        buf = _make_buffer(b'')
        arr = array.array('B', [])
        self.assertEqual(repr(buf), repr(arr))


class TestBufferProtocol(unittest.TestCase):
    """Buffer protocol: memoryview, bytes() via __buffer__."""

    def test_memoryview(self):
        data = b'\x01\x02\x03'
        buf = _make_buffer(data)
        mv = memoryview(buf)
        self.assertEqual(bytes(mv), data)

    def test_bytes_via_memoryview(self):
        data = b'\x01\x02\x03'
        buf = _make_buffer(data)
        self.assertEqual(bytes(memoryview(buf)), data)


class TestBufferCopyPickle(unittest.TestCase):
    """copy and pickle support."""

    def test_copy(self):
        buf = _make_buffer(b'\x01\x02\x03')
        c = copy.copy(buf)
        self.assertEqual(list(c), [0x01, 0x02, 0x03])
        self.assertIsInstance(c, array.array)

    def test_deepcopy(self):
        buf = _make_buffer(b'\x01\x02\x03')
        c = copy.deepcopy(buf)
        self.assertEqual(list(c), [0x01, 0x02, 0x03])
        self.assertIsInstance(c, array.array)

    def test_pickle_roundtrip(self):
        buf = _make_buffer(b'\x01\x02\x03')
        data = pickle.dumps(buf)
        restored = pickle.loads(data)
        self.assertEqual(list(restored), [0x01, 0x02, 0x03])
        self.assertIsInstance(restored, array.array)


class TestBufferSpecificAPI(unittest.TestCase):
    """Buffer-specific methods that are not on array.array."""

    def test_to_bytes(self):
        data = b'\x01\x02\x03'
        buf = _make_buffer(data)
        self.assertEqual(buf.to_bytes(), data)

    def test_to_bytes_no_cpu_conversion(self):
        """to_bytes() should work without triggering _ensure_cpu()."""
        buf = _make_buffer(b'\x01\x02\x03')
        _ = buf.to_bytes()
        self.assertIsNone(object.__getattribute__(buf, '_cpu_array'))

    def test_backend_type_no_cpu_conversion(self):
        """backend_type should not trigger CPU conversion."""
        buf = _make_buffer(b'\x01')
        _ = buf.backend_type
        self.assertIsNone(object.__getattribute__(buf, '_cpu_array'))


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

    def test_buffer_field_iteration(self):
        from sensor_msgs.msg import Image
        msg = Image()
        data = bytes(range(10))
        buf = _make_buffer(data)
        msg.data = buf
        self.assertEqual(list(msg.data), list(data))

    def test_buffer_field_tobytes(self):
        from sensor_msgs.msg import Image
        msg = Image()
        data = bytes(range(10))
        buf = _make_buffer(data)
        msg.data = buf
        self.assertEqual(msg.data.tobytes(), data)

    def test_buffer_field_isinstance_array(self):
        """After assignment, field should still pass isinstance(x, array.array)."""
        from sensor_msgs.msg import Image
        msg = Image()
        buf = _make_buffer(b'\x01\x02\x03')
        msg.data = buf
        self.assertIsInstance(msg.data, array.array)


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
