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

The Buffer class is a drop-in replacement for array.array('B'):
  - isinstance(buf, array.array) returns True
  - All array.array methods are available (tobytes, tolist, etc.)
  - Data is lazily converted to CPU only when array methods are called
  - Backend-specific data stays on-device until accessed

Users never construct Buffer directly.  Instead, backend providers
supply factory functions (e.g. demo_buffer.DemoBuffer) that return
Buffer objects.  Existing rclpy code using array.array('B') continues
to work unchanged.

Example usage:
    from demo_buffer import DemoBuffer

    buf = DemoBuffer(b'\\x00\\x01\\x02\\x03')
    assert buf.backend_type == 'demo'
    assert len(buf) == 4
    assert isinstance(buf, array.array)  # True!
    assert buf.typecode == 'B'
    assert buf.tobytes() == b'\\x00\\x01\\x02\\x03'

    msg = Image()
    msg.data = buf
"""

import array as _array_module

from rcl_buffer import _rcl_buffer_py


class BufferMeta(type):
    """
    Metaclass for Buffer.

    Makes isinstance(x, Buffer) return True for array.array instances,
    so code can use ``isinstance(x, Buffer)`` as a unified check for both
    Buffer objects and plain array.array objects.
    """

    def __instancecheck__(cls, instance):
        if cls is Buffer and isinstance(instance, _array_module.array):
            return True
        return super().__instancecheck__(instance)

    def __subclasscheck__(cls, subclass):
        if cls is Buffer and subclass is _array_module.array:
            return True
        return super().__subclasscheck__(subclass)


class Buffer(metaclass=BufferMeta):
    """
    Array-compatible buffer backed by a vendor-specific memory backend.

    This is a transparent drop-in replacement for ``array.array('B')``.
    ``isinstance(buf, array.array)`` returns True, and all ``array.array``
    methods are available.  Data is lazily converted to CPU (via the
    underlying C++ ``to_vector()``) only when array-compatible methods
    are actually called, so vendor-backed data stays on-device when it
    is simply passed through the publish/subscribe pipeline.
    """

    __slots__ = ('_native', '_cpu_array')

    def __init__(self, native_buffer):
        object.__setattr__(self, '_native', native_buffer)
        object.__setattr__(self, '_cpu_array', None)

    # ------------------------------------------------------------------
    # __class__ property trick: makes isinstance(buf, array.array) True.
    # CPython's isinstance() checks obj.__class__ when it differs from
    # type(obj).  type(buf) still returns Buffer.
    # ------------------------------------------------------------------
    @property
    def __class__(self):
        return _array_module.array

    # ------------------------------------------------------------------
    # Lazy CPU conversion
    # ------------------------------------------------------------------
    def _ensure_cpu(self):
        cpu = object.__getattribute__(self, '_cpu_array')
        if cpu is None:
            native = object.__getattribute__(self, '_native')
            cpu = _array_module.array('B', native.to_bytes())
            object.__setattr__(self, '_cpu_array', cpu)
        return cpu

    # ------------------------------------------------------------------
    # Buffer-specific API (not on array.array)
    # ------------------------------------------------------------------
    @property
    def backend_type(self):
        """Backend type identifier (e.g. 'cpu', 'demo', 'cuda')."""
        return object.__getattribute__(self, '_native').backend_type

    def to_bytes(self):
        """Return buffer contents as bytes (handles any backend)."""
        return object.__getattribute__(self, '_native').to_bytes()

    @property
    def is_cpu(self):
        """True if the buffer is backed by CPU memory."""
        return self.backend_type == 'cpu'

    # ------------------------------------------------------------------
    # array.array read-only properties
    # ------------------------------------------------------------------
    @property
    def typecode(self):
        return 'B'

    @property
    def itemsize(self):
        return 1

    # ------------------------------------------------------------------
    # __len__ — does NOT trigger CPU conversion (size is always available)
    # ------------------------------------------------------------------
    def __len__(self):
        return len(object.__getattribute__(self, '_native'))

    # ------------------------------------------------------------------
    # Read methods — trigger lazy CPU conversion
    # ------------------------------------------------------------------
    def __getitem__(self, key):
        return self._ensure_cpu()[key]

    def __iter__(self):
        return iter(self._ensure_cpu())

    def __reversed__(self):
        return reversed(self._ensure_cpu())

    def __contains__(self, value):
        return value in self._ensure_cpu()

    def tobytes(self):
        return self._ensure_cpu().tobytes()

    def tolist(self):
        return self._ensure_cpu().tolist()

    def tofile(self, f):
        return self._ensure_cpu().tofile(f)

    def buffer_info(self):
        return self._ensure_cpu().buffer_info()

    def count(self, value):
        return self._ensure_cpu().count(value)

    def index(self, value, *args):
        return self._ensure_cpu().index(value, *args)

    # ------------------------------------------------------------------
    # Mutating methods — trigger lazy CPU conversion
    # ------------------------------------------------------------------
    def __setitem__(self, key, value):
        self._ensure_cpu()[key] = value

    def __delitem__(self, key):
        del self._ensure_cpu()[key]

    def append(self, value):
        self._ensure_cpu().append(value)

    def extend(self, iterable):
        self._ensure_cpu().extend(iterable)

    def frombytes(self, data):
        self._ensure_cpu().frombytes(data)

    def fromlist(self, lst):
        self._ensure_cpu().fromlist(lst)

    def fromfile(self, f, n):
        self._ensure_cpu().fromfile(f, n)

    def insert(self, i, value):
        self._ensure_cpu().insert(i, value)

    def pop(self, *args):
        return self._ensure_cpu().pop(*args)

    def remove(self, value):
        self._ensure_cpu().remove(value)

    def reverse(self):
        self._ensure_cpu().reverse()

    def byteswap(self):
        self._ensure_cpu().byteswap()

    # ------------------------------------------------------------------
    # Comparison operators
    # ------------------------------------------------------------------
    def __eq__(self, other):
        if type(other) is Buffer:
            return self._ensure_cpu() == other._ensure_cpu()
        return self._ensure_cpu() == other

    def __ne__(self, other):
        if type(other) is Buffer:
            return self._ensure_cpu() != other._ensure_cpu()
        return self._ensure_cpu() != other

    def __lt__(self, other):
        if type(other) is Buffer:
            return self._ensure_cpu() < other._ensure_cpu()
        return self._ensure_cpu() < other

    def __le__(self, other):
        if type(other) is Buffer:
            return self._ensure_cpu() <= other._ensure_cpu()
        return self._ensure_cpu() <= other

    def __gt__(self, other):
        if type(other) is Buffer:
            return self._ensure_cpu() > other._ensure_cpu()
        return self._ensure_cpu() > other

    def __ge__(self, other):
        if type(other) is Buffer:
            return self._ensure_cpu() >= other._ensure_cpu()
        return self._ensure_cpu() >= other

    # ------------------------------------------------------------------
    # Arithmetic operators
    # ------------------------------------------------------------------
    def __add__(self, other):
        if type(other) is Buffer:
            return self._ensure_cpu() + other._ensure_cpu()
        return self._ensure_cpu() + other

    def __radd__(self, other):
        return other + self._ensure_cpu()

    def __iadd__(self, other):
        if type(other) is Buffer:
            self._ensure_cpu().__iadd__(other._ensure_cpu())
        else:
            self._ensure_cpu().__iadd__(other)
        return self

    def __mul__(self, n):
        return self._ensure_cpu() * n

    def __rmul__(self, n):
        return n * self._ensure_cpu()

    def __imul__(self, n):
        self._ensure_cpu().__imul__(n)
        return self

    # ------------------------------------------------------------------
    # Repr — matches array.array output for full backward compatibility
    # ------------------------------------------------------------------
    def __repr__(self):
        return repr(self._ensure_cpu())

    # ------------------------------------------------------------------
    # Buffer protocol — allows memoryview(buf), bytes(buf), etc.
    # ------------------------------------------------------------------
    def __buffer__(self, flags):
        return self._ensure_cpu().__buffer__(flags)

    # ------------------------------------------------------------------
    # Copy / pickle support
    # ------------------------------------------------------------------
    def __copy__(self):
        import copy
        return copy.copy(self._ensure_cpu())

    def __deepcopy__(self, memo):
        import copy
        return copy.deepcopy(self._ensure_cpu(), memo)

    def __reduce_ex__(self, protocol):
        return self._ensure_cpu().__reduce_ex__(protocol)

    # ------------------------------------------------------------------
    # Hash (array.array is unhashable)
    # ------------------------------------------------------------------
    __hash__ = None


# ------------------------------------------------------------------
# Pipeline helper functions used by generated message code
# ------------------------------------------------------------------

def _take_buffer_from_ptr(ptr):
    """
    Take ownership of a heap-allocated Buffer* and return a Python Buffer.

    Called by generated C->Python conversion code (_msg_support.c).
    """
    native = _rcl_buffer_py._take_buffer_from_ptr(ptr)
    return Buffer(native)


def _get_buffer_ptr(buf):
    """
    Get the raw C++ Buffer pointer as an integer.

    Called by generated Python->C conversion code (_msg_support.c).
    Accepts both the new Python Buffer wrapper and the native pybind11 Buffer.
    """
    if type(buf) is Buffer:
        return _rcl_buffer_py._get_buffer_ptr(
            object.__getattribute__(buf, '_native'))
    return _rcl_buffer_py._get_buffer_ptr(buf)


def is_buffer(obj):
    """Check if the given object is an rcl_buffer.Buffer (not array.array)."""
    return type(obj) is Buffer or _rcl_buffer_py.is_buffer(obj)


__all__ = [
    'Buffer',
    'BufferMeta',
    'is_buffer',
]
