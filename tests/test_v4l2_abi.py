import ctypes
from omniavoid.v4l2 import Format, PixelFormatM, Buffer, Plane, Capability, RequestBuffers, request


def test_linux_64_bit_v4l2_abi_sizes_and_ioctl_numbers():
    assert ctypes.sizeof(Format) == 208
    assert ctypes.sizeof(PixelFormatM) == 192
    assert ctypes.sizeof(Buffer) == 88
    assert ctypes.sizeof(Plane) == 64
    assert ctypes.sizeof(Capability) == 104
    assert ctypes.sizeof(RequestBuffers) == 20
    assert request(3, 5, Format) == 0xC0D05605
    assert request(3, 17, Buffer) == 0xC0585611
    assert request(2, 0, Capability) == 0x80685600
