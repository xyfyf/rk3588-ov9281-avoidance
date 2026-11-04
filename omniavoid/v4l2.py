"""Linux/aarch64 V4L2 mmap capture with kernel timestamps and sequence numbers.

Supports single-plane and multi-plane capture. GREY, YUYV, Y16, NV12/NV21
(including NV12M/NV21M luma) are accepted. Packed RAW10/RAW12 require a
board-specific ISP/unpacking adapter and are deliberately rejected.
"""
from __future__ import annotations

import ctypes as C
import errno
import mmap
import os
import select
import sys
import threading
import time
import cv2
import numpy as np
from .sync import Frame, Synchronizer

U8, U16, U32 = C.c_uint8, C.c_uint16, C.c_uint32


class Capability(C.Structure):
    _fields_ = [("driver", U8 * 16), ("card", U8 * 32), ("bus", U8 * 32),
                ("version", U32), ("caps", U32), ("device_caps", U32), ("reserved", U32 * 3)]


class PixelFormat(C.Structure):
    _fields_ = [(n, U32) for n in ("width", "height", "pixelformat", "field", "bytesperline",
                                 "sizeimage", "colorspace", "priv", "flags", "ycbcr", "quant", "xfer")]


class PlaneFormat(C.Structure):
    _fields_ = [("sizeimage", U32), ("bytesperline", U32), ("reserved", U16 * 6)]


class PixelFormatM(C.Structure):
    _fields_ = [(n, U32) for n in ("width", "height", "pixelformat", "field", "colorspace")] + [
        ("plane_fmt", PlaneFormat * 8), ("num_planes", U8), ("flags", U8),
        ("ycbcr", U8), ("quant", U8), ("xfer", U8), ("reserved", U8 * 7)]


class FormatUnion(C.Union):
    _fields_ = [("pix", PixelFormat), ("pix_mp", PixelFormatM), ("raw", U8 * 200), ("align", C.c_void_p)]


class Format(C.Structure):
    _fields_ = [("type", U32), ("fmt", FormatUnion)]


class RequestBuffers(C.Structure):
    _fields_ = [("count", U32), ("type", U32), ("memory", U32), ("reserved", U32 * 2)]


class Timeval(C.Structure):
    _fields_ = [("seconds", C.c_int64), ("microseconds", C.c_int64)]


class Timecode(C.Structure):
    _fields_ = [("type", U32), ("flags", U32), ("bytes", U8 * 8)]


class PlaneMemory(C.Union):
    _fields_ = [("offset", U32), ("userptr", C.c_uint64), ("fd", C.c_int)]


class Plane(C.Structure):
    _fields_ = [("bytesused", U32), ("length", U32), ("m", PlaneMemory),
                ("data_offset", U32), ("reserved", U32 * 11)]


class BufferMemory(C.Union):
    _fields_ = [("offset", U32), ("userptr", C.c_uint64), ("planes", C.POINTER(Plane)), ("fd", C.c_int)]


class Buffer(C.Structure):
    _fields_ = [("index", U32), ("type", U32), ("bytesused", U32), ("flags", U32),
                ("field", U32), ("timestamp", Timeval), ("timecode", Timecode),
                ("sequence", U32), ("memory", U32), ("m", BufferMemory),
                ("length", U32), ("reserved2", U32), ("request_fd", C.c_int)]


def request(direction, number, struct):
    return (direction << 30) | (C.sizeof(struct) << 16) | (ord("V") << 8) | number


def fourcc(text):
    if len(text) != 4:
        raise ValueError("V4L2 fourcc must contain four characters")
    return sum(ord(ch) << (8 * i) for i, ch in enumerate(text))


class V4L2Camera:
    def __init__(self, device, name, width, height, pixel_format="GREY", buffers=4, y16_shift=8):
        if not sys.platform.startswith("linux") or C.sizeof(C.c_void_p) != 8:
            raise RuntimeError("V4L2 backend requires 64-bit Linux (RK3588 Ubuntu 22.04)")
        import fcntl
        self.ioctl = lambda code, value: fcntl.ioctl(self.fd, code, value, True)
        self.fd, self.maps, self.streaming = None, [], False
        self.name, self.y16_shift = name, int(y16_shift)
        if pixel_format not in ("GREY", "YUYV", "Y16 ", "NV12", "NV21", "NM12", "NM21"):
            raise ValueError("Unsupported pixel format; use ISP grayscale/luma output or write a RAW adapter")
        try:
            self.fd = os.open(device, os.O_RDWR | os.O_NONBLOCK)
            cap = Capability()
            self.ioctl(request(2, 0, Capability), cap)
            caps = cap.device_caps if cap.caps & 0x80000000 else cap.caps
            if not caps & 0x04000000:
                raise RuntimeError("Device does not support streaming capture")
            if caps & 1:
                self.type, self.multi = 1, False
            elif caps & 0x1000:
                self.type, self.multi = 9, True
            else:
                raise RuntimeError("Device is not a video capture node")
            fmt = Format()
            fmt.type = self.type
            pix = fmt.fmt.pix_mp if self.multi else fmt.fmt.pix
            pix.width, pix.height, pix.pixelformat, pix.field = width, height, fourcc(pixel_format), 1
            self.ioctl(request(3, 5, Format), fmt)
            self.width, self.height = int(pix.width), int(pix.height)
            if (self.width, self.height, pix.pixelformat) != (width, height, fourcc(pixel_format)):
                raise RuntimeError("Driver changed requested image dimensions/fourcc; select a matching capture node/config")
            self.pixel_format = pixel_format
            minimum_stride = width * (2 if pixel_format in ("YUYV", "Y16 ") else 1)
            self.stride = int(pix.plane_fmt[0].bytesperline if self.multi else pix.bytesperline) or minimum_stride
            self.plane_count = int(pix.num_planes) if self.multi else 1
            if not 1 <= self.plane_count <= 8 or self.stride < minimum_stride:
                raise RuntimeError("Invalid plane count or stride")
            req = RequestBuffers()
            req.count, req.type, req.memory = buffers, self.type, 1
            self.ioctl(request(3, 8, RequestBuffers), req)
            if req.count < 2:
                raise RuntimeError("At least two mmap buffers are required")
            for i in range(req.count):
                buf, planes = self._buffer(i)
                self.ioctl(request(3, 9, Buffer), buf)
                specs = [(planes[j].length, planes[j].m.offset) for j in range(self.plane_count)] if self.multi else [(buf.length, buf.m.offset)]
                self.maps.append([mmap.mmap(self.fd, length, flags=mmap.MAP_SHARED,
                                           prot=mmap.PROT_READ | mmap.PROT_WRITE, offset=offset)
                                  for length, offset in specs])
                self.ioctl(request(3, 15, Buffer), buf)
            capture_type = U32(self.type)
            self.ioctl(request(1, 18, U32), capture_type)
            self.streaming = True
        except Exception:
            self.close()
            raise

    def _buffer(self, index=0):
        buf, planes = Buffer(), (Plane * 8)()
        buf.index, buf.type, buf.memory = index, self.type, 1
        if self.multi:
            buf.m.planes, buf.length = planes, self.plane_count
        return buf, planes

    def read(self, timeout_s=0.1):
        readable, _, _ = select.select([self.fd], [], [], timeout_s)
        if not readable:
            return None
        buf, planes = self._buffer()
        try:
            self.ioctl(request(3, 17, Buffer), buf)
        except OSError as exc:
            if exc.errno == errno.EAGAIN:
                return None
            raise
        try:
            if buf.flags & 0x40:
                raise RuntimeError(f"{self.name}: V4L2 driver flagged corrupted frame")
            if (buf.flags & 0xE000) != 0x2000:
                raise RuntimeError(f"{self.name}: driver must expose CLOCK_MONOTONIC timestamps")
            if buf.index >= len(self.maps):
                raise RuntimeError("Invalid driver buffer index")
            used = planes[0].bytesused if self.multi else buf.bytesused
            offset = planes[0].data_offset if self.multi else 0
            needed = self.stride * self.height
            if used - offset < needed or offset + needed > len(self.maps[buf.index][0]):
                raise RuntimeError("Truncated video frame")
            raw = np.frombuffer(self.maps[buf.index][0], np.uint8, count=needed, offset=offset).copy()
            rows = raw.reshape(self.height, self.stride)
            if self.pixel_format == "YUYV":
                image = rows[:, :self.width * 2:2].copy()
            elif self.pixel_format == "Y16 ":
                image = (rows[:, :self.width * 2].copy().view("<u2") >> self.y16_shift).clip(0, 255).astype(np.uint8)
            else:
                image = rows[:, :self.width].copy()
            monotonic_ns = int(buf.timestamp.seconds) * 1_000_000_000 + int(buf.timestamp.microseconds) * 1000
            epoch_ns = time.time_ns() - time.monotonic_ns() + monotonic_ns
            return Frame(self.name, int(buf.sequence), monotonic_ns, epoch_ns, image, int(buf.flags))
        finally:
            self.ioctl(request(3, 15, Buffer), buf)

    def close(self):
        if self.fd is not None:
            if self.streaming:
                try:
                    self.ioctl(request(1, 19, U32), U32(self.type))
                except OSError:
                    pass
                self.streaming = False
            for planes in self.maps:
                for plane in planes:
                    plane.close()
            self.maps.clear()
            os.close(self.fd)
            self.fd = None


class CaptureRig:
    def __init__(self, cfg):
        self.cfg = cfg
        self.stop_event = threading.Event()
        self.errors = []
        self.cameras, self.threads = [], []
        self.sync = Synchronizer([d["name"] for d in cfg["cameras"]], cfg["timing"]["max_skew_ms"], cfg["timing"]["max_age_ms"])
        try:
            for d in cfg["cameras"]:
                capture = cfg["capture"]
                camera = V4L2Camera(capture["devices"][d["name"]], d["name"], d["width"], d["height"],
                                    capture["pixel_format"], capture["buffers"], capture.get("y16_shift", 8))
                self.cameras.append(camera)
            for camera in self.cameras:
                worker = threading.Thread(target=self._read, args=(camera,), daemon=True, name="capture-" + camera.name)
                self.threads.append(worker)
                worker.start()
        except Exception:
            self.close()
            raise

    def _read(self, camera):
        try:
            while not self.stop_event.is_set():
                frame = camera.read()
                if frame is not None:
                    self.sync.add(frame)
        except Exception as exc:
            self.errors.append(f"{camera.name}: {exc}")
            self.stop_event.set()

    def next(self, timeout_s=0.2):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.errors:
                raise RuntimeError("; ".join(self.errors))
            group = self.sync.pop(time.monotonic_ns())
            if group:
                return group
            if self.stop_event.wait(0.002):
                break
        return None

    def close(self):
        self.stop_event.set()
        for worker in self.threads:
            worker.join(timeout=1)
        for camera in self.cameras:
            camera.close()
