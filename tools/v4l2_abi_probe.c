#include <time.h>
#include <stdio.h>
#include <stddef.h>
#include <linux/videodev2.h>

int main(void) {
    _Static_assert(sizeof(void *) == 8, "Only 64-bit Linux is supported");
    _Static_assert(sizeof(struct v4l2_format) == 208, "v4l2_format ABI mismatch");
    _Static_assert(sizeof(struct v4l2_buffer) == 88, "v4l2_buffer ABI mismatch");
    _Static_assert(sizeof(struct v4l2_plane) == 64, "v4l2_plane ABI mismatch");
    _Static_assert(sizeof(struct v4l2_pix_format_mplane) == 192, "mplane ABI mismatch");
    _Static_assert(offsetof(struct v4l2_buffer, timestamp) == 24, "timestamp offset mismatch");
    _Static_assert(VIDIOC_S_FMT == 0xc0d05605UL, "S_FMT ioctl mismatch");
    _Static_assert(VIDIOC_DQBUF == 0xc0585611UL, "DQBUF ioctl mismatch");
    puts("V4L2 64-bit ABI matches Python capture backend");
    return 0;
}
