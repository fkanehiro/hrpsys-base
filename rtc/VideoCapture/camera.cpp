#include <cstdio>
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <sys/mman.h>

#include "camera.h"

/* raw camera member functions */
v4l_capture::v4l_capture()
  : dev_name(""), fd(-1), width(640), height(480),
    buffers(NULL), n_buffers(0)
{};

v4l_capture::~v4l_capture()
{
  stop_capturing();
  uninit_device();
  close_device();
};

int
v4l_capture::init(size_t _width, size_t _height, unsigned int devId)
{
  width = _width;
  height = _height;
  if (!init_all(width, height, devId)) return -1;
  frame = cv::Mat(height, width, CV_8UC3);
  return 0;
}

uchar *
v4l_capture::capture ()
{
  if (!write_img(frame.data)) return NULL;
  return frame.data;
}


int v4l_capture::getWidth ()
{
    return width;
}

int v4l_capture::getHeight ()
{
    return height;
}

bool v4l_capture::init_all(size_t _width, size_t _height, unsigned int _devId)
{
  width = _width;
  height = _height;
  std::ostringstream oss("");
  oss << "/dev/video" << _devId;
  dev_name = oss.str();
  if (!open_device()) return false;
  init_device();
  if (!start_capturing()) return false;
  return true;
}

bool v4l_capture::open_device(void)
{
  fprintf(stderr, "Opening device '%s'\n", dev_name.c_str());
  fd = open(dev_name.c_str(), O_RDWR, 0);

  if (fd == -1) {
    fprintf(stderr, "Cannot open '%s': %d, %s\n",
            dev_name.c_str(), errno, strerror(errno));
    return false;
  }
  return true;
}

bool v4l_capture::close_device(void)
{
  if (close(fd) == -1) {
    perror("close");
    return false;
  }
  fd = -1;
  return true;
}

bool v4l_capture::write_img(uchar* ret)
{
  if (!read_frame()) return false;

  for (int i = 0; i < width * height; i += 2) {
    int y, r, g, b;
    int u, v;
    y = ((unsigned char *) buffers[0].start)[i * 2];
    u = ((unsigned char *) buffers[0].start)[i * 2 + 1] - 128;
    v = ((unsigned char *) buffers[0].start)[i * 2 + 3] - 128;

    r = y + 1.40200 * v;
    g = y - 0.71414 * v - 0.34414 * u;
    b = y + 1.77200 * u;
    ret[i * 3 + 0] = (unsigned char) (std::min(std::max(r, 0), 255));
    ret[i * 3 + 1] = (unsigned char) (std::min(std::max(g, 0), 255));
    ret[i * 3 + 2] = (unsigned char) (std::min(std::max(b, 0), 255));

    y = ((unsigned char *) buffers[0].start)[(i + 1) * 2];

    r = y + 1.40200 * v;
    g = y - 0.71414 * v - 0.34414 * u;
    b = y + 1.77200 * u;
    ret[(i + 1) * 3 + 0] = (unsigned char) (std::min(std::max(r, 0), 255));
    ret[(i + 1) * 3 + 1] = (unsigned char) (std::min(std::max(g, 0), 255));
    ret[(i + 1) * 3 + 2] = (unsigned char) (std::min(std::max(b, 0), 255));
  }

  return true;
}

bool v4l_capture::read_frame(void)
{
  struct v4l2_buffer buf;

  memset (&(buf), 0, sizeof (buf));

  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;

  if (ioctl(fd, VIDIOC_DQBUF, &buf) == -1) {
    perror("VIDIOC_DQBUF");
    return false;
  }
  assert(buf.index < n_buffers);

  if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
    perror("VIDIOC_QBUF");
    return false;
  }

  return true;
}

bool v4l_capture::init_mmap(void)
{
  struct v4l2_requestbuffers req;

  memset (&(req), 0, sizeof (req));

  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
    perror("VIDIOC_REQBUFS");
    return false;
  }

  if (req.count < 2) {
    fprintf(stderr, "Insufficient buffer memory on %s\n", dev_name.c_str());
    return false;
  }

  buffers = (buffer*)calloc(req.count, sizeof(*buffers));

  if (!buffers) {
    fprintf(stderr, "Out of memory\n");
    return false;
  }

  for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
    struct v4l2_buffer buf;

    memset (&(buf), 0, sizeof (buf));

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffers;

    if (ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) {
      perror("VIDIOC_QUERYBUF");
      return false;
    }

    buffers[n_buffers].length = buf.length;
    buffers[n_buffers].start = mmap(NULL /* start anywhere */ ,
                                    buf.length, PROT_READ | PROT_WRITE
                                    /* required */ ,
                                    MAP_SHARED /* recommended */ ,
                                    fd, buf.m.offset);

    if (buffers[n_buffers].start == MAP_FAILED) {
      perror("mmap");
      return false;
    }
  }
  return true;
}

bool v4l_capture::uninit_mmap(void)
{
  unsigned int i;

  for (i = 0; i < n_buffers; ++i) {
    if (munmap(buffers[i].start, buffers[i].length) == -1) {
      perror("munmap");
      return false;
    }
  }
  return true;
}

bool v4l_capture::init_device(void)
{
  struct v4l2_capability cap;
  struct v4l2_format fmt;

  if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) {
    if (EINVAL == errno) {
      fprintf(stderr, "%s is no V4L2 device\n", dev_name.c_str());
    }
    perror("VIDIOC_QUERYCAP");
    return false;
  }

  fprintf(stderr, "video capabilities\n");
  fprintf(stderr, "cap.driver        =  %s\n", cap.driver);
  fprintf(stderr, "cap.card          =  %s\n", cap.card);
  fprintf(stderr, "cap.buf_info      =  %s\n", cap.bus_info);
  fprintf(stderr, "cap.version       =  %d\n", cap.version);
  fprintf(stderr, "cap.capabilities  =  0x%08x ", cap.capabilities);
  if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)
    fprintf(stderr, " VIDEO_CAPTURE");
  if (cap.capabilities & V4L2_CAP_VIDEO_OUTPUT)
    fprintf(stderr, " VIDEO_OUTPUT");
  if (cap.capabilities & V4L2_CAP_VIDEO_OVERLAY)
    fprintf(stderr, " VIDEO_OVERLAY");
  if (cap.capabilities & V4L2_CAP_VBI_CAPTURE)
    fprintf(stderr, " VBI_CAPTURE");
  if (cap.capabilities & V4L2_CAP_VBI_OUTPUT)
    fprintf(stderr, " VBI_OUTPUT");
#ifdef V4L2_CAP_SLICED_VBI_CAPTURE
  if (cap.capabilities & V4L2_CAP_SLICED_VBI_CAPTURE)
    fprintf(stderr, " SLICED_VBI_CAPTURE");
#endif
#ifdef V4L2_CAP_SLICED_VBI_OUTPUT
  if (cap.capabilities & V4L2_CAP_SLICED_VBI_OUTPUT)
    fprintf(stderr, " VBI_SLICED_OUTPUT");
#endif
  if (cap.capabilities & V4L2_CAP_RDS_CAPTURE)
    fprintf(stderr, " RDS_CAPTURE");
#if V4L2_CAP_VIDEO_OUTPUT_OVERLAY
  if (cap.capabilities & V4L2_CAP_VIDEO_OUTPUT_OVERLAY)
    fprintf(stderr, " VIDEO_OUTPUT_OVERLAY");
#endif
  if (cap.capabilities & V4L2_CAP_TUNER)
    fprintf(stderr, " TUNER");
    if (cap.capabilities & V4L2_CAP_AUDIO)
	fprintf(stderr, " AUDIO");
    if (cap.capabilities & V4L2_CAP_RADIO)
	fprintf(stderr, " RADIO");
    if (cap.capabilities & V4L2_CAP_READWRITE)
	fprintf(stderr, " READWRITE");
    if (cap.capabilities & V4L2_CAP_ASYNCIO)
	fprintf(stderr, " ASYNCIO");
    if (cap.capabilities & V4L2_CAP_STREAMING)
	fprintf(stderr, " STREAMING");
    fprintf(stderr, "\n");

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
      fprintf(stderr, "%s is no video capture device\n", dev_name.c_str());
      return false;
    }

    memset (&(fmt), 0, sizeof (fmt));

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
        perror("VIDIOC_S_FMT");
        return false;
    }

    init_mmap();
    return true;
}

void v4l_capture::uninit_device(void)
{
    uninit_mmap();
    free(buffers);
}

bool v4l_capture::start_capturing(void)
{
    unsigned int i;
    enum v4l2_buf_type type;

    for (i = 0; i < n_buffers; ++i) {
	struct v4l2_buffer buf;

        memset (&(buf), 0, sizeof (buf));

	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	buf.index = i;

	if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
	    perror("VIDIOC_QBUF");
            return false;
	}
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (ioctl(fd, VIDIOC_STREAMON, &type) == -1) {
	perror("VIDIOC_STREAMON");
        return false;
    }
    return true;
}

bool v4l_capture::stop_capturing(void)
{
    enum v4l2_buf_type type;

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (ioctl(fd, VIDIOC_STREAMOFF, &type) == -1) {
	perror("VIDIOC_STREAMOFF");
        return false;
    }
    return true;
}
