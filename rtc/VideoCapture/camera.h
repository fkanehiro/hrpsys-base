
/* Most of capture.cpp and capture.h are copied from http://jsk-enshu.svn.sourceforge.net/viewvc/jsk-enshu/trunk/keisanki/2009/ */
#include <opencv2/core/core.hpp>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>

/* v4l2 capture class */
class v4l_capture
{
  typedef struct _buffer {
    void *start;
    size_t length;
  } buffer;
  cv::Mat frame;
  std::string dev_name;
  int fd, width, height;
  buffer *buffers;
  unsigned int n_buffers;
  bool open_device();
  bool init_device();
  bool init_mmap();
  bool start_capturing();
  bool stop_capturing();
  void uninit_device();
  bool uninit_mmap();
  bool close_device();
  bool read_frame(void);
  bool write_img(uchar * ret);
  bool init_all(size_t _width, size_t _height, unsigned int _devId);
 public:
  v4l_capture();
  ~v4l_capture();
  uchar *capture ();
  int getHeight ();
  int getWidth ();
  int init (size_t _width, size_t _height, unsigned int devId);
};
