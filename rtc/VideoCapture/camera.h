
/* Most of capture.cpp and capture.h are copied from http://jsk-enshu.svn.sourceforge.net/viewvc/jsk-enshu/trunk/keisanki/2009/ */
#include <opencv/cv.h>
#include <opencv/highgui.h>
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
  void init_device();
  void init_mmap();
  bool start_capturing();
  void stop_capturing();
  void uninit_device();
  void uninit_mmap();
  void close_device();
  void read_frame(void);
  void write_img(uchar * ret);
  bool init_all(size_t _width, size_t _height, unsigned int _devId);
 public:
  v4l_capture();
  ~v4l_capture();
  uchar *capture ();
  int getHeight ();
  int getWidth ();
  int init (size_t _width, size_t _height, unsigned int devId);
};
