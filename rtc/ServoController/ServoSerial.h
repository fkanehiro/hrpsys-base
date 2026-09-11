#ifndef _SERVO_SERIAL_H_
#define _SERVO_SERIAL_H_

#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/select.h>
#include <time.h>
#include <pthread.h>
#include <vector>

//http://www.futaba.co.jp/dbps_data/_material_/localhost/robot/servo/manuals/RS301CR_RS302CD_114.pdf

class ServoSerial {
  enum { MAX_RETURN_BYTES = 8 + 255 };
  pthread_mutex_t mutex;
  bool mutex_ready;
  std::vector<unsigned char> input;

  // Getters hold this across send/echo/return. Recursive acquisition allows
  // the existing public packet helpers to use the same lock.
  class Guard {
    ServoSerial *serial;
  public:
    Guard(ServoSerial *value) : serial(NULL) {
      if (!value->mutex_ready) { errno = EIO; return; }
      int error = pthread_mutex_lock(&value->mutex);
      if (error) { errno = error; return; }
      serial = value;
    }
    ~Guard() {
      if (serial) pthread_mutex_unlock(&serial->mutex);
    }
    bool locked() const { return serial != NULL; }
  private:
    Guard(const Guard &);
    Guard &operator=(const Guard &);
  };
  ServoSerial(const ServoSerial &);
  ServoSerial &operator=(const ServoSerial &);

  static double nowSeconds() {
    struct timespec value;
    if (clock_gettime(CLOCK_MONOTONIC, &value) < 0) return -1;
    return value.tv_sec + value.tv_nsec * 1e-9;
  }

  // One deadline for the entire transfer, including interrupted/partial I/O.
  int transferBytes(void *buffer, int size, double deadline, bool writing,
                    int &done, bool read_some = false) {
    done = 0;
    while (done < size) {
      double now = nowSeconds();
      if (now < 0) return -1;
      double remaining = deadline - now;
      if (remaining <= 0) { errno = ETIMEDOUT; return -1; }
      if (fd < 0 || fd >= FD_SETSIZE) { errno = EBADF; return -1; }
      fd_set set;
      FD_ZERO(&set);
      FD_SET(fd, &set);
      struct timeval timeout;
      timeout.tv_sec = (long)remaining;
      timeout.tv_usec = (long)((remaining - timeout.tv_sec) * 1e6);
      int ready = select(fd + 1, writing ? NULL : &set,
                         writing ? &set : NULL, NULL, &timeout);
      if (ready < 0 && errno == EINTR) continue;
      if (ready < 0) return -1;
      if (!ready) { errno = ETIMEDOUT; return -1; }
      char *next = static_cast<char *>(buffer) + done;
      int count = writing ? write(fd, next, size - done) : read(fd, next, size - done);
      if (count < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;
      if (count <= 0) { if (!count) errno = EIO; return -1; }
      done += count;
      if (!writing && read_some) return done;
    }
    return done;
  }

  // Scan within one deadline. Wrong-ID frames are discarded whole; invalid
  // or incomplete headers must not hide a later complete valid frame.
  int readFrame(const unsigned char *echo, int echo_size, int id, int address,
                int length, unsigned char *result, double deadline) {
    const size_t capacity = echo_size > MAX_RETURN_BYTES ? echo_size : MAX_RETURN_BYTES;
    size_t received = 0;
    while (true) {
      double now = nowSeconds();
      if (now < 0) return -1;
      if (now >= deadline) { errno = ETIMEDOUT; return -1; }
      for (size_t start = 0; start + 8 <= input.size(); ++start) {
        const unsigned char *frame = &input[start];
        bool is_echo = frame[0] == 0xFA && frame[1] == 0xAF;
        bool is_return = frame[0] == 0xFD && frame[1] == 0xDF;
        if (!is_echo && !is_return) continue;
        size_t size = 8 + frame[5] * frame[6];
        if (size > capacity || (is_return && frame[6] != 1)) continue;
        if (start + size > input.size()) continue;
        unsigned char sum = 0;
        for (size_t i = 2; i + 1 < size; ++i) sum ^= frame[i];
        if (sum != frame[size - 1]) continue;
        bool match;
        if (echo) {
          match = is_echo && size == (size_t)echo_size && !memcmp(frame, echo, size);
        } else {
          match = is_return && frame[2] == id &&
            frame[4] == address && frame[5] == length;
        }
        if (match) memcpy(result, frame, size);
        input.erase(input.begin(), input.begin() + start + size);
        if (match) return size;
        start = (size_t)-1;
      }
      // Cap buffered memory and total garbage work independently of elapsed
      // time. These are parser limits, not a bus recovery/silence guarantee.
      if (input.size() > capacity)
        input.erase(input.begin(), input.end() - capacity);
      if (received >= 4 * capacity) { errno = EOVERFLOW; return -1; }
      unsigned char bytes[256];
      size_t remaining = 4 * capacity - received;
      int size = remaining < sizeof(bytes) ? remaining : sizeof(bytes);
      int done;
      int count = transferBytes(bytes, size, deadline, false, done, true);
      if (count < 0) return -1;
      received += count;
      input.insert(input.end(), bytes, bytes + count);
    }
  }

public:
  int fd;

  ServoSerial(const char *devname)  {
    fd = -1;
    mutex_ready = false;
    pthread_mutexattr_t attr;
    int error = pthread_mutexattr_init(&attr);
    if (!error) {
      error = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
      if (!error) error = pthread_mutex_init(&mutex, &attr);
      pthread_mutexattr_destroy(&attr);
    }
    if (error) { errno = error; return; }
    mutex_ready = true;
    fd = open(devname, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd<0) {
      char *pmesg = strerror(errno);
      fprintf (stderr, "[ServoSerial] failed to open %s: %s\n", devname, pmesg);
      return;
    }

    struct termios term;
    int res = tcgetattr(fd, &term);
    if (res<0) {
      char *pmesg = strerror(errno);
      fprintf (stderr, "[ServoSerial] failed to tcgetattr(): %s\n", pmesg);
      close(fd);
      fd = -1;
      return;
    }
    cfmakeraw(&term);
    res = cfsetospeed(&term, B115200);
    if (res == 0) res = cfsetispeed(&term, B115200);
    if (res<0) {
      char *pmesg = strerror(errno);
      fprintf (stderr, "[ServoSerial] failed to cfsetspeed(): %s\n", pmesg);
      close(fd);
      fd = -1;
      return;
    }
    term.c_iflag |= IGNPAR;            // Ignore characters with parity errors
    term.c_cflag |= (CLOCAL | CREAD);  // needed for QNX 6.3.2
    term.c_cflag &= ~PARENB;           // disable parity check
    term.c_cflag |= CS8;               // 8 data bit
    term.c_cflag &= ~CSTOPB;           // 1 stop bit
    term.c_lflag = IEXTEN;
    term.c_lflag &= ~(ECHO | ECHOCTL | ECHONL);  // disable ECHO 


    term.c_cc[VMIN] = 1;
    term.c_cc[VTIME] = 0;
#ifdef __QNX__
    term.c_cflag &= ~(IHFLOW | OHFLOW);
#endif
    res = tcsetattr(fd, TCSANOW, &term);
    if (res<0) {
      char *pmesg = strerror(errno);
      fprintf (stderr, "[ServoSerial] failed to tcsetattr(): %s\n", pmesg);
      close(fd);
      fd = -1;
      return;
    }

    // clear existing packet
    clear_packet();
  }

  ~ServoSerial()  {
      close(fd);
      if (mutex_ready) pthread_mutex_destroy(&mutex);
  }

  int setReset(int id) {
    return sendPacket(0xFAAF, id, 0x20, 0xFF, 0, 0, NULL) < 0 ? -1 : 0;
  }

  int setPosition(int id, double rad) {// #30
    signed short angle = (signed short)(180/M_PI*rad*10);
    printf("[ServoSerial] setPosition %f, %04x\n", 180/M_PI*rad, angle);
    unsigned char data[2] = {0xff & angle, 0xff & (angle>>8)};
    return sendPacket(0xFAAF, id, 0x00, 0x1E, 2, 1, data) < 0 ? -1 : 0;
  }

  int setPositions(int len, int *id, double *rad) {// #30
    unsigned char data[3*len];
    for (int i = 0; i < len; i++) {
      short angle = (int)(180/M_PI*rad[i]*10);
      printf("[ServoSerial] setPositions %d: %f, %04x\n", id[i], 180/M_PI*rad[i], angle);
      data[i*3 + 0] = id[i];
      data[i*3 + 1] = 0xff & angle;
      data[i*3 + 2] = 0xff & (angle>>8);
    }
    return sendPacket(0xFAAF, 0x00, 0x00, 0x1E, 3, len, data) < 0 ? -1 : 0;
  }

  int setPosition(int id, double rad, double sec) {// #32
    short angle = (short)(180/M_PI*rad*10);
    short msec = (short)(sec * 100);
    printf("[ServoSerial] setPosition %f %f, %04x, %04x\n", 180/M_PI*rad, sec, angle, msec);
    unsigned char data[4] = {0xff & angle,0xff & (angle>>8),
			     0xff & msec, 0xff & (msec>>8) };
    return sendPacket(0xFAAF, id, 0x00, 0x1E, 4, 1, data) < 0 ? -1 : 0;
  }

  int setPositions(int len, int *id, double *rad, double *sec) {// #32
    unsigned char data[5*len];
    for (int i = 0; i < len; i++) {
      short angle = (int)(180/M_PI*rad[i]*10);
      short msec = (short)(sec[i] * 100);
      printf("[ServoSerial] setPositions %d: %f %f, %04x, %04x\n", id[i], 180/M_PI*rad[i], sec[i], angle, msec);
      data[i*5 + 0] = id[i];
      data[i*5 + 1] = 0xff & angle;
      data[i*5 + 2] = 0xff & (angle>>8);
      data[i*5 + 3] = 0xff & msec;
      data[i*5 + 4] = 0xff & (msec>>8);
    }
    return sendPacket(0xFAAF, 0x00, 0x00, 0x1E, 5, len, data) < 0 ? -1 : 0;
  }

  int setMaxTorque(int id, short percentage) {// #35
    unsigned char data[1];
    data[0] = percentage;
    return sendPacket(0xFAAF, id, 0x00, 0x23, 1, 1, data) < 0 ? -1 : 0;
  }

  int setTorqueOn(int id) { // #36
    printf("[ServoSerial] setTorqueOn(%d)\n", id);
    unsigned char data[1] = {0x01};
    return sendPacket(0xFAAF, id, 0x00, 0x24, 1, 1, data) < 0 ? -1 : 0;
  }
  int setTorqueOff(int id) { // #36
    printf("[ServoSerial] setTorqueOff(%d)\n", id);
    unsigned char data[1] = {0x00};
    return sendPacket(0xFAAF, id, 0x00, 0x24, 1, 1, data) < 0 ? -1 : 0;
  }
  int setTorqueBreak(int id) { // #36
    unsigned char data[1] = {0x02};
    return sendPacket(0xFAAF, id, 0x00, 0x24, 1, 1, data) < 0 ? -1 : 0;
  }

  int getPosition(int id, double *angle) { // #42
    Guard lock(this);
    if (!lock.locked()) return -1;
    if (sendPacket(0xFAAF, id, 0x09, 0x00, 0, 1, NULL)<0) {
      clear_packet();
      return -1;
    }
    unsigned char data[0x12];
    if ( receivePacket(id, 0x2A, 0x12, data) < 0 ) {
      clear_packet();
      return -1;
    }
    *angle = ((short)(data[1]<<8|data[0]))/10.0;
    return 0;
  }

  int getDuration(int id, double *duration) { // #44
    Guard lock(this);
    if (!lock.locked()) return -1;
    if (sendPacket(0xFAAF, id, 0x09, 0x00, 0, 1, NULL)<0) {
      clear_packet();
      return -1;
    }
    unsigned char data[0x12];
    if ( receivePacket(id, 0x2A, 0x12, data) < 0 ) {
      clear_packet();
      return -1;
    }
    *duration = ((short)(data[3]<<8|data[2]))*10.0;
    return 0;
  }

  int getSpeed(int id, double *duration) { // #46
    Guard lock(this);
    if (!lock.locked()) return -1;
    if (sendPacket(0xFAAF, id, 0x09, 0x00, 0, 1, NULL)<0) {
      clear_packet();
      return -1;
    }
    unsigned char data[0x12];
    if ( receivePacket(id, 0x2A, 0x12, data) < 0 ) {
      clear_packet();
      return -1;
    }
    *duration = ((short)(data[5]<<8|data[4]));
    return 0;
  }

  int getMaxTorque(int id, short *percentage) {
    Guard lock(this);
    if (!lock.locked()) return -1;
    if (sendPacket(0xFAAF, id, 0x0B, 0x00, 0, 1, NULL)<0) {
      clear_packet();
      return -1;
    }
    unsigned char data[0x0C];
    if (receivePacket(id, 0x1E, 0x0C, data) < 0) {
      clear_packet();
      return -1;
    }
    *percentage = (short)(data[5]);
    return 0;
  }

  int getTorque(int id, double *torque) { // #48
    Guard lock(this);
    if (!lock.locked()) return -1;
    if (sendPacket(0xFAAF, id, 0x09, 0x00, 0, 1, NULL)<0) {
      clear_packet();
      return -1;
    }
    unsigned char data[0x12];
    if ( receivePacket(id, 0x2A, 0x12, data) < 0 ) {
      clear_packet();
      return -1;
    }
    *torque = ((short)(data[7]<<8|data[6]));
    return 0;
  }

  int getTemperature(int id, double *temperature) { // #50
    Guard lock(this);
    if (!lock.locked()) return -1;
    if (sendPacket(0xFAAF, id, 0x09, 0x00, 0, 1, NULL)<0) {
      clear_packet();
      return -1;
    }
    unsigned char data[0x12];
    if ( receivePacket(id, 0x2A, 0x12, data) < 0 ) {
      clear_packet();
      return -1;
    }
    *temperature = ((short)(data[9]<<8|data[8]));
    return 0;
  }

  int getVoltage(int id, double *voltage) { // #52
    Guard lock(this);
    if (!lock.locked()) return -1;
    if (sendPacket(0xFAAF, id, 0x09, 0x00, 0, 1, NULL)<0) {
      clear_packet();
      return -1;
    }
    unsigned char data[0x12];
    if ( receivePacket(id, 0x2A, 0x12, data) < 0 ) {
      clear_packet();
      return -1;
    }
    *voltage = ((short)(data[11]<<8|data[10]))/100;
    return 0;
  }

  int getState(int id, unsigned char *data) {
    Guard lock(this);
    if (!lock.locked()) return -1;
    if (sendPacket(0xFAAF, id, 0x05, 0x00, 0, 1, NULL)<0) {
      clear_packet();
      return -1;
    }
    if ( receivePacket(id, 0x1E, 30, data) < 0 ) {
      clear_packet();
      return -1;
    }
    return 0;
  }

  int receivePacket(int id, int address, int length, unsigned char data[]){
    Guard lock(this);
    if (!lock.locked()) return -1;
    if (length < 0 || length > 255 || !data) { errno = EINVAL; return -1; }
    double now = nowSeconds();
    if (now < 0) return -1;
    const double deadline = now + 0.2;
    unsigned char packet[MAX_RETURN_BYTES];
    if (readFrame(NULL, 0, id, address, length, packet, deadline) < 0) return -1;
    // Do not copy unvalidated bytes into the caller's output buffer.
    const unsigned char *payload = packet + 7;
    unsigned char flags = packet[3];
    int ret = 1;

#ifdef SERVO_SERIAL_DEBUG
    const unsigned char *prefix = packet;
    unsigned char ids, addr, len, count, sum;
    unsigned char s = 0;
    fprintf(stderr, "[ServoSerial] received: ");
    printf("%02X %02X ", prefix[0], prefix[1]);
    ids = prefix[2]; s ^= ids;
    printf("%02X " , ids); fflush(stdout);
    flags = prefix[3]; s ^= flags;
    printf("%02X ", flags); fflush(stdout);
    addr = prefix[4]; s ^= addr;
    printf("%02X ", addr); fflush(stdout);
    len = prefix[5]; s ^= len;
    printf("%02X ", len); fflush(stdout);
    count = prefix[6]; s ^= count;
    printf("%02X ", count); fflush(stdout);
    for(int i = 0; i < length; i++){
      s ^= payload[i];
      printf("%02X ", payload[i]); fflush(stdout);
    }
    sum = payload[length];
    printf("%02X - %02X\n", sum, s); fflush(stdout);
#endif

    if ( flags & 0x0002 ) { // 0b00000010
      fprintf(stderr, "[ServoSerial] Failed to receive packet from servo(id:%d) Fail to process received packet\n", id);
      ret = -1;
    }

    if ( flags & 0x0008 ) { // 0b00001000
      fprintf(stderr, "[ServoSerial] Failed to receive packet from servo(id:%d) fail to write Flash ROM\n", id);
      ret = -1;
    }

    if ( flags & 0x0020 ) { // 0b00100000
      fprintf(stderr, "[ServoSerial] Failed to receive packet from servo(id:%d) temperature limit warning\n", id);
      ret = -1;
    }

    if ( flags & 0x0080 ) { // 0b10000000
      fprintf(stderr, "[ServoSerial] Failed to receive packet from servo(id:%d) Temperature limit error\n", id);
      ret = -1;
    }

    if (ret > 0) memcpy(data, payload, length);
    else errno = EIO;
    return ret;
  }

  int sendPacket(int header, int id,
		 int flag,   int address,
		 int length, int count,
		 void *data){

    Guard lock(this);
    if (!lock.locked()) return -1;
    if (length < 0 || length > 255 || count < 0 || count > 255 ||
        (length * count && !data)) { errno = EINVAL; return -1; }

    // Discard only our previous transaction's buffered suffix. Bytes still
    // arriving on the port are framed below; this does not prove freshness.
    input.clear();

    unsigned char c, sum = 0x00, packet[8+length*count];
    c = 0xff & (header>>8); packet[0] = c;
    c = 0xff & header;      packet[1] = c;
    c = id;      packet[2] = c;
    c = flag;    packet[3] = c;
    c = address; packet[4] = c;
    c = length;  packet[5] = c;
    c = count;   packet[6] = c;
    if ( length * count > 0 ) {
      memcpy((void *)(&(packet[7])), (void *)data, length*count);
    }
    for(int i = 2; i < 7 + length*count; i++){
      sum ^= packet[i];
    }
    packet[7+length*count] = sum;

#ifdef SERVO_SERIAL_DEBUG
    fprintf (stderr, "[ServoSerial] sending : ");
    for(int i = 0; i < 7 + length*count + 1; i++){
      fprintf(stderr, "%02X ", packet[i]);
    }
    fprintf(stderr, " - ");
#endif

    int ret1;
    double now = nowSeconds();
    if (now < 0) return -1;
    int written;
    ret1 = transferBytes(packet, 8+length*count, now + 0.2, true, written);
    const int write_error = errno;

#ifdef SERVO_SERIAL_DEBUG
    fprintf(stderr, "%d\n", ret1);
#endif

    if (ret1 != 8+length*count) {
        // These are bytes accepted by write(), not confirmed physical TX.
        // Never restart the packet or flush output after a partial write.
        fprintf(stderr, "[ServoSerial] Failed to send packet to servo(id:%d): %d/%d bytes queued, error:%d\n",
                id, written, 8+length*count, write_error);
	errno = write_error;
	return -1;
    }

    unsigned char echo[8 + length*count];
    int ret2;

    // wait at most 200 msec for the complete echo
    now = nowSeconds();
    if (now < 0) return -1;
    ret2 = readFrame(packet, 8+length*count, id, address, length, echo, now + 0.2);
    const int read_error = errno;

    
#ifdef SERVO_SERIAL_DEBUG
    fprintf(stderr, "[ServoSerial] received: ");
    for(int i = 0; i < ret2; i++){
      fprintf(stderr, "%02X ", echo[i]);
    }
    fprintf(stderr, " - %d\n", ret2);
#endif
    if (ret2 != ret1) {
      fprintf(stderr, "[ServoSerial] Failed to receive packet from servo (id:%d)\n", id);
      errno = read_error;
      clear_packet();
      return -1;
    }
    
    return ret1;
  }

  void clear_packet() {
    Guard lock(this);
    if (!lock.locked()) return;
    const int saved_errno = errno;
    // Discard queued input only; this is not a remote-parser reset and does
    // not prevent a delayed response from arriving after this call.
    input.clear();
    if (fd >= 0 && tcflush(fd, TCIFLUSH) < 0)
      fprintf(stderr, "[ServoSerial] Failed to clear input: %s\n", strerror(errno));
    errno = saved_errno;
  }
};

#endif //_SERVO_SERIAL_H_
