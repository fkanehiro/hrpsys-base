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

//http://www.futaba.co.jp/dbps_data/_material_/localhost/robot/servo/manuals/RS301CR_RS302CD_114.pdf

#define cfsetspeed(term, baudrate)		\
  cfsetispeed(term, baudrate);			\
  cfsetospeed(term, baudrate);


class ServoSerial {
public:
  int fd;

  ServoSerial(const char *devname)  {
    fd = open(devname, O_RDWR);
    if (fd<0) {
      char *pmesg = strerror(errno);
      fprintf (stderr, "[ServoSerial] failed to open %s: %s\n", devname, pmesg);
    }

    struct termios term;
    int res = tcgetattr(fd, &term);
    if (res<0) {
      char *pmesg = strerror(errno);
      fprintf (stderr, "[ServoSerial] failed to tcgetattr(): %s\n", pmesg);
    }
    cfmakeraw(&term);
    res = cfsetspeed(&term, 115200);
    if (res<0) {
      char *pmesg = strerror(errno);
      fprintf (stderr, "[ServoSerial] failed to cfsetspeed(): %s\n", pmesg);
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
    }

    // clear existing packet
    clear_packet();
  }

  ~ServoSerial()  {
      close(fd);
  }

  int setReset(int id) {
    sendPacket(0xFAAF, id, 0x20, 0xFF, 0, 0, NULL);
  }

  int setPosition(int id, double rad) {// #30
    signed short angle = (signed short)(180/M_PI*rad*10);
    printf("[ServoSerial] setPosition %f, %04x\n", 180/M_PI*rad, angle);
    unsigned char data[2] = {0xff & angle, 0xff & (angle>>8)};
    sendPacket(0xFAAF, id, 0x00, 0x1E, 2, 1, data);
    return 0;
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
    sendPacket(0xFAAF, 0x00, 0x00, 0x1E, 3, len, data);
    return 0;
  }

  int setPosition(int id, double rad, double sec) {// #32
    short angle = (short)(180/M_PI*rad*10);
    short msec = (short)(sec * 100);
    printf("[ServoSerial] setPosition %f %f, %04x, %04x\n", 180/M_PI*rad, sec, angle, msec);
    unsigned char data[4] = {0xff & angle,0xff & (angle>>8),
			     0xff & msec, 0xff & (msec>>8) };
    sendPacket(0xFAAF, id, 0x00, 0x1E, 4, 1, data);
    return 0;
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
    sendPacket(0xFAAF, 0x00, 0x00, 0x1E, 5, len, data);
    return 0;
  }

  int setMaxTorque(int id, short percentage) {// #35
    unsigned char data[1];
    data[0] = percentage;
    sendPacket(0xFAAF, id, 0x00, 0x23, 1, 1, data);
    return 0;
  }

  int setTorqueOn(int id) { // #36
    printf("[ServoSerial] setTorqueOn(%d)\n", id);
    unsigned char data[1] = {0x01};
    sendPacket(0xFAAF, id, 0x00, 0x24, 1, 1, data);
    return 0;
  }
  int setTorqueOff(int id) { // #36
    printf("[ServoSerial] setTorqueOff(%d)\n", id);
    unsigned char data[1] = {0x00};
    sendPacket(0xFAAF, id, 0x00, 0x24, 1, 1, data);
    return 0;
  }
  int setTorqueBreak(int id) { // #36
    unsigned char data[1] = {0x02};
    sendPacket(0xFAAF, id, 0x00, 0x24, 1, 1, data);
    return 0;
  }

  int getPosition(int id, double *angle) { // #42
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
    unsigned short header;
    unsigned char ids, flags, addr, len, count, sum;
    unsigned char s = 0;
    int ret;

    fprintf(stderr, "[ServoSerial] received: ");
    read(fd, &header, 2);
    printf("%02X ", header>>8); 
    printf("%02X ", 0xff&header); fflush(stdout);
    read(fd, &ids, 1);	s ^= ids;
    printf("%02X " , ids); fflush(stdout);
    read(fd, &flags, 1);s ^= flags;
    printf("%02X ", flags); fflush(stdout);
    read(fd, &addr, 1);	s ^= addr;
    printf("%02X ", addr); fflush(stdout);
    read(fd, &len, 1);	s ^= len;
    printf("%02X ", len); fflush(stdout);
    read(fd, &count, 1);s ^= count;
    printf("%02X ", count); fflush(stdout);
    read(fd, data, length);
    for(int i = 0; i < length; i++){
      s ^= data[i];
      printf("%02X ", data[i]); fflush(stdout);
    }
    ret = read(fd, &sum, 1);
    printf("%02X - %02X\n", sum, s); fflush(stdout);

    if ( address != addr || length != len || sum != s ) {
      fprintf(stderr, "[ServoSerial] Failed to receive packet from servo(id:%d)\n", id);
      ret = -1;
    }

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

    return ret;
  }

  int sendPacket(int header, int id,
		 int flag,   int address,
		 int length, int count,
		 void *data){

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

    fprintf (stderr, "[ServoSerial] sending : ");
    for(int i = 0; i < 7 + length*count + 1; i++){
      fprintf(stderr, "%02X ", packet[i]);
    }
    fprintf(stderr, " - ");

    int ret1;
    ret1 = write(fd, packet, 8+length*count);

    fprintf(stderr, "%d\n", ret1);

    if (ret1 != 8+length*count) {
        fprintf(stderr, "[ServoSerial] Failed to send packet to servo(id:%d)\n", id);
	return -1;
    }

    unsigned char echo[8 + length*count];
    int ret2;

    // wait at most 200 msec
    fd_set set;
    struct timeval timeout;
    FD_ZERO(&set); /* clear the set */
    FD_SET(fd, &set); /* add our file descriptor to the set */
    timeout.tv_sec = 0;
    timeout.tv_usec = 200*1000;
    select(fd + 1, &set, NULL, NULL, &timeout);
    ret2 = read(fd, &echo, 8+length*count);

    
    fprintf(stderr, "[ServoSerial] received: ");
    for(int i = 0; i < ret2; i++){
      fprintf(stderr, "%02X ", echo[i]);
    }
    fprintf(stderr, " - %d\n", ret2);
    if (ret2 != ret1) {
      fprintf(stderr, "[ServoSerial] Failed to receive packet from servo (id:%d)\n", id);
      clear_packet();
      return -1;
    }
    
    for(int i = 0; i < 8 + length*count; i++){
      if (echo[i] != packet[i]) {
	fprintf(stderr, "[ServoSerial] Failed to confirm packet from servo(id:%d)\n", id);
	clear_packet();
	ret1 = -1;
      }
    }

    return ret1;
  }

  void clear_packet() {
    // clear existing packet
    int oldf = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, oldf | O_NONBLOCK);
    unsigned char c;
    while ( read(fd, &c, 1) != EOF );
    fcntl(fd, F_SETFL, oldf);
  }
};

#endif //_SERVO_SERIAL_H_
