// Linux PTY regression: no robot, ROS graph or physical serial device.
#include <assert.h>
#include <pty.h>
#include <pthread.h>
#include <stdlib.h>
#include "ServoSerial.h"

static void exact(int fd, unsigned char *data, int size, bool writing) {
  while (size > 0) {
    int n = writing ? write(fd, data, size) : read(fd, data, size);
    if (n < 0 && errno == EINTR) continue;
    assert(n > 0);
    data += n;
    size -= n;
  }
}

static void checksum(unsigned char *packet, int size) {
  packet[size - 1] = 0;
  for (int i = 2; i < size - 1; ++i) packet[size - 1] ^= packet[i];
}

struct Peer { int fd; const char *mode; int cut; };

static void *reply(void *value) {
  Peer &peer = *static_cast<Peer *>(value);
  unsigned char echo[8];
  exact(peer.fd, echo, 8, false);
  assert(echo[2] == 3);
  if (!strcmp(peer.mode, "missing-echo")) return NULL;
  unsigned char packet[26] = {0xFD, 0xDF, 3, 0, 0x2A, 18, 1, 123};
  checksum(packet, 26);
  if (!strcmp(peer.mode, "stale")) {
    unsigned char old_echo[8];
    memcpy(old_echo, echo, 8);
    old_echo[2] = 8;
    checksum(old_echo, 8);
    packet[2] = 8;
    checksum(packet, 26);
    exact(peer.fd, old_echo, 8, true);
    exact(peer.fd, packet, 26, true);
    packet[2] = 3;
    checksum(packet, 26);
  }
  if (!strcmp(peer.mode, "coalesced")) {
    unsigned char both[34];
    memcpy(both, echo, 8);
    memcpy(both + 8, packet, 26);
    exact(peer.fd, both, 34, true);
    return NULL;
  }
  // Reproduce the observed three-byte echo read.
  exact(peer.fd, echo, 3, true);
  usleep(10000);
  exact(peer.fd, echo + 3, 5, true);
  if (!strcmp(peer.mode, "missing-return")) return NULL;
  if (!strcmp(peer.mode, "checksum")) packet[25] ^= 1;
  exact(peer.fd, packet, peer.cut, true);
  usleep(10000);
  exact(peer.fd, packet + peer.cut, 26 - peer.cut, true);
  return NULL;
}

int main(int argc, char **argv) {
  assert(argc >= 2);
  const char *mode = argv[1];
  bool valid = !strcmp(mode, "fragment") || !strcmp(mode, "stale") ||
               !strcmp(mode, "coalesced");
  assert(valid || !strcmp(mode, "missing-echo") ||
         !strcmp(mode, "missing-return") || !strcmp(mode, "checksum"));
  int master, slave;
  char path[128];
  assert(openpty(&master, &slave, path, NULL, NULL) == 0);
  ServoSerial serial(path);
  assert(serial.fd >= 0);
  close(slave);
  Peer peer = {master, mode, argc == 3 ? atoi(argv[2]) : 26};
  assert(peer.cut >= 0 && peer.cut <= 26);
  pthread_t thread;
  assert(pthread_create(&thread, NULL, reply, &peer) == 0);
  double angle = -99;
  int result = serial.getPosition(3, &angle);
  assert((result == 0) == valid);
  assert(angle == (valid ? 12.3 : -99));
  assert(pthread_join(thread, NULL) == 0);
  close(master);
  return 0;
}
