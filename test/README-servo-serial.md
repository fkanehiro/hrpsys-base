# ServoController serial regression

On Linux, with g++, pthreads, libutil and the `timeout` command:

```sh
sh test/run-servo-serial.sh /tmp/servo-serial-test
```

The output directory must not exist. This GNU C++98 test uses a pseudo terminal,
not a robot or physical serial port. It reproduces fragmented echo/return reads,
coalesced frames, old different-ID packets, missing replies and bad checksums.
Failed reads must leave the caller's output unchanged. The runner tests packet
logging OFF and ON and applies a three-second watchdog to each case.

This is not a physical bus-recovery or torque-OFF safety test. Identical old/new
replies cannot be distinguished by this protocol. No CI configuration is changed.
