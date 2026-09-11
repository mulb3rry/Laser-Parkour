# Shared protocol code

`include/laser_protocol.h` is the common C/C++ protocol definition used by the
controller and node firmware.

Run the native tests from this directory with:

```sh
pio test -e native
```

The tests verify exact register sizes, little-endian conversion, CRC behavior,
and the portable game engine's state transitions, scoring, abort,
timeout, and fault behavior.
