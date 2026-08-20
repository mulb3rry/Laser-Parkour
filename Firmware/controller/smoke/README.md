# Controller hardware smoke test

Build and upload with the PlatformIO environment `smoke-controller`.

The test passes when:

- the serial output reports that the Wi-Fi access point started;
- a client can join `Laser-Parkour` and receives an IP address;
- the RGB LED shows red, green, and blue in that order;
- the speaker produces three distinguishable event sounds;
- the LCD displays the encoder count and access-point IP address;
- each encoder detent changes the count by one in the expected direction; and
- pressing the encoder resets the displayed count.

Any missing, unstable, or incorrect response is a failure.
