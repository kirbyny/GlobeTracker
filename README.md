# GlobeTracker
Arduino code to read a PS2 optical mouse and transform the movements to lat/long.

This is part of a larger personal project that needed a method of entering latitude/longitude coordinates.

The test hardware was an Arduino Nano ESP32, an HP PS2 optical mouse, and a classroom-style globe of the earth.

Set the globe radius constant as accurately as possible (in inches).

Moving the mouse over the surface of the globe *should* output the lat/long of the mouse on the globe surface.

My application placed the mouse inverted, in a fixed platform.  The globe sits on top.  The globe is rotated to place the desired point on the exact opposite side of the globe as the mouse.

Not all of this code is mine.  Specifically, the PS2 library.

Note: Be mindful of voltage levels.  PS2 is 5V and may not work properly on 3.3V devices.
