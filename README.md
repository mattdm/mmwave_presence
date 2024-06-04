This is a simple library to interact with mmwave radar "human presence detection"
devices, like the LD2410. (Currently, the only thing supported, but I'm intending
this to be generalizable to other sensors which work in a similar way.)

It's intended to be simple and straightforward, and particularly meant for use
with CircuitPython on microcontrollers. Therefore, it expects to have a serial-port-like
object passed in which implements read(number of bytes) and write(buffer of bytes).

Huge Caveat!
------------

I haven't actually tested this with CircuitPython on a microcontroller yet.
But I wanted to share without more delay.

Stability?
----------

No, not yet.

Particularly, I should look a bit more at raising exceptions vs. return values,
and handling errors.

Some of the methods don't really need to be public -- even though they represent
commands, the way I've set this up, `get_firmware()` or `get_resolution()` happen
behind the scenes.

Still... I think this is a pretty usable start.


Completeness?
-------------

I've implemented all of the documented commands and data-gathering for the LD2410,
as of firmware v2.04.something, except for those for bluetooth connections.

This doesn't currently support hardware other than the LD2410, and there may
be some undocumented features not covered.

Bug: setting the last gates doesn't work. However, setting gate sensitivity to 100
is a more flexible approach anyway, and that does work.

There are many TODOs and a few FIXMEs in the file. Will try to eliminate them eventually.

Debugging is mostly through (mostly commented-out) print statements. I didn't want to 
pull in a logging dependency, but maybe there could be something better.


Installation
------------

For Adafruit devices with CircuitPython, the best way is to compile `mmwave_presence.py` with [`mpy-cross`](https://learn.adafruit.com/welcome-to-circuitpython/frequently-asked-questions#faq-3105290), and
then put that in the `/lib/` folder on your device.

Basic Usage
-----------

```python
import serial
import mmwave_presence as mmwave

port = serial.Serial("/dev/ttyUSB0", 57600, timeout=0.1)

# This 
mmwave = mmwave.MMWave(port)

# this is not really meant for anything but debugging...
# but does show the attributes you can reference

print(f"{mmwave}")

```

Random Notes
------------

* What's up with "Engineering Mode"? It seems like the idea is for this
  to be used to find the optimal gate thresholds for some particular
  application, and then to use basic mode in production with only an
  "on/off" kind of response. But, even with a basic microcontroller, you
  can get the individual gate info from "engineering mode" and do more
  interesting things like integration over time, self-learning thresholds,
  etc. There seems to be no real disadvantage (unless you're trying to read
  _really fast_).

* Therefore, engineering mode is enabled by default, and will be automatically
  re-enabled on every read unless `engineering_always` is set to False.

* This module does _not_ attempt to handle serial port exceptions. Catch
  those in the main program (or other modules using this one).

* The documentation is _really_ arbitrary with English-language terms, for 
  example using "gate" and "door" interchangably — or "engineering mode" and
  "project mode". I've tried to pick one and be consistent even when this
  contradicts the docs in places.

* Similarly: all distance units in this library are in centimeters

Compared to LD2410 from PiPI (https://pypi.org/project/LD2410/)
----

* That uses the logging module, which is not so handy under CircuitPython

* That reads constantly in a background thread. That's not always an option
  on little microcontrollers, so that's left to be higher level

* This module is entirely self-contained, with no dependencies outside of the core
  CircuitPython libraries

* That module returns results in a tuple of lists, which you need to understand.
  This module returns an object with named attributes.

* This module's public methods are the high-level commands like `enable_bluetooth()`

* This module does more validation of returned packets, because flaky serial
  lines are real.

* A matter of opinion, but I think this version handles engineering mode more elegantly

* I've _tried_ add comments which explain everything that's going on. Open to 
  improvements (either through questions/complaints or patches)
