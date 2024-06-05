#!/usr/bin/python
"""

This is a simple library to interact with mmwave radar "human presence detection"
devices, like the LD2410. (Currently, the only thing supported, but I'm intending
this to be generalizable to other sensors which work in a similar way.)

It's intended to be simple and straightforward, and particularly meant for use
with CircuitPython on microcontrollers. Therefore, it expects to have a serial-port-like
object passed in which implements read(number of bytes) and write(buffer of bytes).

See README.md for more!
"""

import time

DEFAULT_BAUD = 256000
"""This is the LD2410's default"""


# Constants used in the protocol
# 
# All of these are from the Hi-Link docs which can be found at
# https://www.hlktech.net/ (and then a Google drive link)
#
# There are also some packet details interspersed in the code.
# I don't see high value in naming those separately, but, uh,
# if someone wants to refactor in that way, sure.
# 
REPORT_MODE_ENGINEERING = 1
REPORT_MODE_BASIC = 2

PACKET_LEN_ENGINEERING = 35
PACKET_LEN_BASIC = 13

# These help find the data packets in the byte-stream,
# and also provide rudimentary validation. Note that
# despite the documented name "calibration" this value
# is documented as always zero, so I dunno what that is
# all about. Wouldn't a checksum be better? Oh well!
FRAME_HEADER = bytes.fromhex("f4f3f2f1")
FRAME_FOOTER = bytes.fromhex("f8f7f6f5")
PACKET_HEAD = bytes.fromhex("aa")
PACKET_TAIL = bytes.fromhex("55")
PACKET_CALIBRATION = bytes.fromhex("00")

MOTION_STATE_MASK = 1
STATIC_STATE_MASK = 2

MAX_LEGIT_DISTANCE = 75 * 9 # default gate distance is 75cm, and there are 9 gates.
MAX_LEGIT_ENERGY = 100 # From the docs. Therefore, any higher values are misreads.
MAX_LEGIT_RESPONSE_LENGTH = 28 # see _send_command()
MAX_LEGIT_PRESENCE_TIMEOUT = 65535 # docs 2.2.3


# right now, that's all the LD2410 has. maybe future devices will have more?
LAST_GATE = 8

# From the docs section 2.2. Question: is it more clear to have these all
# in one place, or defined in each command method?
COMMAND_HEADER = bytes.fromhex("fdfcfbfa")
COMMAND_FOOTER = bytes.fromhex("04030201")

COMMAND_CONFIG_ENABLE    = (0x00ff).to_bytes(2,"little")
CONFIG_PROTOCOL_VERSION  = (0x0001).to_bytes(2,"little")
COMMAND_CONFIG_DISABLE   = (0x00fe).to_bytes(2,"little")

COMMAND_BASIC_CONFIG     = (0x0060).to_bytes(2,"little")
COMMAND_READ_CONFIG      = (0x0061).to_bytes(2,"little")
COMMAND_ENG_MODE_ENABLE  = (0x0062).to_bytes(2,"little")
COMMAND_ENG_MODE_DISABLE = (0x0063).to_bytes(2,"little")
COMMAND_GATE_SENSITIVITY = (0x0064).to_bytes(2,"little")
COMMAND_FIRMWARE_VERSION = (0x00A0).to_bytes(2,"little")
COMMAND_BAUD             = (0x00A1).to_bytes(2,"little")
COMMAND_RESET_CONFIG     = (0x00A2).to_bytes(2,"little")
COMMAND_RESTART          = (0x00A3).to_bytes(2,"little")
COMMAND_BLUETOOTH        = (0x00A4).to_bytes(2,"little")
COMMAND_MAC_ADDR         = (0x00A5).to_bytes(2,"little")
COMMAND_BLUETOOTH_PERM   = (0x00A8).to_bytes(2,"little") # not useful over serial
COMMAND_BLUETOOTH_PASS   = (0x00A9).to_bytes(2,"little")
COMMAND_SET_RESOLUTION   = (0x00AA).to_bytes(2,"little")
COMMAND_GET_RESOLUTION   = (0x00AB).to_bytes(2,"little")



class MMWave():

    """TODO: write something helpful here!
    """

    def __init__(self,port,initialize=True,engineering_always=True):
        """Create a MMWave object which communicates over `port`.

        This will also do an initial read of the configuration of the sensor,
        followed by an initial read of sensor values.

        Arguments:

        port -- should be a serial port-like object implementing .read(bytes) and .write(buffer)
        engineering_always -- switch to engineering mode if we end up not in it. Defaults to True.
        
        """        

        self.port = port
        """Serial port."""

        # TODO: should we, like, do something if this reaches some threshold?
        self.serial_failures = 0
        """A count of bad packets, basically."""

        self.engineering_always = engineering_always

        self.engineering_mode = False
        """In "engineering mode", which returns per-gate energy levels."""
        
        self.detected = False
        """At least one sensitivity threshold is exceeded for at least one gate."""

        self.motion_detected = False
        """The motion sensitivity threshold is exceeded for at least one gate."""

        self.static_detected = False
        """The static sensitivity threshold is exceeded for at least one gate."""

        self.motion_target_cm = None
        """Estimated distance to motion target. It is unclear how exactly how
        the sensor derives this. Is it from gate energy, or separate? It's a mystery!
        """

        self.static_target_cm = None
        """Estimated distance to static target. It is unclear how exactly how
        the sensor derives this. Is it from gate energy, or separate? It's a mystery!
        """

        self.detection_cm = None
        """Estimated distance to any target. It is unclear how exactly how
        the sensor derives this. Same mystery as the others, plus it's unclear
        how this relates to those.
        """

        self.motion_energy = None
        """Empirically, this is the motion energy level at the gate with the highest value."""

        self.static_energy = None
        """Empirically, this is the static energy level at the gate with the highest value."""


        self.last_motion_gate = None
        """Motion gates higher than this are ignored. A less-flexible option than
        setting them to sensitivity 100."""

        self.last_static_gate = None
        """Motion gates higher than this are ignored. A less-flexible option than
        setting them to sensitivity 100."""

        self.gate_motion_energy = [None] * 9
        """Energy values for each gate, for moving targets. Range of 0-100.

        Only reported in "engineering mode".
        
        By default, these represent sections of space in 75cm increments
        from the sensor. There is a fine-resolution mode which changes that to 20cm.
        """

        self.gate_static_energy = [None] * 9 
        """Energy values for each gate, for static targets. Range of 0-100.

        Only reported in "engineering mode".

        By default, these represent sections of space in 75cm increments
        from the sensor. There is a fine-resolution mode which changes that to 20cm.
        """

        self.gate_motion_sensitivity = [None] * 9
        """Threshold values for motion detection at each gate, from 1-100. If any
        threshold is exceeded, the basic mode report will report that motion is
        detected.
        """

        self.gate_static_sensitivity = [None] * 9 
        """Threshold values for motion detection at each gate, from 1-100. If any
        threshold is exceeded, the basic mode report will report that static presence
        is detected."""

        self.light_level = None
        """There's a light sensor in here. It's not well-documented. There might
        be a command which makes it factor into the detection threshold, which,
        to be honest, sounds horrible."""
        
        self.presence_timeout = None
        """The basic report will continue to report motion or static presence even
        after the gates fall below the thresholds for at least this long. This can
        be used to avoid bouncing between on/off when detection is right on the edge.
        It doesn't really matter if you're just reading the gate values.        
        """

        self.resolution = None
        """This defines the depth each of the 9 gates (0-8). By default, this is 75cm.
        The other option is 20cm, which gives up distance (over 6m) in exchange for more
        fine control over detection areas (within about 1.8m)
        """

        self.last_updated = None
        """Timestamp of last successful read. This is **NOT** from the sensor itself."""

        if initialize:
            self.get_firmware_version()
            self.get_resolution()
            self.read_config()
            self.read()


    def __str__(self):
        """This is meant for debugging convenience, not really meant to be used normally."""
        s = ""
        for name in dir(self):
            if name[0]=="_":
                continue
            try: 
                val = getattr(self, name)
                if callable(val):
                    continue
                s += f"{name}: {val}\n"
            except:
                pass
        return s

                
    # TODO: timestamp!
    def read(self):
        """Update object attributes with latest data from the sensor.
        Returns True on success, False otherwise. Also, if it fails,
        data attributes will be set to None.
        """
        for failure_count in range(100):

            if not self.engineering_mode and self.engineering_always:
                self.enable_engineering_mode()
            
            if not self._scan_for_header(FRAME_HEADER):
                continue

            # determine report mode

            packet_len = int.from_bytes(self.port.read(2), byteorder='little')
            report_mode = int.from_bytes(self.port.read(1), byteorder='little')

            # various mangled-read checks
            if packet_len == PACKET_LEN_BASIC:
                #print("Basic mode length detected")
                if report_mode != REPORT_MODE_BASIC:
                    #print("But not in basic mode!")
                    continue
                self.engineering_mode = False
            elif packet_len == PACKET_LEN_ENGINEERING:
                #print("Engineering mode length detected")
                if report_mode != REPORT_MODE_ENGINEERING:
                    #print("But not in engineering mode!")
                    continue
                self.engineering_mode = True
            else:
                #print(f"Bad packet length {packet_len}.")
                continue

            # since we already read a byte to check the mode,
            # our packet is one shorter than nominal. But, we
            # also want to check the last for the EOF
            # marker
            packet = self.port.read(packet_len-1)
            footer = self.port.read(4)


            if footer != FRAME_FOOTER:
                #print("End-of_Frame marker doesn't match.")
                continue

            if packet[0].to_bytes() != PACKET_HEAD:
                #print(f"Packet head {packet[0]} isn't right.")
                continue

            target_state = packet[1]
            if target_state > 3:
                #print(f"Detection status invalid.")
                continue

            #print(packet.hex())

            # fill out our values from the basic part of the 
            # packet. This is:
            # byte 1: 0 nothing, 1 motion, 2 static, 3 both (based on gate sensitivity config)
            # byte 2: movement target distance in cm
            # byte 3: (con't)
            # byte 4: motion target energy
            # byte 5: static target distance in cm
            # byte 6: (con't)
            # byte 7: static target energy
            # byte 8: overall detection distance in cm
            # byte 9: (con't)
            # TODO: I'm not getting sensible values out of the
            # overall detection distance. check why not.
            # 
            # byte -2: "tail" value of 55 
            # byte -1: "calibration" value of 00

            if packet[-2].to_bytes() != PACKET_TAIL:
            #print(f"Invalid packet tail value.")
                continue

            if packet[-1].to_bytes() != PACKET_CALIBRATION:
                #print(f"Invalid packet calibration value.")
                continue

            self.detected = bool(target_state)
            self.motion_detected = bool(target_state & MOTION_STATE_MASK)
            self.static_detected = bool(target_state & STATIC_STATE_MASK)

            self.motion_target_cm = None
            self.static_target_cm = None  
            self.motion_energy = None
            self.static_energy = None
            self.detection_cm = None

            # todo: use lower of MAX_LEGIT_DISTANCE and distance resolution * gate limit

            if self.motion_detected:
                self.motion_target_cm = int.from_bytes(packet[2:4], byteorder='little')
                self.motion_energy = int(packet[4])
                if self.motion_target_cm > MAX_LEGIT_DISTANCE:
                    continue
                if self.motion_energy > MAX_LEGIT_ENERGY:
                    continue

            if self.static_detected:
                self.static_target_cm = int.from_bytes(packet[5:7], byteorder='little')
                self.static_energy = int(packet[7])

                if self.static_target_cm > MAX_LEGIT_DISTANCE:
                    continue
                if self.static_energy > MAX_LEGIT_ENERGY:
                    continue

            if target_state:
                self.detection_cm = int.from_bytes(packet[8:10], byteorder='little')
                if self.detection_cm > MAX_LEGIT_DISTANCE:
                    continue

            # clear these, since they're no longer valid
            self.gate_motion_energy = [None] * 9
            self.gate_static_energy = [None] * 9 
        
            # we're done.
            if report_mode == REPORT_MODE_BASIC:
                # sucess (basic mode)
                self.last_updated = time.time()
                return True

            # In "engineering mode":
            # byte 10: highest configured motion gate
            # byte 11: highest configured static gate
            # bytes 12-20: motion energy
            # bytes 21-29: static energy
            # byte 30: light level
            # byte 31: is the output pin on?
            # byte -2: "tail" value of 55 
            # byte -1: "calibration" value of 00
            
            # TODO: this is redundant with info from read_config()
            # Should it be used to fill things out, or as a correctness check?
            self.last_motion_gate = int(packet[10])
            self.last_static_gate = int(packet[11])

            for i in range(self.last_motion_gate+1):
                self.gate_motion_energy[i] = int(packet[12+i])
                if self.gate_motion_energy[i] > MAX_LEGIT_ENERGY:
                    continue
            for i in range(self.last_static_gate+1):
                self.gate_static_energy[i] = int(packet[21+i])
                if self.gate_motion_energy[i] > MAX_LEGIT_ENERGY:
                    continue

            self.light_level = int(packet[30])


            # this is whether the precence line is raised, which
            # should be the same as self.detected
            assert(bool(packet[31]) == self.detected)
            
            # success! (engineering mode)
            self.last_updated = time.time()
            return True

        if failure_count:
            # TODO: raise a timeout error or something.
            print(f"That took {failure_count} attempts.")
        return False
        
    def _send(self,command_data_bytes):
        """Write to sensor. 
        
        command_data_bytes should be the literal "inner packet".
        
        Note that config-mode needs to be enabled before other
        commands, and disabled after. Therefore, _send_command()
        is probably more useful generally.
        
        On ack, returns response as bytestring if any, or True if not.
        On nak, returns None
        
        Should raise an exception for serial port problems.
        """

        # wrap the packet up and send
        packet = COMMAND_HEADER + len(command_data_bytes).to_bytes(2,"little") + command_data_bytes + COMMAND_FOOTER
        
        self.port.write(packet)


        # now listen for the (correct) response
        for _ in range(10):
            # the response uses the same header as the command packet
            if not self._scan_for_header(COMMAND_HEADER):
                #print("Didn't find response header.")
                continue

            packet_len = int.from_bytes(self.port.read(2), byteorder='little')
        
            # It would be more robust to check for the exact expected length
            # for each particular command, but gets clunky. So, at least make
            # sure it isn't unreasonable
            if packet_len > MAX_LEGIT_RESPONSE_LENGTH:
                #print("Response unreasonably long")
                continue

            packet = self.port.read(packet_len)
            status = packet[:4]
            result = packet[4:]
            footer = self.port.read(4)

            if footer != COMMAND_FOOTER:
                #print("End-of_Frame marker doesn't match.")
                continue

            # the response should include echoing back the command, which
            # will be the first byte of the command + data
            if int(status[0]) == 0:
                #print("Uh, that shouldn't be zero")
                continue
            if int(status[0]) != command_data_bytes[0]:
                #print(f"Response {int(packet[0])} doesn't match {command_data_bytes[0]}")
                continue
            if int(status[0]) != command_data_bytes[0]:
                #print(f"Response {int(packet[0])} doesn't match {command_data_bytes[0]}")
                continue


            if status[1] == 0:
                #print("Got failure response to command.")
                return None

            if status[1] != 1:
                #print("Response code something other than 0 or 1")
                continue

            if len(result) == 0:
                # success, but no data for this command
                return True
            
            # success with data (without the status code)
            return(result)
                    
        raise TimeoutError
    
    def _command(self,command,data=None):
        """Enter config mode, send a command and optional data,
        and then exit config mode.
        
        Returns True or a bytestring on success, or None on failure."""
        
        for _retries in range(10):
            try:
                if self._send(COMMAND_CONFIG_ENABLE + CONFIG_PROTOCOL_VERSION) == None:
                    continue          

                if data:
                    rc=self._send(command + data)
                else:
                    rc=self._send(command)

                if rc == False:
                    continue
                
                if self._send(COMMAND_CONFIG_DISABLE) == None:
                    continue

                break

            except TimeoutError:
                continue            
        else:
            # too many timeouts
            return False
        
        # success!
        return rc
                

    def _scan_for_header(self,header):
        # scan for data packet header
        buffer = [0] * 4
        for _ in range(33):
            buffer.pop(0)
            buffer.append(self.port.read(1))
            if not buffer[-1]: # timeout
                break
            if buffer == [x.to_bytes() for x in header]:                                
                return True
        self.serial_failures += 1
        return False

    def set_basic_config(self,last_motion_gate,last_static_gate,presence_timeout):
        """Configures the maximum distance gates and the presence_timeout. See §2.2.3 in the docs.

        This is a weird API. Like, why are these all one command?

        While presence_timeout is handy if you're just want a binary "someone there?"
        value, setting the gate limit is kind of pointless when setting gate sensitivity
        does the same thing in a more flexible way. Still, here it is!

        BUG: setting the gates doesn't actually _work_. Low priority for me because of the above.
        I'm tempted to fix this by just making this `set_presence_timeout` and locking the
        others to 8,8.
        """
        if last_motion_gate<2 or last_motion_gate>LAST_GATE:
            raise ValueError(f"Motion gate number {last_motion_gate} out of range 2-{LAST_GATE}")
        if last_static_gate<2 or last_static_gate>LAST_GATE:
            raise ValueError(f"Static gate number {last_static_gate} out of range 2-{LAST_GATE}")
        if presence_timeout < 0 or presence_timeout > MAX_LEGIT_PRESENCE_TIMEOUT:
            raise ValueError(f"Presence timeout {presence_timeout} out of range 0-{MAX_LEGIT_PRESENCE_TIMEOUT}")

        data = int(0).to_bytes(2,"little") + int(last_motion_gate).to_bytes(4,"little") + int(1).to_bytes(2,"little") + int(last_static_gate).to_bytes(4,"little") + int(2).to_bytes(2,"little") + int(presence_timeout).to_bytes(4,"little")
        
        rc = self._command(COMMAND_BASIC_CONFIG,data)
        self.read_config()
        # TODO: verify that the read-back values are what we just alledgedly set. 
        return rc

    def read_config(self):
        """Reads various configuration parameters and populates the corresponding attributes."""
        for _failure_count in range(10):
            result = self._command(COMMAND_READ_CONFIG)
            if result == None:
                continue

            if result[0] != 0xaa: # internal packet start magic code
                continue

            # next three bytes are configured largest gate.
            
            
            # TODO: this is not documented. How does it relate to the
            # motion and static last gate parameters? Weird.
            _last_gate = result[1]
        
            last_motion_gate = result[2]
            if last_motion_gate > LAST_GATE:
                continue
            last_static_gate = result[3]
            if last_static_gate > LAST_GATE:
                continue
            
            self.last_motion_gate = last_motion_gate
            self.last_static_gate = last_static_gate

            #  the next 9 are motion gate sensitivity
            #  then next 9 are static gate sensitivity
            for i in range(self.last_motion_gate+1):
                self.gate_motion_sensitivity[i] = int(result[4+i])
                if self.gate_motion_sensitivity[i] > MAX_LEGIT_ENERGY:
                    continue
            for i in range(self.last_static_gate+1):
                self.gate_static_sensitivity[i] = int(result[13+i])
                if self.gate_static_sensitivity[i] > MAX_LEGIT_ENERGY:
                    continue


            # then last 2 are little-endian presence timeout
            presence_timeout = int.from_bytes(result[22:], byteorder='little')
            if presence_timeout > MAX_LEGIT_PRESENCE_TIMEOUT:
                continue
            self.presence_timeout = presence_timeout

            return True

        # if we didn't succeed, well, then, must be...
        return False

    def enable_engineering_mode(self):
        """With this on, the LD2410 returns individual gate status 
        in addition to basic data."""
        return self._command(COMMAND_ENG_MODE_ENABLE)

    def disable_engineering_mode(self):
        """With this off, the LD2410 returns only basic data."""
        return self._command(COMMAND_ENG_MODE_DISABLE)
    
    def set_gate_sensitivity(self,gate,motion_sensitivity,static_sensitivity):
        """Sets the motion and static sensitivity thresholds for
        each gate. This affects the basic mode readings; you can
        still process the engineering-mode energy values however you
        want.

        Note that lower is _more_ sensitive. A gate with sensitivity 0 will
        always count as detecting something. Setting a gate's sensitivity to
        100 effectively disables detection for that gate (but, again energy
        levels are still reported in engineering mode.)

        Gates 0 and 1 do not 

        Note: as far as I can tell, the API allows omitting the static value (and
        leaving that unchanged) but does not change that static value if just that
        is provided. This seems to be undocumented behavior, so best to just always
        set both, I think.

        Attributes:
        
        gate -- the gate to configure. Use -1 for all.
        motion sensitivity -- motion energy threshold for this gate (0-100)
        static sensitivity -- static energy threshold for this gate (0-100)
        """
        if motion_sensitivity < 0 or motion_sensitivity > 100:
            raise ValueError(f"Motion sensitivity {motion_sensitivity} out of range 0-100")
        if static_sensitivity < 0 or static_sensitivity > 100:
            raise ValueError(f"Statiction sensitivity {static_sensitivity} out of range 0-100")        
        if gate<-1 or gate>LAST_GATE:
            raise ValueError(f"Gate number {gate} out of range -1-{LAST_GATE}")
        
        if gate == -1:
            gate = 0xFFFF

        data = int(0).to_bytes(2,"little") + int(gate).to_bytes(4,"little") + int(1).to_bytes(2,"little") + int(motion_sensitivity).to_bytes(4,"little") + int(2).to_bytes(2,"little") + int(static_sensitivity).to_bytes(4,"little")
        
        rc = self._command(COMMAND_GATE_SENSITIVITY,data)
        self.read_config()
        # TODO: verify that the read-back values are what we just alledgedly set. 
        return rc

    def get_firmware_version(self):
        """Read firmware version -- call this and then check the `firmware` attribute"""

        for _failure_count in range(10):
            result = self._command(COMMAND_FIRMWARE_VERSION)
            if result == None:
                continue

        # FIXME: proper error handling here
            
        # This is all really weird. The version is actually expressed as
        # hex values in reverse -- with the first two being some kind of
        # undocumented "type"
        if result[:2] != (0x0001).to_bytes(2,"big"):
            raise SystemError(f"Unknown firmware type {result[:2]}.")
        major = f"V{result[3]:x}.{result[2]:02x}"
        minor = f"{result[7]:2x}{result[6]:02x}{result[5]:02x}{result[4]:02x}"
        self.firmware_version = major + "." + minor


    def set_baudrate(self,baud=DEFAULT_BAUD):
        """Changes the port speed. If the default 256000 doesn't work cleanly, 
        it's probably best to use Bluetooth and the app to lower it to 57600 (or
        whatever does work for you). But here's the command in case you want it!
        """
        code = {  9600: 0x01,
                    19200: 0x02,
                    38400: 0x03,
                    57600: 0x04,
                115200: 0x05,
                230400: 0x06,
                256000: 0x07,
                460800: 0x08,
        }            
        return self._command(COMMAND_BAUD,code[baud].to_bytes(2,"little"))

    def reset_config(self):
        """Resets everything to factory defaults, including the values that
        normally persist across reboots. If you need a baud rate lower than
        256000, this may be annoying. This takes effect on the next restart."""
        return self._command(COMMAND_RESET_CONFIG)

    def restart(self):
        """Reboots the device, which resets some settings."""
        return self._command(COMMAND_RESTART)

    def bluetooth(self,on=True):
        """Turns off the bluetooth interface. Probably a good idea to turn
        off for security in production -- but it's very handy to have available
        in development!
        """

        code = 0x01 if on else 0x00
        return self._command(COMMAND_BLUETOOTH,(0x00).to_bytes(2,"little"))
    
    # TODO (and I guess call this in __init__, why not)
    def mac_addr(self):    
        raise NotImplemented
    
    # TODO -- except, I think this _only_ makes sense on a bluetooth link!
    def bluetooth_permission(self,password):
        raise NotImplemented
    
    # TODO
    def bluetooth_password(self,password):
        raise NotImplemented
    
    def set_resolution(self,resolution):
        """
        Sets the distance represnted by each gate, in cm. The default is 75, which gives a
        total range of about 6.75m. You can also switch to 20m, for about 1.8m.

        Note: takes effect after restart.

        Attributes:

        resolution: One of 20 or 75

        
        """
        code = {75: 0x00,
                20: 0x01,
        }
            
        return self._command(COMMAND_SET_RESOLUTION,code[resolution].to_bytes(2,"little"))
        # Note: _not_ calling self.get_resolution() because the value gets written but
        # doesn't take effect until restart.     
        
    
    def get_resolution(self):
        """Get current distance resolution -- the depth of each gate.
           With the LD2410, this is either 20 or 75.
        """
        code = {(0x00).to_bytes(2,"little"): 75,
                (0x01).to_bytes(2,"little"): 20,
        }


        for _failure_count in range(10):
            result = self._command(COMMAND_GET_RESOLUTION)
            if result == None:
                continue
            try:
                resolution = code[result]
            except KeyError:
                continue
            
            # TODO: proper error handling.

            self.resolution = resolution
            return self.resolution

