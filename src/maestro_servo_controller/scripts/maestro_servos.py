__author__ = "Stuart Marshall"
__copyright__ = "Copyright 2019, bThere.ai"

import config
import blog
import os.path
import binascii
import math
import string
import struct
import time

__author__ = 'stuart'

print("maestro_servos")

if config.args['mock_servos']:
    blog.i("Using mock serial")
    import serial_mock as serial
else:
    blog.i("Using real serial")
    import serial as serial

# mac ports are not valid - just guesses right now
command_port_mac = "/dev/cu.usbmodem00654321"
ttl_serial_port_mac = "/dev/cu.usbmodem00654322"
command_port_linux = "/dev/ttyACM0"
ttl_serial_port_linux = "/dev/ttyACM0"

time_to_wait_for_usb = 1000

if config.args['platform'] == "linux":
    command_serial_port = config.get_config_or_default(
        "maestro_command_serial_port", command_port_linux)
    ttl_serial_port = config.get_config_or_default(
        "maestro_ttl_serial_port", ttl_serial_port_linux)
    blog.i("Linux platform, so setting command serial port to " +
           command_serial_port + " and ttl serial port to " + ttl_serial_port)
elif config.args['platform'] == "mac":
    command_serial_port = config.get_config_or_default(
        "maestro_command_serial_port", command_port_mac)
    ttl_serial_port = config.get_config_or_default(
        "maestro_ttl_serial_port", ttl_serial_port_mac)
    blog.i("Mac platform, so setting command serial port to " +
           command_serial_port + " and ttl serial port to " + ttl_serial_port)
else:
    blog.e("Can't set serial port because platform is neither linux nor mac")
    command_serial_port = ""
    ttl_serial_port = ""

flush_never = 0
flush_on_command = 1
flush_on_write = 2

flush_policy = flush_on_command

verbose = False
is_setup = False

ser = None

use_simple_serial = True

# todo: move to config
baud_rate = 0xAA
default_controller_address = 12
pan_servo_channel = 0
tilt_servo_channel = 1

set_target_cmd = 0x84

pan_min = 0
pan_max = 100
tilt_min = 0
tilt_max = 100

servo_min = 2100
servo_max = 900
servo_scale_factor = 4

currentPan = 50
currentTilt = 50


def clamp(input, lower_bound, upper_bound):
    if input < lower_bound:
        return lower_bound
    elif input > upper_bound:
        return upper_bound
    else:
        return input


def cleanup():
    global ser
    if ser is not None:
        ser.close()
        ser = None
    blog.i("serial cleaned up")


def get_servo_data_value(percent):
    servo_range = servo_max - servo_min
    return int((((float(percent)/100) * servo_range) + servo_min) * servo_scale_factor)


def panTilt(panValue, tiltValue):
    blog.i('panTilt input. Pan: ' + str(panValue) + ' Tilt: ' + str(tiltValue))
    global currentPan
    global currentTilt
    newPan = currentPan + panValue
    newTilt = currentTilt + tiltValue
    pan(newPan)
    tilt(newTilt)


def pan(amount):
    amount = clamp(amount, 0, 100)
    global currentPan
    if amount == currentPan:
        return
    currentPan = amount
    data = get_servo_data_value(amount)
    control_servo(pan_servo_channel, data)
    blog.i("Panning to " + str(amount))


def tilt(amount):
    amount = clamp(amount, 0, 100)
    global currentTilt
    if amount == currentTilt:
        return
    currentTilt = amount
    data = get_servo_data_value(amount)
    control_servo(tilt_servo_channel, data)
    blog.i("Tilting to " + str(amount))


def control_servo(channel, amount):
    packed = pack_command_to_channel(channel, set_target_cmd, amount)
    ser.write(packed)
    if flush_policy == flush_on_write:
        ser.flushOutput()


def reset_cameras():
    global pan_position
    global tilt_position
    pan_position = 50
    tilt_position = 50
    pan(pan_position)
    tilt(tilt_position)


def pack(number):
    packed = struct.pack("B", number)
    blog.i(binascii.hexlify(packed))
    if verbose:
        blog.i("Packed data: " + binascii.hexlify(packed))
    return packed


def pack_command_to_channel(channel, command, data):
    packed = struct.pack("BBBB", command, channel,
                         data & 0x7F, (data >> 7) & 0x7F)
    if verbose:
        blog.i("Packed data: " + binascii.hexlify(packed))
    return packed


def setup():
    global ser
    global is_setup
    if not is_setup:
        # spin looking to make sure the ttyUSB exists
        if config.args['mock_servos']:
            blog.i(
                "Using mock serial, so not gonna check if the serial port device exists")
        else:
            if os.path.exists(command_serial_port) is False:
                blog.i("Serial port not available yet. Waiting " +
                       str(time_to_wait_for_usb) + " milliseconds in hopes that it'll show up")
                time.sleep(time_to_wait_for_usb/1000.0)
                if os.path.exists(command_serial_port) is False:
                    blog.e("Serial port still isn't available. Giving up")
                    raise Exception(
                        "Serial port not available: " + command_serial_port)
        ser = serial.Serial(
            port=command_serial_port,
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0,
            write_timeout=0)
        is_setup = True
        blog.i("connected to: " + ser.portstr)
