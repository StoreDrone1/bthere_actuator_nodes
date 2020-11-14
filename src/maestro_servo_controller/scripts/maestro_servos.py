__copyright__ = "Copyright 2019, bThere.ai"

# This file handles communication with a Pololu maestro servo controller.
# This file assumes communication is in USB dual mode.
# There's also an assumption that there are two servos attached to the controller, on channels 1 and 2.

import config
import bthere_log
import os.path
import binascii
import math
import string
import struct
import time

print("maestro_servos")

if config.args['mock_servos']:
    bthere_log.i("Using mock serial")
    import serial_mock as serial
else:
    bthere_log.i("Using real serial")
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
    bthere_log.i("Linux platform, so setting command serial port to " +
                 command_serial_port + " and ttl serial port to " + ttl_serial_port)
elif config.args['platform'] == "mac":
    command_serial_port = config.get_config_or_default(
        "maestro_command_serial_port", command_port_mac)
    ttl_serial_port = config.get_config_or_default(
        "maestro_ttl_serial_port", ttl_serial_port_mac)
    bthere_log.i("Mac platform, so setting command serial port to " +
                 command_serial_port + " and ttl serial port to " + ttl_serial_port)
else:
    bthere_log.e(
        "Can't set serial port because platform is neither linux nor mac")
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
pan_servo2_channel = 2
tilt_servo2_channel = 3

set_target_cmd = 0x84

pan_min = 0
pan_max = 100
tilt_min = 0
tilt_max = 100

servo_min = 2100
servo_max = 900
servo_scale_factor = 4

currentPan1 = 50
currentTilt1 = 50
currentPan2 = 50
currentTilt2 = 50


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
    bthere_log.i("serial cleaned up")


def get_servo_data_value(percent):
    servo_range = servo_max - servo_min
    return int((((float(percent)/100) * servo_range) + servo_min) * servo_scale_factor)


def panTilt(panValue, tiltValue, servoNum):
    bthere_log.i('panTilt input. Pan: ' + str(panValue) +
                 ' Tilt: ' + str(tiltValue))
    global currentPan1
    global currentTilt1
    global currentPan2
    global currentTilt2
    
    if servoNum == 1:
        newPan = currentPan1 + panValue
        newTilt = currentTilt1 + tiltValue
        pan(newPan, servoNum)
        tilt(newTilt, servoNum)
    if servoNum == 2:
        newPan = currentPan2 + panValue
        newTilt = currentTilt2 + tiltValue
        pan(newPan, servoNum)
        tilt(newTilt, servoNum)

def pan(amount, servoNum):
    amount = clamp(amount, 0, 100)
    global currentPan1
    global currentPan2
    
    if servoNum == 1:
        if amount == currentPan1:
            return
        currentPan1 = amount
        data = get_servo_data_value(amount)
        control_servo(pan_servo_channel, data)
        bthere_log.i("Panning to " + str(amount) + " on servo: " + str(servoNum))
    if servoNum == 2:
        if amount == currentPan2:
            return
        currentPan2 = amount
        data = get_servo_data_value(amount)
        control_servo(pan_servo2_channel, data)
        bthere_log.i("Panning to " + str(amount) + " on servo: " + str(servoNum))

def tilt(amount, servoNum):
    amount = clamp(amount, 0, 100)
    global currentTilt1
    global currentTilt2
    
    if servoNum == 1:
        if amount == currentTilt1:
            return
        currentTilt1 = amount
        data = get_servo_data_value(amount)
        control_servo(tilt_servo_channel, data)
        bthere_log.i("Tilting to " + str(amount) + " on servo: " + str(servoNum))
    if servoNum == 2:
        if amount == currentTilt2:
            return
        currentTilt2 = amount
        data = get_servo_data_value(amount)
        control_servo(tilt_servo2_channel, data)
        bthere_log.i("Tilting to " + str(amount) + " on servo: " + str(servoNum))

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
    pan(pan_position, 1)
    pan(pan_position, 2)
    tilt(tilt_position, 1)
    tilt(tilt_position, 2)


def pack(number):
    packed = struct.pack("B", number)
    bthere_log.i(binascii.hexlify(packed))
    if verbose:
        bthere_log.i("Packed data: " + binascii.hexlify(packed))
    return packed


def pack_command_to_channel(channel, command, data):
    packed = struct.pack("BBBB", command, channel,
                         data & 0x7F, (data >> 7) & 0x7F)
    if verbose:
        bthere_log.i("Packed data: " + binascii.hexlify(packed))
    return packed


def setup():
    global ser
    global is_setup
    if not is_setup:
        # spin looking to make sure the ttyUSB exists
        if config.args['mock_servos']:
            bthere_log.i(
                "Using mock serial, so not gonna check if the serial port device exists")
        else:
            if os.path.exists(command_serial_port) is False:
                bthere_log.i("Serial port not available yet. Waiting " +
                             str(time_to_wait_for_usb) + " milliseconds in hopes that it'll show up")
                time.sleep(time_to_wait_for_usb/1000.0)
                if os.path.exists(command_serial_port) is False:
                    bthere_log.e(
                        "Serial port still isn't available. Giving up")
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
        bthere_log.i("connected to: " + ser.portstr)
