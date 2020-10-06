#!/usr/bin/env python

__copyright__ = "Copyright 2020, bThere.ai"

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import enum
import sys

LINEAR_SPEED_SCALE_FACTOR = 100

# Dutycycle to switch off motors
STOP = 0


class Motor(object):
    """generic parent class for motor classes."""
    pwm_pin = -1
    pwm_obj = None
    max_allowable_dc = 100
    pwm_frequency = 20

    def __init__(self, pwm_pin, max_allowable_dc, pwm_frequency):
        self.pwm_pin = pwm_pin
        self.pwm_frequency = pwm_frequency
        self.max_allowable_dc = max_allowable_dc
        GPIO.setup(pwm_pin, GPIO.OUT, initial=GPIO.LOW)
        self.pwm_obj = GPIO.PWM(pwm_pin, pwm_frequency)
        self.pwm_obj.start(0)  # start stopped

    def set_power(self, power):
        """set only the power pwm."""
        if power < 0 or power > 1:
            raise ValueError(
                "set_power can only take a value between 0 and 1.")

        # To achieve linear response use the square of the normalized speeds
        power *= abs(power)
        power *= LINEAR_SPEED_SCALE_FACTOR
        power = min(power, self.max_allowable_dc)
        self.pwm_obj.ChangeDutyCycle(round(power))

    def stop(self):
        self.set_power(STOP)


class Motor2Pin(Motor):
    """class to control a motor (through an H-bridge) using 2 pins: pwm and direction.
    This class acts as an abstraction layer for a GPIO interface for a motor controller
    or H-bridge that uses sign-magnitude control via one pin providing PWM for magnitude
    and the other being set to either high or low to specify the direction.
    This class was specifically made for use with a Cytron MDD10A motor driver, but will
    also work with other motor controllers and H-bridges that use the same system for control."""
    dir_pin = -1
    dir_forward_val = -1
    dir_backward_val = -1

    def __init__(self, pwm_pin, dir_pin, dir_forward_val, max_allowable_dc, pwm_frequency):
        super(Motor2Pin, self).__init__(
            pwm_pin, max_allowable_dc, pwm_frequency)
        self.dir_pin = dir_pin
        self.dir_forward_val = dir_forward_val
        self.dir_backward_val = GPIO.HIGH if dir_forward_val == GPIO.LOW else GPIO.LOW
        GPIO.setup(dir_pin, GPIO.OUT, initial=dir_forward_val)
        print("Adding Motor2Pin - pwm_pin: " + pwm_pin + ", direction pin: " + dir_pin +
              "forward value: " + ("high" if dir_forward_val == GPIO.HIGH else "low"))

    def set(self, power):
        """sets power and direction, using power, a value between -1 (backwards) and 1 (forwards)"""
        if power > 1 or power < -1:
            raise ValueException(
                "set() can only take a value between -1 and 1.")

        self.set_power(abs(power))
        if power > 0:  # forwards
            GPIO.output(self.dir_pin, self.dir_forward_val)
        elif power < 0:  # backwards
            GPIO.output(self.dir_pin, self.dir_backward_val)


class Motor3Pin(Motor):
    """class to control a motor (through an H-bridge) using 3 pins: pwm, forward, and back.
    This class acts as an abstraction layer for a GPIO interface with a motor controller or
    H-bridge that uses a variation of sign-magnitude control where one pin is used for 
    magnitude control using PWM, and two pins are used to control the direction (a particular
    pin is set high and the other low for one direction, and inverse for the opposite direction.)
    This class was specifically made for use with L298N H-bridges, but will also work with 
    other H-bridges or motor controllers that use the same control system."""
    forward_pin = -1
    backward_pin = -1

    def __init__(self, pwm_pin, forward_pin, backward_pin, max_allowable_dc, pwm_frequency):
        super(Motor3Pin, self).__init__(
            pwm_pin, max_allowable_dc, pwm_frequency)
        self.forward_pin = forward_pin
        self.backward_pin = backward_pin
        GPIO.setup([forward_pin, backward_pin], GPIO.OUT, initial=GPIO.LOW)
        print("Adding Motor3Pin - pwm_pin: " + pwm_pin + ", forward pin: " + forward_pin + ", backward "
              + "pin:" + backward_pin)

# print("adding motor" + "br" + ", pwm pin: " + str(22) + ", dir pin: " + str(17))

    def set(self, power):
        """sets power and direction, using power, a value between -1 (backwards) and 1 (forwards)"""
        if power > 1 or power < -1:
            raise ValueException(
                "set() can only take a value between -1 and 1.")

        self.set_power(abs(power))

        if power > 0:  # forwards
            GPIO.output(self.forward_pin, GPIO.HIGH)
            GPIO.output(self.backward_pin, GPIO.LOW)
        elif power < 0:  # backwards
            GPIO.output(self.forward_pin, GPIO.LOW)
            GPIO.output(self.backward_pin, GPIO.HIGH)


class ControlMode(enum.Enum):
    """supported types of control.
    tank: anything similar to a tracked chassis, i.e. controlled with a left speed and
          a right speed only.
    mecanum: a four-wheel standard mecanum chassis."""
    tank = 1
    mecanum = 2


# ========================================================================================================
# CONFIG

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Maximum allowable dutycycle
MAX_ALLOWABLE_DC = 100

# PWM Frequency.
# this is going to depend on your motors and H-bridge.

# TODO: [for visibility]
# !!! CONFIGURE THIS PER ROBOT !!!
# actually todo: maybe use a config file of some kind to ensure this isn't run before being configured?
#      it could be dangerous for someone to run this script unconfigured.
motors = {}

# EXAMPLE CONFIG:
#   This is the configuration for a mecanum-drive based robot using two Cytron MDD10A motor controllers.
#   The Motor2Pin constructor takes 5 arguments: the pwm pin #, the direction pin #, the pin status
#   (GPIO.HIGH vs GPIO.LOW) that is "forward", the maximum allowable duty cycle (0-100, to avoid burning
#   out motors with too high of voltage), and the PWM freqency.
#   See the Motor2Pin docstring for more info on this class.
#
# motors['fl'] = Motor2Pin(15, 24, GPIO.LOW, 100, FREQ)
# motors['bl'] = Motor2Pin(23, 14, GPIO.LOW, 100, FREQ)
# motors['fr'] = Motor2Pin(27, 18, GPIO.HIGH, 100, FREQ)
# motors['br'] = Motor2Pin(22, 17, GPIO.HIGH, 100, FREQ)
#
# the control mode to be used:
# control_mode = ControlMode.mecanum

# EXAMPLE CONFIG:
#   This shows an example of how this script can be configured to control a robot that uses a tank-style
#   chassis and an L298N H-bridge.
#   the Motor3Pin constructor takes the same parameters as Motor2Pin, but instead of taking direction pin
#   number and a value for which is forward, it takes a forward pin and backward pin number.
#   for more info see the docstring for Motor3Pin.
#
# motors['l'] = Motor3Pin(15, 24, 23, 100, FREQ)
# motors['r'] = Motor3Pin(14, 27, 18, 100, FREQ)
#
# the control mode to be used:
# control_mode = ControlMode.tank


# FIXME: uncomment out this next line once you've added your configuration.
control_mode = None

# ========================================================================================================


def StopMotors():
    """stop all motors"""
    # GPIO.output(dir_pins, GPIO.LOW)
    for motor in motors:
        motors[motor].stop()


def set_and_log_pwr(motorname, power):
    """double checks the given motor name is configured, clamps the power to the
    range of -1 <= power <= 1, and sets the power using the motor object in motors."""
    if not motorname in motors:
        raise ValueError("No motor of name " + motorname + " configured!")
    power = max(min(power, 1), -1)
    rospy.loginfo('setting ' + motorname + 'power to: ' + str(power))
    motors[motorname].set(power)


def cmd_callback(cmdMessage):
    linear_x = cmdMessage.linear.x
    angular_z = cmdMessage.angular.z

    if control_mode == None:
        rospy.logerr("cannot run rpi motor controller, control_mode not set!"
                     + "\nMake sure you have configured the script before use!")
    elif control_mode == ControlMode.tank:
        right_speed = 1.0 * linear_x - angular_z
        left_speed = 1.0 * linear_x + angular_z

        set_and_log_pwr("l", left_speed)
        set_and_log_pwr("r", right_speed)

    elif control_mode == ControlMode.mecanum:
        fr_speed = cmdMessage.linear.x - cmdMessage.linear.y + angular_z
        fl_speed = cmdMessage.linear.x + cmdMessage.linear.y - angular_z
        bl_speed = cmdMessage.linear.x - cmdMessage.linear.y - angular_z
        br_speed = cmdMessage.linear.x + cmdMessage.linear.y + angular_z

        set_and_log_pwr("fr", fr_speed)
        set_and_log_pwr("fl", fl_speed)
        set_and_log_pwr("bl", bl_speed)
        set_and_log_pwr("br", br_speed)

    # This script should optimally be using a deadman switch so
    # if whatever is publishing to the input topic breaks, the
    # robot doesn't go driving off ad infinitum


def init_subscriber():
    topic = rospy.get_param("~topic_name", "/cmd_vel/teleop")
    rospy.loginfo("listening for input on " + topic)
    rospy.Subscriber(topic, Twist, cmd_callback)


def create_node():
    rospy.init_node('pi_motor_controller', anonymous=False)


if __name__ == '__main__':
    rospy.logerr("cannot run rpi motor controller, control_mode not set!"
                 + "\nMake sure you have configured the script before use!")
    sys.exit()

    create_node()
    init_subscriber()
    rospy.loginfo("initialized subscriber")
    rospy.spin()
    StopMotors()
    GPIO.cleanup()
