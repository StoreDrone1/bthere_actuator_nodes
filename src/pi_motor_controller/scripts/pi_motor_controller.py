#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import RPi.GPIO as GPIO

LINEAR_SPEED_SCALE_FACTOR = 100
ANGULAR_SPEED_SCALE_FACTOR = 1/4    # Inverse of time minimum time to make one revolution
RADIANS_PER_REVOLUTION = 6.28319

w_dist = 0.23       # The separation between the drive wheels
lwheel_pub = None   # Publisher for left wheel speed
rwheel_pub = None   # Publisher for right wheel speed

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Pin configurations
IN1 = 17    # Right direction control
IN2 = 18    # Right direction control
IN3 = 27    # Left direction control
IN4 = 22    # Left direction control
ENA = 12    # Right motor control
ENB = 13    # Left motor control

# Maximum allowable dutycycle
MAX_ALLOWABLE_DC = 73

# Hertz
FREQ = 20

# Dutycycle to switch off motors
STOP = 0

# Setup GPIO pins as outputs
dir_pins = [IN1, IN2, IN3, IN4]
GPIO.setup(dir_pins, GPIO.OUT, initial=GPIO.LOW)

# Setup PWM pins
PWM_PINS = [ENA, ENB]
GPIO.setup(PWM_PINS, GPIO.OUT, initial=GPIO.LOW)
pwmMotorR = GPIO.PWM(ENA, FREQ)
pwmMotorL = GPIO.PWM(ENB, FREQ)

pwmMotorR.start(STOP)
pwmMotorL.start(STOP)


def StopMotors():
    GPIO.output(dir_pins, GPIO.LOW)


def right_direction(direction):
    if direction == 'F':
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
    else:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)


def left_direction(direction):
    if direction == 'F':
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
    else:
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)


def cmd_callback(cmdMessage):
    global rwheel_pub, lwheel_pub
    linear_x = cmdMessage.linear.x
    angular_z = cmdMessage.angular.z

    right_speed = 1.0 * linear_x - angular_z
    left_speed = 1.0 * linear_x + angular_z

    # To achieve linear response use the square of the normalized speeds
    right_speed = right_speed * abs(right_speed)
    left_speed = left_speed * abs(left_speed)

    # Amplify speeds to correct levels
    right_speed = right_speed * LINEAR_SPEED_SCALE_FACTOR
    left_speed = left_speed * LINEAR_SPEED_SCALE_FACTOR

    # Publish the left and right speeds
    print(right_speed, left_speed)
    rwheel_pub.publish(right_speed)
    lwheel_pub.publish(left_speed)

    if right_speed > 0:
        right_direction('F')
    else:
        right_direction('B')

    if left_speed > 0:
        left_direction('F')
    else:
        left_direction('B')

    if abs(right_speed) > MAX_ALLOWABLE_DC:
        right_speed = MAX_ALLOWABLE_DC

    if abs(left_speed) > MAX_ALLOWABLE_DC:
        left_speed = MAX_ALLOWABLE_DC

    pwmMotorR.ChangeDutyCycle(int(abs(right_speed)))
    pwmMotorL.ChangeDutyCycle(int(abs(left_speed)))

    # You should build in a deadman switch, however, the bThereClient already has one


def init_publisher():
    global lwheel_pub, rwheel_pub
    lwheel_pub = rospy.Publisher('lwheel_vtarget', Float32, queue_size=10)
    rwheel_pub = rospy.Publisher('rwheel_vtarget', Float32, queue_size=10)


def init_subscriber():
    rospy.Subscriber('/cmd_vel/teleop', Twist, cmd_callback)


def create_node():
    rospy.init_node('pi_motor_controller', anonymous=False)


if __name__ == '__main__':
    try:
        init_publisher()
    except rospy.ROSInterruptException:
        pass

    create_node()
    init_subscriber()
    rospy.spin()
    StopMotors()
    GPIO.cleanup()
