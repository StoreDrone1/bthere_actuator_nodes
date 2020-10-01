#!/usr/bin/env python

import blog
import rospy
import std_msgs
from geometry_msgs.msg import Twist
import maestro_servos

increment_scalar = 1


def callback(twist):
    rospy.loginfo(rospy.get_caller_id() + " Received: " + str(twist))
    pan = int(twist.angular.z * increment_scalar)
    tilt = int(twist.angular.y * increment_scalar)
    blog.i('Pan incrementing by ' + str(pan) +
           ' Tilt incrementing by ' + str(tilt))
    maestro_servos.panTilt(str(pan) + ' ' + str(tilt))


def listener(topic):
    rospy.init_node('bthere_maestro_servos')
    rospy.Subscriber(topic, Twist, callback)
    rospy.spin()


def main():
    blog.i("bthere_servos running...")
    topic = rospy.get_param("~topic_name", "/camera_servo1/teleop")
    maestro_servos.setup()
    listener(topic)
