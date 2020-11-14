#!/usr/bin/env python

# This is the main code that listens on a ROS topic, interprets received twists and calls the servo code

__copyright__ = "Copyright 2020, bThere.ai"

import bthere_log
import rospy
import std_msgs
from geometry_msgs.msg import Twist
import maestro_servos

increment_scalar = 1


def callback1(twist):
    rospy.loginfo(rospy.get_caller_id() + " Received: " + str(twist))
    pan = int(twist.angular.z * increment_scalar)
    tilt = int(twist.angular.y * increment_scalar)
    if (pan == 0 and tilt == 0):
        return
    bthere_log.i('Pan incrementing by ' + str(pan) +
                 ' Tilt incrementing by ' + str(tilt) + 
                 ' on servo 1 ')
    maestro_servos.panTilt(pan, tilt, 1)
    
def callback2(twist):
    rospy.loginfo(rospy.get_caller_id() + " Received: " + str(twist))
    pan = int(twist.angular.z * increment_scalar)
    tilt = int(twist.angular.y * increment_scalar)
    if (pan == 0 and tilt == 0):
        return
    bthere_log.i('Pan incrementing by ' + str(pan) +
                 ' Tilt incrementing by ' + str(tilt) +
                 ' on servo 2 ')
    maestro_servos.panTilt(pan, tilt, 2)    


def listener(topic1, topic2):
    rospy.init_node('bthere_maestro_servos')
    rospy.Subscriber(topic1, Twist, callback1)
    rospy.Subscriber(topic2, Twist, callback2)
    rospy.spin()


def main():
    global top1
    global top2
    
    bthere_log.i("bthere_servos running...")
    top1 = rospy.get_param("~topic_name", "/camera_servo1/teleop")
    top2 = rospy.get_param("~topic_name", "/camera_servo2/teleop")
    maestro_servos.setup()
    listener(top1, top2)
