__author__ = "Stuart Marshall"
__copyright__ = "Copyright 2019, bThere.ai"

import MLog

def cleanup():
    MLog.i("servo mock cleaned up")


def pan(amount):
    MLog.i("servo mock panning to " + str(amount))


def tilt(amount):
    MLog.i("servo mock tilting to " + str(amount))


def reset_cameras():
    global pan_position
    global tilt_position
    pan_position = 50
    tilt_position = 50
    pan(pan_position)
    tilt(tilt_position)


def setup():
    MLog.i("mock servos setup")

