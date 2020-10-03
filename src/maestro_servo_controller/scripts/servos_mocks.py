__copyright__ = "Copyright 2019, bThere.ai"

import bthere_log


def cleanup():
    bthere_log.i("servo mock cleaned up")


def pan(amount):
    bthere_log.i("servo mock panning to " + str(amount))


def tilt(amount):
    bthere_log.i("servo mock tilting to " + str(amount))


def reset_cameras():
    global pan_position
    global tilt_position
    pan_position = 50
    tilt_position = 50
    pan(pan_position)
    tilt(tilt_position)


def setup():
    bthere_log.i("mock servos setup")
