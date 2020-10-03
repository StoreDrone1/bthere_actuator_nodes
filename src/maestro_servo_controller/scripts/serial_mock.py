__copyright__ = "Copyright 2019, bThere.ai"

# This file assist with testing the outer code without the actual presence of or communication with a servo controller

import bthere_log


PARITY_NONE = True
STOPBITS_ONE = True
EIGHTBITS = True


class Serial(object):

    def __init__(self, port, baudrate, parity, stopbits, bytesize, timeout, write_timeout):
        self.portstr = port
        bthere_log.i("mock Serial.__init__")

    def close(self):
        bthere_log.i("Serial mock close called")

    def write(self, data):
        bthere_log.i("Serial mock write called")

    def flushOutput(self):
        bthere_log.i("Serial mock flushOutput called")
