__author__ = "Stuart Marshall"
__copyright__ = "Copyright 2019, bThere.ai"

import blog


PARITY_NONE = True
STOPBITS_ONE = True
EIGHTBITS = True


class Serial(object):

    def __init__(self, port, baudrate, parity, stopbits, bytesize, timeout, write_timeout):
        self.portstr = port
        blog.i("mock Serial.__init__")

    def close(self):
        blog.i("Serial mock close called")

    def write(self, data):
        blog.i("Serial mock write called")

    def flushOutput(self):
        blog.i("Serial mock flushOutput called")
